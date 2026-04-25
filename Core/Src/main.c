/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "comp.h"
#include "cordic.h"
#include "dac.h"
#include "dma.h"
#include "fmac.h"
#include "opamp.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_RATE         100000.0f   // 100 kHz DAC/ADC 主采样率
#define BUFFER_SIZE         256         // DMA 缓冲区大小
#define HALF_BUFFER_SIZE    (BUFFER_SIZE / 2)
#define DAC_MID_VALUE       2048        // 12-bit DAC 中值 (1.65V)
#define ADC_MID_VALUE       2048.0f     // 12-bit ADC 中值浮点表示

/*
 * 闭环 PI-IQ 控制器参数：
 * 自适应追踪未知 LTI 的衰减与相移
 */
#define ANC_KP              1.0f
#define ANC_KI              0.05f
#define ANC_OUT_LIMIT       2040.0f     // DAC码值限幅，防止削顶失真
#define ANC_INT_LIMIT       (ANC_OUT_LIMIT / ANC_KI)

/*
 * STM32G474 TIM5 时钟频率 (Hz)，用于硬件测频换算
 * 默认系统时钟 170 MHz，TIM5 挂载 APB1，TIM 时钟 = 170 MHz
 */
#define TIM5_CLOCK_HZ       170000000UL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern CORDIC_HandleTypeDef hcordic;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;
extern COMP_HandleTypeDef hcomp1;

/* [BUG FIX #3]
 * 原代码将 adc_buffer / dac_buffer 声明为标量 uint16_t，
 * 但后续代码以数组方式访问（pADC_Data[i]、HAL_ADC_Start_DMA 传长度256）。
 * 修正：声明为长度 BUFFER_SIZE 的数组。
 */
uint16_t adc_buffer[BUFFER_SIZE];
uint16_t dac_buffer[BUFFER_SIZE];

/* [BUG FIX #5]
 * 原代码声明 tim5_ic_buffer[1]（仅1个元素），
 * 但测频逻辑需要比较索引 [0] 和 [1]（两个相邻捕获值），
 * 且 HAL_TIM_IC_Start_DMA 传入长度2，会发生越界写入。
 * 修正：声明为2个元素的数组。
 */
uint32_t tim5_ic_buffer[2];

/* [BUG FIX #1]
 * 原代码 typedef struct 定义不完整，结构体被截断，
 * 缺少 phase_step / I_out / Q_out / Kp / Ki / I_integral / Q_integral /
 * I_err_acc / Q_err_acc 字段，且没有结束的 } ANC_State_t; 及变量声明。
 * 修正：补全完整结构体定义并声明全局实例 anc。
 */
typedef struct {
    float target_freq;   // 动态追踪的单音频率 (Hz)
    float phase;         // 实时相位累加器，范围 [0, 2π)
    float phase_step;    // 每采样点相位步进 = 2π * target_freq / SAMPLE_RATE
    float I_out;         // IQ 输出系数：同相分量幅度
    float Q_out;         // IQ 输出系数：正交分量幅度
    float Kp;            // PI 控制器比例增益
    float Ki;            // PI 控制器积分增益
    float I_integral;    // I 通道积分项累积
    float Q_integral;    // Q 通道积分项累积
    float I_err_acc;     // 当前帧 I 误差累加（帧内临时使用）
    float Q_err_acc;     // 当前帧 Q 误差累加（帧内临时使用）
} ANC_State_t;

static ANC_State_t anc = {
    .target_freq  = 1000.0f,   // 初始追踪频率，上电后由测频模块动态更新
    .phase        = 0.0f,
    .phase_step   = 2.0f * (float)M_PI * 1000.0f / SAMPLE_RATE,
    .I_out        = 0.0f,
    .Q_out        = 0.0f,
    .Kp           = ANC_KP,
    .Ki           = ANC_KI,
    .I_integral   = 0.0f,
    .Q_integral   = 0.0f,
    .I_err_acc    = 0.0f,
    .Q_err_acc    = 0.0f,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float clampf(float val, float lo, float hi);
static void ANC_Control_Loop(uint16_t *pADC_Data, uint16_t *pDAC_Data, uint16_t length);
static void update_freq(uint32_t timer_diff);
static void CORDIC_Config_Cosine(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  浮点限幅辅助函数
 */
static float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * @brief  配置 CORDIC 外设为余弦模式（q1.31，双结果：cos + sin）
 *
 * [BUG FIX #7]
 * 原代码从未配置 CORDIC->CSR 就直接写 WDATA，行为未定义。
 * STM32G474 CORDIC 上电默认 CSR=0（函数字段=0b0000，对应余弦），
 * 但精度、数据格式必须显式配置，否则结果不可靠。
 * 本函数设置：
 *   - FUNC   = 0000b → 余弦
 *   - PRECISION = 6  → 6次迭代（精度约 20 bit，满足音频应用）
 *   - ARGSIZE = 0    → 32-bit q1.31 输入
 *   - RESSIZE = 0    → 32-bit q1.31 输出
 *   - NARGS  = 0     → 1个输入寄存器（仅角度）
 *   - NRES   = 1     → 2个结果寄存器（cos 在前，sin 在后）
 * 应在外设初始化完成后、首次调用 ANC_Control_Loop 前调用一次。
 */
static void CORDIC_Config_Cosine(void)
{
    /* CORDIC CSR 位域（参见 RM0440 Table 196）：
     * [3:0]  FUNC      = 0000 → cosine
     * [7:4]  PRECISION = 0110 → 6 iterations
     * [9:8]  SCALE     = 00   → no scaling
     * [16]   IEN       = 0    → 无中断
     * [17]   DMAREN    = 0    → 不使用 DMA
     * [18]   DMAWEN    = 0
     * [19]   NRES      = 1    → 读两次 RDATA（cos, sin）
     * [20]   NARGS     = 0    → 写一次 WDATA（角度）
     * [21]   RESSIZE   = 0    → 32-bit 结果
     * [22]   ARGSIZE   = 0    → 32-bit 输入
     */
    CORDIC->CSR = (0x0UL  << CORDIC_CSR_FUNC_Pos)      /* cosine        */
                | (0x6UL  << CORDIC_CSR_PRECISION_Pos)  /* 6 iterations  */
                | (0x0UL  << CORDIC_CSR_SCALE_Pos)
                | (0x1UL  << CORDIC_CSR_NRES_Pos)       /* 2 results     */
                | (0x0UL  << CORDIC_CSR_NARGS_Pos)      /* 1 argument    */
                | (0x0UL  << CORDIC_CSR_RESSIZE_Pos)    /* 32-bit result */
                | (0x0UL  << CORDIC_CSR_ARGSIZE_Pos);   /* 32-bit input  */
}

/**
 * @brief  根据 TIM5 输入捕获差值更新追踪频率与相位步进
 *
 * [BUG FIX #9]
 * 原代码调用了 update_freq() 但从未定义该函数，导致链接错误。
 *
 * @param  timer_diff  两次相邻上升沿的 TIM5 计数差值（一个周期的计数数）
 *                     timer_diff = 0 时跳过（捕获溢出或初始化阶段保护）
 */
static void update_freq(uint32_t timer_diff)
{
    if (timer_diff == 0) return;

    float new_freq = (float)TIM5_CLOCK_HZ / (float)timer_diff;

    /* 简单低通滤波，防止偶发噪声突变 */
    anc.target_freq = 0.9f * anc.target_freq + 0.1f * new_freq;

    /* 同步更新相位步进；phase 累加器不复位，保持相位连续 */
    anc.phase_step  = 2.0f * (float)M_PI * anc.target_freq / SAMPLE_RATE;
}

/**
 * @brief  ANC 核心控制环：对一段 ADC 数据执行 IQ 解调 + PI 闭环 + DAC 输出生成
 *
 * @param  pADC_Data  ADC 采样数据指针（当前待处理半段首地址）
 * @param  pDAC_Data  DAC 输出数据指针（当前待写出半段首地址）
 * @param  length     本次处理长度（= HALF_BUFFER_SIZE）
 */
static void ANC_Control_Loop(uint16_t *pADC_Data, uint16_t *pDAC_Data, uint16_t length)
{
    anc.I_err_acc = 0.0f;
    anc.Q_err_acc = 0.0f;

    for (uint16_t i = 0; i < length; i++)
    {
        /* --- 1. 提取物理叠加误差 --- */
        float error_signal = (float)pADC_Data[i] - ADC_MID_VALUE;

        /* --- 2. CORDIC 极速正余弦解算 ---
         *
         * [BUG FIX #8]
         * 原代码将 anc.phase（维护在 [0, 2π) 内）减去 π 后赋给 map_phase，
         * 意图将其折叠到 [-π, π]，但只做了 > π 的单向判断，
         * 且折叠后仍需映射到 CORDIC q1.31 的 [-1, 1] 表示（即除以 π）。
         * 实际上 anc.phase 已在 [0, 2π) 内，直接线性映射到 [-1, 1] 即可：
         *   q1.31_val = (phase / π - 1.0) * 2^31
         * 等价地：将 [0, 2π) 映射为 [-π, π) 再除以 π：
         *   mapped = phase - π  （当 phase ∈ [0, 2π)，mapped ∈ [-π, π)）
         *   q1.31  = mapped / π * 2^31
         */
        float mapped_phase = anc.phase - (float)M_PI;   // 映射到 [-π, π)
        int32_t input_q31  = (int32_t)(mapped_phase / (float)M_PI * 2147483647.0f);

        CORDIC->WDATA = (uint32_t)input_q31;
        /* 轮询等待结果就绪（NRES=1 时需读两次 RDATA） */
        while (!(CORDIC->CSR & CORDIC_CSR_RRDY));
        int32_t cos_q31 = (int32_t)CORDIC->RDATA;  /* 第一次读：cos */
        int32_t sin_q31 = (int32_t)CORDIC->RDATA;  /* 第二次读：sin */

        float local_cos = (float)cos_q31 / 2147483648.0f;
        float local_sin = (float)sin_q31 / 2147483648.0f;

        /* --- 3. IQ 误差投影解调 --- */
        anc.I_err_acc += error_signal * local_cos;
        anc.Q_err_acc += error_signal * local_sin;

        /* --- 4. 生成抵消波形送入 DAC --- */
        float   dac_f   = anc.I_out * local_cos + anc.Q_out * local_sin;
        int32_t dac_out = (int32_t)dac_f + DAC_MID_VALUE;

        if (dac_out < 0)    dac_out = 0;
        if (dac_out > 4095) dac_out = 4095;
        pDAC_Data[i] = (uint16_t)dac_out;

        /* --- 5. 基准相位递推 --- */
        anc.phase += anc.phase_step;
        if (anc.phase >= 2.0f * (float)M_PI)
            anc.phase -= 2.0f * (float)M_PI;
    }

    /* --- 6. 帧均值化与 PI 闭环寻优 --- */
    float inv_len = 1.0f / (float)length;
    anc.I_err_acc *= inv_len;
    anc.Q_err_acc *= inv_len;

    anc.I_integral = clampf(anc.I_integral + anc.I_err_acc, -ANC_INT_LIMIT, ANC_INT_LIMIT);
    anc.Q_integral = clampf(anc.Q_integral + anc.Q_err_acc, -ANC_INT_LIMIT, ANC_INT_LIMIT);

    /* 更新输出系数：自适应补偿声道 LTI 的相移和衰减 */
    anc.I_out = clampf(anc.I_out + anc.Kp * anc.I_err_acc + anc.Ki * anc.I_integral,
                       -ANC_OUT_LIMIT, ANC_OUT_LIMIT);
    anc.Q_out = clampf(anc.Q_out + anc.Kp * anc.Q_err_acc + anc.Ki * anc.Q_integral,
                       -ANC_OUT_LIMIT, ANC_OUT_LIMIT);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_CORDIC_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_COMP1_Init();
  MX_TIM5_Init();
  MX_FMAC_Init();
  /* USER CODE BEGIN 2 */

  /* [BUG FIX #7] 在使用 CORDIC 前显式配置外设寄存器 */
  CORDIC_Config_Cosine();

  /* 启动模拟前端 */
  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);
  HAL_COMP_Start(&hcomp1);

  /* 初始化 DAC 输出为静默中值，避免启动瞬态 */
  for (uint16_t i = 0; i < BUFFER_SIZE; i++)
  {
      dac_buffer[i] = DAC_MID_VALUE;
  }

  /* 启动外设：先启 DAC DMA，再启 ADC DMA，最后启 TIM2 产生触发节拍 */
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                    (uint32_t *)dac_buffer, BUFFER_SIZE, DAC_ALIGN_12B_R);

  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, BUFFER_SIZE);

  /* TIM5 输入捕获 DMA：捕获 COMP1 输出上升沿，用于硬件测频 */
  HAL_TIM_IC_Start_DMA(&htim5, TIM_CHANNEL_1,
                        tim5_ic_buffer, 2);

  /* TIM2 最后启动，开始产生 DAC/ADC 触发 */
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 所有实时处理均在 DMA 中断回调中完成，主循环可用于低优先级任务 */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ================= DMA 中断回调逻辑 ================= */

/**
 * @brief  ADC DMA 传输过半回调：处理缓冲区前半段
 *
 * [BUG FIX #4]
 * 原代码两次回调（半完成/完成）均传入 &adc_buffer / &dac_buffer，
 * 实际上始终处理前半段，后半段数据从未被处理（双缓冲机制失效）。
 * 修正：半完成处理 [0, HALF_BUFFER_SIZE)，完成处理 [HALF_BUFFER_SIZE, BUFFER_SIZE)。
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        ANC_Control_Loop(&adc_buffer[0],
                         &dac_buffer[0],
                         HALF_BUFFER_SIZE);
    }
}

/**
 * @brief  ADC DMA 传输完成回调：处理缓冲区后半段
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        ANC_Control_Loop(&adc_buffer[HALF_BUFFER_SIZE],
                         &dac_buffer[HALF_BUFFER_SIZE],
                         HALF_BUFFER_SIZE);
    }
}

/**
 * @brief  TIM5 DMA 输入捕获过半回调：用新旧捕获值计算周期
 *
 * [BUG FIX #6]
 * 原代码使用 tim5_ic_buffer（数组名，即指针值）参与减法运算，
 * 应使用 tim5_ic_buffer[0] / tim5_ic_buffer[1]（元素值）。
 *
 * DMA 以循环模式写入 tim5_ic_buffer[2]：
 *   半完成时 [0] 已更新，[1] 是上一轮旧值 → diff = [0] - [1]（32-bit 无符号回绕安全）
 *   完成时   [1] 已更新，[0] 是本轮旧值   → diff = [1] - [0]
 */
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        uint32_t diff = tim5_ic_buffer[0] - tim5_ic_buffer[1];
        update_freq(diff);
    }
}

/**
 * @brief  TIM5 DMA 输入捕获完成回调
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        uint32_t diff = tim5_ic_buffer[1] - tim5_ic_buffer[0];
        update_freq(diff);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
