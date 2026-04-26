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
typedef enum {
    ANC_STATE_INIT = 0,         // 初始化
    ANC_STATE_MEASURE_BASE,     // 测量环境底噪
    ANC_STATE_SEARCH_PHASE,     // 寻找 LTI 系统的相移补偿角
    ANC_STATE_TRACKING          // 锁定相位并持续降噪
} ANC_SM_t;

typedef struct {
    float target_freq;   // 动态追踪的单音频率 (Hz)
    float phase;         // 实时相位累加器，范围[0, 2π)
    float phase_step;    // 相位步进
    float I_out;         // DAC 输出 I 分量
    float Q_out;         // DAC 输出 Q 分量
    float Kp;
    float Ki;
    float I_integral;
    float Q_integral;
    
    // --- LTI 相位自适应与状态机变量 ---
    ANC_SM_t state;
    float phase_comp;    // LTI 系统相位补偿角 (解耦 I/Q)
    uint32_t state_timer;
    float current_power; // 当前帧的误差信号能量
    float base_power;    // 未开启降噪时的参考底噪能量
    float best_power;    // 寻优时记录的最佳能量
    float best_phase;    // 寻优时记录的最佳相位
    uint8_t test_idx;    // 寻优测试索引 (0~3)
} ANC_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_RATE         100000.0f
#define BUFFER_SIZE         256
#define HALF_BUFFER_SIZE    (BUFFER_SIZE / 2)
#define DAC_MID_VALUE       2048
#define ADC_MID_VALUE       2048.0f

#define ANC_KP          0.02f
#define ANC_KI          0.001f
#define ANC_OUT_LIMIT   2000.0f      // 扩大输出限幅以对抗 LTI 衰减，最大不超过 2047
#define ANC_INT_LIMIT   (ANC_OUT_LIMIT / ANC_KI)
#define TIM5_CLOCK_HZ   170000000UL
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

uint16_t adc_buffer[BUFFER_SIZE];
uint16_t dac_buffer[BUFFER_SIZE];
uint32_t tim5_ic_buffer[2];

static ANC_State_t anc = {
    .target_freq  = 1000.0f,
    .phase        = 0.0f,
    .phase_step   = 2.0f * (float)M_PI * 1000.0f / SAMPLE_RATE,
    .I_out        = 0.0f,
    .Q_out        = 0.0f,
    .Kp           = ANC_KP,
    .Ki           = ANC_KI,
    .I_integral   = 0.0f,
    .Q_integral   = 0.0f,
    .state        = ANC_STATE_INIT,
    .phase_comp   = 0.0f,
    .current_power= 0.0f
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

static float clampf(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static void CORDIC_Config_Cosine(void) {
    CORDIC->CSR = (0x0UL  << CORDIC_CSR_FUNC_Pos)
                | (0x6UL  << CORDIC_CSR_PRECISION_Pos)
                | (0x0UL  << CORDIC_CSR_SCALE_Pos)
                | (0x1UL  << CORDIC_CSR_NRES_Pos)
                | (0x0UL  << CORDIC_CSR_NARGS_Pos)
                | (0x0UL  << CORDIC_CSR_RESSIZE_Pos)
                | (0x0UL  << CORDIC_CSR_ARGSIZE_Pos);
}

static void update_freq(uint32_t timer_diff) {
    if (timer_diff == 0) return;
    float new_freq = (float)TIM5_CLOCK_HZ / (float)timer_diff;

    // 限制在合理频率范围，忽略错误捕获
    if (new_freq < 20.0f || new_freq > 20000.0f) return;

    // 如果频率发生剧烈变化 (>10%)，LTI系统的相移也会改变，重新触发寻优
    if (fabsf(new_freq - anc.target_freq) > anc.target_freq * 0.1f) {
        anc.state = ANC_STATE_INIT;
    }

    anc.target_freq = 0.9f * anc.target_freq + 0.1f * new_freq;
    anc.phase_step  = 2.0f * (float)M_PI * anc.target_freq / SAMPLE_RATE;
}

static void ANC_Control_Loop(uint16_t *pADC_Data, uint16_t *pDAC_Data, uint16_t length) {
    float I_err_acc = 0.0f;
    float Q_err_acc = 0.0f;
    float frame_energy = 0.0f;

    /* --- 1. 高速信号合成与解调 (Sample by Sample) --- */
    for (uint16_t i = 0; i < length; i++) {
        float error_signal = (float)pADC_Data[i] - ADC_MID_VALUE;
        frame_energy += error_signal * error_signal;

        float mapped_phase = anc.phase - (float)M_PI;
        int32_t input_q31  = (int32_t)(mapped_phase / (float)M_PI * 2147483647.0f);

        CORDIC->WDATA = (uint32_t)input_q31;
        while (!(CORDIC->CSR & CORDIC_CSR_RRDY));
        float local_cos = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;
        float local_sin = (float)((int32_t)CORDIC->RDATA) / 2147483648.0f;

        I_err_acc += error_signal * local_cos;
        Q_err_acc += error_signal * local_sin;

        float   dac_f   = anc.I_out * local_cos + anc.Q_out * local_sin;
        int32_t dac_out = (int32_t)dac_f + DAC_MID_VALUE;

        if (dac_out < 0)    dac_out = 0;
        if (dac_out > 4095) dac_out = 4095;
        pDAC_Data[i] = (uint16_t)dac_out;

        anc.phase += anc.phase_step;
        if (anc.phase >= 2.0f * (float)M_PI) anc.phase -= 2.0f * (float)M_PI;
    }

    /* --- 2. 帧数据平滑 --- */
    float inv_len = 1.0f / (float)length;
    I_err_acc *= inv_len;
    Q_err_acc *= inv_len;
    float mean_power = frame_energy * inv_len;
    anc.current_power = 0.8f * anc.current_power + 0.2f * mean_power; // 简单的低通滤波

    /* --- 3. 自适应 LTI 寻优与 PI 闭环控制 (Frame by Frame) --- */
    switch (anc.state) {
        case ANC_STATE_INIT:
            anc.I_out = 0.0f; anc.Q_out = 0.0f;
            anc.I_integral = 0.0f; anc.Q_integral = 0.0f;
            anc.state_timer = 0;
            anc.state = ANC_STATE_MEASURE_BASE;
            break;

        case ANC_STATE_MEASURE_BASE:
            // 等待约 25ms 测定未经降噪的原始 LTI 环境底噪
            if (++anc.state_timer > 20) {
                anc.base_power = anc.current_power;
                anc.state = ANC_STATE_SEARCH_PHASE;
                anc.test_idx = 0;
                anc.state_timer = 0;
                anc.best_power = 1e12f; // 初始一个极大的值
            }
            break;

        case ANC_STATE_SEARCH_PHASE:
        case ANC_STATE_TRACKING:
        {
            // 在寻优状态中，我们每隔 30 帧 (约38ms) 切换一次相位
            if (anc.state == ANC_STATE_SEARCH_PHASE) {
                anc.phase_comp = anc.test_idx * (M_PI / 2.0f); // 测试 0, 90, 180, 270 度
                anc.state_timer++;
                
                if (anc.state_timer == 30) {
                    if (anc.current_power < anc.best_power) {
                        anc.best_power = anc.current_power;
                        anc.best_phase = anc.phase_comp;
                    }
                    // 如果能量下降到了原来的一半以下，说明这个相位能让 PI 稳定下降，立刻锁定！
                    if (anc.current_power < anc.base_power * 0.5f) {
                        anc.state = ANC_STATE_TRACKING;
                    } else {
                        anc.test_idx++;
                        if (anc.test_idx >= 4) {
                            // 4个方向都测完了，挑一个最好的强行进入追踪
                            anc.phase_comp = anc.best_phase;
                            anc.state = ANC_STATE_TRACKING;
                        } else {
                            // 准备测试下一个相位，重置积分器
                            anc.I_integral = 0; anc.Q_integral = 0;
                            anc.I_out = 0; anc.Q_out = 0;
                            anc.state_timer = 0;
                        }
                    }
                }
            } else {
                // TRACKING 状态：监视系统是否再次发散
                if (anc.current_power > anc.base_power * 1.5f) {
                    anc.state = ANC_STATE_INIT; // 异常发散，重新寻优
                }
            }

            /* --- 核心：坐标旋转解耦 ---
             * 通过 phase_comp 逆向旋转误差向量，抵消外部 LTI 的相移
             * 使得 I/Q 通道重新变得独立、正交，恢复 PI 算法的稳定性
             */
            float cos_c = cosf(anc.phase_comp);
            float sin_c = sinf(anc.phase_comp);
            float I_rot = I_err_acc * cos_c + Q_err_acc * sin_c;
            float Q_rot = Q_err_acc * cos_c - I_err_acc * sin_c;

            anc.I_integral = clampf(anc.I_integral + I_rot * anc.Ki, -ANC_INT_LIMIT, ANC_INT_LIMIT);
            anc.Q_integral = clampf(anc.Q_integral + Q_rot * anc.Ki, -ANC_INT_LIMIT, ANC_INT_LIMIT);

            anc.I_out = clampf(anc.I_integral + I_rot * anc.Kp, -ANC_OUT_LIMIT, ANC_OUT_LIMIT);
            anc.Q_out = clampf(anc.Q_integral + Q_rot * anc.Kp, -ANC_OUT_LIMIT, ANC_OUT_LIMIT);
            break;
        }
    }
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
  MX_OPAMP4_Init();
  /* USER CODE BEGIN 2 */

  CORDIC_Config_Cosine();

  HAL_OPAMP_Start(&hopamp2);
  HAL_OPAMP_Start(&hopamp3);
  HAL_COMP_Start(&hcomp1);
    HAL_OPAMP_Start(&hopamp4);
  for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
      dac_buffer[i] = DAC_MID_VALUE;
  }

  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_buffer, BUFFER_SIZE, DAC_ALIGN_12B_R);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, BUFFER_SIZE);
  HAL_TIM_IC_Start_DMA(&htim5, TIM_CHANNEL_1, tim5_ic_buffer, 2);
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

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC2) {
        ANC_Control_Loop(&adc_buffer[0], &dac_buffer[0], HALF_BUFFER_SIZE);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC2) {
        ANC_Control_Loop(&adc_buffer[HALF_BUFFER_SIZE], &dac_buffer[HALF_BUFFER_SIZE], HALF_BUFFER_SIZE);
    }
}

void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM5) {
        uint32_t diff = tim5_ic_buffer[0] - tim5_ic_buffer[1];
        update_freq(diff);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM5) {
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
