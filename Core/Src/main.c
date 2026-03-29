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
#include "cordic.h"
#include "dac.h"
#include "dma.h"
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
#define SAMPLE_RATE         100000.0f  // 100 kHz 采样率
#define BUFFER_SIZE         256        // DMA 缓冲区大小，256次采样为一帧
#define HALF_BUFFER_SIZE    (BUFFER_SIZE / 2)
#define DAC_MID_VALUE       2048       // 12-bit DAC 中值 (0V对应偏置)
#define ADC_MID_VALUE       2048       // 12-bit ADC 中值 (假设输入偏置为VDD/2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern CORDIC_HandleTypeDef hcordic;
extern TIM_HandleTypeDef htim2;

uint16_t adc_buffer[BUFFER_SIZE];
uint16_t dac_buffer[BUFFER_SIZE];

typedef struct {
    float target_freq; // 当前单音频率 (500Hz - 2kHz)
    float phase;       // 实时相位 [0, 2*pi)
    float phase_step;  // 相位步进量
    
    // 累积量 (用于低通滤波/均值)
    float I_err_acc;   
    float Q_err_acc;   
    
    // PI 控制器参数
    float Kp;
    float Ki;
    
    // 积分项
    float I_integral;  
    float Q_integral;  
    
    // 输出幅值系数
    float I_out;       
    float Q_out;       
} ANC_Controller_t;

ANC_Controller_t anc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ANC_Init(float init_freq);
void ANC_Control_Loop(uint16_t* pADC_Data, uint16_t* pDAC_Data, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_CORDIC_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* ADC 自校准（单端模式） */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  /* 启动 ANC 系统，初始化频率为 1kHz */
  ANC_Init(1000.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

void ANC_Init(float init_freq) {
    // 1. 初始化控制变量
    anc.target_freq = init_freq;
    anc.phase = 0.0f;
    anc.phase_step = 2.0f * M_PI * anc.target_freq / SAMPLE_RATE;
    
    anc.Kp = 0.05f;   // 需根据实际硬件LTI特性(增益调节)进行整定
    anc.Ki = 0.002f;
    
    anc.I_integral = 0.0f;
    anc.Q_integral = 0.0f;
    anc.I_out = 0.0f;
    anc.Q_out = 0.0f;

    // 2. 配置 CORDIC 加速器计算 Sin/Cos
    CORDIC_ConfigTypeDef sCordicConfig = {0};
    sCordicConfig.Function         = CORDIC_FUNCTION_COSINE; // 同时计算 Cos 和 Sin
    sCordicConfig.Precision        = CORDIC_PRECISION_5CYCLES; 
    sCordicConfig.Scale            = CORDIC_SCALE_0;
    sCordicConfig.NbWrite          = CORDIC_NBWRITE_1; // 写一个参数 (Phase)
    sCordicConfig.NbRead           = CORDIC_NBREAD_2;  // 读两个参数 (Cos, Sin)
    sCordicConfig.InSize           = CORDIC_INSIZE_32BITS;
    sCordicConfig.OutSize          = CORDIC_OUTSIZE_32BITS;
    HAL_CORDIC_Configure(&hcordic, &sCordicConfig);

    // 3. 填充初始DAC Buffer (输出0)
    for(int i=0; i<BUFFER_SIZE; i++) {
        dac_buffer[i] = DAC_MID_VALUE;
    }

    // 4. 启动外设
    // 开启 DAC1 Channel 1 并启动 DMA 循环传输
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_buffer, BUFFER_SIZE, DAC_ALIGN_12B_R);
    
    // 开启 ADC1 并启动 DMA 循环采样，由 TIM2 TRGO 触发
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE);
    
    // 开启 TIM2 产生 100kHz TRGO 信号
    HAL_TIM_Base_Start(&htim2);
}

void ANC_Control_Loop(uint16_t* pADC_Data, uint16_t* pDAC_Data, uint16_t length) {
    anc.I_err_acc = 0.0f;
    anc.Q_err_acc = 0.0f;
    
    // 修正：length 此时代表 128 个连续的 ADC 采样点
    for (uint16_t i = 0; i < length; i++) { // 步进改为 1
        float error_signal = (float)(pADC_Data[i]) - ADC_MID_VALUE;
        
        // CORDIC 计算相位映射
        float map_phase = anc.phase;
        if(map_phase > M_PI) map_phase -= 2.0f * M_PI;
        int32_t input_q31 = (int32_t)((map_phase / M_PI) * 2147483648.0f);
        
        CORDIC->WDATA = input_q31; 
        int32_t cos_q31 = (int32_t)CORDIC->RDATA; 
        int32_t sin_q31 = (int32_t)CORDIC->RDATA;
        
        float local_cos = (float)cos_q31 / 2147483648.0f;
        float local_sin = (float)sin_q31 / 2147483648.0f;
        
        anc.I_err_acc += error_signal * local_cos;
        anc.Q_err_acc += error_signal * local_sin;
        
        // 合成发射信号 B 并填充 DAC 缓冲区
        float dac_f = anc.I_out * local_cos + anc.Q_out * local_sin;
        int32_t dac_out = (int32_t)(dac_f) + DAC_MID_VALUE;
        pDAC_Data[i] = (uint16_t)__SSAT(dac_out, 12); // 索引改为 i
        
        // 更新相位
        anc.phase += anc.phase_step;
        if (anc.phase >= 2.0f * M_PI) anc.phase -= 2.0f * M_PI;
    }
    
    // 闭环更新系数 (samples_count 改为 length)
    float samples_count = (float)length;
    anc.I_err_acc /= samples_count;
    anc.Q_err_acc /= samples_count;
    // PI 调节逻辑维持不变 
    anc.I_integral += anc.I_err_acc;
    anc.Q_integral += anc.Q_err_acc;
    anc.I_out -= (anc.Kp * anc.I_err_acc + anc.Ki * anc.I_integral);
    anc.Q_out -= (anc.Kp * anc.Q_err_acc + anc.Ki * anc.Q_integral);
}

// DMA 传输过半回调
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        ANC_Control_Loop(&adc_buffer[0], &dac_buffer[0], HALF_BUFFER_SIZE);
    }
}

// DMA 传输完成回调
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        ANC_Control_Loop(&adc_buffer[HALF_BUFFER_SIZE], &dac_buffer[HALF_BUFFER_SIZE], HALF_BUFFER_SIZE);
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
