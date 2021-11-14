/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
typedef enum {SIN_WAVE, 
              TRIANGLE_WAVE,
              SQUARE_WAVE}  Signal_Type;

Signal_Type SignalType;

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
uint16_t sinewave[60] = {
0x07ff,0x08cb,0x0994,0x0a5a,0x0b18,0x0bce,0x0c79,0x0d18,0x0da8,0x0e29,0x0e98,0x0ef4,0x0f3e,0x0f72,0x0f92,0x0f9d,
0x0f92,0x0f72,0x0f3e,0x0ef4,0x0e98,0x0e29,0x0da8,0x0d18,0x0c79,0x0bce,0x0b18,0x0a5a,0x0994,0x08cb,0x07ff,0x0733,
0x066a,0x05a4,0x04e6,0x0430,0x0385,0x02e6,0x0256,0x01d5,0x0166,0x010a,0x00c0,0x008c,0x006c,0x0061,0x006c,0x008c,
0x00c0,0x010a,0x0166,0x01d5,0x0256,0x02e6,0x0385,0x0430,0x04e6,0x05a4,0x066a,0x0733};

/* USER CODE END PFP */
uint16_t dac_sin_freq;
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* FFT settings */

#define FFT_SIZE                 1024         /* FFT size is always the same size as we have samples, so 180 in our case */
#define ADC_SAMPLE                FFT_SIZE*2 
#define SAMPLES                  FFT_SIZE*2             /* 1800 real party and 1800 imaginary parts */
 /* Global variables */
float32_t Input[SAMPLES*2];
float32_t Output[FFT_SIZE];
uint32_t  uOutput[FFT_SIZE];

uint16_t ConvData[ADC_SAMPLE];

/* Global variables */
uint8_t conv_done,re_start=0;
/* USER CODE END 0 */
uint16_t Fft_freq;

/* Global variables */

 arm_cfft_radix4_instance_f32 S;    /* ARM CFFT module */
 float32_t maxValue;                /* Max FFT value is stored here */
 uint32_t maxIndex;                /* Index in Output array where max value is */
QueueHandle_t xQueueFFT;
    
/* USER CODE END 0 */
void FFT_Task(void *p);
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint16_t i;
  

  /*Select signal type*/
  SignalType=SIN_WAVE;
  //SignalType=TRIANGLE_WAVE;
  //SignalType=SQUARE_WAVE;
  
  
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
  if(SignalType!=SQUARE_WAVE)
  {
  MX_DAC_Init();
  }
  /*TIM3*/
  MX_TIM3_Init();
  
  /*DAC TIM6 */
  MX_TIM6_Init();
  
  
  
  /* USER CODE BEGIN 2 */
    // 60 sample each sample take 400ùs  
  dac_sin_freq= 1000000 / (60 * (TIM6->ARR+1U));

  /*FFT FReq step */  
  Fft_freq = 1000000 / ((TIM3->ARR+1U)*1024);
  
     /*Start TIM3*/
   if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
    /* Counter enable error */
    Error_Handler();
  }
  
  
   if(SignalType!=SQUARE_WAVE)
    {
     /*Start TIM6*/
     if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
     {
      /* Counter enable error */
       Error_Handler();
      }
    }
    else
    {
    HAL_TIM_Base_Start_IT(&htim6);    
     }
  
  /* USER CODE END 2 */
  /*Create Queu used for Cycle in Run mode */
  xQueueFFT = xQueueCreate(6U, sizeof( uint8_t ) );


  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  
  xTaskCreate(FFT_Task, "FFT", 500U , NULL,  (tskIDLE_PRIORITY+3U),NULL );

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}








/* USER CODE BEGIN 4 */
void FFT_Task(void *p)
{
  uint8_t Step;
  
  
  while(1)
  {
    
    
    if (xQueueReceive( xQueueFFT, &( Step ), (TickType_t)80U)==pdPASS)
    {
      
      if (Step==1U) /*first Half Buffer*/
      {
          uint16_t j=0;
          for (uint16_t i=0;(i<ADC_SAMPLE/2) ; i++)
          {
            Input[j]=(float32_t)  ((float32_t)ConvData[i] - (float32_t)(4095.0) ) /(float32_t)(4095.0);
            Input[j+1]=(float32_t)0;
            j=j+2;
          }      
      }
      else  /*Second HalfBuffer*/
      {

        uint16_t j=0;
          for (uint16_t i=ADC_SAMPLE/2;(i<ADC_SAMPLE) ; i++)
          {
            Input[j]=(float32_t)  ((float32_t)ConvData[i] - (float32_t)(4095.0) ) /(float32_t)(4095.0);
            Input[j+1]=(float32_t)0;
            j=j+2;
          }

        
      }
          
       /* Process the data through the CFFT/CIFFT module */
        arm_cfft_radix4_f32(&S, Input);
        
        /* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
        arm_cmplx_mag_f32(Input, Output, FFT_SIZE);
        
        /* Calculates maxValue and returns corresponding value */
        arm_max_f32(Output, FFT_SIZE, &maxValue, &maxIndex);
        
        for (uint16_t i=0;i<FFT_SIZE;i++)
        uOutput[i]=Output[i];
        
        /*Toggle PD12 Led */
        GPIOD->ODR ^=1<<12;
        
        HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ConvData,ADC_SAMPLE);
        
   }
    
    
    
    
  }
  
  
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
 if(SignalType!=SQUARE_WAVE)
  {
  if(SignalType==SIN_WAVE)
  {
   if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,  (uint32_t *)sinewave,60, DAC_ALIGN_12B_R) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else
  {
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  }
  }
   
  
    /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
    arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
        
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ConvData,ADC_SAMPLE);
    
    
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(1);
  }
  /* USER CODE END 5 */
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
 conv_done=1;
 
   uint8_t val=2;
  BaseType_t xHigherPriorityTaskWoken=pdFALSE;
  
  /*Process First Buffer */
  (void)xQueueSendFromISR(xQueueFFT,(void* )&val,&xHigherPriorityTaskWoken);
  /*Force Contex Switch ...*/
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
 
 }

/**
  * @brief  Regular conversion half DMA transfer callback in non blocking mode 
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
 
  uint8_t val=1;
  BaseType_t xHigherPriorityTaskWoken=pdFALSE;
  
  /*Process First Buffer */
  (void)xQueueSendFromISR(xQueueFFT,(void* )&val,&xHigherPriorityTaskWoken);
  /*Force Contex Switch ...*/
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  
  
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE; ////bad config;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Noise wave generation on DAC OUT1
  */
 if(SignalType==TRIANGLE_WAVE)
{
  HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_1, DAC_TRIANGLEAMPLITUDE_4095);
  
  /* USER CODE BEGIN DAC_Init 2 */
}
  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 50; //50us
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65;//65µS  ==> FADC=15384hz == > fft_freq=15384/1024=15Hz   ==> bandwidth 15hz*512=7.68khz

  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;

  if(SignalType!=SQUARE_WAVE)
   {
     if(SignalType==TRIANGLE_WAVE)
     {
         htim6.Init.Prescaler = 1; //1us
     }
     else
     {
       htim6.Init.Prescaler = 50; //1us
     }
       htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
      if(SignalType==TRIANGLE_WAVE)
     {
       htim6.Init.Period = 10; //42*60µs = 2.5ms == > freq ~=400hz
     }
     else
     {
       //htim6.Init.Period = 41; //42*60µs = 2.5ms == > freq ~=400hz
       htim6.Init.Period = 58; //58*60µs = 3.58ms == > freq ~=285hz
     }
  }
  else
  {
      htim6.Init.Prescaler = 50; //1us
      htim6.Init.Period = 3590; //3.59ms
  }
  
  
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  
  
   if(SignalType==SQUARE_WAVE)
  {
  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    
  }
  
  
  
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  
 if (htim->Instance == TIM6) 
 {
if(SignalType==SQUARE_WAVE)
{
  GPIOA->ODR ^= 0x10;
}
 }
  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
