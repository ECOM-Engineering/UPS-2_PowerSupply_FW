/* USER CODE BEGIN Header */
/*! @addtogroup Main
@{ */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Klaus Mezger, ECOM ENGINEERING
  * All rights reserved.</center></h2>
  *
  * Development base software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "timer.h"
#include "interface.h"
#include "lowlevel.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	KEY_STATE_WAIT,
	KEY_STATE_DEBOUNCE,
	KEY_STATE_PRESSED,
	KEY_STATE_RELEASED,
	KEY_STATE_CHECK_DOUBLE,
	KEY_STATE_SUPER_LONG,
	KEY_STATE_LONG,
	KEY_STATE_SHORT,
	KEY_STATE_WAIT_RELEASE,
	KEY_STATE_DOUBLE
}eKeyStates_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define KEY_T_SHORT 		200   	//ms
#define KEY_T_LONG 			(2000)  //ms
#define KEY_T_SUPER_LONG	(5000)
#define KEY_T_DEBOUNCE 		20		//ms
#define KEY_T_DBL_WINDOW	350	//ms
#define ADC_DMA_BUF_SIZE	32		//bytes --> 4 * 16Bit elements

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile eKeyStates_t g_keyState;
__IO ITStatus UartReady = RESET;
uint8_t g_adcRdy;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
eKeyPress_t ecGetKey(void);
void ecDecodeKey(void);
void ecRx_Init(void);
//void ecTxString(char *String, size_t Size); //proto in main.h

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char *ecFormatFixptStr(int16_t Raw_value)
{
    static char retStr[16];
    snprintf(retStr, sizeof(retStr)-1, "%d.%01d", Raw_value/10, (Raw_value % 10));
    return retStr;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    eKeyPress_t keyPress;
    ePiState_t piState = PI_STATE_UNKNOWN;
    char mainMessage[81];
    uint16_t charsWritten;
#ifdef SERIAL_DEBUG
    char rxStrBuf[80];
#endif


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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_Base_Start_IT(&htim3); //1ms Timer
  ecSWTimerInit();
  ecLEDinit();

  if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0) //check if bootloader requested
  {

	  ecSetLED(LED_Pi, LED_ON);
	  ecSetLED(LED_Main, LED_ON);
	  ecSetLED(LED_Batt, LED_ON);
	  ecSWTimerStart(DELAY_TIMER, KEY_T_LONG); //wait
	  int key = 1;
	  while(ecSWTimerRead(DELAY_TIMER) > 0)
	  {
		  key = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin); //check if button still active
		  if (key == 1)
			  break;
	  }
	  if(key == 0)
	  {
		JumpToBootloader();
	  }
	  ecSetLED(LED_Pi, LED_OFF);
	  ecSetLED(LED_Main, LED_OFF);
	  ecSetLED(LED_Batt, LED_OFF);

  }



  /* Run ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  g_sys_Mode = HAL_GPIO_ReadPin( Mode_GPIO_Port , Mode_Pin);


  strcpy(mainMessage, "\r\nUPS-2 Starting up ...\r\n" );
  ecTxString(mainMessage, strlen(mainMessage));
  ecRx_Init();

  HAL_GPIO_WritePin(CMD_OUT_GPIO_Port, CMD_OUT_Pin, GPIO_PIN_SET); //prepare Pi startup
  HAL_GPIO_WritePin(EN_5V_GPIO_Port,  EN_5V_Pin, GPIO_PIN_SET); //turn 5V Pi ON

  ecSWTimerStart(DELAY_TIMER, 1000); //wait s, allow system and power stabilization
  while(ecSWTimerRead(DELAY_TIMER));


//  HAL_UART_Receive_IT(&huart2,(uint8_t *)aRxBuffer, 1 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

      if(g_adcRdy)
	  {
		  g_adcRdy = 0;
		  g_powerState = ecAnalogHandler(g_adc_voltages);
          if(ecSWTimerRead(MESSAGE_TIMER) == 0)
         {
             ecSWTimerStart(MESSAGE_TIMER, 1000);
             charsWritten = sprintf(g_analogStr,"%s",ecFormatFixptStr(g_adc_voltages.main_voltage));
             charsWritten += sprintf(&g_analogStr[charsWritten],",%s",ecFormatFixptStr(g_adc_voltages.batt_voltage));
             charsWritten += sprintf(&g_analogStr[charsWritten],",%d\n",g_adc_voltages.CPU_temp/10);

#ifdef SERIAL_DEBUG
//             ecTxString(mainMessage, strlen(mainMessage));
#endif
         }

	  }
	  else
	  {
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&g_adc_buf, 4);
	  }

#ifdef SERIAL_DEBUG
      charsWritten = ecGetRxString(rxStrBuf, sizeof(rxStrBuf));
      if(charsWritten)
      {
          sprintf(mainMessage, "Rx: %s, %d chars\r\n", rxStrBuf,charsWritten);

          ecTxString(mainMessage, strlen(mainMessage));
      }
#endif


	  keyPress = ecGetKey();
	  if(g_sys_Mode == SYS_MODE_PARALLEL)
	  {
	      ecPortExecCommand(keyPress);
	      piState = ecPortHandleFeedback(keyPress, piState);
	  }
	  else
	  {
          ecSerialExecCommand(keyPress);
          piState = ecSerialHandleFeedback(keyPress, piState);
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
//  g_hadc1 = hadc1; //make handle global
  /* USER CODE END ADC1_Init 2 */

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
  htim3.Init.Prescaler = 8000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 38400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_DisableDMADeactOnRxErr(USART2);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pi_Pin|LED_Batt_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Main_GPIO_Port, LED_Main_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_5V_Pin|CMD_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pi_Pin LED_Batt_Pin */
  GPIO_InitStruct.Pin = LED_Pi_Pin|LED_Batt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Main_Pin */
  GPIO_InitStruct.Pin = LED_Main_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Main_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_5V_Pin LED_Pin */
  GPIO_InitStruct.Pin = EN_5V_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : nBatt_Pin Mode_Pin */
  GPIO_InitStruct.Pin = nBatt_Pin|Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CMD_OUT_Pin */
  GPIO_InitStruct.Pin = CMD_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CMD_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACK_IN_Pin */
  GPIO_InitStruct.Pin = ACK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  		Triggers Software timers with TIM3 period
  * @param[in]		*htim
  * param[out]		None
  * @retval 		None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		ecSWTimerClk();
		ecDecodeKey();
		ecBlinkHandler();
	}
}

/**
  * @brief 		Key press mode analyzer.
  * @param[in]  *hadc1 					adc1 handle
  * @param[out]	g_adc_voltages 			global data structure
  * @param[out]	g_adcRdy				global sync flag measurement complete
  * @retval 	None
	  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1)
{

    // values in mV with divider 510kk - 62kk
    g_adc_voltages.main_voltage = (((g_adc_buf.main_voltage / 2) * 15) / 100) + 3;    //[100mV] incl input diode threshold
    g_adc_voltages.batt_voltage = (((g_adc_buf.batt_voltage / 2) * 15) / 100) + 3;    //[100mV] incl input diode threshold
    //ignore diode threshold if no input present
    if(g_adc_voltages.main_voltage < 5)
        g_adc_voltages.main_voltage = 0;
    if(g_adc_voltages.batt_voltage < 5)
        g_adc_voltages.batt_voltage = 0;


    // >19 --> Pi is powered by USB connector
    g_adc_voltages.Pi_voltage = g_adc_buf.Pi_voltage / 124; //result in 1/10 V

    g_adc_voltages.CPU_temp = ecGetCPUTemp(g_adc_buf.CPU_temp);

    g_adcRdy = 1;
};

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc1)
{
//	adc_value = 0;
};


/**
  * @brief 		Key press mode analyzer.
  * @param[in] 	None
  * @param[out]	global keyState
  * @retval 	None
	  */
	/* Function called by timer interrupt */
void ecDecodeKey(void)
{

    static uint16_t keyTimer;
    switch (g_keyState)
    {
        case KEY_STATE_WAIT:
            if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0) //low active input
            {
                keyTimer = 0;
                g_keyState = KEY_STATE_DEBOUNCE;
            }
            break;

        case KEY_STATE_DEBOUNCE:
            keyTimer ++;
            if(keyTimer > KEY_T_DEBOUNCE)
            {
              if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0) //still active
                  g_keyState = KEY_STATE_PRESSED;
              else
                  g_keyState = KEY_STATE_WAIT;
            }

            break;

        case KEY_STATE_PRESSED:
            keyTimer ++;
            if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 1) //key released?
            {
                g_keyState = KEY_STATE_RELEASED;
            }
            break;

        case KEY_STATE_RELEASED:
            if(keyTimer > KEY_T_SUPER_LONG)
            {
                g_keyState = KEY_STATE_SUPER_LONG;
            }else
            if(keyTimer > KEY_T_LONG)
            {
                g_keyState = KEY_STATE_LONG;
            }else
            {
                g_keyState = KEY_STATE_CHECK_DOUBLE;
                keyTimer = 0;
            }
            break;
        case KEY_STATE_CHECK_DOUBLE: //new key within double klick window?
            if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 1) //key still released?
            {
                keyTimer++;
                if(keyTimer > KEY_T_DBL_WINDOW)
                {
                    g_keyState = KEY_STATE_SHORT;
                }
            }
            else //key pressed again
            {
                g_keyState = KEY_STATE_WAIT_RELEASE;
                keyTimer = 0;
            }
            break;

        case KEY_STATE_WAIT_RELEASE:
            if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 1)
            {
                g_keyState = KEY_STATE_DOUBLE;
            }
            break;

        default:
            break;
    }


}

/**
  * @brief 		public access function
  * @param[in] 	global keyState
  * @param[out]	None
  * @retval 	key action type
  */
eKeyPress_t ecGetKey(void)
{

	eKeyPress_t retVal;
 	retVal = KEY_NO_PRESS;

 	switch (g_keyState)
 	{
 	case KEY_STATE_PRESSED:
 		retVal = KEY_PENDING;
 		break;
 	case KEY_STATE_SUPER_LONG:
		retVal = KEY_SUPER_LONG_PRESS;
		g_keyState = KEY_STATE_WAIT;
		break;
 	case KEY_STATE_LONG:
		retVal = KEY_LONG_PRESS;
		g_keyState = KEY_STATE_WAIT;
		break;
	case KEY_STATE_SHORT:
		retVal = KEY_SHORT_PRESS;
		g_keyState = KEY_STATE_WAIT;
		break;
	case KEY_STATE_DOUBLE:
		retVal = KEY_DOUBLE_PRESS;
		g_keyState = KEY_STATE_WAIT;
		break;
	default:
		break;
 	}


	return retVal;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
