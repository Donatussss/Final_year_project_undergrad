/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* GAS CODE BEGIN Includes */
#include "gas.h"
#include <string.h>
/* GAS CODE END Includes */

/* water CODE BEGIN Includes */
#include "water.h"
/* water CODE END Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* GAS CODE BEGIN PV */
int *electrode_power_status;
uint16_t oxygen_concentration;
uint16_t oxygen_flowrate;
uint16_t oxygen_temperature;
uint16_t oxygen_params_buf[3];
uint16_t oxygen_params[3];
uint32_t oxygen_gas_bit;
uint32_t *oxygen_pressure;
int16_t oxygen_deltas[3];

/*uart_variables*/
uint8_t rx_buf[RXBUFSIZE];
uint8_t main_buf[MAINBUFSIZE];
uint16_t old_pos = 0;
uint16_t new_pos = 0;
/* GAS CODE END PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* GAS CODE BEGIN PFP */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t buf_size);
/* GAS CODE END PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* GAS CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t buf_size)
{
    if (huart->Instance == USART2)
    {
        old_pos = new_pos; // Update the last position before copying new data

        /* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
         * This is to maintain the circular buffer
         * The old data in the main buffer will be overlapped
         */
        if (old_pos + buf_size > MAINBUFSIZE) // If the current position + new data size is greater than the main buffer
        {
            uint16_t datatocopy = MAINBUFSIZE - old_pos;             // find out how much space is left in the main buffer
            memcpy((uint8_t *)main_buf + old_pos, rx_buf, datatocopy); // copy data in that remaining space
            old_pos = 0;
            memcpy((uint8_t *)main_buf, (uint8_t *)rx_buf + datatocopy, (buf_size - datatocopy)); // copy the remaining data
            new_pos = (buf_size - datatocopy);                                                   // update the position
        }

        /* if the current position + new data size is less than the main buffer
         * we will simply copy the data into the buffer and update the position
         */
        else
        {
            memcpy((uint8_t *)main_buf + old_pos, rx_buf, buf_size);
            new_pos = buf_size + old_pos;
        }

        /* start the DMA again */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)rx_buf, RXBUFSIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}
/* GAS CODE END 0 */

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
  waterInitialization();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* GAS CODE BEGIN 2 */

  /* GAS CODE END 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* GAS CODE BEGIN WHILE */
		int chamber_min_1 = HAL_GPIO_ReadPin(Float_Chamber1_min_GPIO_Port, Float_Chamber1_min_Pin);
		int chamber_max_1 = HAL_GPIO_ReadPin(Float_Chamber1_max_GPIO_Port, Float_Chamber1_max_Pin);
		int chamber_min_2 = HAL_GPIO_ReadPin(Float_Chamber2_min_GPIO_Port, Float_Chamber2_min_Pin);
		int chamber_max_2 = HAL_GPIO_ReadPin(Float_Chamber2_max_GPIO_Port, Float_Chamber2_max_Pin);
		int chamber_min_3 = HAL_GPIO_ReadPin(Float_Chamber3_min_GPIO_Port, Float_Chamber3_min_Pin);
		int chamber_max_3 = HAL_GPIO_ReadPin(Float_Chamber3_max_GPIO_Port, Float_Chamber3_max_Pin);
		int chamber_min_4 = HAL_GPIO_ReadPin(Float_Chamber4_min_GPIO_Port, Float_Chamber4_min_Pin);
		int chamber_max_4 = HAL_GPIO_ReadPin(Float_Chamber4_max_GPIO_Port, Float_Chamber4_max_Pin);

		waterLevel_Chamber_1(chamber_min_1, chamber_max_1);
		waterLevel_Chamber_2(chamber_min_2, chamber_max_2);
		waterLevel_Chamber_3(chamber_min_3, chamber_max_3);
		waterLevel_Chamber_4(chamber_min_4, chamber_max_4);

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  oxygen_gas_bit = HAL_ADC_GetValue(&hadc1);
	  gas_bit_to_bar(oxygen_gas_bit, oxygen_pressure);

	  /* store oxygen parameters to buffer before updating */
	  if (oxygen_params[o2temp_ind] != 0) /* if temperature is not equal to 0 */
	  {
		  for (int i = 0; i < 3; i++)
		  {
			  oxygen_params_buf[i] = oxygen_params[i];
		  }
	  }

	  /* update oxygen parameters */
	  get_oxygen_params(rx_buf, main_buf, oxygen_params);
	  if (oxygen_params[2] != 0) /* if temperature is not equal to 0. Valid data */
	  {
		  for (int i = 0; i < 3; i++)
		  {
			  oxygen_deltas[i] = oxygen_params[i] - oxygen_params_buf[i];
		  }

		  oxygen_concentration = oxygen_params[o2conc_ind];
		  oxygen_flowrate = oxygen_params[o2flow_ind];
		  oxygen_temperature = oxygen_params[o2temp_ind];

		  if (oxygen_concentration < CONCENTRATION_THRESH)
		  {
			  /* if concentration is lower than threshold
			   * and is increasing and electrode power is on, could be system has been powered on for the 1st time
			   * and is decreasing and electrode power is on, there is a likelihood of leakage
			   * power off electrodes
			   */
			  if (oxygen_deltas[o2conc_ind] < 0 && electrode_power_status)
			  {
				  // indicate leakage and turning off of electrodes to oled
				  power_electrodes(0, electrode_power_status);
			  }
		  }

		  if (oxygen_flowrate > FLOWRATE_UPPER_THRESH)
		  {
			  /* if flowrate is greater than upper threshold
			   * and electrode power is on, we should power off electrodes
			   */
			  power_electrodes(0, electrode_power_status);
		  }
		  else if (oxygen_flowrate < FLOWRATE_LOWER_THRESH)
		  {
			  /* if flowrate is less than lower threshold
			   * and is increasing and electrode power is on, could be system has been powered back on
			   * and is decreasing and electrode power is off, power on electrodes
			   * and is decreasing and electrode power is on, there is a likelihood of leakage , power off electrodes
			   */
			  if (oxygen_deltas[o2flow_ind] < 0 && !electrode_power_status)
			  {
				  power_electrodes(1, electrode_power_status);
			  }
			  else if (oxygen_deltas[o2flow_ind] < 0 && !electrode_power_status)
			  {
				  power_electrodes(1, electrode_power_status);
			  }
		  }

		  if (oxygen_temperature > TEMPERATURE_THRESH)
		  {
			  /* if temperature is greater than upper threshold
			   * power off electrodes
			   */
			  power_electrodes(0, electrode_power_status);
		  }
	  }

	  if (*oxygen_pressure > PRESSURE_THRESH && oxygen_flowrate > 0)
	  {
		  /* if pressure is greater than upper threshold
		   * and there is flow into the collection tank
		   * power off electrodes
		   */
		  power_electrodes(0, electrode_power_status);
	  }
	/* GAS CODE END WHILE */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Electrode1_Output_Pin|Electrode2_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Electrode3_Output_Pin|Electrode4_Output_Pin|Solenoid1_Output_Pin|Solenoid2_Output_Pin
                          |Solenoid3_Output_Pin|Solenoid4_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Float_Chamber1_min_Pin Float_Chamber1_max_Pin Float_Chamber2_min_Pin Float_Chamber2_max_Pin */
  GPIO_InitStruct.Pin = Float_Chamber1_min_Pin|Float_Chamber1_max_Pin|Float_Chamber2_min_Pin|Float_Chamber2_max_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Float_Chamber3_min_Pin Float_Chamber3_max_Pin Float_Chamber4_min_Pin Float_Chamber4_max_Pin */
  GPIO_InitStruct.Pin = Float_Chamber3_min_Pin|Float_Chamber3_max_Pin|Float_Chamber4_min_Pin|Float_Chamber4_max_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Electrode1_Output_Pin Electrode2_Output_Pin */
  GPIO_InitStruct.Pin = Electrode1_Output_Pin|Electrode2_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Electrode3_Output_Pin Electrode4_Output_Pin Solenoid1_Output_Pin Solenoid2_Output_Pin
                           Solenoid3_Output_Pin Solenoid4_Output_Pin */
  GPIO_InitStruct.Pin = Electrode3_Output_Pin|Electrode4_Output_Pin|Solenoid1_Output_Pin|Solenoid2_Output_Pin
                          |Solenoid3_Output_Pin|Solenoid4_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
