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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFSIZE 12	/*uart*/
#define MAINBUFSIZE 12	/*uart*/
#define PRESSURE_RXBUFSIZE 12	/*uart*/
#define PRESSURE_R_DROP 200	/*oxygen_pressure resistor dropper*/
#define	CONCENTRATION_THRESH	20*10	/*percentage lower limit*/
#define	FLOWRATE_UPPER_THRESH	5*10	/*litres per minute*/
#define	FLOWRATE_LOWER_THRESH	1	/*0.1 litres per minute*/
#define TEMPERATURE_THRESH 60*10	/*degrees celsius upper limit*/
#define PRESSURE_THRESH 5	/*bar*/
#define o2conc_ind 0
#define o2flow_ind 1
#define o2temp_ind 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint16_t oxygen_concentration;
uint16_t oxygen_flowrate;
uint16_t oxygen_temperature;
uint16_t oxygen_params_buf[3];
uint16_t oxygen_params[3];
uint32_t oxygen_pressure;
int16_t oxygen_deltas[3];
int feedback_pin;	/*feedback pin*/

/*uart_variables*/
uint8_t RxBuf[RXBUFSIZE];
uint8_t MainBuf[MAINBUFSIZE];
uint16_t oldPos = 0;
uint16_t newPos = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
int check_main_buf(uint8_t *buf, uint16_t size);
int modder(int num);
void get_oxygen_params(uint8_t *buf_p, uint16_t *oxygen_params);
int electrode_power_status(void);
void power_electrodes(int power_direction);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		oldPos = newPos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (oldPos+Size > MAINBUFSIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = MAINBUFSIZE-oldPos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)MainBuf+oldPos, RxBuf, datatocopy);  // copy data in that remaining space

			oldPos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)MainBuf, (uint8_t *)RxBuf+datatocopy, (Size-datatocopy));  // copy the remaining data
			newPos = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)MainBuf+oldPos, RxBuf, Size);
			newPos = Size+oldPos;
		}


		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RXBUFSIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	}


}


int check_main_buf(uint8_t *buf_p, uint16_t buf_size)
{
	int j, buf_ok = -1;
	for (int i = 0; i < buf_size * 2; i++)
	{
		j = i % buf_size;
		if ((RxBuf[j] == 0x16) && (RxBuf[j+1] == 0x09) && (RxBuf[j+1] == 0x01))
		{
			buf_ok = j;
			break;
		}
	}

	return (buf_ok);
}

int modder(int num)
{
	return (num % MAINBUFSIZE);
}

void get_oxygen_params(uint8_t *buf_p, uint16_t *oxygen_params)
{
	int buf_ok = check_main_buf(buf_p, MAINBUFSIZE);

	if (buf_ok > 0)
	{
		oxygen_params[0] = buf_p[modder(buf_ok + 3)] * 256 + buf_p[modder(buf_ok + 4)];
		oxygen_params[1] = buf_p[modder(buf_ok + 5)] * 256 + buf_p[modder(buf_ok + 6)];
		oxygen_params[2] = buf_p[modder(buf_ok + 7)] * 256 + buf_p[modder(buf_ok + 8)];
	}

	else
	{
		oxygen_params[0] = 0;
		oxygen_params[1] = 0;
		oxygen_params[2] = 0;
	}
}

int electrode_power_status(void)
{
	return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0));
}

void power_electrodes(int power_direction)	/* power off electrodes using relay */
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, power_direction);
	/* confirm electrodes power to be off
	 * if not off flash the led
	 */
	while (electrode_power_status() != power_direction)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(250);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	/* pressure sensor output is 4-20mA and takes input of 10-30VDC
	 * pressure sensor can measure 0-16bar
	 * atmospheric pressure is 1.01325bar
	 * 4mA is 1.01325bar and we can say 20mA is 16bar
	 * We can assume the pressure-current relationship is linear and use a formula
	 * y = mx + c
	 * m = (20-4)/(16-1) = (1/3)[mA/bar]
	 * with a defined resistance r, m can be (r/3)[mV/bar]
	 * voltage read from the drop across the resistor can be converted to pressure in bar
	 * x = y/m
	 */

	oxygen_pressure = (HAL_ADC_GetValue(&hadc1) * (4/4096)) / (PRESSURE_R_DROP / 3);

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)oxygen_pressure, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* store oxygen parameters to buffer before updating */
	  if (oxygen_params[2] != 0)	/* if temperature is not equal to 0 */
	  {
		  for (int i = 0; i < 3; i++)
		  {
			  oxygen_params_buf[i] = oxygen_params[i];
		  }
	  }

	  /* update oxygen parameters */
	  get_oxygen_params(MainBuf, oxygen_params);
	  if (oxygen_params[2] != 0)	/* if temperature is not equal to 0. Valid data */
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
			  if (oxygen_deltas[o2conc_ind] < 0 && electrode_power_status())
			  {
				  power_electrodes(0);
			  }
		  }

		  if (oxygen_flowrate > FLOWRATE_UPPER_THRESH)
		  {
			  /* if flowrate is greater than upper threshold
			   * and electrode power is on, we should power off electrodes
			   */
			  if (electrode_power_status())
			  {
				  power_electrodes(0);
			  }
		  }
		  else if (oxygen_flowrate < FLOWRATE_LOWER_THRESH)
		  {
			  /* if flowrate is less than lower threshold
			   * and is increasing and electrode power is on, could be system has been powered back on
			   * and is decreasing and electrode power is off, power on electrodes
			   * and is decreasing and electrode power is on, there is a likelihood of leakage , power off electrodes
			   */
			  if (oxygen_deltas[o2flow_ind] < 0 && !electrode_power_status())
			  {
				  power_electrodes(1);
			  }
			  else if (oxygen_deltas[o2flow_ind] < 0 && !electrode_power_status())
			  {
				  power_electrodes(1);
			  }
		  }

		  if (oxygen_temperature > TEMPERATURE_THRESH)
		  {
			  /* if temperature is greater than upper threshold
			   * power off electrodes
			   */
			  power_electrodes(0);
		  }
	  }


	  if (oxygen_pressure > PRESSURE_THRESH && oxygen_flowrate > 0)
	  {
		  /* if pressure is greater than upper threshold
		   * and there is flow into the collection tank
		   * power off electrodes
		   */
		  power_electrodes(0);
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
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
