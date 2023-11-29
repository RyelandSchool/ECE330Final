/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c ECE330L Lab 6
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
//int DelayValue = 50;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
void Seven_Segment_Digit (unsigned char digit, unsigned char hex_char);
void Seven_Segment(unsigned int HexValue);



/* USER CODE BEGIN PFP */

/* USER function prototypes */
void Play_Note(int note,int size,int tempo,int space);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Global variables */
int TONE = 0;
int Note = 0;

int Song[100][3] =
{
		{E5,_8th,0},{D5,_16th,0},{A4,_16th,0},{A4,_8th,0},{F4,_8th,0}, {E5,_8th,0},{D5,_16th,0},{A4,_16th,0},{A4,_8th,0},{F4,_8th,0},
		{C5,_8th,0},{B4,_16th,0},{F4,_16th,0},{F4,_8th,0},{D4,_8th,0}, {C5,_8th,0},{B4,_16th,0},{F4,_16th,0},{F4,_8th,0},{D4,_8th,0},
		{B4,_8th,0},{A4,_16th,0},{F4,_16th,0},{F4,_8th,0},{D4,_8th,0},{B3,half,10},
		{A3,_8th,10},{B3,_8th,10},{C4,_8th,10},{D4,_8th,10},{E4,_8th,10},{F4,_8th,10},{G4,_8th,10},{A4,_8th,10},

		{E5,_8th,0},{D5,_16th,0},{A4,_16th,0},{A4,_8th,0},{F4,_8th,0}, {E5,_8th,0},{D5,_16th,0},{A4,_16th,0},{A4,_8th,0},{F4,_8th,0},
		{C5,_8th,0},{B4,_16th,0},{F4,_16th,0},{F4,_8th,0},{D4,_8th,0}, {C5,_8th,0},{B4,_16th,0},{F4,_16th,0},{F4,_8th,0},{D4,_8th,0},
		{B4,_8th,0},{A4,_16th,0},{F4,_16th,0},{F4,_8th,0},{D4,_8th,0},{B3,half,10},
		{A3,_8th,10},{B3,_8th,10},{C4,_8th,10},{D4,_8th,10},{E4,_8th,10},{F4,_8th,10},{G4,_8th,10},{A4,_8th,10},

};

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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  /*** Configure GPIOs ***/
  GPIOD->MODER = 0x55555555; // set all Port D pins to outputs
  GPIOE->MODER |= 0x55555555;
  GPIOA->MODER |= 0x000000FF;
  GPIOE->OTYPER |= 0xFF00;

  //Configure ADC1
  RCC->APB2ENR |= 1<<8;
  ADC1->SMPR2 |= 1;
  ADC1->CR2 |= 1;
  /*****************************************************************************************************
  These commands are handled as part of the MX_TIM7_Init() function and don't need to be enabled:
  RCC->AHB1ENR |= 1<<5; // Enable clock for timer 7
  __enable_irq(); // Enable interrupts
  NVIC_EnableIRQ(TIM7_IRQn); // Enable Timer 7 Interrupt in the NVIC controller
  *******************************************************************************************************/

  TIM7->PSC = 167; //1Mhz timer clock prescaler value, 1Mhz = 168Mhz / (167 + 1)
  TIM7->ARR = 3; // 1MHz/(3+1) = 250Khz interrupt rate to increment byte counter for 250Khz/256 = 976Hz PWM
  TIM7->DIER |= 1; // Enable timer 7 interrupt
  TIM7->CR1 |= 1; // Enable timer counting

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
	  Seven_Segment(0x5AFE0000);

	  ADC1->SQR3 = 1;
	  ADC1->CR2 |= 1<<30;

	  int myVar = ADC1->DR;

	  /*We need to make a range to switch so there are only
	   * 2 LEDS on at once....*/
//	  if(myVar > 2000){
//		  GPIOD->ODR = 0xff00;
//	  } else {
//		  GPIOD->ODR = 0x00ff;
//	  }

	   // Moving the paddle around based on ADC movement
	  const int DELIM = 512;
	  if(myVar > 0 && myVar < DELIM){
		  GPIOD->ODR = 3;
	  } else if (myVar < DELIM*2){
		  GPIOD->ODR = 3<<2;
	  } else if (myVar < DELIM*3){
		  GPIOD->ODR = 3<<4;
	  } else if (myVar < DELIM*4){
		  GPIOD->ODR = 3<<6;
	  } else if (myVar < DELIM*5){
		  GPIOD->ODR = 3<<8;
	  } else if (myVar < DELIM*6){
		  GPIOD->ODR = 3<<10;
	  } else if (myVar < DELIM*7){
		  GPIOD->ODR = 3<<12;
	  } else if (myVar < DELIM*8){
		  GPIOD->ODR = 3<<14;
	  }

	  int i;

	  //The rightmost LED is in use by the Piezo buzzer, don't heck with it yo
	  /* play the tune defined in the array Song */
//	  for (i = 0;i<(sizeof(Song)/sizeof(Song[0]));i++) // determine number of elements in array for loop maximum
//	  {
//		  Play_Note(Song[i][0],Song[i][1],3200,Song[i][2]); // Call function to play each note
//	  }

//	  HAL_Delay(1000); // Delay for 1000 milliseconds (1 second)

    /* USER CODE END WHILE */

  }

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Play_Note(int note,int size,int tempo,int space)
{
	Note = note; // Assign passed note parameter to global Note variable

	// Delay to hold note for the duration of the note size, e.g. Whole note, half note, etc.
	if (size > 0 && size <= 64) HAL_Delay(tempo/size-space); // Delay based on tempo, note size, and note separation (space)
	Note = 0; // Turn note off
	HAL_Delay(space); // Delay for the amount of silence needed to separate notes from each other
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
void Seven_Segment_Digit (unsigned char digit, unsigned char hex_char)
{
/*******************************************************************************
Code to mask and bit shift 0-7 value of digit and 0-15 value of hex_char
to output correct bit pattern to GPIO_Output
*******************************************************************************/

        GPIOE->ODR = 0xC000 | ((~digit & 3) << 12) | ((hex_char & 0xF) << 8);

        HAL_Delay(1); // delay for 1 milliseconds
        // if digit > 3 set upper character chip select bit low using a mask value in GPIO_Output
        if (digit > 3) GPIOE->ODR &= ~(1 << 15);
        // else set lower character chip select bit low using a mask value in GPIO_Output
        else GPIOE->ODR &= ~(1 << 14);

        HAL_Delay(1); // delay for 1 milliseconds
        GPIOE->ODR |= 0xC000;
        return;
}

void Seven_Segment(unsigned int HexValue)
{
/******************************************************************************
Use a for loop to output HexValue to 7 segment display digits
*******************************************************************************/
        char digit;
        // Send hex values to lower 4 digits
        for (digit=0 ; digit<8 ; digit++)
        {
           Seven_Segment_Digit(digit,(HexValue >> (digit*4)) & 0xF);
        }

        return;
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
