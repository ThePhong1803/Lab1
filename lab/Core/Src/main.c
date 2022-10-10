/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  //to turn off all the led from PA4 to PA5, which mean to set these pin high while ignore
  //PA0 to PA3, we just need to OR the Output Data register with the value 0xfff0
  void clearAllClock(){
	  GPIOA -> ODR |= 0xfff0;
  }

  //to set the led on, we need to pull the corresponding pin low while ignore other pin
  //to do that we just need to AND the Output Data Register with the suitable bit's pattern
  //to turn it on.
  void setNumberOnClock(int num){
	  if(num == 1) {
		  GPIOA -> ODR &= 0xffef;   //number 1 on clock
	  }
	  if(num == 2) {
		  GPIOA -> ODR &= 0xffdf;   //number 2 on clock 
	  }
	  if(num == 3) {
		  GPIOA -> ODR &= 0xffbf;   //number 3 on clock
	  }
	  if(num == 4) {
		  GPIOA -> ODR &= 0xff7f;   //number 4 on clock
	  }
	  if(num == 5) {
		  GPIOA -> ODR &= 0xfeff;   //number 5 on clock
	  }
	  if(num == 6) {
		  GPIOA -> ODR &= 0xfdff;   //number 6 on clock
	  }
	  if(num == 7) {
		  GPIOA -> ODR &= 0xfbff;   //number 7 on clock
	  }
	  if(num == 8) {
		  GPIOA -> ODR &= 0xf7ff;   //number 8 on clock
	  }
	  if(num == 9) {
		  GPIOA -> ODR &= 0xefff;   //number 9 on clock
	  }
	  if(num == 10) {
		  GPIOA -> ODR &= 0xdfff;   //number 10 on clock
	  }
	  if(num == 11) {
		  GPIOA -> ODR &= 0xbfff;   //number 11 on clock
	  }
	  if(num == 0) {
		  GPIOA -> ODR &= 0x7fff;   //number 0 on clock
	  }
  }

  //to clear a pin, we just need to OR the Output Data Register with the position of the pin is 1, the other are 0
  void clearNumberOnClock(int num){
	  if(num == 1) {
		  GPIOA -> ODR |= 0x0010;   //clear number 1 on clock
	  }
	  if(num == 2) {
		  GPIOA -> ODR |= 0x0020;   //clear number 2 on clock
	  }
	  if(num == 3) {
		  GPIOA -> ODR |= 0x0040;   //clear number 3 on clock
	  }
	  if(num == 4) {
		  GPIOA -> ODR |= 0x0080;   //clear number 4 on clock
	  }
	  if(num == 5) {
		  GPIOA -> ODR |= 0x0100;   //clear number 5 on clock
	  }
	  if(num == 6) {
		  GPIOA -> ODR |= 0x0200;   //clear number 6 on clock
	  }
	  if(num == 7) {
		  GPIOA -> ODR |= 0x0400;   //clear number 7 on clock
	  }
	  if(num == 8) {
		  GPIOA -> ODR |= 0x0800;   //clear number 8 on clock
	  }
	  if(num == 9) {
		  GPIOA -> ODR |= 0x1000;   //clear number 9 on clock
	  }
	  if(num == 10) {
		  GPIOA -> ODR |= 0x2000;   //clear number 10 on clock
	  }
	  if(num == 11) {
		  GPIOA -> ODR |= 0x4000;   //clear number 11 on clock
	  }
	  if(num == 0) {
		  GPIOA -> ODR |= 0x8000;   //clear number 0 on clock
	  }
  }

  /* USER CODE END 2 */
  clearAllClock();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L1_Pin|L2_Pin|L3_Pin|L4_Pin
                          |L5_Pin|L6_Pin|L7_Pin|L8_Pin
                          |L9_Pin|L10_Pin|L11_Pin|L12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L1_Pin L2_Pin L3_Pin L4_Pin
                           L5_Pin L6_Pin L7_Pin L8_Pin
                           L9_Pin L10_Pin L11_Pin L12_Pin */
  GPIO_InitStruct.Pin = L1_Pin|L2_Pin|L3_Pin|L4_Pin
                          |L5_Pin|L6_Pin|L7_Pin|L8_Pin
                          |L9_Pin|L10_Pin|L11_Pin|L12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
