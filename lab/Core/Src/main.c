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
  void display7Segment1(int num){
		if(num == 0){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0040;
		}
		if(num == 1){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0079;
		}
		if(num == 2){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0024;
		}
		if(num == 3){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0030;
		}
		if(num == 4){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0019;
		}
		if(num == 5){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0012;
		}
		if(num == 6){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0002;
		}
		if(num == 7){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0078;
		}
		if(num == 8){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0080;
		}
		if(num == 9){
			GPIOB -> ODR &= 0xff00;
			GPIOB -> ODR |= 0x0090;
		}
	}

  void display7Segment2(int num){
  		if(num == 0){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x4000;
  		}
  		if(num == 1){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x7900;
  		}
  		if(num == 2){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x2400;
  		}
  		if(num == 3){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x3000;
  		}
  		if(num == 4){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x1900;
  		}
  		if(num == 5){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x1200;
  		}
  		if(num == 6){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x0200;
  		}
  		if(num == 7){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x7800;
  		}
  		if(num == 8){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x8000;
  		}
  		if(num == 9){
  			GPIOB -> ODR &= 0x00ff;
  			GPIOB -> ODR |= 0x9000;
  		}
  	}
  HAL_GPIO_WritePin(LED_RED1_GPIO_Port, LED_RED1_Pin, RESET);
  HAL_GPIO_WritePin(LED_YELLOW1_GPIO_Port, LED_YELLOW1_Pin, SET);
  HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, SET);

  HAL_GPIO_WritePin(LED_RED2_GPIO_Port, LED_RED2_Pin, SET);
  HAL_GPIO_WritePin(LED_YELLOW2_GPIO_Port, LED_YELLOW2_Pin, SET);
  HAL_GPIO_WritePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin, RESET);

  int rcd1 = 5, rcd2 = -1;
  int ycd1 = -1, ycd2 = -1;
  int gcd1 = -1, gcd2 = 3;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);
	  if(rcd1 > 0) {
		  rcd1--;
		  display7Segment1(rcd1);
	  }
	  if(ycd1 > 0) {
		  ycd1--;
		  display7Segment1(ycd1);
	  }
	  if(gcd1 > 0) {
		  gcd1--;
		  display7Segment1(gcd1);
	  }

	  if(rcd2 > 0) {
		  rcd2--;
		  display7Segment2(rcd2);
	  }
	  if(ycd2 > 0) {
		  ycd2--;
		  display7Segment2(ycd2);
	  }
	  if(gcd2 > 0) {
		  gcd2--;
		  display7Segment2(gcd2);
	  }


	  if(!rcd1){
		  HAL_GPIO_TogglePin(LED_RED1_GPIO_Port, LED_RED1_Pin);
		  HAL_GPIO_TogglePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin);
		  rcd1 = -1;
		  gcd1 = 3;
	  }

	  if(!ycd1){
		  HAL_GPIO_TogglePin(LED_YELLOW1_GPIO_Port, LED_YELLOW1_Pin);
		  HAL_GPIO_TogglePin(LED_RED1_GPIO_Port, LED_RED1_Pin);
		  ycd1 = -1;
		  rcd1 = 5;
	  }

	  if(!gcd1){
		  HAL_GPIO_TogglePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin);
		  HAL_GPIO_TogglePin(LED_YELLOW1_GPIO_Port, LED_YELLOW1_Pin);
		  gcd1 = -1;
		  ycd1 = 2;
	  }

	  if(!rcd2){
		  HAL_GPIO_TogglePin(LED_RED2_GPIO_Port, LED_RED2_Pin);
		  HAL_GPIO_TogglePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin);
		  rcd2 = -1;
		  gcd2= 3;
	  }

	  if(!ycd2){
		  HAL_GPIO_TogglePin(LED_YELLOW2_GPIO_Port, LED_YELLOW2_Pin);
		  HAL_GPIO_TogglePin(LED_RED2_GPIO_Port, LED_RED2_Pin);
		  ycd2 = -1;
		  rcd2 = 5;
	  }

	  if(!gcd2){
		  HAL_GPIO_TogglePin(LED_GREEN2_GPIO_Port, LED_GREEN2_Pin);
		  HAL_GPIO_TogglePin(LED_YELLOW2_GPIO_Port, LED_YELLOW2_Pin);
		  gcd2 = -1;
		  ycd2 = 2;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED1_Pin|LED_YELLOW1_Pin|LED_GREEN1_Pin|LED_RED2_Pin
                          |LED_YELLOW2_Pin|LED_GREEN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, a1_Pin|b1_Pin|c1_Pin|c2_Pin
                          |d2_Pin|e2_Pin|f2_Pin|g2_Pin
                          |d1_Pin|e1_Pin|f1_Pin|g1_Pin
                          |a2_Pin|b2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED1_Pin LED_YELLOW1_Pin LED_GREEN1_Pin LED_RED2_Pin
                           LED_YELLOW2_Pin LED_GREEN2_Pin */
  GPIO_InitStruct.Pin = LED_RED1_Pin|LED_YELLOW1_Pin|LED_GREEN1_Pin|LED_RED2_Pin
                          |LED_YELLOW2_Pin|LED_GREEN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : a1_Pin b1_Pin c1_Pin c2_Pin
                           d2_Pin e2_Pin f2_Pin g2_Pin
                           d1_Pin e1_Pin f1_Pin g1_Pin
                           a2_Pin b2_Pin */
  GPIO_InitStruct.Pin = a1_Pin|b1_Pin|c1_Pin|c2_Pin
                          |d2_Pin|e2_Pin|f2_Pin|g2_Pin
                          |d1_Pin|e1_Pin|f1_Pin|g1_Pin
                          |a2_Pin|b2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
