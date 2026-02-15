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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
float Accel_X_OFF = 0, Accel_Y_OFF = 0, Accel_Z_OFF = 0; // will take avg of sensor reading while still to calibrate.
float velocity_x = 0;
float distance_x = 0;
float peak_velocity = 0;
uint32_t last_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#include <stdio.h>

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
	while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t check; // wake up sensor first!
  uint8_t Data;

  HAL_I2C_Mem_Read(&hi2c1, 0xD2, 0x75, 1, &check, 1, 1000); // checks device ID - 0x75 is address for MPU6050
  if (check == 0x68) { // 0x68 is default response for a healthy MPU6050
	  Data = 0;        // Register 0x6B is Power Management
	  HAL_I2C_Mem_Write(&hi2c1, 0xD2, 0x6B, 1, &Data, 1, 1000); // writing 0 wakes sensor up
  }

  printf("Calibrating... Keep the sensor flat!\r\n"); // calibrate sensor next!
  float tempX = 0, tempY = 0, tempZ = 0;

  uint8_t Cal_Data[6];
  int16_t raw_x, raw_y, raw_z;

  for(int i = 0; i < 100; i++) {
      // Read your RAW data here (same logic as your loop)
	  // FIXED: ADDED ACTUAL I2C READ AND BIT SHIFTING TO CALIBRATION LOOP TO POPULATE temp VARIABLES
	  if(HAL_I2C_Mem_Read(&hi2c1, 0xD2, 0x3B, 1, Cal_Data, 6, 1000) == HAL_OK) {
		  raw_x = (int16_t)(Cal_Data[0] << 8 | Cal_Data[1]);
		  raw_y = (int16_t)(Cal_Data[2] << 8 | Cal_Data[3]);
		  raw_z = (int16_t)(Cal_Data[4] << 8 | Cal_Data[5]);

		  // Add to tempX, tempY, tempZ
		  tempX += (raw_x / 16384.0);
		  tempY += (raw_y / 16384.0);
		  tempZ += (raw_z / 16384.0);
	  }
      HAL_Delay(10);
  }

  Accel_X_OFF = tempX / 100.0;
  Accel_Y_OFF = tempY / 100.0;
  Accel_Z_OFF = (tempZ / 100.0) - 1.0; // We subtract 1.0 because Z should see gravity
  printf("Calibration Done!\r\n");

  // FIXED: CHANGED hi2c1 TO htim2 TO ACTUALLY START THE TIMER
  HAL_TIM_Base_Start(&htim2); // start timer next!

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t Rec_Data[6];
	  int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
	  float ax, ay, az;

	  if(HAL_I2C_Mem_Read(&hi2c1, 0xD2, 0x3B, 1, Rec_Data, 6, 1000) == HAL_OK) // read 6 bytes starting from ACCEL_XOUT_H register (0x3B)
	  {
		  // COMBINE 2 8-bit registers into 1 16-bit signed int
		  Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		  Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		  Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	  	  // convert to 'g' force assuming default range of +/- 2g, sensitivaitvy factor for +/- 2g is 16384.0
	  	  ax = (Accel_X_RAW / 16384.0) - Accel_X_OFF;
	  	  ay = (Accel_Y_RAW / 16384.0) - Accel_Y_OFF;
	  	  az = (Accel_Z_RAW / 16384.0) - Accel_Z_OFF;
	  }

	  uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim2);
	  float dt = (current_time - last_time) / 1000000.0f; // convert microseconds to seconds
	  last_time = current_time; //update

	  // FIXED: SLIGHTLY INCREASED DEADZONE TO 0.1 TO PREVENT DRIFT DURING DOUBLE INTEGRATION
	  if (ax > 0.1 || ax < -0.1) {
		  velocity_x += (ax * 9.81f) * dt;
		  // FIXED: INTEGRATED VELOCITY TO GET DISTANCE
		  distance_x += velocity_x * dt;

		  // FIXED: TRACK PEAK VELOCITY (ABSOLUTE VALUE TO ACCOUNT FOR BACKWARD MOVES)
		  float current_v_abs = (velocity_x < 0) ? -velocity_x : velocity_x;
		  if (current_v_abs > peak_velocity) peak_velocity = current_v_abs;

		  printf("V: %.2f | D: %.2f\r\n", velocity_x, distance_x);
	  }

	  else {
		  // FIXED: ADDED LOGIC TO PRINT SUMMARY AND RESET ONCE MOVEMENT STOPS
		  if (velocity_x != 0) {
			  printf("---- MOVE COMPLETE ----\r\n");
			  printf("Peak: %.2f m/s | Total Dist: %.2f m\r\n", peak_velocity, distance_x);
			  printf("-----------------------\r\n");

			  velocity_x = 0;
			  distance_x = 0;
			  peak_velocity = 0;
		  }
	  }

	  // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); i just used this to check that microcontroller works (this blinks green)
	  HAL_Delay(10); // 100hz refresh rate for now
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
