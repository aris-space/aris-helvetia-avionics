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
#include "i2c.h"
#include "icache.h"
#include "spi.h"
#include "app_usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "aris-helvetia-state-estimation/Inc/main_state_est.h" //Needs to be removed after testing
#include "aris-helvetia-state-estimation/Inc/bno055_stm32.h"
#include "Drivers/TE_Connectivity/Inc/ms5607.h"
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Flags for state estimation: TODO: remove after testing
uint8_t uart_rx_flag = 0; //flag which enables receiving commands over UART
uint8_t rx_data_uart = 0; //buffer to store commands received over UART
uint8_t uart_rx_data_parse_flag = 0; //flag which enables parsing of incoming UART commands
uint8_t uart_tx_data_enable_flag = 0;
uint8_t run_flag = 0; //flag which enables state estimation
uint8_t calibration_on_startup_flag = 0; // Set true to calibrate sensor
uint8_t system_calibration = 0;
uint8_t sensor_before_calibration_flag = 1; // If sensor is not calibrated yet
uint8_t state_estimation_first_loop_flag = 1; // In first loop only z prev can be set
uint8_t state_estimation_flag = 0;
uint8_t state_estimation_calibration_flag = 0; // Flag to get a better accuracy in state estimation by bias correction
uint8_t correction_state = 0;
double correction_accel_x = 0;
double correction_accel_y = 0;
double correction_accel_z = 0;
double correction_gyro_x = 0;
double correction_gyro_y = 0;
double correction_gyro_z = 0;

double min_value = 0.05;
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
  MX_USB_PCD_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_SPI2_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */
  bno055_assignI2C(&hi2c1); //don't know which i2c connection is used
  bno055_setup();
  ms5607_assignI2C(&hi2c3); //don't know which i2c connection is used
  ms5607_setup();

  //Test whether bno055 is working correctly
  bno055_self_test_result_t self_test = bno055_getSelfTestResult();
  if (self_test.gyrState == 0 || self_test.magState == 0 || self_test.accState == 0) return 0;
  bno055_setPage(1);
  bno055_writeData(BNO055_ACC_CONFIG, 0b00001111); //set acceleration to 16g instead of 4g by default
  bno055_setPage(0);
  bno055_setOperationModeNDOF();

  // Initialisation of all state estimation variables need to be removed after testing
  ekf_state_t ekf_state;
  flight_phase_detection_t flight_phase_detection;
  flight_phase_detection.flight_phase = IDLE; // No idea why this isn't done by default
  state_est_meas_t z;
  state_est_meas_t z_prev;
  env_t env;
  state_est_data_t state_est_data;

  uint32_t t;
  uint32_t t0 = HAL_GetTick();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
