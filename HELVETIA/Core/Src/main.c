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
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 1);
	  if(run_flag){ //state estimation is running
		  if(sensor_before_calibration_flag){
			  if(calibration_on_startup_flag && system_calibration < 3){
				  bno055_calibration_state_t cal_state = bno055_getCalibrationState();
				  uint8_t sending_buffer [5] = {56,cal_state.sys,cal_state.gyro,cal_state.mag,cal_state.accel};
				  HAL_UART_Transmit(&huart3, (uint8_t*)sending_buffer, 5, 10);
				  system_calibration = min(cal_state.mag,min(cal_state.gyro,cal_state.accel));
			  }
			  else if(calibration_on_startup_flag){
				  // TO DO: Results not checked!
				  bno055_calibration_data_t cal_data = bno055_getCalibrationData();
				  uint8_t sending_buffer[23] = {0};
				  sending_buffer[0]=59;
				  sending_buffer[1]=(uint8_t)(cal_data.offset.gyro.x >> 8);
				  sending_buffer[2]=(uint8_t)(cal_data.offset.gyro.x & 0xff);
				  sending_buffer[3]=(uint8_t)(cal_data.offset.gyro.y >> 8);
				  sending_buffer[4]=(uint8_t)(cal_data.offset.gyro.y & 0xff);
				  sending_buffer[5]=(uint8_t)(cal_data.offset.gyro.z >> 8);
				  sending_buffer[6]=(uint8_t)(cal_data.offset.gyro.z & 0xff);
				  sending_buffer[7]=(uint8_t)(cal_data.offset.mag.x >> 8);
				  sending_buffer[8]=(uint8_t)(cal_data.offset.mag.x & 0xff);
				  sending_buffer[9]=(uint8_t)(cal_data.offset.mag.y >> 8);
				  sending_buffer[10]=(uint8_t)(cal_data.offset.mag.y & 0xff);
				  sending_buffer[11]=(uint8_t)(cal_data.offset.mag.z >> 8);
				  sending_buffer[12]=(uint8_t)(cal_data.offset.mag.z & 0xff);
				  sending_buffer[13]=(uint8_t)(cal_data.offset.accel.x >> 8);
				  sending_buffer[14]=(uint8_t)(cal_data.offset.accel.x & 0xff);
				  sending_buffer[15]=(uint8_t)(cal_data.offset.accel.y >> 8);
				  sending_buffer[16]=(uint8_t)(cal_data.offset.accel.y & 0xff);
				  sending_buffer[17]=(uint8_t)(cal_data.offset.accel.z >> 8);
				  sending_buffer[18]=(uint8_t)(cal_data.offset.accel.z & 0xff);
				  sending_buffer[19]=(uint8_t)(cal_data.radius.mag >> 8);
				  sending_buffer[20]=(uint8_t)(cal_data.radius.mag & 0xff);
				  sending_buffer[21]=(uint8_t)(cal_data.radius.accel >> 8);
				  sending_buffer[22]=(uint8_t)(cal_data.radius.accel & 0xff);
				  HAL_UART_Transmit(&huart3, (uint8_t*)sending_buffer, sizeof(sending_buffer), 100);
				  uint8_t command = 77;
				  HAL_Delay(10);
				  HAL_UART_Transmit(&huart3,&command,sizeof(command),100);
				  sensor_before_calibration_flag = 0;
			  }
			  else if(system_calibration < 3){ // If command calibration_on_startup flag is not set load old config
				  bno055_calibration_data_t cal_data;
				  cal_data.offset.accel.x = CONFIG_BNO_ACCEL_OFFSET_X;
				  cal_data.offset.accel.y = CONFIG_BNO_ACCEL_OFFSET_Y;
				  cal_data.offset.accel.z = CONFIG_BNO_ACCEL_OFFSET_Z;
				  cal_data.radius.accel = CONFIG_BNO_ACCEL_OFFSET_R;
				  cal_data.offset.gyro.x = CONFIG_BNO_GYRO_OFFSET_X;
				  cal_data.offset.gyro.y = CONFIG_BNO_GYRO_OFFSET_Y;
				  cal_data.offset.gyro.z = CONFIG_BNO_GYRO_OFFSET_Z;
				  cal_data.offset.mag.x = CONFIG_BNO_MAG_OFFSET_X;
				  cal_data.offset.mag.y = CONFIG_BNO_MAG_OFFSET_Y;
				  cal_data.offset.mag.z = CONFIG_BNO_MAG_OFFSET_Z;
				  cal_data.radius.mag = CONFIG_BNO_MAG_OFFSET_R;
				  bno055_setCalibrationData(cal_data);
				  sensor_before_calibration_flag = 0; // sensor now calibrated
			  }

		  }// sensor calibration part end
		  bno055_vector_t gyro_vec = bno055_getVectorGyroscope();
		  bno055_vector_t mag_vec = bno055_getVectorMagnetometer();
		  bno055_vector_t accel_vec = bno055_getVectorLinearAccel(); // Linear Acceleration already has gravity compensation
		  bno055_vector_t euler = bno055_getVectorEuler();
		  bno055_vector_t quat = bno055_getVectorQuaternion();
      ms5607_Update()
		  if (state_estimation_flag){
			  if (fabs(gyro_vec.x - correction_gyro_x) < min_value) gyro_vec.x = 0;
			  else gyro_vec.x -= correction_gyro_x;

			  if (fabs(gyro_vec.y - correction_gyro_y) < min_value) gyro_vec.y = 0;
			  else gyro_vec.y -= correction_gyro_y;

			  if (fabs(gyro_vec.z - correction_gyro_z) < min_value) gyro_vec.z = 0;
			  else gyro_vec.z -= correction_gyro_z;

			  if (fabs(accel_vec.x - correction_accel_x) < min_value) accel_vec.x = 0;
			  else accel_vec.x -= correction_accel_x;

			  if (fabs(accel_vec.y - correction_accel_y) < min_value) accel_vec.y = 0;
			  else accel_vec.y -= correction_accel_y;

			  if (fabs(accel_vec.z - correction_accel_z) < min_value) accel_vec.z = 0;
			  else accel_vec.z -= correction_accel_z;
		  }
		  float temp_imu = convert_temperature(bno055_getTemp());
		  int32_t gyro_x = (int32_t)(gyro_vec.x*10000);
		  int32_t gyro_y = (int32_t)(gyro_vec.y*10000);
		  int32_t gyro_z = (int32_t)(gyro_vec.z*10000);
		  int32_t mag_x = (int32_t)(mag_vec.x*10000);
		  int32_t mag_y = (int32_t)(mag_vec.y*10000);
		  int32_t mag_z = (int32_t)(mag_vec.z*10000);
		  int32_t accel_x = (int32_t)(accel_vec.x*10000);
		  int32_t accel_y = (int32_t)(accel_vec.y*10000);
		  int32_t accel_z = (int32_t)(accel_vec.z*10000);
 		  //uint8_t sending_buffer[19] = {0};
		  uint8_t sending_buffer[37] = {0};
 		  sending_buffer[0] = 62;
 		  sending_buffer[1] = (uint8_t) (gyro_x >> 24);
 		  sending_buffer[2] = (uint8_t) (gyro_x >> 16);
 		  sending_buffer[3] = (uint8_t) (gyro_x >> 8);
 		  sending_buffer[4] = (uint8_t) (gyro_x & 0xff);
 		  sending_buffer[5] = (uint8_t) (gyro_y >> 24);
 		  sending_buffer[6] = (uint8_t) (gyro_y >> 16);
 		  sending_buffer[7] = (uint8_t) (gyro_y >> 8);
 		  sending_buffer[8] = (uint8_t) (gyro_y & 0xff);
 		  sending_buffer[9] = (uint8_t) (gyro_z >> 24);
 		  sending_buffer[10] = (uint8_t) (gyro_z>> 16);
 		  sending_buffer[11] = (uint8_t) (gyro_z >> 8);
 		  sending_buffer[12] = (uint8_t) (gyro_z & 0xff);
 		  sending_buffer[13] = (uint8_t) (mag_x >> 24);
 		  sending_buffer[14] = (uint8_t) (mag_x >> 16);
 		  sending_buffer[15] = (uint8_t) (mag_x >> 8);
 		  sending_buffer[16] = (uint8_t) (mag_x & 0xff);
 		  sending_buffer[17] = (uint8_t) (mag_y >> 24);
 		  sending_buffer[18] = (uint8_t) (mag_y >> 16);
 		  sending_buffer[19] = (uint8_t) (mag_y >> 8);
 		  sending_buffer[20] = (uint8_t) (mag_y & 0xff);
 		  sending_buffer[21] = (uint8_t) (mag_z >> 24);
 		  sending_buffer[22] = (uint8_t) (mag_z >> 16);
 		  sending_buffer[23] = (uint8_t) (mag_z >> 8);
 		  sending_buffer[24] = (uint8_t) (mag_z & 0xff);
 		  sending_buffer[25] = (uint8_t) (accel_x >> 24);
 		  sending_buffer[26] = (uint8_t) (accel_x >> 16);
 		  sending_buffer[27] = (uint8_t) (accel_x >> 8);
 		  sending_buffer[28] = (uint8_t) (accel_x & 0xff);
 		  sending_buffer[29] = (uint8_t) (accel_y >> 24);
 		  sending_buffer[30] = (uint8_t) (accel_y >> 16);
 		  sending_buffer[31] = (uint8_t) (accel_y >> 8);
 		  sending_buffer[32] = (uint8_t) (accel_y & 0xff);
 		  sending_buffer[33] = (uint8_t) (accel_z >> 24);
 		  sending_buffer[34] = (uint8_t) (accel_z >> 16);
 		  sending_buffer[35] = (uint8_t) (accel_z >> 8);
 		  sending_buffer[36] = (uint8_t) (accel_z & 0xff);
 		 /*
		  sending_buffer[0] = 62;
		  sending_buffer[1] = gyro_vec.x_uint8[0];
		  sending_buffer[2] = gyro_vec.x_uint8[1];
		  sending_buffer[3] = gyro_vec.y_uint8[0];
		  sending_buffer[4] = gyro_vec.y_uint8[1];
		  sending_buffer[5] = gyro_vec.z_uint8[0];
		  sending_buffer[6] = gyro_vec.z_uint8[1];
		  sending_buffer[7] = mag_vec.x_uint8[0];
		  sending_buffer[8] = mag_vec.x_uint8[1];
		  sending_buffer[9] = mag_vec.y_uint8[0];
		  sending_buffer[10] = mag_vec.y_uint8[1];
		  sending_buffer[11] = mag_vec.z_uint8[0];
		  sending_buffer[12] = mag_vec.z_uint8[1];
		  sending_buffer[13] = accel_vec.x_uint8[0];
		  sending_buffer[14] = accel_vec.x_uint8[1];
		  sending_buffer[15] = accel_vec.y_uint8[0];
		  sending_buffer[16] = accel_vec.y_uint8[1];
		  sending_buffer[17] = accel_vec.z_uint8[0];
		  sending_buffer[18] = accel_vec.z_uint8[1];
		  */

		  HAL_UART_Transmit(&huart3, (uint8_t*)sending_buffer, sizeof(sending_buffer), 100);
		  if (state_estimation_calibration_flag){
			  correction_accel_x += accel_vec.x;
			  correction_accel_y += accel_vec.y;
			  correction_accel_z += accel_vec.z;
			  correction_gyro_x += gyro_vec.x;
			  correction_gyro_y += gyro_vec.y;
			  correction_gyro_z += gyro_vec.z;
			  if (correction_state  == 100){ // create the average of the first x measurements to get a better bias correction
				  correction_accel_x /= correction_state;
				  correction_accel_y /= correction_state;
				  correction_accel_z /= correction_state;
				  correction_gyro_x /= correction_state;
				  correction_gyro_y /= correction_state;
				  correction_gyro_z /= correction_state;
				  correction_state = 0;
				  state_estimation_calibration_flag = 0;
				  state_estimation_flag = 1;
			  }
			  correction_state++;
		  }
		  if(state_estimation_flag){ //state estimation part of code
			  if(state_estimation_first_loop_flag){ //fist loop of state estimation -> only initialisation

				  reset_state(&ekf_state);
				  //t = clock();
				  //z_prev.timestamp = ((float)t-t0)/CLOCKS_PER_SEC;
				  t = HAL_GetTick();
				  z_prev.timestamp = (t - t0)*pow(10,-3);
				  // Accel
				  /*accel_vec.x -= correction_accel_x;
				  if (fabs(accel_vec.x) > min_value) z_prev.acc[0][0] = accel_vec.x;
				  else z_prev.acc[0][0] = 0;*/
				  z_prev.acc[0][0] = accel_vec.x; //- correction_accel_x;
				  /*accel_vec.y -= correction_accel_y;
				  if (fabs(accel_vec.y) > min_value) z_prev.acc[1][0] = accel_vec.y;
				  else z_prev.acc[1][0] = 0;*/
				  z_prev.acc[1][0] = accel_vec.y; //- correction_accel_y;
				  /*accel_vec.z -= correction_accel_z;
				  if (fabs(accel_vec.z) > min_value) z_prev.acc[2][0] = accel_vec.z;
				  else z_prev.acc[2][0] = 0;*/
				  z_prev.acc[2][0] = accel_vec.z; //- correction_accel_z;

				  // Gyro
				  /*gyro_vec.x -= correction_gyro_x;
				  if (fabs(gyro_vec.x) > min_value) z_prev.gyro[0][0] = gyro_vec.x;
				  else z_prev.gyro[0][0] = 0;*/
				  z_prev.gyro[0][0] = gyro_vec.x; //- correction_gyro_x;
				  /*gyro_vec.y -= correction_gyro_y;
				  if (fabs(gyro_vec.y) > min_value) z_prev.gyro[1][0] = gyro_vec.y;
				  else z_prev.gyro[1][0] = 0;*/
				  z_prev.gyro[1][0] = gyro_vec.y; //- correction_gyro_y;
				  /*gyro_vec.x -= correction_gyro_x;
				  if (fabs(gyro_vec.z) > min_value) z_prev.gyro[2][0] = gyro_vec.z;
				  else z_prev.gyro[2][0] = 0;*/
				  z_prev.gyro[2][0] = gyro_vec.z; // - correction_gyro_z;

				  // Magn
				  z_prev.mag[0][0] = mag_vec.x;
				  z_prev.mag[1][0] = mag_vec.y;
				  z_prev.mag[2][0] = mag_vec.z;

				  // add barometer here if used
          z_prev.pressure = ms5607_getPreassure();

				  // Temperature
				  z_prev.temperature_imu = temp_imu;
				  // add temperature of barometer here
          z_prev.temperature_baro = ms5607_getTemperature();
				  //set reference pressure and temperature
          set_reference_values(z_prev,env);
				  // Euler angles
				  z_prev.euler[0][0] = euler.x;
				  z_prev.euler[1][0] = euler.y;
				  z_prev.euler[2][0] = euler.z;

				  // Quaternion
				  z_prev.quat[0][0] = quat.w;
				  z_prev.quat[1][0] = quat.x;
				  z_prev.quat[2][0] = quat.y;
				  z_prev.quat[3][0] = quat.z;

				  // First initialisation of the state
				  ekf_state.X_e[0][0] = 0;
				  ekf_state.X_e[1][0] = 0;
				  ekf_state.X_e[2][0] = 0;
				  ekf_state.X_e[3][0] = 0;
				  ekf_state.X_e[4][0] = 0;
				  ekf_state.X_e[5][0] = 0;
				  ekf_state.X_e[6][0] = quat.w;
				  ekf_state.X_e[7][0] = quat.x;
				  ekf_state.X_e[8][0] = quat.y;
				  ekf_state.X_e[9][0] = quat.z;



				  state_estimation_first_loop_flag = 0;
			  }
			  else{ // not first loop of state estimation
				  //t = clock();
				  //z.timestamp = ((float)t-t0)/CLOCKS_PER_SEC;
				  t = HAL_GetTick();
				  z.timestamp = (t - t0)*pow(10,-3);
				  //HAL_GetTick();
				  // Accel
				  state_est_data.acc[0][0] = accel_vec.x; //- correction_accel_x;
				  z.acc[0][0] = accel_vec.x; //- correction_accel_x;
				  state_est_data.acc[1][0] = accel_vec.y; //- correction_accel_y;
				  z.acc[1][0] = accel_vec.y; //- correction_accel_y;
				  state_est_data.acc[2][0] = accel_vec.z; //- correction_accel_z;
				  z.acc[2][0] = accel_vec.z; //- correction_accel_z;
				  // Gyro
				  state_est_data.gyro[0][0] = gyro_vec.x; //- correction_gyro_x;
				  z.gyro[0][0] = gyro_vec.x; //- correction_gyro_x;
				  state_est_data.gyro[1][0] = gyro_vec.y; //- correction_gyro_y;
				  z.gyro[1][0] = gyro_vec.y; //- correction_gyro_y;
				  state_est_data.gyro[2][0] = gyro_vec.z; //- correction_gyro_z;
				  z.gyro[2][0] = gyro_vec.z; //- correction_gyro_z;
				  // Magn
				  state_est_data.mag[0][0] = accel_vec.x;
				  z.mag[0][0] = mag_vec.x;
				  state_est_data.mag[1][0] = mag_vec.y;
				  z.mag[1][0] = mag_vec.y;
				  state_est_data.mag[2][0] = mag_vec.z;
				  z.mag[2][0] = mag_vec.z;

				  // add barometer here if used
          z.pressure = ms5607_getPreassure();
				  // Temperature
				  z.temperature_imu = temp_imu;
				  // add temperature of barometer here
          z.temperature_baro = ms5607_getTemperature();
				  // Euler angles
				  z.euler[0][0] = euler.x;
				  z.euler[1][0] = euler.y;
				  z.euler[2][0] = euler.z;

				  // Quaternion
				  z.quat[0][0] = quat.w;
				  z.quat[1][0] = quat.x;
				  z.quat[2][0] = quat.y;
				  z.quat[3][0] = quat.z;

				  // Calculate mach number
				  state_est_data.mach_number = mach_number(&state_est_data,&z);

				  detect_flight_phase(&flight_phase_detection, &state_est_data); // Detects current state of rocket

				  ekf_estimation_cycle(&ekf_state, &z, &z_prev, &flight_phase_detection, &env);

				  // Update z_prev
				  z_prev.timestamp = z.timestamp;
				  // Accel
				  z_prev.acc[0][0] = z.acc[0][0];
				  z_prev.acc[1][0] = z.acc[1][0];
				  z_prev.acc[2][0] = z.acc[2][0];
				  // Gyro
				  z_prev.gyro[0][0] = z.gyro[0][0];
				  z_prev.gyro[1][0] = z.gyro[0][0];
				  z_prev.gyro[2][0] = z.gyro[0][0];
				  // Magn
				  z_prev.mag[0][0] = z.mag[0][0];
				  z_prev.mag[1][0] = z.mag[0][0];
				  z_prev.mag[2][0] = z.mag[0][0];

				  // add barometer here if used
          z_prev.pressure = z.pressure;
				  // Temperature
				  z_prev.temperature_imu = z.temperature_imu;
				  // add temperature of barometer here
          z_prev.temperature_baro = z.temperature_baro;

				  // Euler angles
				  z_prev.euler[0][0] = z.euler[0][0];
				  z_prev.euler[1][0] = z.euler[1][0];
				  z_prev.euler[2][0] = z.euler[2][0];

				  // Quaternion
				  z_prev.quat[0][0] = z.quat[0][0];
				  z_prev.quat[1][0] = z.quat[1][0];
				  z_prev.quat[2][0] = z.quat[2][0];
				  z_prev.quat[3][0] = z.quat[3][0];


				  // Return current position and velocity
				  int32_t pos_x = (int32_t)(ekf_state.X_e[0][0]*1000);
				  int32_t pos_y = (int32_t)(ekf_state.X_e[1][0]*1000);
				  int32_t pos_z = (int32_t)(ekf_state.X_e[2][0]*1000);
				  int32_t vel_x = (int32_t)(ekf_state.X_e[3][0]*1000);
				  int32_t vel_y = (int32_t)(ekf_state.X_e[4][0]*1000);
				  int32_t vel_z = (int32_t)(ekf_state.X_e[5][0]*1000);
				  int32_t q_w = (int32_t)(ekf_state.X_e[6][0]*1000);
				  int32_t q_x = (int32_t)(ekf_state.X_e[6][0]*1000);
				  int32_t q_y = (int32_t)(ekf_state.X_e[6][0]*1000);
				  int32_t q_z = (int32_t)(ekf_state.X_e[6][0]*1000);

				  uint8_t send_buffer [41] = {0};
				  send_buffer[0] = 65;
				  send_buffer[1] = (uint8_t) (pos_x >> 24);
				  send_buffer[2] = (uint8_t) (pos_x >> 16);
				  send_buffer[3] = (uint8_t) (pos_x >> 8);
				  send_buffer[4] = (uint8_t) (pos_x & 0xff);
				  send_buffer[5] = (uint8_t) (pos_y >> 24);
				  send_buffer[6] = (uint8_t) (pos_y >> 16);
				  send_buffer[7] = (uint8_t) (pos_y >> 8);
				  send_buffer[8] = (uint8_t) (pos_y & 0xff);
				  send_buffer[9] = (uint8_t) (pos_z >> 24);
				  send_buffer[10] = (uint8_t) (pos_z >> 16);
				  send_buffer[11] = (uint8_t) (pos_z >> 8);
				  send_buffer[12] = (uint8_t) (pos_z & 0xff);
				  send_buffer[13] = (uint8_t) (vel_x >> 24);
				  send_buffer[14] = (uint8_t) (vel_x >> 16);
				  send_buffer[15] = (uint8_t) (vel_x >> 8);
				  send_buffer[16] = (uint8_t) (vel_x & 0xff);
				  send_buffer[17] = (uint8_t) (vel_y >> 24);
				  send_buffer[18] = (uint8_t) (vel_y >> 16);
				  send_buffer[19] = (uint8_t) (vel_y >> 8);
				  send_buffer[20] = (uint8_t) (vel_y & 0xff);
				  send_buffer[21] = (uint8_t) (vel_z >> 24);
				  send_buffer[22] = (uint8_t) (vel_z >> 16);
				  send_buffer[23] = (uint8_t) (vel_z >> 8);
				  send_buffer[24] = (uint8_t) (vel_z & 0xff);
				  send_buffer[25] = (uint8_t) (q_w >> 24);
				  send_buffer[26] = (uint8_t) (q_w >> 16);
				  send_buffer[27] = (uint8_t) (q_w >> 8);
				  send_buffer[28] = (uint8_t) (q_w & 0xff);
				  send_buffer[29] = (uint8_t) (q_x >> 24);
				  send_buffer[30] = (uint8_t) (q_x >> 16);
				  send_buffer[31] = (uint8_t) (q_x >> 8);
				  send_buffer[32] = (uint8_t) (q_x & 0xff);
				  send_buffer[33] = (uint8_t) (q_y >> 24);
				  send_buffer[34] = (uint8_t) (q_y >> 16);
				  send_buffer[35] = (uint8_t) (q_y >> 8);
				  send_buffer[36] = (uint8_t) (q_y & 0xff);
				  send_buffer[37] = (uint8_t) (q_z >> 24);
				  send_buffer[38] = (uint8_t) (q_z >> 16);
				  send_buffer[39] = (uint8_t) (q_z >> 8);
				  send_buffer[40] = (uint8_t) (q_z & 0xff);

				  HAL_UART_Transmit(&huart3, (uint8_t*)send_buffer, sizeof(send_buffer), 100);
			  }
		  }

	  }
	  if(!uart_rx_flag){//receiving commands over UART
		  uart_rx_flag = 1;
		  rx_data_uart = 0;
		  HAL_UART_Receive(&huart3, &rx_data_uart, 1,10);
		  uart_rx_data_parse_flag = 1;
	  }
	  if (uart_rx_data_parse_flag){
		  if (rx_data_uart == 17){ //calibration activated
			  calibration_on_startup_flag = 1;
		  }
		  else if(rx_data_uart == 5){ //start command
			  run_flag = 1;
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 0);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
		  }
		  else if(rx_data_uart == 21){// start state estimation command
			  if (calibration_on_startup_flag){
				  state_estimation_calibration_flag = 1;
			  }
			  else {
				  state_estimation_flag = 1;
			  }
			  state_estimation_first_loop_flag = 1;
		  }
		  else if(rx_data_uart == 13){//stop command
			  run_flag = 0;
			  state_estimation_flag = 0;
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 0);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
		  }
		  uart_rx_data_parse_flag = 0;
		  uart_rx_flag = 0;
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//uart_rx_flag = 0; //reset uart receive ongoing flag
	uart_rx_data_parse_flag = 1; //enable message parsing
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
