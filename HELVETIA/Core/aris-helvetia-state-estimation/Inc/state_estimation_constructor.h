#ifndef STATE_ESTIMATION_CONSTRUCTOR_H
#define STATE_ESTIMATION_CONSTRUCTOR_H

#include "state_estimation_config.h"
#include <stdbool.h>
#include "stdint.h"
// structure with flight phase definitions
typedef enum {
	IDLE = 1, THRUSTING, COASTING, DROGUE_DESCENT, BALLISTIC_DESCENT, TOUCHDOWN // default value IDLE
} flight_phase_e;

// Mach regime
typedef enum {
	SUBSONIC = 1, TRANSONIC, SUPERSONIC // default value SUBSONIC
} mach_regime_e;


typedef struct {
    /* pressure in [Pa] and temperature in [°C] */
	
	float pressure;
	float temperature_baro;
    /* acceleration in [m/s^2] and angular velocity in [rad/s] */
    /* all in rocket frame where x-dir is along length of rocket */
	float acc[3][1];
	float gyro[3][1];
	float mag[3][1];

	// Additional data from BNO055
	float euler[3][1];
	float quat[4][1];
	float temperature_imu;

	float timestamp;

} state_est_meas_t;


typedef struct {
	float acc[3][1];
	float gyro[3][1];
	float mag[3][1];
	float pos[3][1];
	float vel[3][1];

	// Additional data from BNO055
	float euler[3][1];
	float quat[4][1];
	float temperature[2][1];

	float mach_number;
	float timestamp;
} state_est_data_t;


// Structure to detect flight phase
typedef struct {
	flight_phase_e flight_phase;
	mach_regime_e mach_regime;
	float mach_number;
	int8_t safety_counter[3];
	int8_t t_bias_reset_start;
} flight_phase_detection_t;

typedef struct {
  //a struct to save the sensor standard deviations
    float sigma_acc[3][1];
    float sigma_gyro[3][1];
    float sigma_mag[3][1];
	float sigma_baro;
} std_dev_t;

// Structure for extended kalman filter
typedef struct{
	// A priori state vector and covariance matrix
	float X_prior[NUM_STATES][1];
	float P_prior[NUM_STATES][NUM_STATES];
	// Posterior state vector and covariance matrix
	float X_e[NUM_STATES][1];
	float P_e[NUM_STATES][NUM_STATES];

	// Kalman Gain matrix
	float K[NUM_STATES][3];

	// State space matrixes of system
	float A[NUM_STATES][NUM_STATES];
	float B[NUM_STATES][NUM_INPUTS];
	float H[NUM_MEASUREMENTS][NUM_STATES];

	// noise covariance matices
	float Q[NUM_STATES][NUM_STATES];
	float R[NUM_MEASUREMENTS][NUM_MEASUREMENTS];

	// helper matrices
	float R_IB[3][3];  // rotation matrix from body to world
	float Xi[4][3]; //useful for quaternion operations 
	float R_IB_prior[3][3]; //rotation
	float Xi_prior[4][3]; // useful for quaternion operations

	//placeholder matrices for increased speed
	float Q_quat[4][4]; //quaternion part of the covariance matrix
	float H_quat[3][4]; // observation matrix part of the quaternion
	float Xi_T[3][4]; //Xi transposed
	float R_BI[3][3]; // rotation matrix from body to world
	
	// sensor standard evs
	std_dev_t std_dev;

	// Flight phase state
	flight_phase_detection_t flight_phase_detection;

} ekf_state_t;

#endif
