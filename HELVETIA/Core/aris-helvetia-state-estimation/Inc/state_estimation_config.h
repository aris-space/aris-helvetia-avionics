#include <stdbool.h>

#ifndef STATE_ESTIMATION_CONFIG_H_
#define STATE_ESTIMATION_CONFIG_H_

#define NUM_STATES 10
#define NUM_INPUTS 6
#define STATE_ESTIMATION_FREQUENCY 100
#define NUM_IMU 1
#define NUM_BARO 1
#define NUM_MEASUREMENTS 4

#define LAUNCH_RAIL_ANGLE 90.0f // launch rail angle in degrees
#define X_COG 2.262f // longitudinal distance from tip nose-cone to the CoG of the rocket [m]
#define X_IMU 2.0476680f // longitudinal distance from the tip of the nose-cone to the IMU sensors [m]
#define X_BARO 2.0476680f // longitudinal distance from the tip of the nose-cone to the barometric sensors [m]
#define R_IMU 0.023f // radial offset of the IMU sensors from the center axis of the rocket [m]

/* flight phase detection config */
#define FPD_SAFETY_COUNTER_THRESH 8 // how many (non-consecutive) measurements are required to detect an event and switch the flight phase
#define FPD_LIFTOFF_ACC_THRESH 20 // acceleration threshold to detect lift-off and enter thrusting flight phase [m/s^2]
#define FPD_LIFTOFF_ALT_ACTIVE true // wheter liftoff can also be determined by altitude
#define FPD_LIFTOFF_ALT_THRESH 150 // altitude above ground level to detect lift-off and enter thrusting flight phase [m]
#define FPD_BIAS_RESET_ACTIVATION_MACH_NUMBER 0 // the bias reset window is activated below this mach number[mach]
#define FPD_BIAS_RESET_TIME 0 // duration of the bias reset window. Set it to 0s to not use the bias reset window [s]
#define FPD_MAIN_DESCENT_ALT_THRESH 400 // altitude above ground level to enter main descent and activate the main parachute in [m]
#define FPD_BALLISTIC_ACTIVE true // wether to use ballistic flight phase
#define FPD_BALLISTIC_VEL_THRESH_HIGH 75 // upper velocity threshold to enter ballistic flight phase in [m/s]
#define FPD_BALLISTIC_VEL_THRESH_LOW 60 // lower velocity threshold to leave ballistic flight phase in [m/s]
#define FPD_DROGUE_LIN_ACC_THRESH 20 // linear accel needed to enter drogue descent in [1/s^2]
#define FPD_DROGUE_GYR_THRESH 1.2 // angular velocity theshold to detect drogue descent?? (Marvin's Suggestion)
#define FPD_TOUCHDOWN_SAFETY_COUNTER_THRESH 20 // how many (non-consecutive) measurements are required to detect touchdown
#define FPD_TOUCHDOWN_VEL_THRESH 2 // velocity threshold to assume touchdown (together with altitude threshold) in [m/s]
#define FPD_TOUCHDOWN_GYR_THRESH 0.8 // angular velocity threshold to assume touchdown in [rad/s]
#define FPD_TOUCHDOWN_VEL_ACTIVE true // wheter touchdown can also be determined by the velocity

#define MAG_REF_VECTOR_X 1
#define MAG_REF_VECTOR_Y 2
#define MAG_REF_VECTOR_Z 3

#define USE_BARO_IN_CONTROL_PHASE true // wether to use barometer measurements during control phase or exclude them because of dynamic pressure
#define BIAS_RESET_AIRBRAKE_EXTENSION_THRESH 0.05 // threshold of airbrake extensino under which we use baro to reset bias

/* use moving average to determine velocity instead of state estimation during main and drogue descent */
#define USE_STATE_EST_DESCENT false // wether to use the state estimation during drogue and main descent
#define MAX_LENGTH_MOVING_AVERAGE 10

/* sensor elimination by extrapolation config */
#define USE_SENSOR_ELIMINATION_BY_EXTRAPOLATION false // set to true to activate sensor elimination by extrapolation for barometer and temperature [m]
#define MAX_LENGTH_ROLLING_MEMORY 18
#define EXTRAPOLATION_POLYFIT_DEGREE 2

#endif