//Header file for testing purposes
//TO DO: Remove for final product

#ifndef MAIN_STATE_EST_H_
#define MAIN_STATE_EST_H_



#include "bno055_stm32.h"
#include "string.h"
#include "sensor_config.h"
#include "environment.h"
#include "flight_phase_detection.h"
#include "state_estimation_config.h"
#include "state_estimation_constructor.h"
#include "extended_kalman_filter.h"

void run_state_est();

#endif