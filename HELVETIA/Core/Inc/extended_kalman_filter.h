#ifndef EXTENDEND_KALMAN_FILTER_H
#define EXTENDEND_KALMAN_FILTER_H

#include "state_estimation_constructor.h"
#include "environment.h"
#include "flight_phase_detection.h"
#include "Util/math_utils.h"
#include <stdbool.h>

// Resets state estimation
void reset_state(ekf_state_t* ekf_state); 

// Updates A state space matrix
void update_A(float dt, ekf_state_t* ekf_state);

// Updates B state space matrix
void update_B(float dt, ekf_state_t* ekf_state);

// Updates state estimation data used for flight phase detection and output
void update_state_est_data(state_est_meas_t* state_est_meas,ekf_state_t* ekf_state,state_est_data_t* state_est_data);

void select_noise_models(float dt, ekf_state_t* ekf_state,state_est_meas_t* state_est_meas, flight_phase_detection_t* flight_phase_detection);

// Update H state matrix
void update_H(ekf_state_t* ekf_state, state_est_meas_t* state_est_meas);

void prior_update(ekf_state_t* ekf_state, state_est_meas_t* z, state_est_meas_t* z_prev, flight_phase_detection_t* flight_phase_detection);

void posterior_update(ekf_state_t* ekf_state, state_est_meas_t* state_est_meas,state_est_data_t* state_est_data, env_t* env);

void ekf_estimation_cycle(ekf_state_t* ekf_state, state_est_meas_t* z, state_est_meas_t* z_prev, flight_phase_detection_t* flight_phase_detection, state_est_data_t* state_est_data,env_t* env);

#endif
