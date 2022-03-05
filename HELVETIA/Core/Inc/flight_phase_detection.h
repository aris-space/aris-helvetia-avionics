#ifndef FLIGHT_PHASE_DETECTION_H_
#define FLIGHT_PHASE_DETECTION_H_

#include "state_estimation_constructor.h"
#include "environment.h"
#include <stdbool.h>

void detect_flight_phase(flight_phase_detection_t *flight_phase_detection, state_est_data_t *state_est_data);

void reset_flight_phase_detection(flight_phase_detection_t *flight_phase_detection);

#endif
