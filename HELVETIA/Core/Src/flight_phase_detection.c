#include "flight_phase_detection.h"

// State Machine to detect the current state of flight. All parameters can be configured in config file.
void detect_flight_phase(flight_phase_detection_t* flight_phase_detection, state_est_data_t* state_est_data)
{
    switch(flight_phase_detection->flight_phase)
    {
        case IDLE: // Idle state before and after launch
            if (fabsf((float)(state_est_data->acc[2][0])) > FPD_LIFTOFF_ACC_THRESH) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= FPD_SAFETY_COUNTER_THRESH) {
                    flight_phase_detection->flight_phase = THRUSTING;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                    flight_phase_detection->safety_counter[2] = 0;
                }
            }
            #if FPD_LIFTOFF_ALT_ACTIVE == true
                else if (((float)(state_est_data->pos[2][0])) > FPD_LIFTOFF_ALT_THRESH) {
                    flight_phase_detection->safety_counter[1] += 1;
                    if (flight_phase_detection->safety_counter[1] >= FPD_SAFETY_COUNTER_THRESH) {
                        flight_phase_detection->flight_phase = THRUSTING;
                        flight_phase_detection->safety_counter[0] = 0;
                        flight_phase_detection->safety_counter[1] = 0;
                        flight_phase_detection->safety_counter[2] = 0;
                    }
            #endif
            }
        break;

        case THRUSTING: // State during thrust phase
            if (((float)(state_est_data->acc[2][0])) < 0) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= FPD_SAFETY_COUNTER_THRESH) {
                    flight_phase_detection->flight_phase = COASTING;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                    flight_phase_detection->safety_counter[2] = 0;
                }
            }
        break;

        case COASTING: // State during coasting
            if (euledian_norm(3,state_est_data->acc) < FPD_DROGUE_LIN_ACC_THRESH) { // If no active control is enabled on the flight
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= FPD_SAFETY_COUNTER_THRESH) {
                    flight_phase_detection->flight_phase = DROGUE_DESCENT;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                    flight_phase_detection->safety_counter[2] = 0;
                }
            }
            else if (euclidean_norm(3,state_est_data->gyro) > FPD_DROGUE_GYR_THRESH) { //Heavy rotation due to separation
            flight_phase_detection->safety_counter[1] += 1;
            if (flight_phase_detection->safety_counter[1] >= FPD_SAFETY_COUNTER_THRESH) {
              flight_phase_detection->flight_phase = DROGUE_DESCENT;
              flight_phase_detection->safety_counter[0] = 0;
              flight_phase_detection->safety_counter[1] = 0;
              flight_phase_detection->safety_counter[2] = 0;
            }
        }
        break;

        case DROGUE_DESCENT:
            if (euclidean_norm(3, state_est_data->gyro) < FPD_TOUCHDOWN_GYR_THRESH) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= FPD_TOUCHDOWN_SAFETY_COUNTER_THRESH) {
                    flight_phase_detection->flight_phase = TOUCHDOWN;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                    flight_phase_detection->safety_counter[2] = 0;
                }
            }
            #if FPD_TOUCHDOWN_VEL_ACTIVE == true
                else if (fabsf(((float)(state_est_data->vel[2][0]))) < FPD_TOUCHDOWN_VEL_THRESH){
                   flight_phase_detection->safety_counter[1] += 1;
                if (flight_phase_detection->safety_counter[1] >= FPD_TOUCHDOWN_SAFETY_COUNTER_THRESH) {
                    flight_phase_detection->flight_phase = TOUCHDOWN;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                    flight_phase_detection->safety_counter[2] = 0;
                } 
                }
            #endif
            #if FPD_BALLISTIC_ACTIVE == true 
            /* we assume a ballistic descent when the absolute velocity of the rocket in vertical direction is larger than 75 m/s */
                else if (fabsf(((float)(state_est_data->vel[2][0]))) > FPD_BALLISTIC_VEL_THRESH_HIGH) {
                    flight_phase_detection->safety_counter[2] += 1;
                    if (flight_phase_detection->safety_counter[2] >= FPD_SAFETY_COUNTER_THRESH) {
                        flight_phase_detection->flight_phase = BALLISTIC_DESCENT;
                        flight_phase_detection->safety_counter[0] = 0;
                        flight_phase_detection->safety_counter[1] = 0;
                        flight_phase_detection->safety_counter[2] = 0;
                    }
                }
            #endif
        break;

        case BALLISTIC_DESCENT: // State if during decent the ballistic regime is reached
            /* we assume a touchdown event when the absolute value of the altitude is smaller than 400m 
               and the absolute velocity of the rocket is smaller than 2 m/s */
            if (fabs(((float)(state_est_data->vel[2][0]))) < FPD_TOUCHDOWN_VEL_THRESH 
                && (euclidean_norm(3, state_est_data->gyro)) < FPD_TOUCHDOWN_GYR_THRESH) {
                flight_phase_detection->safety_counter[0] += 1;
                if (flight_phase_detection->safety_counter[0] >= FPD_TOUCHDOWN_SAFETY_COUNTER_THRESH) {
                    flight_phase_detection->flight_phase = TOUCHDOWN;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                    flight_phase_detection->safety_counter[2] = 0;
                }
            }
            /* we assume a normal descent with parachute when the absolute velocity of the rocket in vertical direction is smaller than 40 m/s */
            else if (fabs(((float)(state_est_data->vel[2][0]))) < FPD_BALLISTIC_VEL_THRESH_LOW) {
                flight_phase_detection->safety_counter[1] += 1;
                if (flight_phase_detection->safety_counter[1] >= FPD_SAFETY_COUNTER_THRESH) {
                    flight_phase_detection->flight_phase = DROGUE_DESCENT;
                    flight_phase_detection->safety_counter[0] = 0;
                    flight_phase_detection->safety_counter[1] = 0;
                    flight_phase_detection->safety_counter[2] = 0;
                }
            }
        break;

        default:
        break;
    }

    flight_phase_detection->mach_number = (float)(state_est_data->mach_number);

    // Determine the mach number and therefore the flight regime for triggers to different states
    if (flight_phase_detection->mach_number >= 1.3) {
        flight_phase_detection->mach_regime = SUPERSONIC;
    } else if (flight_phase_detection->mach_number >= 0.8)
    {
        flight_phase_detection->mach_regime = TRANSONIC;
    } else
    {
        flight_phase_detection->mach_regime = SUBSONIC;
    }
}


// reset the state of the state machine
void reset_flight_phase_detection(flight_phase_detection_t *flight_phase_detection){
    flight_phase_detection->flight_phase = IDLE;
    flight_phase_detection->mach_regime = SUBSONIC;
    flight_phase_detection->mach_number = 0.0;
    flight_phase_detection->safety_counter[0] = 0;
    flight_phase_detection->safety_counter[1] = 0;
    flight_phase_detection->safety_counter[2] = 0;

}
