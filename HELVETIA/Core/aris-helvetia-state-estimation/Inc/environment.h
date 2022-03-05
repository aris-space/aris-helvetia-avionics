#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include "state_estimation_constructor.h"
#include <math.h>

#define GRAVITATION 9.81
#define R_star 8.3144598
#define M 0.0289644
#define T_GRAD 0.0065
#define T_0 273.15
#define GAMMA 1.4

#define TEMP_IMU_WEIGHT 0.5

typedef struct env_t{
    float p_g; // Pressure at ground level [Pa]
    float T_g; // Temperature at ground level [K]
} env_t;


float convert_temperature(float temp_in_C);
void set_reference_values(state_est_meas_t* state_est_meas,env_t* env);
float alitiude_from_pressure(env_t *env, float pressure);
float mach_number(state_est_data_t* state_est_data);

#endif
