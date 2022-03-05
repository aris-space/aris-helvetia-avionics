#include "../Inc/environment.h"

// Convert Temprature from Celcius (Output of sensors) to Kelvin
float convert_temperature(float temp_in_C){
    return temp_in_C+T_0;
}

// Set the reference temperature and pressure before flight-start in IDLE state
void set_reference_values(state_est_meas_t* state_est_meas,env_t* env){
    env->p_g = state_est_meas->pressure;
    env->T_g = TEMP_IMU_WEIGHT*state_est_meas->temperature_imu+(1-TEMP_IMU_WEIGHT)*state_est_meas->temperature_baro; // Create weighted average of the both temperature measurements
}

// Calculate the altitude from the pressure measurements
float alitiude_from_pressure(env_t *env, float pressure){
	return env->T_g / T_GRAD * (powf(pressure / env->p_g, - R_star * T_GRAD / (GRAVITATION*M))-1); // TO DO Check formula previously different!
}


// Calculates mach number
float mach_number(state_est_data_t* state_est_data){
    float T = TEMP_IMU_WEIGHT*state_est_data->temperature[0][0]+(1-TEMP_IMU_WEIGHT)*state_est_data->temperature[1][0];
    float a = sqrt(GAMMA*R_star*T);
    return fabsf(state_est_data->vel[3][0])/a;
}
