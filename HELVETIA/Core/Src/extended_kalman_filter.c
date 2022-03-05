#include "extended_kalman_filter.h"

// Resets state estimation
void reset_state(ekf_state_t* ekf_state){
    // reset state matrices to zero
    float A_init[NUM_STATES][NUM_STATES] = {0};
    float B_init[NUM_STATES][NUM_INPUTS] = {0};
    float H_init[NUM_STATES][NUM_INPUTS] = {0};
    memcpy(ekf_state->A, A_init, sizeof(ekf_state->A));
    memcpy(ekf_state->B, B_init, sizeof(ekf_state->B));
    memcpy(ekf_state->H, H_init, sizeof(ekf_state->H));

    // reset noise convariance matrices
    float Q_init[NUM_STATES][NUM_STATES] = {0};
    float R_init[NUM_INPUTS][NUM_INPUTS] = {0};
    memcpy(ekf_state->Q, Q_init, sizeof(ekf_state->Q));
    memcpy(ekf_state->R, R_init, sizeof(ekf_state->R));
    for(int i=0;i<3;i++) {
        ekf_state->Q[i][i] = 0.1;
        ekf_state->Q[i+3][i+3] = 0.1;
        ekf_state->R[i][i] = 1.0E-7;
    }
    ekf_state->R[3][3] = 0.01; //R value added by barometer

    // reset state and convariance matrix
    float X_init[NUM_STATES][1] = {0};
    float P_init[NUM_STATES][NUM_STATES] = {0};
    for(int i=0;i<3;i++) {
        X_init[i][0] = 0.0;
        X_init[i+3][0] = 0.0;
        P_init[i][i] = 1.0E-4;
        P_init[i+3][i+3] = 1.0E-4;
    }
    X_init[6][0] = 1;
    memcpy(ekf_state->X_prior, X_init, sizeof(ekf_state->X_prior));
    memcpy(ekf_state->P_prior, P_init, sizeof(ekf_state->P_prior));
    memcpy(ekf_state->X_e, X_init, sizeof(ekf_state->X_e));
    memcpy(ekf_state->P_e, P_init, sizeof(ekf_state->P_e));
}

// Updates A state space matrix
void update_A(float dt, ekf_state_t* ekf_state){
    eye(NUM_STATES,ekf_state->A);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            ekf_state->A[i][j+3] = dt*ekf_state->R_IB[i][j];
        }
    }
}

// Updates B state space matrix
void update_B(float dt, ekf_state_t* ekf_state){
    float B[NUM_STATES][NUM_INPUTS] = {0};
    for (int i=0; i<4; i++){
        for (int j = 0; j < 3; j++){
            // attitude part
            B[i+6][j+3] = dt*ekf_state->Xi[i][j];
        }
        if (i<3){
            // position and velocity section
            B[i+3][i] = dt;
        }
    }
    
    memcpy(ekf_state->B,B,sizeof(ekf_state->B));
}

// Updates state estimation data used for flight phase detection and output
void update_state_est_data(state_est_meas_t* state_est_meas,ekf_state_t* ekf_state,state_est_data_t* state_est_data){
    memcpy(state_est_data->acc,state_est_meas->acc,sizeof(state_est_data->acc));
    memcpy(state_est_data->gyro,state_est_meas->gyro,sizeof(state_est_data->gyro));
    if (state_est_meas->mag!=NULL){
        memcpy(state_est_data->mag,state_est_meas->mag,sizeof(state_est_data->mag));
    }
    else{
        memset(state_est_data->mag,0,sizeof(state_est_data));
    }
    memcpy(state_est_data->euler,state_est_meas->euler,sizeof(state_est_data->euler));
    memcpy(state_est_data->quat,state_est_meas->quat,sizeof(state_est_data->quat));
    float temp[2][1] = {state_est_meas->temperature_imu,state_est_meas->temperature_baro};
    memcpy(state_est_data->temperature,temp,sizeof(state_est_data->temperature));
    state_est_data->timestamp = state_est_meas->timestamp;
    float pos[3][1] = {ekf_state->X_e[0][0],ekf_state->X_e[1][0],ekf_state->X_e[2][0]};
    float vel[3][1] = {ekf_state->X_e[3][0],ekf_state->X_e[4][0],ekf_state->X_e[5][0]};
    
    memcpy(state_est_data->pos,pos,sizeof(state_est_data->pos));
    memcpy(state_est_data->vel,vel,sizeof(state_est_data->vel));

    state_est_data->mach_number = mach_number(state_est_data); // Update Mach number
}

void select_noise_models(float dt, ekf_state_t* ekf_state,state_est_meas_t* state_est_meas, flight_phase_detection_t* flight_phase_detection){
    /*select the correct Q & R matrices for our flight phase*/
    float gyro_stdev_norm = 0.01; // noise stdev which scales proportionally with the norm of the angular velocity
    /* we set a minimal angular velocity norm of 0.1 rad/s */
    float omega_norm = max(euclidean_norm(3, state_est_meas->gyro), 0.1);
    for (int i = 0; i<3; i++) {
        ekf_state->std_dev.sigma_gyro[i][0] = omega_norm * gyro_stdev_norm;
    }
    /*define magnetometer noise levels*/
    float sigma_mag[3][1] = {0.05,0.05,0.05};
    memcpy(ekf_state->std_dev.sigma_mag,sigma_mag,sizeof(ekf_state->std_dev.sigma_mag));

    // State machine to calculate sigma_acc and sigma_baro
    // No idea where this values are comming from
    switch (flight_phase_detection->flight_phase) {
        case IDLE:
            ekf_state->std_dev.sigma_acc[0][0] = 0.080442;
            ekf_state->std_dev.sigma_acc[1][0] = 0.080442;
            ekf_state->std_dev.sigma_acc[2][0] = 0.080442;
            ekf_state->std_dev.sigma_baro = 1.869;
        break;
        case THRUSTING:
            ekf_state->std_dev.sigma_acc[0][0] = 4.5126;
            ekf_state->std_dev.sigma_acc[1][0] = 4.5126;
            ekf_state->std_dev.sigma_acc[2][0] = 1.376343;
            ekf_state->std_dev.sigma_baro = 13.000;
        break;
        case COASTING:  //Removed supersonic and transsonic values
            ekf_state->std_dev.sigma_acc[0][0] = 0.805401;
            ekf_state->std_dev.sigma_acc[1][0] = 0.805401;
            ekf_state->std_dev.sigma_acc[2][0] = 0.1;
            ekf_state->std_dev.sigma_baro = 7.380;
        break;
        default:
        break;
    }

    for(int i=0;i<3;i++){
        ekf_state->Q[i][i] = dt*ekf_state->Q[i+3][i+3]; // position covariance (we integrate the previous velocity covariance)
        ekf_state->Q[i+3][i+3] = dt*powf(ekf_state->std_dev.sigma_acc[i][0],2); // velocity covariance
        ekf_state->R[i][i] = dt*powf(ekf_state->std_dev.sigma_mag[i][0],2); // magnetometer covariance
    }
    ekf_state->R[3][3] = dt*powf(ekf_state->std_dev.sigma_baro,2);

    /* update the quternion part of Q*/
    // Q_quat = Dt/2^2 * Xi * sigma^2 * Xi^T

    // TO DO: needs to be checked!!
    float factor = powf(dt/2, 2)*powf(ekf_state->std_dev.sigma_gyro[0][0], 2);
    transpose(4 , 3, ekf_state->Xi, ekf_state->Xi_T);
    matmul(4, 3, 4, ekf_state->Xi, ekf_state->Xi_T, ekf_state->Q_quat, 1);
    scalarmatprod(4, 4, factor, ekf_state->Q_quat, ekf_state->Q_quat);
    // overwrite quaternion part of Q with Q_quat
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            ekf_state->Q[i+6][j+6] = ekf_state->Q_quat[i][j];
        }
    }


}

// Update H state matrix
void update_H(ekf_state_t* ekf_state, state_est_meas_t* state_est_meas){
    float q_prior[4] = {ekf_state->X_prior[6][0],ekf_state->X_prior[7][0],ekf_state->X_prior[8][0],ekf_state->X_prior[9][0]};
    // Update prior matrices
    rot_from_quat(q_prior,ekf_state->R_IB_prior);
    Xi_from_quat(q_prior,ekf_state->Xi_prior);
    // placeholder variables
    float mag_ref[3][1] = {MAG_REF_VECTOR_X,MAG_REF_VECTOR_Y,MAG_REF_VECTOR_Z};
    float mag_ref_rotated[3][1];
    float cross1[3][1];
    float cross2[3][1];
    float cross3[3][1];

    const float cartesian_i[3][1] = {1, 0, 0};
    const float cartesian_j[3][1] = {0, 1, 0};
    const float cartesian_k[3][1] = {0, 0, 1};

    // compute H_quat
    transpose(3, 3, ekf_state->R_IB_prior, ekf_state->R_BI);
    matvecprod(3, 3, ekf_state->R_BI, mag_ref, mag_ref_rotated, true);

    veccrossprod(cartesian_i, mag_ref_rotated, cross1);
    veccrossprod(cartesian_j, mag_ref_rotated, cross2);
    veccrossprod(cartesian_k, mag_ref_rotated, cross3);

    float H_q[3][4]= {0};
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 4; j++){
            H_q[0][j] += 2*ekf_state->Xi[i][j] * cross1[j][0];
            H_q[1][j] += 2*ekf_state->Xi[i][j] * cross2[j][0];
            H_q[2][j] += 2*ekf_state->Xi[i][j] * cross3[j][0];
        }
    }

    memcpy(ekf_state->H_quat,H_q,sizeof(ekf_state->H_quat));
    float H[NUM_MEASUREMENTS][NUM_STATES] = {0};
    for (int i=0;i<3;i++){
        for (int j=0;j<4;j++){
            H[i][j+6] = H_q[i][j];
        }
    }
    H[3][2] = 1; // H part depending on barometer
    memcpy(ekf_state->H,H,sizeof(ekf_state->H));
}

void prior_update(ekf_state_t* ekf_state, state_est_meas_t* z, state_est_meas_t* z_prev, flight_phase_detection_t* flight_phase_detection){
    float dt = z->timestamp - z_prev->timestamp;
    // compute rotation and quaternion matrix
    float q_e[4] = {ekf_state->X_e[6][0],ekf_state->X_e[7][0],ekf_state->X_e[8][0],ekf_state->X_e[9][0]};
    rot_from_quat(q_e,ekf_state->R_IB);
    Xi_from_quat(q_e,ekf_state->Xi);
    // compute the linearized system matrices
    update_A(dt,ekf_state);
    update_B(dt,ekf_state);
    // compute the covariance matrix
    select_noise_models(dt,ekf_state, z, flight_phase_detection);

    //construct the input vector where the input is the previous IMU measurement
    float u[NUM_STATES][1];
    for(int i=0;i<3;i++){
        u[i][0] = z_prev->acc[i][0];
        u[i+3][0] = z_prev->gyro[i][0];
    }
    // ====================
    // perform prior update
    // ====================
    // X_{k+1} = A * X_{k} + B*u{k}
    float prod1[NUM_STATES][1];
    float prod2[NUM_STATES][1];

    matvecprod(NUM_STATES, NUM_STATES, ekf_state->A, ekf_state->X_e, prod1, true);
    matvecprod(NUM_STATES, NUM_INPUTS, ekf_state->B, u, prod2, true);
    vecadd(NUM_STATES, prod1, prod2, ekf_state->X_prior);

    // P_{k+1} = A * P_{k} * A_{k}^T + Q_{k}
    // to check whether Q_{k} has to be transposed
    float P_propagated1[NUM_STATES][NUM_STATES];
    float P_propagated2[NUM_STATES][NUM_STATES];
    float A_transposed[NUM_STATES][NUM_STATES];

    transpose(NUM_STATES, NUM_STATES, *(ekf_state->A), *A_transposed);
    matmul(NUM_STATES, NUM_STATES, NUM_STATES, *(ekf_state->A), *(ekf_state->P_e), *P_propagated1, true); 
    matmul(NUM_STATES, NUM_STATES, NUM_STATES, *P_propagated1, *A_transposed, *(ekf_state->P_prior), true);
    matadd(NUM_STATES, NUM_STATES, *(ekf_state->P_e), *(ekf_state->Q), *(ekf_state->P_e));

}

void posterior_update(ekf_state_t* ekf_state, state_est_meas_t* state_est_meas,state_est_data_t* state_est_data, env_t* env){
    if((state_est_meas->mag==NULL) || (state_est_meas->pressure == 0)){
    // we copy the prior to the posterior as magnotometer is five times slower then other sensors
    memcpy(ekf_state->X_e, ekf_state->X_prior, sizeof(ekf_state->X_e));
    memcpy(ekf_state->P_e, ekf_state->P_prior, sizeof(ekf_state->P_e));
    } 
    // TO DO: ADD CASESES WHERE ONLY ONE SENSOR HAS NO DATA
    /*else if (state_est_meas->mag==NULL){

    }
    else if (state_est_meas->pressure == 0){

    }*/
    else{
        update_H(ekf_state, state_est_meas);

        // compute the K matrix
        float mat1[NUM_MEASUREMENTS][NUM_STATES];
        float mat2[NUM_MEASUREMENTS][NUM_MEASUREMENTS];
        float mat3[NUM_MEASUREMENTS][NUM_MEASUREMENTS];
        float mat4[NUM_STATES][NUM_MEASUREMENTS];
        float H_transposed[NUM_STATES][NUM_MEASUREMENTS];
        float inv[NUM_MEASUREMENTS][NUM_MEASUREMENTS];

        transpose(NUM_MEASUREMENTS, NUM_STATES, ekf_state->H, H_transposed);
        matmul(NUM_MEASUREMENTS, NUM_STATES, NUM_STATES, ekf_state->H, ekf_state->P_prior, mat1, true);
        matmul(NUM_MEASUREMENTS, NUM_STATES, NUM_MEASUREMENTS, mat1, H_transposed, mat2, true);
        matadd(NUM_MEASUREMENTS, NUM_MEASUREMENTS, mat2, ekf_state->R, mat3); 

        float lambda = 0.; //damping factor
        bool invertible = inverse(NUM_MEASUREMENTS, mat3, inv, lambda);

        while (~invertible){ // check that matrix is definitly inverted
            invertible = pseudo_inverse(NUM_MEASUREMENTS,NUM_MEASUREMENTS,mat3,inv,lambda);
            lambda += 0.1;
        }

        matmul(NUM_STATES, NUM_MEASUREMENTS, NUM_MEASUREMENTS, H_transposed, inv, mat4, true);
        matmul(NUM_STATES, NUM_STATES, NUM_MEASUREMENTS, ekf_state->P_prior, mat4, ekf_state->K, true);
        // K matrix computed

        // Now compute the posterior state
        float z_expected[NUM_MEASUREMENTS][1];
        float innovation[NUM_MEASUREMENTS][1];
        float dX[NUM_STATES][1];
        float h = alitiude_from_pressure(env, state_est_meas->pressure);
        float z[NUM_MEASUREMENTS][1] = {state_est_meas->mag[0][0],state_est_meas->mag[1][0],state_est_meas->mag[2][0],h};
        matvecprod(NUM_MEASUREMENTS, NUM_STATES, ekf_state->H, ekf_state->X_prior, z_expected, true);
        vecsub(NUM_MEASUREMENTS,z, z_expected, innovation);
        matvecprod(NUM_STATES, NUM_MEASUREMENTS, ekf_state->K, innovation, dX, true);
        vecadd(NUM_STATES, ekf_state->X_prior, dX, ekf_state->X_e);

        // compute posterior covariance
        float mat5[NUM_STATES][NUM_STATES];
        float mat6[NUM_STATES][NUM_STATES];
        float mat7[NUM_STATES][NUM_STATES];

        eye(NUM_STATES, mat5);
        matmul(NUM_STATES, NUM_MEASUREMENTS, NUM_STATES, ekf_state->K, ekf_state->H, mat6, true);
        matsub(NUM_STATES, NUM_STATES, mat5, mat6, mat7);
        matmul(NUM_STATES, NUM_STATES, NUM_STATES, mat7, ekf_state->P_prior, ekf_state->P_e, true);
    }
    update_state_est_data(state_est_meas,ekf_state,state_est_data);
}

void ekf_estimation_cycle(ekf_state_t* ekf_state, state_est_meas_t* z, state_est_meas_t* z_prev, flight_phase_detection_t* flight_phase_detection, state_est_data_t* state_est_data,env_t* env){
  /*perform on estimation cycle with prior update and posterior update*/
  prior_update(ekf_state, z, z_prev, flight_phase_detection);
  posterior_update(ekf_state, z, state_est_data,env);
  // normalize the quaternion
  float mu = sqrt(powf(ekf_state->X_e[6][0],2) + powf(ekf_state->X_e[7][0],2) + powf(ekf_state->X_e[8][0],2) + powf(ekf_state->X_e[9][0],2));
  for(int i=0;i<4;i++){
    ekf_state->X_e[6+i][0] = ekf_state->X_e[6+i][0] / mu;
  }
}
