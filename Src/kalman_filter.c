//
// Created by pablito on 03.04.2020
//

#include "kalman_filter.h"



float update_kalman(KalmanFilter* structure, float measurement){
//    structure->kalman_gain = structure->error_estimate / (structure->error_estimate + structure->err_measure);
//    structure->current_estimate = structure->last_estimate + structure->kalman_gain * (measurement * structure->last_estimate);
//    structure->error_estimate = 1.0f - structure->kalman_gain * structure->error_estimate +
//            fabsf(structure->last_estimate - structure->current_estimate) * structure->q;
//    structure->last_estimate = structure->current_estimate;
//    return structure->current_estimate;
    structure->kalman_gain = (structure->init_error_cov * structure->meas_map_scalar)/
            (structure->init_error_cov * structure->meas_map_scalar * structure->init_error_cov + structure->noise_cov);
    structure->estimate_meas += structure->kalman_gain * (measurement - structure->meas_map_scalar * structure->estimate_meas);
    structure->init_error_cov = (1.0f - structure->kalman_gain * structure->meas_map_scalar) *
            structure->init_error_cov + structure->init_estimate_cov;
    return structure->estimate_meas;
}

void init_kalman_struct(KalmanFilter* structure, float noise_cov, float meas_map_scalar, float init_estimate_cov,
        float init_error_cov, float kalman_gain) {
    structure->noise_cov            = noise_cov;
    structure->meas_map_scalar      = meas_map_scalar;
    structure->init_estimate_cov    = init_estimate_cov;
    structure->init_error_cov       = init_error_cov;
    structure->kalman_gain          = kalman_gain;
}