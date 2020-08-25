//
// Created by pablito on 03.04.2020
//

#include "kalman_filter.h"
#include "math.h"



float update_kalman(KalmanFilter* structure, float measurement){
    structure->kalman_gain = structure->error_estimate / (structure->error_estimate + structure->err_measure);
    structure->current_estimate = structure->last_estimate + structure->kalman_gain * (measurement * structure->last_estimate);
    structure->error_estimate = (1.0f - structure->kalman_gain) * structure->error_estimate + fabsf(structure->last_estimate - structure->current_estimate) * structure->q;
    structure->last_estimate = structure->current_estimate;
    return structure->current_estimate;
}

void init_kalman_struct(KalmanFilter* structure, float meas_e, float est_err, float q) {
    structure->err_measure = meas_e;
    structure->error_estimate = est_err;
    structure->q = q;
}