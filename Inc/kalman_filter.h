//
// Created by pablito on 03.04.2020.
//

#ifndef FC_SOFT_KALMAN_FILTER_H
#define FC_SOFT_KALMAN_FILTER_H
#include "mpu6000.h"

typedef struct{
    float err_measure;
    float error_estimate;
    float q;
    float current_estimate;
    float last_estimate;
    float kalman_gain;
} KalmanFilter;

KalmanFilter altitude_estimate;


float update_kalman(KalmanFilter* structure, float measurement);
void init_kalman_struct(KalmanFilter* structure, float meas_e, float est_err, float q);
#endif //FC_SOFT_KALMAN_FILTER_H
