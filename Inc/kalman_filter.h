//
// Created by pablito on 03.04.2020.
//
#ifndef FC_SOFT_KALMAN_FILTER_H
#define FC_SOFT_KALMAN_FILTER_H

typedef struct{
    float noise_cov;
    float meas_map_scalar;
    float init_estimate_cov;
    float init_error_cov;
    float estimate_meas;
    float kalman_gain;
} KalmanFilter;

KalmanFilter altitude_estimate;
KalmanFilter gyro_estimate[3];
KalmanFilter accel_estimate[3];

float update_kalman(KalmanFilter*, float measurement);
void init_kalman_struct(KalmanFilter* structure, float noise_cov, float meas_map_scalar, float init_estimate_cov,
                        float init_error_cov, float kalman_gain);
#endif //FC_SOFT_KALMAN_FILTER_H