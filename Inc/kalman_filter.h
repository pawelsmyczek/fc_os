//
// Created by pablito on 03.04.2020.
//

#ifndef FC_SOFT_KALMAN_FILTER_H
#define FC_SOFT_KALMAN_FILTER_H
#include "mpu6000.h"


float calculate_gyro(float);
float calculate_accel(float);
float calculate_kalman(float, float);
float update_kalman(float, float);

#endif //FC_SOFT_KALMAN_FILTER_H
