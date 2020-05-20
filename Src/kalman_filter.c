//
// Created by pablito on 03.04.2020.
//

#include "kalman_filter.h"

float gyro_out = 0;
float accel_out = 0;
float kalman_out = 0;


float calculate_gyro(float gyro){
    gyro /= 256.0;

    if(gyro >= 0 && gyro < 128){
        gyro -= 1;
        return gyro;
    }
    if(gyro >= 127 && gyro < 256){
        gyro = 255 - gyro;
        return (-1)*gyro;
    }
    return 0;
}

float calculate_accel(float accel){
    accel /= 256.0;

    if(accel >= 200 && accel < 256){
        accel = 256 - accel;
        return accel * 3;
    }
    else if(accel > 0 && accel < 40){
        return (-3)*accel;
    }
    return 0;
}

float calculate_kalman(float accel, float gyro){
    gyro_out = gyro; //calculate_gyro((float)rawGyro[ROLL].value);
    accel_out = accel; //calculate_accel((float)rawAccel[ROLL].value);
    kalman_out = update_kalman(gyro_out, accel_out);
    return kalman_out;
}

float update_kalman(float gyro_rate, float accel_angle){
    // Inputs.
    float u = gyro_rate;
    float y = accel_angle;

    // Output.
    float x_00 = 0.0;
    float x_10 = 0.0;

    // Persistant states.
    static float P_00 = 0.001;
    static float P_01 = 0.003;
    static float P_10 = 0.003;
    static float P_11 = 0.003;

    // Constants.

    // These are the delta in seconds between samples.
    const float A_01 = -0.01;
    const float B_00 = 0.01;

    // Data read from 1000 samples of the accelerometer had a variance of 0.07701688.
    const float Sz = accelRTError[YAW];

    // Data read from 1000 samples of the gyroscope had a variance of 0.00025556.
    // XXX The values below were pulled from the autopilot site, but I'm not sure how to
    // XXX plug them into the process noise covariance matrix.  This needs to be
    // XXX further explored.
    const float Sw_00 = 0.001;
    const float Sw_01 = 0.003;
    const float Sw_10 = 0.003;
    const float Sw_11 = 0.003;

    // Temp.
    float s_00;
    float inn_00;
    float K_00;
    float K_10;
    float AP_00;
    float AP_01;
    float AP_10;
    float AP_11;
    float APAT_00;
    float APAT_01;
    float APAT_10;
    float APAT_11;
    float KCPAT_00;
    float KCPAT_01;
    float KCPAT_10;
    float KCPAT_11;

    // Update the state estimate by extrapolating current state estimate with input u.
    // x = A * x + B * u
    x_00 += (A_01 * x_10) + (B_00 * u);

    // Compute the innovation -- error between measured value and state.
    // inn = y - c * x
    inn_00 = y - x_00;

    // Compute the covariance of the innovation.
    // s = C * P * C' + Sz
    s_00 = P_00 + Sz;

    // Compute AP matrix for use below.
    // AP = A * P
    AP_00 = P_00 + A_01 * P_10;
    AP_01 = P_01 + A_01 * P_11;
    AP_10 = P_10;
    AP_11 = P_11;

    // Compute the kalman gain matrix.
    // K = A * P * C' * inv(s)
    K_00 = AP_00 / s_00;
    K_10 = AP_10 / s_00;

    // Update the state estimate.
    // x = x + K * inn
    x_00 += K_00 * inn_00;
    x_10 += K_10 * inn_00;

    // Compute the new covariance of the estimation error.
    // P = A * P * A' - K * C * P * A' + Sw
    APAT_00 = AP_00 + (AP_01 * A_01);
    APAT_01 = AP_01;
    APAT_10 = AP_10 + (AP_11 * A_01);
    APAT_11 = AP_11;
    KCPAT_00 = (K_00 * P_00) + (K_00 * P_01) * A_01;
    KCPAT_01 = (K_00 * P_01);
    KCPAT_10 = (K_10 * P_00) + (K_10 * P_01) * A_01;
    KCPAT_11 = (K_10 * P_01);
    P_00 = APAT_00 - KCPAT_00 + Sw_00;
    P_01 = APAT_01 - KCPAT_01 + Sw_01;
    P_10 = APAT_10 - KCPAT_10 + Sw_10;
    P_11 = APAT_11 - KCPAT_11 + Sw_11;

    // Return the estimate.
    return x_00;
}
