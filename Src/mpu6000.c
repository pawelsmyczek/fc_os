//
// Created by pablito on 03.04.2020.
//

#include "mpu6000.h"
#include "math.h"
#include "MadgwickAHRS.h"

float accelRTError[3]      = {0.0f, 0.0f, 0.0f};
float gyroRTError[3]       = {0.0f, 0.0f, 0.0f};
float accelCTBias[3]       = {0.0f, 0.0f, 0.0f};
float position[3]          = {0.0f, 0.0f, 0.0f};
float position_previous[3] = {0.0f, 0.0f, 0.0f};
float velocity[3]          = {0.0f, 0.0f, 0.0f};
float velocity_previous[3] = {0.0f, 0.0f, 0.0f};
float gyroCTBias[3]        = {0.0f, 0.0f, 0.0f};
float mpu6000Temperature1  = 0.0f;
uint8_t counter[3]         = {0, 0, 0};
float accel_sum[3]         = {0.0f, 0.0f, 0.0f};
float previous_accelSum[3] = {0.0f, 0.0f, 0.0f};
float accel_angle[3]       = {0.0f, 0.0f, 0.0f};
float gyro_sum[3]          = {0.0f, 0.0f, 0.0f};
float rotation_matrix[9]        = {0.0f, 0.0f, 0.0f,
                                   0.0f, 0.0f, 0.0f,
                                   0.0f, 0.0f, 0.0f};
float angle[3]             = {0.0f, 0.0f, 0.0f};
float angle_from_rot[3]             = {0.0f, 0.0f, 0.0f};

const uint16_t SAMPLES_NUM = 20000;
float oneG = 9.81f;
uint8_t raw_mpu[15] = {MPU6000_ACCEL_XOUT_H | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


bool detect_mpu(void){
    unsigned int response_mpu = 0;

    set_spi_divisor(&MPU6000, 2);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_PWR_MGMT_1);          // Device Reset
    spi_transfer(&MPU6000, BIT_H_RESET);
    spi_disable(&MPU6000_CS);

    uint8_t attemptsRemaining = 5;
    do {
        delay_ms(150);

        const uint8_t whoAmI = spi_transfer(&MPU6000, MPU6000_WHOAMI);
        if (whoAmI == MPU6000_WHOAMI) {
            return true;
        }
    } while (attemptsRemaining--);
    return false;
}

bool init_mpu(void){

    set_spi_divisor(&MPU6000, 2);                         // 21 MHz SPI clock (within 20 +/- 10%)

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_PWR_MGMT_1);          // Device Reset
    spi_transfer(&MPU6000, BIT_H_RESET);
    spi_disable(&MPU6000_CS);

    delay_ms(150);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, SIGNAL_PATH_RESET);
    spi_transfer(&MPU6000, 0x07); //
    spi_disable(&MPU6000_CS);

    delay_ms(150);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_PWR_MGMT_1);          // Clock Source PPL with Z axis gyro reference
    spi_transfer(&MPU6000, MPU_CLK_SEL_PLLGYROZ);
    spi_disable(&MPU6000_CS);

    delay_ms(1);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_USER_CTRL);           // spi_disable(&MPU6000_CS) Primary I2C Interface
    spi_transfer(&MPU6000, BIT_I2C_IF_DIS);
    spi_disable(&MPU6000_CS);

    delay_ms(1);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_PWR_MGMT_2);
    spi_transfer(&MPU6000, 0x00);
    spi_disable(&MPU6000_CS);

    delay_ms(1);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_SMPLRT_DIV);          // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
    spi_transfer(&MPU6000, 0x00);
    spi_disable(&MPU6000_CS);

    delay_ms(1);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_CONFIG);              // Accel and Gyro DLPF Setting
    spi_transfer(&MPU6000, BITS_DLPF_CFG_10HZ);
    spi_disable(&MPU6000_CS);

    delay_ms(1);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_ACCEL_CONFIG);        // Accel +/- 4 G Full Scale
    spi_transfer(&MPU6000, BITS_FS_4G);
    spi_disable(&MPU6000_CS);

    delay_ms(1);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_GYRO_CONFIG);
    spi_transfer(&MPU6000, BITS_FS_500DPS);
    spi_disable(&MPU6000_CS);

    delay_ms(1);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_INT_PIN_CFG);
    spi_transfer(&MPU6000, BIT_INT_ANYRD_2CLEAR);
    spi_disable(&MPU6000_CS);

    delay_ms(1);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_INT_ENABLE);         // Gyro spi_enable(&MPU6000_CS) interrupts
    spi_transfer(&MPU6000, BIT_RAW_RDY_EN);
    spi_disable(&MPU6000_CS);

    delay_ms(1);

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_WHOAMI | 0x80);
    response.value = spi_transfer(&MPU6000, 0x00);
    spi_disable(&MPU6000_CS);

    delay_ms(100);

    // if(response.value < 100) return false;

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);

    EXTI_InitTypeDef            EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Line = EXTI_Line4;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitTypeDef            NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x04;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x04;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    ///////////////////////////////////

    // delay_ms(100);

    calibrate_mpu();

    return true;
}

void read_mpu(void)
{

    spi_enable(&MPU6000_CS);
    spi_transfer(&MPU6000, MPU6000_ACCEL_XOUT_H | 0x80);
    raw_accel[ROLL   ].bytes[1]            = spi_transfer(&MPU6000, 0x00);
    raw_accel[ROLL   ].bytes[0]            = spi_transfer(&MPU6000, 0x00);
    raw_accel[PITCH  ].bytes[1]            = spi_transfer(&MPU6000, 0x00);
    raw_accel[PITCH  ].bytes[0]            = spi_transfer(&MPU6000, 0x00);
    raw_accel[YAW    ].bytes[1]            = spi_transfer(&MPU6000, 0x00);
    raw_accel[YAW    ].bytes[0]            = spi_transfer(&MPU6000, 0x00);

    rawMPU6000Temperature.bytes[1]         = spi_transfer(&MPU6000, 0x00);
    rawMPU6000Temperature.bytes[0]         = spi_transfer(&MPU6000, 0x00);

    raw_gyro[ROLL    ].bytes[1]            = spi_transfer(&MPU6000, 0x00);
    raw_gyro[ROLL    ].bytes[0]            = spi_transfer(&MPU6000, 0x00);
    raw_gyro[PITCH   ].bytes[1]            = spi_transfer(&MPU6000, 0x00);
    raw_gyro[PITCH   ].bytes[0]            = spi_transfer(&MPU6000, 0x00);
    raw_gyro[YAW     ].bytes[1]            = spi_transfer(&MPU6000, 0x00);
    raw_gyro[YAW     ].bytes[0]            = spi_transfer(&MPU6000, 0x00);
    spi_disable(&MPU6000_CS);

}

void data_transfer_callback_mpu(void) {
    raw_accel[ROLL   ].value       = (int16_t)((raw_mpu[1] << 8) | raw_mpu[2]);
    raw_accel[PITCH  ].value       = (int16_t)((raw_mpu[3] << 8) | raw_mpu[4]);
    raw_accel[YAW    ].value       = (int16_t)((raw_mpu[5] << 8) | raw_mpu[6]);
    rawMPU6000Temperature.value    = (int16_t)((raw_mpu[7] << 8) | raw_mpu[8]);
    raw_gyro[ROLL    ].value       = (int16_t)((raw_mpu[9] << 8) | raw_mpu[10]);
    raw_gyro[PITCH   ].value       = (int16_t)((raw_mpu[11] << 8) | raw_mpu[12]);
    raw_gyro[YAW     ].value       = (int16_t)((raw_mpu[13] << 8) | raw_mpu[14]);
}

void read_mpu_dma(void){
    mpu_timestamp = micros();
    raw_mpu[0] = MPU6000_ACCEL_XOUT_H | 0x80;
    dma_transfer(&MPU6000, raw_mpu, raw_mpu, 15, &data_transfer_callback_mpu);
}


void calibrate_mpu(void) {


    for (uint16_t samples = 0; samples < SAMPLES_NUM; samples++) {
        compute_mpu_tc_bias();
        accelRTError[ROLL ] += ((float)raw_accel[ROLL ].value / 8192.0f) - accelCTBias[ROLL ];
        accelRTError[PITCH] += ((float)raw_accel[PITCH].value / 8192.0f) - accelCTBias[PITCH];
        accelRTError[YAW  ] += ((float)raw_accel[YAW  ].value / 8192.0f) - accelCTBias[YAW  ];

        gyroRTError[ROLL ]  += ((float)raw_gyro[ROLL ].value / 65.5f) - gyroCTBias[ROLL ];
        gyroRTError[PITCH]  += ((float)raw_gyro[PITCH].value / 65.5f) - gyroCTBias[PITCH];
        gyroRTError[YAW  ]  += ((float)raw_gyro[YAW  ].value / 65.5f) - gyroCTBias[YAW  ];
    }

    for (uint8_t axis = 0; axis < 3; axis++) {
        accelRTError[axis]   = accelRTError[axis] / (float)SAMPLES_NUM;
        gyroRTError[axis] = gyroRTError[axis] / (float)SAMPLES_NUM;
    }
    calibrated = true;
//    gyro_sum[ROLL ] = (float)raw_gyro[ROLL ].value / 32.8f - gyroRTError[ROLL ];
//    gyro_sum[PITCH] = (float)raw_gyro[PITCH].value / 32.8f - gyroRTError[PITCH];
//    gyro_sum[YAW  ] = (float)raw_gyro[YAW  ].value / 32.8f - gyroRTError[YAW  ];
    oneG = sqrtf(powf((float)raw_accel[ROLL ].value, 2.0f) + powf((float)raw_accel[PITCH].value, 2.0f) + powf((float)raw_accel[YAW ].value, 2.0f));
}

void compute_mpu_tc_bias(void)
{
    mpu6000Temperature1 = (float) (rawMPU6000Temperature.value) / 340.0f + 35.0f;

    accelCTBias[ROLL ] *= mpu6000Temperature1;
    accelCTBias[PITCH] *= mpu6000Temperature1;
    accelCTBias[YAW  ] *= mpu6000Temperature1;

    gyroCTBias[ROLL ]  *= mpu6000Temperature1;
    gyroCTBias[PITCH]  *= mpu6000Temperature1;
    gyroCTBias[YAW  ]  *= mpu6000Temperature1;
}


void compute_angles(void){
    uint64_t us_prev = mpu_timestamp;
    mpu_timestamp = micros();
    float seconds_diff = (float)(mpu_timestamp - us_prev) / 100000.0f;  // TODO should be 1us, but is 10 us instead
    accel_sum[ROLL ] = ((float)(raw_accel[ROLL ].value) / 8192.0f) - accelRTError[ROLL ];// asinff(((float)(raw_accel[ROLL ].value) / 16384.0f) / oneG);
    accel_sum[PITCH] = ((float)(raw_accel[PITCH].value) / 8192.0f) - accelRTError[PITCH];// asinff((-1*(float)(raw_accel[PITCH].value) / 16384.0f) / oneG);
    accel_sum[YAW  ] = ((float)(raw_accel[YAW  ].value) / 8192.0f) - accelRTError[YAW  ];

    accel_angle[ROLL] = atanf(accel_sum[ROLL ] / sqrtf(powf(accel_sum[PITCH], 2.0f) + powf(accel_sum[YAW  ], 2.0f)));
    accel_angle[PITCH] = atanf(accel_sum[PITCH] / sqrtf(powf(accel_sum[ROLL ], 2.0f) + powf(accel_sum[YAW  ], 2.0f)));

    gyro_sum[ROLL ] += (((float)raw_gyro[ROLL ].value / 65.5f) - gyroRTError[ROLL ]) * seconds_diff;//gyro_sum[ROLL] + (((float)(raw_gyro[ROLL].value) / 16.4) * seconds_diff);
    gyro_sum[PITCH] += (((float)raw_gyro[PITCH].value / 65.5f) - gyroRTError[PITCH]) * seconds_diff; //gyro_sum[PITCH] + (((float)(raw_gyro[PITCH].value) / 16.4) * seconds_diff);
    gyro_sum[YAW  ] += (((float)raw_gyro[YAW  ].value / 65.5f) - gyroRTError[YAW  ]) * seconds_diff;//gyro_sum[YAW] + (((float)(raw_gyro[YAW].value) / 16.4) * seconds_diff);


//    gyro_sum[ROLL ] -= gyro_sum[PITCH] * sinff(gyro_sum[YAW  ] * ((float)M_PI / 180.0f));//gyro_sum[ROLL] + (((float)(raw_gyro[ROLL].value) / 16.4) * seconds_diff);
//    gyro_sum[PITCH] += gyro_sum[ROLL ] * sinff(gyro_sum[YAW  ] * ((float)M_PI / 180.0f)); //gyro_sum[PITCH] + (((float)(raw_gyro[PITCH].value) / 16.4) * seconds_diff);


    // here we check for data received not valid
    angle[ROLL ] = gyro_sum[ROLL ] > 359.9f || gyro_sum[ROLL ] < -359.9f? 0.0f : 0.996f * (gyro_sum[ROLL ]) + 0.004f * accel_angle[ROLL ];
    angle[PITCH] = gyro_sum[PITCH] > 359.9f || gyro_sum[PITCH] < -359.9f? 0.0f : 0.996f * (gyro_sum[PITCH]) + 0.004f * accel_angle[PITCH];
    // we keep the standard method for yaw angle, since the error for that angle is relatively small
    angle[YAW  ] = gyro_sum[YAW  ] > 359.9f || gyro_sum[YAW  ] < -359.9f? 0.0f : 0.996f * (gyro_sum[YAW  ]) + 0.004f * accel_sum[YAW  ];


    MadgwickAHRSupdateIMU(
            (((float)raw_gyro[ROLL ].value / 65.5f) - gyroRTError[ROLL ]) * ((float)M_PI / 180.0f) ,
            (((float)raw_gyro[PITCH].value / 65.5f) - gyroRTError[PITCH]) * ((float)M_PI / 180.0f) ,
            (((float)raw_gyro[YAW  ].value / 65.5f) - gyroRTError[YAW  ]) * ((float)M_PI / 180.0f) ,
            ((float)(raw_accel[ROLL ].value) / 8192.0f) - accelRTError[ROLL ]  ,
            ((float)(raw_accel[PITCH].value) / 8192.0f) - accelRTError[PITCH]  ,
            (float)(raw_accel[YAW  ].value) / 8192.0f
    );

    angle_from_rot[ROLL ] = atan2f( 2.0f * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3 ) * 180.0f / (float)M_PI ;
    angle_from_rot[PITCH] = -asinf( 2.0f * (q1*q3 - q0*q2) )                                * 180.0f / (float)M_PI ;
    angle_from_rot[YAW  ] = atan2f( 2.0f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3 ) * 180.0f / (float)M_PI ;

    // oneG = sqrtf(powf(accel_sum[ROLL ], 2.0f) + powf(accel_sum[PITCH], 2.0f) + powf(accel_sum[YAW  ], 2.0f));
}


void positions_estimate(void){
    uint64_t ms_prev = mpu_timestamp;
    mpu_timestamp = micros();
    float seconds_diff = (float)(mpu_timestamp - ms_prev) / 1000.0f;  // TODO should be 1us, but is 10 us instead
    velocity[ROLL ] = velocity_previous[ROLL ] + previous_accelSum[ROLL ] + (accel_sum[ROLL ] - previous_accelSum[ROLL ]);
    position[ROLL ] = position_previous[ROLL ] + velocity_previous[ROLL ] + (velocity[ROLL ] - velocity_previous[ROLL ]);
    velocity[PITCH] = velocity_previous[PITCH] + previous_accelSum[PITCH] + (accel_sum[PITCH] - previous_accelSum[PITCH]);
    position[PITCH] = position_previous[PITCH] + velocity_previous[PITCH] + (velocity[PITCH] - velocity_previous[PITCH]);
    velocity[YAW  ] = velocity_previous[YAW  ] + previous_accelSum[YAW  ] + (accel_sum[YAW  ] - previous_accelSum[YAW  ]);
    position[YAW  ] = position_previous[YAW  ] + velocity_previous[YAW  ] + (velocity[YAW  ] - velocity_previous[YAW  ]);

    previous_accelSum[ROLL ] = accel_sum[ROLL ];
    previous_accelSum[PITCH] = accel_sum[PITCH];
    previous_accelSum[YAW  ] = accel_sum[YAW  ];

    velocity_previous[ROLL ] = velocity[ROLL ];
    velocity_previous[PITCH] = velocity[PITCH];
    velocity_previous[YAW  ] = velocity[YAW  ];

    movement_end_check();

    position_previous[ROLL ] = position[ROLL ];
    position_previous[PITCH] = position[PITCH];
    position_previous[YAW  ] = position[YAW  ];

}

void create_rotation_matrix(void){
    rotation_matrix[0] = cosf(angle[PITCH])*cosf(angle[YAW]);
    rotation_matrix[1] = sinf(angle[ROLL])*sinf(angle[PITCH])*cosf(angle[YAW]) - cosf(angle[ROLL])*sinf(angle[YAW]);
    rotation_matrix[2] = cosf(angle[ROLL])*sinf(angle[PITCH])*cosf(angle[YAW]) + sinf(angle[ROLL])*sinf(angle[YAW]);
    rotation_matrix[3] = cosf(angle[PITCH])*sinf(angle[YAW]);
    rotation_matrix[4] = sinf(angle[ROLL])*sinf(angle[PITCH])*sinf(angle[YAW]) + cosf(angle[ROLL])*cosf(angle[YAW]);
    rotation_matrix[5] = cosf(angle[ROLL])*sinf(angle[PITCH])*sinf(angle[YAW]) - sinf(angle[ROLL])*cosf(angle[YAW]);
    rotation_matrix[6] = -1.0f*sinf(angle[PITCH]);
    rotation_matrix[7] = sinf(angle[ROLL])*cosf(angle[PITCH]);
    rotation_matrix[8] = cosf(angle[ROLL])*cosf(angle[PITCH]);

}

void movement_end_check(void)
{
    if(previous_accelSum[ROLL] > -0.02f && previous_accelSum[ROLL] < 0.02f) { counter[ROLL]++;}  else { counter[ROLL] = 0;}
    if (counter[ROLL]>=5) { velocity_previous[ROLL] = 0.0f;   velocity[ROLL] = 0.0f;}

    if(previous_accelSum[PITCH] > -0.02f && previous_accelSum[PITCH] < 0.02f) { counter[PITCH]++;}  else { counter[PITCH] = 0;}
    if (counter[PITCH]>=5) { velocity_previous[PITCH] = 0.0f;   velocity[PITCH] = 0.0f;}

    if(previous_accelSum[YAW] > -0.02f && previous_accelSum[YAW] < 0.02f) { counter[YAW]++;}  else { counter[YAW] = 0;}
    if (counter[YAW]>=5) { velocity_previous[YAW] = 0.0f;   velocity[YAW] = 0.0f;}
}


void EXTI4_IRQHandler(void) {
    EXTI_ClearITPendingBit(EXTI_Line4);
    read_mpu_dma();
}

