//
// Created by pablito on 03.04.2020.
//

#include "mpu6000.h"
#include "math.h"

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
float accelSum[3]          = {0.0f, 0.0f, 0.0f};
float previous_accelSum[3] = {0.0f, 0.0f, 0.0f};
float gyroSum[3]           = {0.0f, 0.0f, 0.0f};
float angle[3]            = {0.0f, 0.0f, 0.0f};
const uint16_t SAMPLES_NUM = 20000;
float oneG = 9.81f;



bool detect_mpu(void){
    unsigned int response_mpu = 0;

    set_spi_divisor(2);

    ENABLE_SPI;
    spi_transfer(MPU6000_PWR_MGMT_1);          // Device Reset
    spi_transfer(BIT_H_RESET);
    DISABLE_SPI;

    uint8_t attemptsRemaining = 5;
    do {
        delay_ms(150);

        const uint8_t whoAmI = spi_transfer(MPU6000_WHOAMI);
        if (whoAmI == MPU6000_WHOAMI) {
            return true;
        }
    } while (attemptsRemaining--);
    return false;
}

bool init_mpu(void){

    set_spi_divisor(2);                         // 21 MHz SPI clock (within 20 +/- 10%)

    ENABLE_SPI;
    spi_transfer(MPU6000_PWR_MGMT_1);          // Device Reset
    spi_transfer(BIT_H_RESET);
    DISABLE_SPI;

    delay_ms(150);

    ENABLE_SPI;
    spi_transfer(SIGNAL_PATH_RESET);
    spi_transfer(0x07); //
    DISABLE_SPI;

    delay_ms(150);

    ENABLE_SPI;
    spi_transfer(MPU6000_PWR_MGMT_1);          // Clock Source PPL with Z axis gyro reference
    spi_transfer(MPU_CLK_SEL_PLLGYROZ);
    DISABLE_SPI;

    delay_ms(1);

    ENABLE_SPI;
    spi_transfer(MPU6000_USER_CTRL);           // DISABLE_SPI Primary I2C Interface
    spi_transfer(BIT_I2C_IF_DIS);
    DISABLE_SPI;

    delay_ms(1);

    ENABLE_SPI;
    spi_transfer(MPU6000_PWR_MGMT_2);
    spi_transfer(0x00);
    DISABLE_SPI;

    delay_ms(1);

    ENABLE_SPI;
    spi_transfer(MPU6000_SMPLRT_DIV);          // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
    spi_transfer(0x00);
    DISABLE_SPI;

    delay_ms(1);

    ENABLE_SPI;
    spi_transfer(MPU6000_CONFIG);              // Accel and Gyro DLPF Setting
    spi_transfer(BITS_DLPF_CFG_10HZ);
    DISABLE_SPI;

    delay_ms(1);

    ENABLE_SPI;
    spi_transfer(MPU6000_ACCEL_CONFIG);        // Accel +/- 4 G Full Scale
    spi_transfer(BITS_FS_4G);
    DISABLE_SPI;

    delay_ms(1);

    ENABLE_SPI;
    spi_transfer(MPU6000_GYRO_CONFIG);
    spi_transfer(BITS_FS_2000DPS);
    DISABLE_SPI;

    delay_ms(1);

    ENABLE_SPI;
    spi_transfer(MPU6000_INT_PIN_CFG);
    spi_transfer(BIT_INT_ANYRD_2CLEAR);
    DISABLE_SPI;

    delay_ms(1);

    ENABLE_SPI;
    spi_transfer(MPU6000_INT_ENABLE);         // Gyro ENABLE_SPI interrupts
    spi_transfer(BIT_RAW_RDY_EN);
    DISABLE_SPI;

    delay_ms(1);

    ENABLE_SPI;
    spi_transfer(MPU6000_WHOAMI | 0x80);
    response.value = spi_transfer(0x00);
    DISABLE_SPI;



    // if(response.value < 100) return false;

    delay_ms(100);


    ///////////////////////////////////

    // delay_ms(100);

    calibrate_mpu();

    return true;
}

void read_mpu(void)
{

    ENABLE_SPI;
    spi_transfer(MPU6000_ACCEL_XOUT_H | 0x80);
    raw_accel[ROLL   ].bytes[1]            = spi_transfer(0x00);
    raw_accel[ROLL   ].bytes[0]            = spi_transfer(0x00);
    raw_accel[PITCH  ].bytes[1]            = spi_transfer(0x00);
    raw_accel[PITCH  ].bytes[0]            = spi_transfer(0x00);
    raw_accel[YAW    ].bytes[1]            = spi_transfer(0x00);
    raw_accel[YAW    ].bytes[0]            = spi_transfer(0x00);

    rawMPU6000Temperature.bytes[1]        = spi_transfer(0x00);
    rawMPU6000Temperature.bytes[0]        = spi_transfer(0x00);

    raw_gyro[ROLL    ].bytes[1]            = spi_transfer(0x00);
    raw_gyro[ROLL    ].bytes[0]            = spi_transfer(0x00);
    raw_gyro[PITCH   ].bytes[1]            = spi_transfer(0x00);
    raw_gyro[PITCH   ].bytes[0]            = spi_transfer(0x00);
    raw_gyro[YAW     ].bytes[1]            = spi_transfer(0x00);
    raw_gyro[YAW     ].bytes[0]            = spi_transfer(0x00);
    DISABLE_SPI;

}


void calibrate_mpu(void) {


    for (uint16_t samples = 0; samples < SAMPLES_NUM; samples++)
    {
        compute_mpu_tc_bias();
        accelRTError[ROLL ] += ((float)raw_accel[ROLL ].value / 8192.0f) - accelCTBias[ROLL ];
        accelRTError[PITCH] += ((float)raw_accel[PITCH].value / 8192.0f) - accelCTBias[PITCH];
        accelRTError[YAW  ] += ((float)raw_accel[YAW  ].value / 8192.0f) - accelCTBias[YAW  ];

        gyroRTError[ROLL ]  += ((float)raw_gyro[ROLL ].value / 4.1f) - gyroCTBias[ROLL ];
        gyroRTError[PITCH]  += ((float)raw_gyro[PITCH].value / 4.1f) - gyroCTBias[PITCH];
        gyroRTError[YAW  ]  += ((float)raw_gyro[YAW  ].value / 4.1f) - gyroCTBias[YAW  ];

    }

    for (uint8_t axis = 0; axis < 3; axis++)
    {
        accelRTError[axis]   = accelRTError[axis] / (float)SAMPLES_NUM;
        gyroRTError[axis] = gyroRTError[axis] / (float)SAMPLES_NUM;
    }

//    gyroSum[ROLL ] = (float)raw_gyro[ROLL ].value / 32.8f - gyroRTError[ROLL ];
//    gyroSum[PITCH] = (float)raw_gyro[PITCH].value / 32.8f - gyroRTError[PITCH];
//    gyroSum[YAW  ] = (float)raw_gyro[YAW  ].value / 32.8f - gyroRTError[YAW  ];
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
    uint32_t us_prev = us;
    us = millis();
    float seconds_diff = (float)(us - us_prev) / 100.0f;  // should be 1ms, but is 10 us TODO
    accelSum[ROLL ] = ((float)(raw_accel[ROLL ].value) / 8192.0f) - accelRTError[ROLL ];// asinf(((float)(raw_accel[ROLL ].value) / 16384.0f) / oneG);
    accelSum[PITCH] = ((float)(raw_accel[PITCH].value) / 8192.0f) - accelRTError[PITCH];// asinf((-1*(float)(raw_accel[PITCH].value) / 16384.0f) / oneG);
    accelSum[YAW  ] = ((float)(raw_accel[YAW  ].value) / 8192.0f) - accelRTError[YAW  ];

    gyroSum[ROLL ] += (((((float)raw_gyro[ROLL ].value / 4.1f))) - gyroRTError[ROLL ]) * seconds_diff;//gyroSum[ROLL] + (((float)(raw_gyro[ROLL].value) / 16.4) * seconds_diff);
    gyroSum[PITCH] += (((((float)raw_gyro[PITCH].value / 4.1f))) - gyroRTError[PITCH]) * seconds_diff; //gyroSum[PITCH] + (((float)(raw_gyro[PITCH].value) / 16.4) * seconds_diff);
    gyroSum[YAW  ] += (((((float)raw_gyro[YAW  ].value / 4.1f))) - gyroRTError[YAW  ]) * seconds_diff;//gyroSum[YAW] + (((float)(raw_gyro[YAW].value) / 16.4) * seconds_diff);

    angle[ROLL ] = 0.996f * (gyroSum[ROLL ]) + 0.004f * accelSum[ROLL ];
    angle[PITCH] = 0.996f * (gyroSum[PITCH]) + 0.004f * accelSum[PITCH];
    angle[YAW  ] = 0.996f * (gyroSum[YAW  ]) + 0.004f * accelSum[YAW  ];

    oneG = sqrtf(powf(accelSum[ROLL ], 2.0f) + powf(accelSum[PITCH], 2.0f) + powf(accelSum[YAW  ], 2.0f));
}


void positions_estimate(void){
    uint32_t ms_prev = us;
    us = millis();
    float seconds_diff = (float)(us - ms_prev) / 1000.0f;  // should be 1ms, but is 10 us more TODO
    velocity[ROLL ] = velocity_previous[ROLL ] + previous_accelSum[ROLL ] + (accelSum[ROLL ] - previous_accelSum[ROLL ]);
    position[ROLL ] = position_previous[ROLL ] + velocity_previous[ROLL ] + (velocity[ROLL ] - velocity_previous[ROLL ]);
    velocity[PITCH] = velocity_previous[PITCH] + previous_accelSum[PITCH] + (accelSum[PITCH] - previous_accelSum[PITCH]);
    position[PITCH] = position_previous[PITCH] + velocity_previous[PITCH] + (velocity[PITCH] - velocity_previous[PITCH]);
    velocity[YAW  ] = velocity_previous[YAW  ] + previous_accelSum[YAW  ] + (accelSum[YAW  ] - previous_accelSum[YAW  ]);
    position[YAW  ] = position_previous[YAW  ] + velocity_previous[YAW  ] + (velocity[YAW  ] - velocity_previous[YAW  ]);

    previous_accelSum[ROLL ] = accelSum[ROLL ];
    previous_accelSum[PITCH] = accelSum[PITCH];
    previous_accelSum[YAW  ] = accelSum[YAW  ];

    velocity_previous[ROLL ] = velocity[ROLL ];
    velocity_previous[PITCH] = velocity[PITCH];
    velocity_previous[YAW  ] = velocity[YAW  ];

    movement_end_check();

    position_previous[ROLL ] = position[ROLL ];
    position_previous[PITCH] = position[PITCH];
    position_previous[YAW  ] = position[YAW  ];

}


void movement_end_check(void)
{
    if(previous_accelSum[ROLL] > -0.05f && previous_accelSum[ROLL] < 0.05f) { counter[ROLL]++;}  else { counter[ROLL] = 0;}
    if (counter[ROLL]>=5) { velocity_previous[ROLL] = 0.0f;   velocity[ROLL] = 0.0f;}

    if(previous_accelSum[PITCH] > -0.05f && previous_accelSum[PITCH] < 0.05f) { counter[PITCH]++;}  else { counter[PITCH] = 0;}
    if (counter[PITCH]>=5) { velocity_previous[PITCH] = 0.0f;   velocity[PITCH] = 0.0f;}

    if(previous_accelSum[YAW] > -0.05f && previous_accelSum[YAW] < 0.05f) { counter[YAW]++;}  else { counter[YAW] = 0;}
    if (counter[YAW]>=5) { velocity_previous[YAW] = 0.0f;   velocity[YAW] = 0.0f;}

}
