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



bool detect_mpu6000(void){
    unsigned int response_mpu = 0;

    set_spi_divisor(2);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_PWR_MGMT_1);          // Device Reset
    spi_transfer(BIT_H_RESET);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

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

bool initMPU6000(void){

    set_spi_divisor(2);                         // 21 MHz SPI clock (within 20 +/- 10%)

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_PWR_MGMT_1);          // Device Reset
    spi_transfer(BIT_H_RESET);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(150);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(SIGNAL_PATH_RESET);
    spi_transfer(0x07); //
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(150);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_PWR_MGMT_1);          // Clock Source PPL with Z axis gyro reference
    spi_transfer(MPU_CLK_SEL_PLLGYROZ);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(1);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_USER_CTRL);           // Disable Primary I2C Interface
    spi_transfer(BIT_I2C_IF_DIS);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(1);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_PWR_MGMT_2);
    spi_transfer(0x00);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(1);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_SMPLRT_DIV);          // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
    spi_transfer(0x00);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(1);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_CONFIG);              // Accel and Gyro DLPF Setting
    spi_transfer(BITS_DLPF_CFG_20HZ);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(1);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_ACCEL_CONFIG);        // Accel +/- 4 G Full Scale
    spi_transfer(BITS_FS_4G);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(1);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_GYRO_CONFIG);
    spi_transfer(BITS_FS_1000DPS);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(1);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_INT_PIN_CFG);
    spi_transfer(BIT_INT_ANYRD_2CLEAR);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(1);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_INT_ENABLE);         // Gyro enable interrupts
    spi_transfer(BIT_RAW_RDY_EN);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    delay_ms(1);

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_WHOAMI | 0x80);
    response.value = spi_transfer(0x00);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);



    // if(response.value < 100) return false;

    delay_ms(100);


    ///////////////////////////////////

    // delay_ms(100);

    mpu6000Calibration();

    return true;
}

void readMPU6000(void)
{

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    spi_transfer(MPU6000_ACCEL_XOUT_H | 0x80);
    rawAccel[ROLL   ].bytes[1]            = spi_transfer(0x00);
    rawAccel[ROLL   ].bytes[0]            = spi_transfer(0x00);
    rawAccel[PITCH  ].bytes[1]            = spi_transfer(0x00);
    rawAccel[PITCH  ].bytes[0]            = spi_transfer(0x00);
    rawAccel[YAW    ].bytes[1]            = spi_transfer(0x00);
    rawAccel[YAW    ].bytes[0]            = spi_transfer(0x00);

    rawMPU6000Temperature.bytes[1]  = spi_transfer(0x00);
    rawMPU6000Temperature.bytes[0]  = spi_transfer(0x00);

    rawGyro[ROLL    ].bytes[1]             = spi_transfer(0x00);
    rawGyro[ROLL    ].bytes[0]             = spi_transfer(0x00);
    rawGyro[PITCH   ].bytes[1]             = spi_transfer(0x00);
    rawGyro[PITCH   ].bytes[0]             = spi_transfer(0x00);
    rawGyro[YAW     ].bytes[1]             = spi_transfer(0x00);
    rawGyro[YAW     ].bytes[0]             = spi_transfer(0x00);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

}


void mpu6000Calibration(void) {


    for (uint16_t samples = 0; samples < SAMPLES_NUM; samples++)
    {
        readMPU6000();
        computeMPU6000TCBias();
        accelRTError[ROLL ] += ((float)rawAccel[ROLL ].value / 16384.0f) - accelCTBias[ROLL ];
        accelRTError[PITCH] += ((float)rawAccel[PITCH].value / 16384.0f) - accelCTBias[PITCH];
        accelRTError[YAW  ] += ((float)rawAccel[YAW  ].value / 16384.0f) - accelCTBias[YAW  ];

        gyroRTError[ROLL ]  += ((float)rawGyro[ROLL ].value / 65.5f) - gyroCTBias[ROLL ];
        gyroRTError[PITCH]  += ((float)rawGyro[PITCH].value / 65.5f) - gyroCTBias[PITCH];
        gyroRTError[YAW  ]  += ((float)rawGyro[YAW  ].value / 65.5f) - gyroCTBias[YAW  ];

        delay_ms(1);
    }

    for (uint8_t axis = 0; axis < 3; axis++)
    {
        accelRTError[axis]   = accelRTError[axis] / (float)SAMPLES_NUM;
        gyroRTError[axis] = gyroRTError[axis] / (float)SAMPLES_NUM;
    }

//    gyroSum[ROLL ] = (float)rawGyro[ROLL ].value / 32.8f - gyroRTError[ROLL ];
//    gyroSum[PITCH] = (float)rawGyro[PITCH].value / 32.8f - gyroRTError[PITCH];
//    gyroSum[YAW  ] = (float)rawGyro[YAW  ].value / 32.8f - gyroRTError[YAW  ];
    oneG = sqrtf(powf((float)rawAccel[ROLL ].value, 2.0f) + powf((float)rawAccel[PITCH].value, 2.0f) + powf((float)rawAccel[YAW ].value, 2.0f));
}

void computeMPU6000TCBias(void)
{
    mpu6000Temperature1 = (float) (rawMPU6000Temperature.value) / 340.0f + 35.0f;

    accelCTBias[0] *= mpu6000Temperature1;
    accelCTBias[1] *= mpu6000Temperature1;
    accelCTBias[2] *= mpu6000Temperature1;

    gyroCTBias[0 ]  *= mpu6000Temperature1;
    gyroCTBias[1]   *= mpu6000Temperature1;
    gyroCTBias[2 ]  *= mpu6000Temperature1;
}


void setGyroAccelSums(void){
    uint32_t ms_prev = ms;
    ms = millis();
    float seconds_diff = (float)(ms - ms_prev) / 100.0f;  // should be 1ms, but is 10 ms TODO
    accelSum[ROLL ] = ((float)(rawAccel[ROLL ].value) / 16384.0f) - accelRTError[ROLL ];// asinf(((float)(rawAccel[ROLL ].value) / 16384.0f) / oneG);    
    accelSum[PITCH] = ((float)(rawAccel[PITCH].value) / 16384.0f) - accelRTError[PITCH];// asinf((-1*(float)(rawAccel[PITCH].value) / 16384.0f) / oneG); 
    accelSum[YAW  ] = ((float)(rawAccel[YAW  ].value) / 16384.0f) - accelRTError[YAW  ];

    gyroSum[ROLL ] += (((((float)rawGyro[ROLL ].value / 65.5f))) - gyroRTError[ROLL ]) * seconds_diff;//gyroSum[ROLL] + (((float)(rawGyro[ROLL].value) / 16.4) * seconds_diff);
    gyroSum[PITCH] += (((((float)rawGyro[PITCH].value / 65.5f))) - gyroRTError[PITCH]) * seconds_diff; //gyroSum[PITCH] + (((float)(rawGyro[PITCH].value) / 16.4) * seconds_diff);
    gyroSum[YAW  ] += (((((float)rawGyro[YAW  ].value / 65.5f))) - gyroRTError[YAW  ]) * seconds_diff;//gyroSum[YAW] + (((float)(rawGyro[YAW].value) / 16.4) * seconds_diff);

    angle[ROLL ] = 0.996f * (gyroSum[ROLL ]) + 0.004f * accelSum[ROLL ];
    angle[PITCH] = 0.996f * (gyroSum[PITCH]) + 0.004f * accelSum[PITCH];
    angle[YAW  ] = 0.996f * (gyroSum[YAW  ]) + 0.004f * accelSum[YAW  ];

    oneG = sqrtf(powf(accelSum[ROLL ], 2.0f) + powf(accelSum[PITCH], 2.0f) + powf(accelSum[YAW  ], 2.0f));
}


void positions_estimate(void){
    uint32_t ms_prev = ms;
    ms = millis();
    float seconds_diff = (float)(ms - ms_prev) / 1000.0f;  // should be 1ms, but is 10 ms more TODO
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
