//
// Created by pablito on 03.04.2020.
//

#ifndef FC_SOFT_MPU6000_H
#define FC_SOFT_MPU6000_H


#include "stm32f4xx.h"
#include "setup.h"
#include <stdbool.h>


#define TIM_ARR                          (uint16_t)1999
#define TIM_CCR                          (uint16_t)1000

#define MPU6000_CONFIG		    	0x1A
#define MPU6000_SMPLRT_DIV	    	0x19
#define MPU6000_GYRO_CONFIG	    	0x1B
#define MPU6000_ACCEL_CONFIG  		0x1C
#define MPU6000_FIFO_EN		    	0x23
#define MPU6000_INT_PIN_CFG	    	0x37
#define MPU6000_INT_ENABLE	    	0x38
#define MPU6000_INT_STATUS	    	0x3A
#define MPU6000_ACCEL_XOUT_H 		0x3B
#define MPU6000_ACCEL_XOUT_L 		0x3C
#define MPU6000_ACCEL_YOUT_H 		0x3D
#define MPU6000_ACCEL_YOUT_L 		0x3E
#define MPU6000_ACCEL_ZOUT_H 		0x3F
#define MPU6000_ACCEL_ZOUT_L    	0x40
#define MPU6000_TEMP_OUT_H	    	0x41
#define MPU6000_TEMP_OUT_L	    	0x42
#define MPU6000_GYRO_XOUT_H	    	0x43
#define MPU6000_GYRO_XOUT_L	    	0x44
#define MPU6000_GYRO_YOUT_H	    	0x45
#define MPU6000_GYRO_YOUT_L	     	0x46
#define MPU6000_GYRO_ZOUT_H	    	0x47
#define MPU6000_GYRO_ZOUT_L	    	0x48
#define SIGNAL_PATH_RESET           0x68
#define MPU6000_USER_CTRL	    	0x6A
#define MPU6000_PWR_MGMT_1	    	0x6B
#define MPU6000_PWR_MGMT_2	    	0x6C
#define MPU6000_FIFO_COUNTH	    	0x72
#define MPU6000_FIFO_COUNTL	    	0x73
#define MPU6000_FIFO_R_W		   	0x74
#define MPU6000_WHOAMI		    	0x75


// Bits

#define BIT_SLEEP				    0x40
#define BIT_H_RESET				    0x80
#define BITS_CLKSEL				    0x07
#define MPU_CLK_SEL_PLLGYROX	    0x01
#define MPU_CLK_SEL_PLLGYROZ	    0x03
#define MPU_EXT_SYNC_GYROX		    0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN			    0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA		    0x01

#define ROLL                        0
#define PITCH                       1
#define YAW                         2

typedef union {
    int16_t value;
    uint8_t bytes[2];
} uint8_int16_t;

float accelRTError[3]      ;
float gyroRTError[3]       ;
float accelCTBias[3]       ;
float position[3]          ;
float position_previous[3] ;
float velocity[3]          ;
float velocity_previous[3] ;
float gyroCTBias[3]        ;
float mpu6000Temperature1  ;
uint8_t counter[3]         ;
float accelSum[3]          ;
float previous_accelSum[3] ;
float gyroSum[3]           ;
float angle[3]            ;

uint8_int16_t rawAccel[3];

uint8_int16_t rawGyro[3];
uint8_int16_t rawMPU6000Temperature;
uint8_int16_t response;

uint32_t ms;

bool initMPU6000(void);
bool detect_mpu6000(void);
void readMPU6000(void);
void mpu6000Calibration(void);
void computeMPU6000TCBias(void);
void setGyroAccelSums(void);
void movement_end_check();
void positions_estimate(void);

#endif //FC_SOFT_MPU6000_H