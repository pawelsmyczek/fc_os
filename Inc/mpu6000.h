//
// Created by pablito on 03.04.2020.
//

#ifndef FC_SOFT_MPU6000_H
#define FC_SOFT_MPU6000_H


#include "spi.h"
//#include "setup.h"
#include <stdbool.h>


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

extern float accelRTError[3];
extern float gyroRTError[3];
extern float accelCTBias[3];
extern float position[3];
extern float position_previous[3];
extern float velocity[3];
extern float velocity_previous[3];
extern float gyroCTBias[3];
extern float mpu6000Temperature1;
extern uint8_t counter[3];
extern float accel_sum[3];
extern float previous_accelSum[3];
extern float angle_init[3];
extern float gyro_sum[3];
// float rotation_matrix[9];
extern float angle[3];
extern float real_angle[3];

extern uint8_int16_t raw_accel[3];
extern uint8_int16_t raw_gyro[3];
extern uint8_int16_t rawMPU6000Temperature;
extern uint8_int16_t response;
extern bool calibrated;

extern __IO uint64_t mpu_timestamp;

class MPU6000_
{
    MPU6000_(const MPU6000_&) = delete;
    const MPU6000_& operator=(const MPU6000_&) = delete;
public:
    MPU6000_(SPI* spi) noexcept;
    ~MPU6000_() noexcept;
    bool init_mpu(void);
    bool detect_mpu(void);
    void read_mpu(void);
    void read_mpu_dma(void);
    void calibrate_mpu(void);
    void compute_mpu_tc_bias(void);
    void compute_angles(void);
// void create_rotation_matrix(void);
    void movement_end_check();
    void positions_estimate(void);

private:
    SPI* spi;
    float accelRTError[3];
    float gyroRTError[3];
    float accelCTBias[3];
    float position[3];
    float position_previous[3];
    float velocity[3];
    float velocity_previous[3];
    float gyroCTBias[3];
    float mpu6000Temperature1;
    uint8_t counter[3];
    float accel_sum[3];
    float previous_accelSum[3];
    float angle_init[3];
    float gyro_sum[3];
    float real_angle[3];
    uint8_int16_t raw_accel[3];

    uint8_int16_t raw_gyro[3];
    uint8_int16_t rawMPU6000Temperature;
    uint8_int16_t response;
    // float rotation_matrix[9];
    float angle[3];
};


bool init_mpu(void);
bool detect_mpu(void);
void read_mpu(void);
void read_mpu_dma(void);
void calibrate_mpu(void);
void compute_mpu_tc_bias(void);
void compute_angles(void);
// void create_rotation_matrix(void);
void movement_end_check();
void positions_estimate(void);

extern "C"
{
void EXTI4_IRQHandler(void);
}
#endif //FC_SOFT_MPU6000_H
