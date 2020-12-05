//
// Created by pablito on 28.11.2020.
//

#ifndef FC_SOFT_BMP280_H
#define FC_SOFT_BMP280_H

#include "iic.h"

#define BMP280_I2C_ADDRESS_READ              0xED
#define BMP280_I2C_ADDRESS_WRITE              0xEC
#define BMP280_CHIP_ID                  0x58 /* BMP280 has chip-id 0x58 */
#define BMP280_CHIP_ID_ADDR             0xD0
#define BMP280_CALIB_PROM_ID            0x88
#define BMP280_CALIB_DATA_LEN           24
#define BMP280_REG_PRESS_MSB            0xF7
#define BMP280_REG_CONFIG               0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL                 0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_RESET                0xE0
#define BMP280_VAL_RESET                0xB6
#define BMP280_REG_STATUS               0xF3 /* bits: 3 measuring; 0 im_update */
/**
 * Mode of BMP280 module operation.
 * Forced - Measurement is initiated by user.
 * Normal - Continues measurement.
 */
typedef enum {
    BMP280_MODE_SLEEP = 0,
    BMP280_MODE_FORCED = 1,
    BMP280_MODE_NORMAL = 3
} BMP280_Mode;

typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
} BMP280_Filter;

typedef enum {
    BMP280_SKIPPED = 0,          /* no measurement  */
    BMP280_ULTRA_LOW_POWER = 1,  /* oversampling x1 */
    BMP280_LOW_POWER = 2,        /* oversampling x2 */
    BMP280_STANDARD = 3,         /* oversampling x4 */
    BMP280_HIGH_RES = 4,         /* oversampling x8 */
    BMP280_ULTRA_HIGH_RES = 5    /* oversampling x16 */
} BMP280_Oversampling;


/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
    BMP280_STANDBY_05 =     0,      /* stand by time 0.5ms */
    BMP280_STANDBY_62 =     1,      /* stand by time 62.5ms */
    BMP280_STANDBY_125 =    2,     /* stand by time 125ms */
    BMP280_STANDBY_250 =    3,     /* stand by time 250ms */
    BMP280_STANDBY_500 =    4,     /* stand by time 500ms */
    BMP280_STANDBY_1000 =   5,    /* stand by time 1s */
    BMP280_STANDBY_2000 =   6,    /* stand by time 2s BMP280, 10ms BME280 */
    BMP280_STANDBY_4000 =   7,    /* stand by time 4s BMP280, 20ms BME280 */
} BMP280_StandbyTime;


/**
 * Configuration parameters for BMP280 module.
 * Use function bmp280_init_default_params to use default configuration.
 */
typedef struct {
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_StandbyTime standby;
} bmp280_params_t;

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibrationTypedef;

typedef enum {
    BMP_ERROR = 0,
    BMP_SUCCESS = !BMP_ERROR
}BMP280_RESULT;


class BMP280
{
    BMP280(const BMP280&) = delete;
    const BMP280& operator =(const BMP280&) = delete;
public:
    BMP280(I2C* dev) noexcept;
    BMP280& operator =(BMP280&&) = default;
    BMP280_RESULT check(void);
    void read_calibration(void);

    void temperature_calculate(void);
    void pressure_calculate(void);
    void altitude_calculate(void);

    void update(void);
    uint8_t read_reg(uint8_t reg);
    void write_reg(uint8_t reg, uint8_t value);
    void status_read();
    void soft_reset(void);
    void write_config();
    float get_press();
    float get_temp();
    bool read_data_perform();
    void low_pass_filter(float* output, float input, float cut_off_freq);

private:
    I2C* dev;
    uint16_t addr;
    BMP280_CalibrationTypedef bmp_cali;
    uint8_t  id;        /* Chip ID */
    float pressure;
    float temperature;
    bmp280_params_t params;
    uint64_t up;
    int32_t ut;
    int32_t fine_temp;
    uint8_t data_buffer[6];


};


#endif //FC_SOFT_BMP280_H
