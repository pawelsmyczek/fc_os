//
// Created by pablito on 29.05.2020.
//

#ifndef FC_SOFT_BMP180_H
#define FC_SOFT_BMP180_H


#include <cstdint>
#include "iic.h"

#define BMP180_I2C_PORT                 I2C2 // I2C port where the BMP180 connected

// BMP180 I2C related
#define BMP180_ADDR_WRITE               0xEE // BMP180 I2C address
#define BMP180_ADDR_READ                0xEF // BMP180 I2C address

// BMP180 registers
#define BMP180_PROM_START_ADDR          0xAA // E2PROM calibration data start register
#define BMP180_PROM_DATA_LEN            22   // E2PROM length
#define BMP180_CHIP_ID_REG              0xD0 // Chip ID
#define BMP180_VERSION_REG              0xD1 // Version
#define BMP180_CTRL_MEAS_REG            0xF4 // Measurements control (OSS[7.6], SCO[5], CTL[4.0]
#define BMP180_ADC_OUT_MSB_REG          0xF6 // ADC out MSB  [7:0]
#define BMP180_ADC_OUT_LSB_REG          0xF7 // ADC out LSB  [7:0]
#define BMP180_ADC_OUT_XLSB_REG         0xF8 // ADC out XLSB [7:3]
#define BMP180_SOFT_RESET_REG           0xE0 // Soft reset control

// BMP180 control values
#define BMP180_T_MEASURE                0x2E // temperature measurement
#define BMP180_P0_MEASURE               0x34 // pressure measurement (OSS=0, 4.5ms)
#define BMP180_P1_MEASURE               0x74 // pressure measurement (OSS=1, 7.5ms)
#define BMP180_P2_MEASURE               0xB4 // pressure measurement (OSS=2, 13.5ms)
#define BMP180_P3_MEASURE               0xF4 // pressure measurement (OSS=3, 25.5ms)

// BMP180 Pressure calculation constants
#define BMP180_PARAM_MG                 3038
#define BMP180_PARAM_MH                -7357
#define BMP180_PARAM_MI                 3791
#define PRESS_SAMPLE_COUNT              15

// reference for altitude calculation
#define SEA_LEVEL_PRESSURE              101325


typedef enum {
    BMP180_LOWPOWER = 0,      // Ultra low power mode (oss = 0)
    BMP180_STANDARD = 1,      // Standard mode (oss = 1)
    BMP180_HIRES    = 2,      // High resolution (oss = 2)
    BMP180_UHIRES   = 3,      // Ultra high resolution (oss = 3)
    BMP180_ADVRES   = 4       // Advanced resolution (oss = 3, software oversampling)
} BMP180_Mode_TypeDef;

typedef enum {
    START_TEMP_READ     = 0,
    READ_TEMP           = 1,
    START_PRESS_READ    = 2,
    READ_PRESS          = 3
} BMP180_State;

typedef struct {
    uint16_t OSS_delay;
    uint8_t OSS_cmd;
} BMP180_OSS_TypeDef;

typedef enum {
    BMP180_ERROR   = 0,
    BMP180_SUCCESS = !BMP180_ERROR
} BMP180_RESULT;


// Calibration parameters structure
typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
    int32_t B5;
} BMP180_Calibration_TypeDef;


class BMP_180
{

    BMP_180(const BMP_180&) = delete;
    const BMP_180& operator=(const BMP_180&) = delete;
public:
    BMP_180(I2C* i2c) noexcept;

    ~BMP_180() noexcept;

    void reset();
    void write_reg(uint8_t reg, uint8_t value);
    uint8_t reaad_reg(uint8_t reg);


    BMP180_RESULT check(void);
    void read_calibration(void);

    void temperature_calculate(void);
    void pressure_calculate(void);
    void altitude_calculate(void);

    void bmp180_update(void);

    bool read_temp_start();
    bool read_temp_perform();
    bool read_press_start();
    bool read_press_perform();
    void low_pass_filter(float* output, float input, float cut_off_freq);
private:
    I2C* i2c;
    uint8_t temperature_buffer[2]{};
    uint8_t pressure_buffer[3]{};
    int32_t pressure_average_buffer[PRESS_SAMPLE_COUNT]{};
    int32_t pressure_total{};
    int32_t pressure_read{};
    float pressure_average{};
    float pressure_average_slow{};

    float ground_altitude{};
    float delta_altitude{};
    float altitude_current{};
    float last_altitude{};
    float velocity_current{};
    float altitude_calculated{};
    float low_pass_filtered{};

    int16_t temperature_read{};

    float dt{};
    uint32_t last_update_ms{};
    uint32_t next_update_ms;

    BMP180_State bmp_state;
    BMP180_Mode_TypeDef mode;
    bool new_data_available;

    BMP180_Calibration_TypeDef  bmp_calibration{}; // Calibration parameters from E2PROM of BMP180

    int16_t temperature_trigger{};

};




typedef enum {
    START_TEMP_READ_CB  = 0,
    READ_TEMP_CB        = 1,
    START_PRESS_READ_CB = 2,
    READ_PRESS_CB       = 3
} BMP180_Callback;


typedef struct{
    uint8_t temperature_buffer[2];
    uint8_t pressure_buffer[3];
    int32_t pressure_average_buffer[PRESS_SAMPLE_COUNT];
    int32_t pressure_total;
    int32_t pressure_read;
    float pressure_average;
    float pressure_average_slow;

    float ground_altitude;
    float delta_altitude;
    float altitude_current;
    float last_altitude;
    float velocity_current;
    float altitude_calculate;
    float low_pass_filtered;

    int16_t temperature_read;

    float dt;
    uint32_t last_update_ms;
    uint32_t next_update_ms;

    BMP180_State bmp_state;
    BMP180_Mode_TypeDef mode;
    bool new_data_available;
} BMP180_Data;

extern BMP180_Data                 bmp_data;


extern BMP180_Calibration_TypeDef  bmp_calibration; // Calibration parameters from E2PROM of BMP180

extern int16_t temperature_trigger;

// Delay and Commands for different BMP180 oversampling levels
static const BMP180_OSS_TypeDef BMP_OSS[] = {
        { 6, BMP180_P0_MEASURE},
        { 9, BMP180_P1_MEASURE},
        {15, BMP180_P2_MEASURE},
        {27, BMP180_P3_MEASURE}
};


// Function prototypes
BMP180_RESULT BMP180_Check(void);
void BMP180_Reset();
uint8_t BMP180_GetVersion(void);
void BMP180_ReadCalibration(void);

int16_t BMP180_GetTemperature(void);
int32_t BMP180_GetPressure(BMP180_Mode_TypeDef mode);
BMP180_RESULT BMP180_GetReadings(int16_t *RT, int32_t *RP, BMP180_Mode_TypeDef mode);

int32_t BMP180_hPa_to_mmHg(int32_t hPa);
int32_t BMP180_hPa_to_Altitude(int32_t hPa);


void init_bmp180(void);

void temperature_calculate(void);
void pressure_calculate(void);
void altitude_calculate(void);

void bmp180_update(void);

bool read_temp_start();
bool read_temp_perform();
bool read_press_start();
bool read_press_perform();

void read_temp_start_callback(uint8_t result);
void read_temp_perform_callback(uint8_t result);
void read_press_start_callback(uint8_t result);
void read_press_perform_callback(uint8_t result);
void low_pass_filter(float* output, float input, float cut_off_freq);

#endif //FC_SOFT_BMP180_H
