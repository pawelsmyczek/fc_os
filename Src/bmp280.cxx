//
// Created by pablito on 28.11.2020.
//
#include "bmp280.h"

static uint64_t next_update_ms = 0;
static uint64_t last_update_ms = 0;
// Delay and Commands for different BMP180 oversampling levels
static uint8_t oss;
static  bool new_data_available = false;
static const uint8_t bmp_oss[5] = {
        6,
        8,
        12,
        21,
        40
};

BMP280::BMP280(I2C_Dev* _dev) noexcept
    : dev(_dev)
    , addr(0)
    , id(0)
    , pressure(.0f)
    , temperature(.0f)
{
    if(!dev)
        return ;

    for(uint8_t i = 0; i < 6; ++i) {
        data_buffer[i] = 0;
    }
    params = {
            .mode = BMP280_MODE_NORMAL,
            .filter = BMP280_FILTER_4,
            .oversampling_pressure = BMP280_HIGH_RES,
            .oversampling_temperature = BMP280_HIGH_RES,
            .standby = BMP280_STANDBY_05
    };
    oss = params.oversampling_pressure;

}

void BMP280::write_reg(uint8_t reg, uint8_t value) {
    write_data_async(dev,BMP280_I2C_ADDRESS,reg, &value,nullptr, true);
}

uint8_t BMP280::read_reg(uint8_t reg) {
    uint8_t value;
    read_data(dev, BMP280_I2C_ADDRESS, reg, &value, 1);
    return value;
}

BMP280_RESULT
BMP280::check(void)
{
    uint8_t value;
    value = read_reg(BMP280_CHIP_ID);
    return (value == BMP280_CHIP_ID) ? BMP_SUCCESS : BMP_ERROR;

}


void
BMP280::soft_reset(void)
{
    write_reg(BMP280_REG_RESET, BMP280_VAL_RESET);
}

void
BMP280::status_read()
{
    for( ; ;)
    {
        if(read_reg(BMP280_REG_STATUS))
            break;
    }
}

void BMP280::write_config()
{
    uint8_t param = (params.standby << 5) | (params.filter << 2);
    write_reg(BMP280_REG_CONFIG, param);

    param = (params.oversampling_temperature << 5)
            | (params.oversampling_pressure << 2) | (params.mode);

    write_reg(BMP280_REG_CTRL, param);
}

void BMP280::read_calibration(void)
{
    uint8_t buffer[BMP280_CALIB_DATA_LEN];
    memset(buffer, 0, sizeof(buffer));

    read_data(dev, BMP280_I2C_ADDRESS, BMP280_CALIB_PROM_ID, buffer, BMP280_CALIB_DATA_LEN);
    bmp_cali.dig_T1 = (buffer[0] << 8) | buffer[1];
    bmp_cali.dig_T2 = (buffer[2] << 8) | buffer[3];
    bmp_cali.dig_T3 = (buffer[4] << 8) | buffer[5];
    bmp_cali.dig_P1 = (buffer[6] << 8) | buffer[7];
    bmp_cali.dig_P2 = (buffer[8] << 8) | buffer[9];
    bmp_cali.dig_P3 = (buffer[10] << 8) | buffer[11];
    bmp_cali.dig_P4 = (buffer[12] << 8) | buffer[13];
    bmp_cali.dig_P5 = (buffer[14] << 8) | buffer[15];
    bmp_cali.dig_P6 = (buffer[16] << 8) | buffer[17];
    bmp_cali.dig_P7 = (buffer[18] << 8) | buffer[19];
    bmp_cali.dig_P8 = (buffer[20] << 8) | buffer[21];
    bmp_cali.dig_P9 = (buffer[22] << 8) | buffer[23];


}

void
BMP280::temperature_calculate(void)
{
    int32_t var1, var2, adc_temp;
    adc_temp = data_buffer[3] << 12 | data_buffer[4] << 4 | data_buffer[5] >> 4;
    var1 = ((((adc_temp >> 3) - ((int32_t) bmp_cali.dig_T1 << 1)))
            * (int32_t) bmp_cali.dig_T2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t) bmp_cali.dig_T1)
              * ((adc_temp >> 4) - (int32_t) bmp_cali.dig_T1)) >> 12)
            * (int32_t) bmp_cali.dig_T3) >> 14;

    fine_temp = var1 + var2;
    ut = (fine_temp * 5 + 128) >> 8;
}
void
BMP280::pressure_calculate(void)
{
    int64_t var1, var2, p;
    int32_t adc_press;
    adc_press = data_buffer[0] << 12 | data_buffer[1] << 4 | data_buffer[2] >> 4;

    var1 = (int64_t) fine_temp - 128000;
    var2 = var1 * var1 * (int64_t) bmp_cali.dig_P6;
    var2 = var2 + ((var1 * (int64_t) bmp_cali.dig_P5) << 17);
    var2 = var2 + (((int64_t) bmp_cali.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t) bmp_cali.dig_P3) >> 8)
           + ((var1 * (int64_t) bmp_cali.dig_P2) << 12);
    var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) bmp_cali.dig_P1) >> 33;

    if (var1 == 0) {
        return ;  // avoid exception caused by division by zero
    }

    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t) bmp_cali.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t) bmp_cali.dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t) bmp_cali.dig_P7 << 4);
    up = p;
}
void
BMP280::altitude_calculate(void)
{

}

void BMP280::update(void)
{
    uint64_t now_ms = millis();
    if(now_ms > next_update_ms)
        if(!read_data_perform())
            next_update_ms += 1 ;

    if(new_data_available)
    {
        temperature_calculate();
        pressure_calculate();
        altitude_calculate();
    }
}

void data_perform_callback(uint8_t val)
{
    (void)val;
    last_update_ms = millis();
    next_update_ms = last_update_ms + bmp_oss[oss];

    new_data_available = true;
}

bool BMP280::read_data_perform()
{
    return read_data_async(dev, BMP280_I2C_ADDRESS,
                           BMP280_REG_PRESS_MSB,
                           6, data_buffer,
                           &data_perform_callback, false) > 0;
}

float
BMP280::get_press()
{
    return static_cast<float>(up)/256.f;
}
float
BMP280::get_temp()
{
    return static_cast<float>(ut)/100.f;
}

void BMP280::low_pass_filter(float* output, float input, float cut_off_freq)
{
    (void)output; (void)input; (void)cut_off_freq;
}

