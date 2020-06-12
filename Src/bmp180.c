//
// Created by pablito on 29.05.2020.
//

#include "bmp180.h"


void init_bmp180(void){
    bmp_data.next_update_ms = millis();
    bmp_data.new_data_available = false;
    bmp_data.bmp_state = START_TEMP_READ;
    bmp_data.mode = BMP180_STANDARD;
}

void BMP180_WriteReg(uint8_t reg, uint8_t value) {
    write_data_async(&BMP180,BMP180_ADDR_WRITE,reg, &value, NULL, true);
}



uint8_t BMP180_ReadReg(uint8_t reg) {
    uint8_t value;
    read_data(&BMP180, BMP180_ADDR_READ, reg, &value, 1);
    return value;
}


BMP180_RESULT BMP180_Check(void) {
    uint8_t value;
    value = BMP180_ReadReg(BMP180_CHIP_ID_REG);
    return (value == 0x55) ? BMP180_SUCCESS : BMP180_ERROR;
}

void BMP180_Reset() {
    BMP180_WriteReg(BMP180_SOFT_RESET_REG,0xb6); // Do software reset
}

void BMP180_ReadCalibration(void) {
    uint8_t buffer[BMP180_PROM_DATA_LEN];

    read_data(&BMP180, BMP180_ADDR_READ, BMP180_PROM_START_ADDR, &buffer, BMP180_PROM_DATA_LEN);
    bmp_calibration.AC1 = (buffer[0] << 8) | buffer[1];
    bmp_calibration.AC2 = (buffer[2] << 8) | buffer[3];
    bmp_calibration.AC3 = (buffer[4] << 8) | buffer[5];
    bmp_calibration.AC4 = (buffer[6] << 8) | buffer[7];
    bmp_calibration.AC5 = (buffer[8] << 8) | buffer[9];
    bmp_calibration.AC6 = (buffer[10] << 8) | buffer[11];
    bmp_calibration.B1  = (buffer[12] << 8) | buffer[13];
    bmp_calibration.B2  = (buffer[14] << 8) | buffer[15];
    bmp_calibration.MB  = (buffer[16] << 8) | buffer[17];
    bmp_calibration.MC  = (buffer[18] << 8) | buffer[19];
    bmp_calibration.MD  = (buffer[20] << 8) | buffer[21];
}

void BMP180_Read_UT(uint16_t* UT) {
    uint8_t* buf;
    BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP180_T_MEASURE);
    delay_ms(5); // Wait for 4.5ms by datasheet
    read_data_async(&BMP180, BMP180_ADDR_READ, BMP180_ADC_OUT_MSB_REG,2, buf, NULL, true);
    *UT = (buf[0] << 8) | buf[1];
}

void BMP180_Read_PT(uint32_t *PT, uint8_t oss) {
    uint8_t buf[3];

    // Start pressure measurement
    BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP_OSS[oss].OSS_cmd);
    delay_ms(BMP_OSS[oss].OSS_delay);
    // Read pressure value
    read_data_async(&BMP180, BMP180_ADDR_READ, BMP180_ADC_OUT_MSB_REG, 3, buf, NULL, true);
    *PT = ((buf[0] << 16) | (buf[1] << 8) | buf[0]) >> (8 - oss);
}

int16_t BMP180_Calc_RT(uint16_t UT) {
    bmp_calibration.B5  = (((int32_t)UT - (int32_t)bmp_calibration.AC6) * (int32_t)bmp_calibration.AC5) >> 15;
    bmp_calibration.B5 += ((int32_t)bmp_calibration.MC << 11) / (bmp_calibration.B5 + bmp_calibration.MD);

    return (bmp_calibration.B5 + 8) >> 4;
}

int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss) {
    int32_t B3,B6,X3,p;
    uint32_t B4,B7;

    B6 = bmp_calibration.B5 - 4000;
    X3 = ((bmp_calibration.B2 * ((B6 * B6) >> 12)) >> 11) + ((bmp_calibration.AC2 * B6) >> 11);
    B3 = (((((int32_t)bmp_calibration.AC1) * 4 + X3) << oss) + 2) >> 2;
    X3 = (((bmp_calibration.AC3 * B6) >> 13) + ((bmp_calibration.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
    B4 = (bmp_calibration.AC4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (50000 >> oss);
    if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
    p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

    return p;
}

// Fast integer Pa -> mmHg conversion (Pascals to millimeters of mercury)
int32_t BMP180_hPa_to_mmHg(int32_t hPa) {
    return (hPa * 75) / 10000;
}

// Get pressure and temperature values from sensor
// input:
//   RT - pointer to temperature value
//   RP - pointer to pressure value
//   mode - BMP180 measure accuracy mode
// return:
//   BMP180_ERROR if it was an I2C timeout, BMP180_SUCCESS otherwise
BMP180_RESULT BMP180_GetReadings(int16_t *RT, int32_t *RP, BMP180_Mode_TypeDef mode) {
    uint32_t UP = 0;
    uint16_t UT = 0;

    BMP180_Read_UT(&UT);
    *RT = BMP180_Calc_RT(UT); // Temperature must be calculated before calculating pressure
    BMP180_Read_PT(&UP, (uint8_t)mode);
    *RP = BMP180_Calc_RP(UP,(uint8_t)mode);
    return BMP180_ERROR;
}


/**
 * I2C CALLBACKS CODE FOR BMP180 ACCORDING TO SEQUENCE FROM MANUAL,
 * DEPENDING ON STATE, THE UPDATE PROCEDURE WOULD HAVE DIFFERENT TIMINGS,
 * FOR CONSISTENCY, EVERY STATE CHANGE HAPPENS AROUND 20 ms AFTER PREVIOUS
 * */

void temperature_calculate(void){
    uint16_t uncompensated_temp = 0;
    /*TEMPERATURE CALC*/
    uncompensated_temp = (bmp_data.temperature_buffer[0] << 8) | bmp_data.temperature_buffer[1];
    bmp_calibration.B5  = (((int32_t)uncompensated_temp - (int32_t)bmp_calibration.AC6) * (int32_t)bmp_calibration.AC5) >> 15;
    bmp_calibration.B5 += ((int32_t)bmp_calibration.MC << 11) / (bmp_calibration.B5 + bmp_calibration.MD);

    bmp_data.temperature_read = (bmp_calibration.B5 + 8) >> 4;

}

void pressure_calculate(void){
    int32_t B3,B6,X3,p;
    uint32_t B4,B7;
    uint32_t uncompensated_press = 0;

    /*PRESSURE CALC*/
    uncompensated_press = ((bmp_data.pressure_buffer[0] << 16) | (bmp_data.pressure_buffer[1] << 8) |
                           bmp_data.pressure_buffer[0]) >> (8 - (uint8_t)bmp_data.mode);
    B6 = bmp_calibration.B5 - 4000;
    X3 = ((bmp_calibration.B2 * ((B6 * B6) >> 12)) >> 11) + ((bmp_calibration.AC2 * B6) >> 11);
    B3 = (((((int32_t)bmp_calibration.AC1) * 4 + X3) << (uint8_t)bmp_data.mode) + 2) >> 2;
    X3 = (((bmp_calibration.AC3 * B6) >> 13) + ((bmp_calibration.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
    B4 = (bmp_calibration.AC4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)uncompensated_press - B3) * (50000 >> (uint8_t)bmp_data.mode);
    if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
    p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

    bmp_data.pressure_read = p;
}

void altitude_calculate(void){
    bmp_data.altitude_current = 44330.0f*(1.0f-powf((float)(bmp_data.pressure_read)/SEA_LEVEL_PRESSURE, 1.0f / 5.255f));
}

void bmp180_update(void){
    uint32_t now_ms = millis();

    if(now_ms > bmp_data.next_update_ms){
        switch(bmp_data.bmp_state){
            case START_TEMP_READ:
                if(read_temp_start())
                    bmp_data.next_update_ms += 5;
                break;
            case READ_TEMP:
                if(read_temp_perform())
                    bmp_data.next_update_ms += 5;
                break;
            case START_PRESS_READ:
                if(read_press_start())
                    bmp_data.next_update_ms += BMP_OSS[bmp_data.mode].OSS_delay;
                break;
            case READ_PRESS:
                if(read_press_perform())
                    bmp_data.next_update_ms += 5;
                break;
            default:
                bmp_data.bmp_state = START_TEMP_READ;
                break;
        }
    }

    if(bmp_data.new_data_available){
        temperature_calculate();
        pressure_calculate();
        altitude_calculate();
    }
}

bool read_temp_start(){
    uint8_t value = BMP180_T_MEASURE;
    return write_data_async(&BMP180,BMP180_ADDR_WRITE, BMP180_CTRL_MEAS_REG, &value, &read_temp_start_callback, true) > 0;
}

bool read_temp_perform(){
    return read_data_async(&BMP180, BMP180_ADDR_READ, BMP180_ADC_OUT_MSB_REG,2, bmp_data.temperature_buffer, &read_temp_perform_callback, true) > 0;
}

bool read_press_start(){
    uint8_t value = BMP_OSS[bmp_data.mode].OSS_cmd;
    return write_data_async(&BMP180,BMP180_ADDR_WRITE, BMP180_CTRL_MEAS_REG, &value, &read_press_start_callback, true) > 0;
}

bool read_press_perform(){
    return read_data_async(&BMP180, BMP180_ADDR_READ, BMP180_ADC_OUT_MSB_REG,3, bmp_data.pressure_buffer, &read_press_perform_callback, true) > 0;
}

void read_temp_start_callback(uint8_t result){
    (void)result;
    bmp_data.last_update_ms = millis();
    bmp_data.next_update_ms = bmp_data.last_update_ms + 5;
    bmp_data.bmp_state = READ_TEMP;
    bmp_data.new_data_available = true;
}

void read_temp_perform_callback(uint8_t result){
    (void)result;
    bmp_data.last_update_ms = millis();
    bmp_data.next_update_ms = bmp_data.last_update_ms + 5;
    bmp_data.bmp_state = START_PRESS_READ;
    bmp_data.new_data_available = true;
}

void read_press_start_callback(uint8_t result){
    (void)result;
    bmp_data.last_update_ms = millis();
    bmp_data.next_update_ms = bmp_data.last_update_ms + 5;
    bmp_data.bmp_state = READ_PRESS;
    bmp_data.new_data_available = true;
}

void read_press_perform_callback(uint8_t result){
    (void)result;
    bmp_data.last_update_ms = millis();
    bmp_data.next_update_ms = bmp_data.last_update_ms + 5;
    bmp_data.bmp_state = START_TEMP_READ;
    bmp_data.new_data_available = true;
}
