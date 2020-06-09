//
// Created by pablito on 29.05.2020.
//

#include "bmp180.h"

void init_bmp180(void){
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);

    EXTI_InitTypeDef            EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Line = EXTI_Line4;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitTypeDef            NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x05;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void BMP180_WriteReg(uint8_t reg, uint8_t value) {
    write_data(&BMP180,BMP180_ADDR_WRITE,reg, &value);
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
    BMP180_Calibration.AC1 = (buffer[0]  << 8) | buffer[1];
    BMP180_Calibration.AC2 = (buffer[2]  << 8) | buffer[3];
    BMP180_Calibration.AC3 = (buffer[4]  << 8) | buffer[5];
    BMP180_Calibration.AC4 = (buffer[6]  << 8) | buffer[7];
    BMP180_Calibration.AC5 = (buffer[8]  << 8) | buffer[9];
    BMP180_Calibration.AC6 = (buffer[10] << 8) | buffer[11];
    BMP180_Calibration.B1  = (buffer[12] << 8) | buffer[13];
    BMP180_Calibration.B2  = (buffer[14] << 8) | buffer[15];
    BMP180_Calibration.MB  = (buffer[16] << 8) | buffer[17];
    BMP180_Calibration.MC  = (buffer[18] << 8) | buffer[19];
    BMP180_Calibration.MD  = (buffer[20] << 8) | buffer[21];
}

void BMP180_Read_UT(uint16_t* UT) {
    uint8_t buf[2];

    BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP180_T_MEASURE);
    delay_ms(6); // Wait for 4.5ms by datasheet
    read_data(&BMP180, BMP180_ADDR_READ, BMP180_ADC_OUT_MSB_REG, buf,2);
    *UT = (buf[0] << 8) | buf[1];
}

void BMP180_Read_PT(uint32_t *PT, uint8_t oss) {
    uint8_t buf[3];

    // Start pressure measurement
    BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP_OSS[oss].OSS_cmd);
    delay_ms(BMP_OSS[oss].OSS_delay);

    // Read pressure value
    buf[0] = BMP180_ADC_OUT_MSB_REG;

    read_data(&BMP180, BMP180_ADDR_READ, BMP180_ADC_OUT_MSB_REG, buf, 3);
    *PT = ((buf[0] << 16) | (buf[1] << 8) | buf[0]) >> (8 - oss);
}

int16_t BMP180_Calc_RT(uint16_t UT) {
    BMP180_Calibration.B5  = (((int32_t)UT - (int32_t)BMP180_Calibration.AC6) * (int32_t)BMP180_Calibration.AC5) >> 15;
    BMP180_Calibration.B5 += ((int32_t)BMP180_Calibration.MC << 11) / (BMP180_Calibration.B5 + BMP180_Calibration.MD);

    return (BMP180_Calibration.B5 + 8) >> 4;
}

int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss) {
    int32_t B3,B6,X3,p;
    uint32_t B4,B7;

    B6 = BMP180_Calibration.B5 - 4000;
    X3 = ((BMP180_Calibration.B2 * ((B6 * B6) >> 12)) >> 11) + ((BMP180_Calibration.AC2 * B6) >> 11);
    B3 = (((((int32_t)BMP180_Calibration.AC1) * 4 + X3) << oss) + 2) >> 2;
    X3 = (((BMP180_Calibration.AC3 * B6) >> 13) + ((BMP180_Calibration.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
    B4 = (BMP180_Calibration.AC4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (50000 >> oss);
    if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
    p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

    return p;
}

// Fast integer Pa -> mmHg conversion (Pascals to millimeters of mercury)
int32_t BMP180_kpa_to_mmhg(int32_t Pa) {
    return (Pa * 75) / 10000;
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
    *RT = BMP180_Calc_RT(UT); // Temperature must calculated before calculating pressure
    BMP180_Read_PT(&UP, (uint8_t)mode);
    *RP = BMP180_Calc_RP(UP,(uint8_t)mode);
    return BMP180_ERROR;
}


