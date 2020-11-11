//
// Created by pablito on 24.05.2020.
//

#ifndef FC_SOFT_BOARD_H
#define FC_SOFT_BOARD_H

#include "setup.h"

#define INFO_LED_ON         GPIO_SetBits(GPIOC, GPIO_Pin_14)
#define INFO_LED_OFF        GPIO_ResetBits(GPIOC, GPIO_Pin_14)

extern SPI_Dev MPU6000;
extern CS_Pin MPU6000_CS;

extern SPI_Dev M25P16;
extern CS_Pin M25P16_CS;

extern I2C_Dev BMP180;

extern UART_dev RC;

#endif //FC_SOFT_BOARD_H
