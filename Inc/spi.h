//
// Created by pablito on 20.05.2020.
//

#ifndef FC_SOFT_SPI_H
#define FC_SOFT_SPI_H

#include "setup.h"

int spi_init(void);
void spi_tx(uint8_t address, uint8_t data);
uint8_t spi_transfer(uint8_t data);
void set_spi_divisor(uint16_t data);


#endif //FC_SOFT_SPI_H
