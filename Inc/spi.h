//
// Created by pablito on 20.05.2020.
//

#ifndef FC_SOFT_SPI_H
#define FC_SOFT_SPI_H

#include "setup.h"

static uint8_t* in_buffer;
static uint8_t* out_buffer;
uint8_t* dummyread;
uint8_t dma_buffer_size;
void (*cb)(void);


int spi_init(void);
void spi_tx(uint8_t address, uint8_t data);
uint8_t spi_transfer(uint8_t data);
void spi_perform_transfer();
void dma_transfer(uint8_t* in_data, uint8_t* out_data, uint8_t number_of_bytes, void (*callback)(void));
void set_spi_divisor(uint16_t data);
void DMA2_Stream3_IRQHandler();

#endif //FC_SOFT_SPI_H
