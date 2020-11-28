//
// Created by pablito on 20.05.2020.
//

#ifndef FC_SOFT_SPI_H
#define FC_SOFT_SPI_H

#include "board.h"


extern uint8_t* dummyread;

class SPI
{
    SPI(const SPI&) = delete;
    const SPI& operator=(const SPI&) = delete;
public:
    SPI(const SPI_Dev_*) noexcept;
    ~SPI() noexcept;

    void chip_select_init(GPIO_TypeDef* gpio, uint16_t pinNumber);
    int spi_init(SPI_Dev* dev, SPI_TypeDef* SPI, uint16_t clockPolarity, DMA_InitTypeDef* dmaInitStructure, CS_Pin* ChipSelect,
                 uint8_t irqChannel, uint32_t dmaChannel, DMA_Stream_TypeDef *txDmaStream,
                 DMA_Stream_TypeDef *rxDmaStream, uint32_t dmaTxFlag, uint32_t dmaRxFlag, uint8_t priority);
    uint8_t spi_transfer(uint8_t data);
    void spi_enable();
    void spi_disable();
    void spi_perform_transfer();
    void dma_mem_write(uint8_t* out_data, uint32_t number_of_bytes);
    void dma_transfer(uint8_t* out_data, uint8_t* in_data, uint32_t number_of_bytes, void (*callback)(void));
    bool is_busy();
    void transfer_complete_callback(void);
    void set_spi_divisor(uint16_t data);

private:
    const SPI_Dev_*     dev;
    DMA_InitTypeDef     dma;
    void                (*callback)(void);
    uint8_t*            in_buffer;
    const uint8_t*      out_buffer;
    bool                busy;
};


void chip_select_init(CS_Pin* pin, GPIO_TypeDef* gpio, uint16_t pinNumber);
int spi_init(SPI_Dev* dev, SPI_TypeDef* SPI, uint16_t clockPolarity, CS_Pin* ChipSelect,
        uint8_t irqChannel, uint32_t dmaChannel, DMA_Stream_TypeDef *txDmaStream,
        DMA_Stream_TypeDef *rxDmaStream, uint32_t dmaTxFlag, uint32_t dmaRxFlag, uint8_t priority);


uint8_t spi_transfer(SPI_Dev *dev, uint8_t data);
void spi_enable(const CS_Pin *pin);
void spi_disable(const CS_Pin *pin);
void spi_perform_transfer(SPI_Dev *dev);
void dma_mem_write(SPI_Dev* dev, uint8_t* out_data, uint32_t number_of_bytes);
void dma_transfer(SPI_Dev *dev, uint8_t* out_data, uint8_t* in_data, uint32_t number_of_bytes, void (*callback)(void));
bool is_busy(SPI_Dev* dev);
void mpu_transfer_complete_callback(void);
void m25p16_transfer_complete_callback(void);
void set_spi_divisor(SPI_Dev* dev, uint16_t data);


#endif //FC_SOFT_SPI_H
