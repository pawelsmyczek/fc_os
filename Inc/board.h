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


const SPI_Dev_ MPU6000_ =
        {
        .SPI =                SPI1,
        .ChipSelect =         &MPU6000_CS,
        .IRQChannel =         DMA2_Stream3_IRQn,
        .DMA_Channel =        DMA_Channel_3,
        .DMA_InitStructure =  &dmaInitStructure1,
        .TX_DMA_Stream =      DMA2_Stream3,
        .RX_DMA_Stream =      DMA2_Stream2,
        .SPI_CPol =           SPI_CPOL_High,
        .DMA_FLAG_TX =        DMA_FLAG_TCIF3,
        .DMA_FLAG_RX =        DMA_FLAG_TCIF2,
        .IRQ_Prio =           0x01
        };

extern SPI_Dev M25P16;
extern CS_Pin M25P16_CS;


const SPI_Dev_ M25P16_ =
        {
                .SPI =                  SPI2,
                .ChipSelect =           &M25P16_CS,
                .IRQChannel =           DMA1_Stream4_IRQn,
                .DMA_Channel =          DMA_Channel_0,
                .DMA_InitStructure =    &dmaInitStructure2,
                .TX_DMA_Stream =        DMA1_Stream4,
                .RX_DMA_Stream =        DMA1_Stream3,
                .SPI_CPol =             SPI_CPOL_Low,
                .DMA_FLAG_TX =          DMA_FLAG_TCIF4,
                .DMA_FLAG_RX =          DMA_FLAG_TCIF3,
                .IRQ_Prio =             0x02
        };

extern I2C_Dev BMP180;



const I2C_Dev_ BMP180_ =
        {

        .I2C               = I2C1,
        .SCL_Pin           = GPIO_Pin_8,
        .SDA_Pin           = GPIO_Pin_9,
        .DMA_InitStructure = &dmaInitStructure3,
        .I2C_ClockSpeed    = 400000,
        .I2C_EV_IRQn       = I2C1_EV_IRQn,
        .I2C_ER_IRQn       = I2C1_ER_IRQn,
        .DMA_Stream        = DMA1_Stream0,
        .DMA_Channel       = DMA_Channel_1,
        .DMA_IRQn          = DMA1_Stream0_IRQn,
        .DMA_TCIF          = DMA_FLAG_TCIF0
        };


extern UART_dev RC;

const UART_dev_ someDev =
        {
        .UART           = USART2,
        .GPIO_RX        = GPIOA,
        .GPIO_TX        = GPIOA,
        .RX_Pin         = GPIO_Pin_3,
        .Tx_Pin         = GPIO_Pin_2,
        .RX_PinSource   = GPIO_PinSource3,
        .TX_PinSource   = GPIO_PinSource2,
        .USART_IRQn     = USART2_IRQn,
        .Rx_DMA_IRQn    = DMA1_Stream4_IRQn,
        .Tx_DMA_IRQn    = DMA1_Stream5_IRQn,
        .Rx_DMA_Stream  = DMA1_Stream4,
        .Tx_DMA_Stream  = DMA1_Stream5,
        .DMA_Channel    = DMA_Channel_4,
        .DMA_Rx_IT_Bit  = DMA_IT_TCIF4,
        .DMA_Tx_IT_Bit  = DMA_IT_TCIF5,
        };

#endif //FC_SOFT_BOARD_H
