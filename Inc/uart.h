//
// Created by pablito on 05.11.2020.
//

#ifndef FC_SOFT_UART_H
#define FC_SOFT_UART_H
#include "board.h"

typedef enum {
    MODE_8N1,
    MODE_8E2

} UART_Mode;

void uart_init(
        UART_dev*           dev,
        USART_TypeDef*      UART,
        uint16_t            RX_Pin,
        uint16_t            Tx_Pin,
        uint8_t             RX_PinSource,
        uint8_t             TX_PinSource,
        IRQn_Type           USART_IRQn,
        IRQn_Type           Rx_DMA_IRQn,
        IRQn_Type           Tx_DMA_IRQn,
        DMA_Stream_TypeDef* Rx_DMA_Stream,
        DMA_Stream_TypeDef* Tx_DMA_Stream,
        uint32_t            DMA_Channel,
        uint32_t            DMA_Rx_IT_Bit,
        uint32_t            DMA_Tx_IT_Bit,
        uint32_t            baudrate_,
        UART_Mode           mode
);
uint32_t uart_tx_bytes_free(UART_dev* dev);
bool set_mode(uint32_t baud, UART_Mode mode);
bool tx_buffer_empty();
bool flush();
void uart_register_rx_callback(UART_dev* dev, void (*cb)(uint8_t data));
void uart_unregister_rx_callback(UART_dev* dev);
void uart_write(UART_dev* dev, const uint8_t* ch, uint8_t len);
void uart_start_dma(UART_dev* dev);
uint8_t uart_read_byte(UART_dev* dev);
void uart_put_byte(UART_dev* dev, uint8_t byte);
uint32_t uart_rx_bytes_waiting(UART_dev* dev);

void UART_DMA_Tx_IRQ_callback(UART_dev* dev);
void UART_DMA_Rx_IRQ_callback(UART_dev* dev);
void USART_IRQ_callback();

void USART1_IRQHandler(void);
void USART3_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);

void DMA2_Stream7_IRQHandler(void);

void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);




#endif //FC_SOFT_UART_H
