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

class UART
{
    UART(const UART&) = delete;
    const UART& operator=(const UART&) = delete;

public:
    UART(UART_dev_* dev,uint32_t baudrate, UART_Mode mode) noexcept;
    ~UART() noexcept;

    uint32_t uart_tx_bytes_free();
    bool set_mode(uint32_t baud, UART_Mode mode);
    bool tx_buffer_empty();
    bool flush();
    void uart_register_rx_callback(void (*cb)(uint8_t data));
    void uart_unregister_rx_callback();
    void uart_write(const uint8_t* ch, uint8_t len);
    void uart_start_dma();
    uint8_t uart_read_byte();
    void uart_put_byte(uint8_t byte);
    uint32_t uart_rx_bytes_waiting();
    void dma_rx_irq_callback();
    void dma_tx_irq_callback();
private:
    UART_dev_* dev;
    DMA_InitTypeDef dma;
    uint32_t  baudrate;                 // the baudrate for the connection
    UART_Mode mode;
    uint8_t   rx_buffer_[UART_RX_BUFFER_SIZE]; // the buffer for incoming data
    uint8_t   tx_buffer_[UART_TX_BUFFER_SIZE]; // the buffer for outgoing data
    uint16_t  rx_buffer_head_;
    uint16_t  rx_buffer_tail_;
    uint16_t  tx_buffer_head_;
    uint16_t  tx_buffer_tail_;
    void      (*rx_cb)(uint8_t data);

};

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

extern "C"
{
    void USART2_IRQHandler(void);
    void USART4_IRQHandler(void);
    void USART5_IRQHandler(void);
    void DMA1_Stream5_IRQHandler(void);
//    void DMA1_Stream4_IRQHandler(void);
}






#endif //FC_SOFT_UART_H
