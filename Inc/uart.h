//
// Created by pablito on 05.11.2020.
//

#ifndef FC_SOFT_UART_H
#define FC_SOFT_UART_H
#include "board.h"

#define UART_RX_BUFFER_SIZE 512
#define UART_TX_BUFFER_SIZE 512


typedef enum {
    MODE_8N1,
    MODE_8E2

} UART_Mode;

class UART_
{
    UART_(const UART_&) = delete;
    const UART_& operator=(const UART_&) = delete;

public:
    UART_(const UART_dev_* dev,uint32_t baudrate, UART_Mode mode) noexcept;
    ~UART_() noexcept;

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
    const UART_dev_* dev;
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


extern "C"
{
//    void USART2_IRQHandler(void);
    void UART4_IRQHandler(void);
//    void UART5_IRQHandler(void);
    void DMA1_Stream4_IRQHandler(void);
    void DMA1_Stream2_IRQHandler(void);
}






#endif //FC_SOFT_UART_H
