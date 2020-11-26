//
// Created by pablito on 26.11.2020.
//

#ifndef FC_SOFT_SETUP_H
#define FC_SOFT_SETUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"


#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  ((volatile uint32_t *)0xE0001004)
#define CYCCNTENA   (1 << 0)

#define MOTOR_1     0
#define MOTOR_2     1
#define MOTOR_3     2
#define MOTOR_4     3

#define UART_RX_BUFFER_SIZE 512
#define UART_TX_BUFFER_SIZE 512



typedef struct
{
    GPIO_TypeDef* GPIO;
    uint16_t PinNumber;
}CS_Pin;

typedef struct
{
    SPI_TypeDef*        SPI;
    CS_Pin*             ChipSelect;
    uint8_t             IRQChannel;
    uint32_t            DMA_Channel;
    DMA_InitTypeDef*    DMA_InitStructure;
    DMA_Stream_TypeDef* TX_DMA_Stream;
    DMA_Stream_TypeDef* RX_DMA_Stream;
    uint32_t            DMA_FLAG_TX;
    uint32_t            DMA_FLAG_RX;
    void                (*callback)(void);
    uint8_t*            in_buffer;
    const uint8_t*      out_buffer;
    bool                busy;

}SPI_Dev;

typedef struct
{
    SPI_TypeDef*        SPI;
    CS_Pin*             ChipSelect;
    uint8_t             IRQChannel;
    uint32_t            DMA_Channel;
    DMA_Stream_TypeDef* TX_DMA_Stream;
    DMA_Stream_TypeDef* RX_DMA_Stream;
    uint16_t            SPI_CPol;
    uint32_t            DMA_FLAG_TX;
    uint32_t            DMA_FLAG_RX;
    uint8_t            IRQ_Prio;
}SPI_Dev_;


typedef struct
{
    I2C_TypeDef*        I2C;
    uint16_t            SCL_Pin;
    uint16_t            SDA_Pin;
    DMA_InitTypeDef*    DMA_InitStructure;
    uint32_t            I2C_ClockSpeed;
    IRQn_Type           I2C_EV_IRQn;
    IRQn_Type           I2C_ER_IRQn;
    DMA_Stream_TypeDef* DMA_Stream;
    uint32_t            DMA_Channel;
    IRQn_Type           DMA_IRQn;
    uint32_t            DMA_TCIF;
    uint8_t             return_code;
    void                (*callback)(uint8_t);
    uint8_t             address;
    uint8_t             length;
    uint8_t             reg;
    uint8_t*            data;
    bool                subaddress_sent;
    bool                done;
    bool                initialised;
    uint8_t             current_status;
} I2C_Dev;


typedef struct
{
    I2C_TypeDef*        I2C;
    uint16_t            SCL_Pin;
    uint16_t            SDA_Pin;
    uint32_t            I2C_ClockSpeed;
    IRQn_Type           I2C_EV_IRQn;
    IRQn_Type           I2C_ER_IRQn;
    DMA_Stream_TypeDef* DMA_Stream;
    uint32_t            DMA_Channel;
    IRQn_Type           DMA_IRQn;
    uint32_t            DMA_TCIF;
} I2C_Dev_;



typedef struct
{
    USART_TypeDef*      UART;
    uint16_t            RX_Pin;
    uint16_t            Tx_Pin;
    uint8_t             RX_PinSource;
    uint8_t             TX_PinSource;
    IRQn_Type           USART_IRQn;
    IRQn_Type           Rx_DMA_IRQn;
    IRQn_Type           Tx_DMA_IRQn;
    DMA_Stream_TypeDef* Rx_DMA_Stream;
    DMA_Stream_TypeDef* Tx_DMA_Stream;
    uint32_t            DMA_Channel;
    uint32_t            DMA_Rx_IT_Bit;
    uint32_t            DMA_Tx_IT_Bit;
    void                (*rx_cb)(uint8_t data);
    uint32_t            baudrate;                 // the baudrate for the connection
    uint8_t             rx_buffer_[UART_RX_BUFFER_SIZE]; // the buffer for incoming data
    uint8_t             tx_buffer_[UART_TX_BUFFER_SIZE]; // the buffer for outgoing data
    uint16_t            rx_buffer_head_;
    uint16_t            rx_buffer_tail_;
    uint16_t            tx_buffer_head_;
    uint16_t            tx_buffer_tail_;
} UART_dev;


typedef struct
{
    USART_TypeDef*      UART;
    GPIO_TypeDef*       GPIO_RX;
    GPIO_TypeDef*       GPIO_TX;
    uint16_t            RX_Pin;
    uint16_t            Tx_Pin;
    uint8_t             RX_PinSource;
    uint8_t             TX_PinSource;
    IRQn_Type           USART_IRQn;
    IRQn_Type           Rx_DMA_IRQn;
    IRQn_Type           Tx_DMA_IRQn;
    DMA_Stream_TypeDef* Rx_DMA_Stream;
    DMA_Stream_TypeDef* Tx_DMA_Stream;
    uint32_t            DMA_Channel;
    uint32_t            DMA_Rx_IT_Bit;
    uint32_t            DMA_Tx_IT_Bit;
} UART_dev_;

typedef struct
{
    GPIO_TypeDef*   GPIO;
	uint16_t        GPIO_Pin;
	uint8_t         GPIO_PinSource;
	TIM_TypeDef*    TIM;
	uint8_t         TIM_Channel;
	uint8_t         GIPO_AF_TIM;
	IRQn_Type       TIM_IRQn;
	uint16_t        TIM_IT_CC;
} Timer_dev;



// INERNAL TYPEDEFS



extern volatile uint32_t usTicks;

// USER DEFINED STRUCTS




void SysTick_Handler(void);

uint32_t millis(void);

uint64_t micros(void);

void delay_us(uint32_t us);

void delay_ms(uint32_t ms);

void Fail_Handler(void);

#ifdef __cplusplus
}
#endif


#endif //FC_SOFT_SETUP_H
