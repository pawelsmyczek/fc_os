//
// Created by pablito on 22.03.2020.
//

#ifndef FC_SOFT_SETUP_H
#define FC_SOFT_SETUP_H



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

// INERNAL TYPEDEFS



volatile uint32_t sysTickCycleCounter;
volatile uint32_t usTicks;

// USER DEFINED STRUCTS


DMA_InitTypeDef             dmaInitStructure1;
DMA_InitTypeDef             dmaInitStructure2;
DMA_InitTypeDef             dmaInitStructure3;
RCC_ClocksTypeDef           RCC_Clocks;
GPIO_InitTypeDef            GPIO_InitStructure;
TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
TIM_OCInitTypeDef           TIM_OCInitStructure;

typedef struct{
    GPIO_TypeDef* GPIO;
    uint16_t PinNumber;
}CS_Pin;

typedef struct{
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



typedef struct{
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




void SysTick_Handler(void);
volatile uint32_t millis(void);
volatile uint64_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void Fail_Handler(void);

#endif //FC_SOFT_SETUP_H
