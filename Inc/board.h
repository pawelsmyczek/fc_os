//
// Created by pablito on 24.05.2020.
//

#ifndef FC_SOFT_BOARD_H
#define FC_SOFT_BOARD_H


#ifdef __cplusplus
extern "C"
{
#endif
#include "setup.h"
#ifdef __cplusplus
}
#endif

#define INFO_LED_ON         GPIO_SetBits(GPIOC, GPIO_Pin_14)
#define INFO_LED_OFF        GPIO_ResetBits(GPIOC, GPIO_Pin_14)

//extern SPI_Dev MPU6000;
//extern CS_Pin MPU6000_CS;

const CS_Pin MPU6000_CS =
        {
        .GPIO = GPIOA,
        .PinNumber = GPIO_Pin_4
        };

const SPI_Dev_ MPU6000_Dev =
        {
        .SPI =                SPI1,
        .ChipSelect =         &MPU6000_CS,
        .IRQChannel =         DMA2_Stream3_IRQn,
        .DMA_Channel =        DMA_Channel_3,
        .TX_DMA_Stream =      DMA2_Stream3,
        .RX_DMA_Stream =      DMA2_Stream2,
        .SPI_CPol =           SPI_CPOL_High,
        .DMA_FLAG_TX =        DMA_FLAG_TCIF3,
        .DMA_FLAG_RX =        DMA_FLAG_TCIF2,
        .IRQ_Prio =           0x01
        };

extern SPI_Dev M25P16;
extern CS_Pin M25P16_CS;


//const SPI_Dev_ M25P16_ =
//        {
//                .SPI =                  SPI2,
//                .ChipSelect =           &M25P16_CS,
//                .IRQChannel =           DMA1_Stream4_IRQn,
//                .DMA_Channel =          DMA_Channel_0,
//                .TX_DMA_Stream =        DMA1_Stream4,
//                .RX_DMA_Stream =        DMA1_Stream3,
//                .SPI_CPol =             SPI_CPOL_Low,
//                .DMA_FLAG_TX =          DMA_FLAG_TCIF4,
//                .DMA_FLAG_RX =          DMA_FLAG_TCIF3,
//                .IRQ_Prio =             0x02
//        };

extern I2C_Dev BMP180;



//const I2C_Dev_ BMP180_ =
//        {
//
//        .I2C               = I2C1,
//        .SCL_Pin           = GPIO_Pin_8,
//        .SDA_Pin           = GPIO_Pin_9,
//        .I2C_ClockSpeed    = 400000,
//        .I2C_EV_IRQn       = I2C1_EV_IRQn,
//        .I2C_ER_IRQn       = I2C1_ER_IRQn,
//        .DMA_Stream        = DMA1_Stream0,
//        .DMA_Channel       = DMA_Channel_1,
//        .DMA_IRQn          = DMA1_Stream0_IRQn,
//        .DMA_TCIF          = DMA_FLAG_TCIF0
//        };


//extern UART_dev RC;

//const UART_dev_ someDev =
//        {
//        .UART           = USART2,
//        .GPIO_RX        = GPIOA,
//        .GPIO_TX        = GPIOA,
//        .RX_Pin         = GPIO_Pin_3,
//        .Tx_Pin         = GPIO_Pin_2,
//        .RX_PinSource   = GPIO_PinSource3,
//        .TX_PinSource   = GPIO_PinSource2,
//        .USART_IRQn     = USART2_IRQn,
//        .Rx_DMA_IRQn    = DMA1_Stream4_IRQn,
//        .Tx_DMA_IRQn    = DMA1_Stream5_IRQn,
//        .Rx_DMA_Stream  = DMA1_Stream4,
//        .Tx_DMA_Stream  = DMA1_Stream5,
//        .DMA_Channel    = DMA_Channel_4,
//        .DMA_Rx_IT_Bit  = DMA_IT_TCIF4,
//        .DMA_Tx_IT_Bit  = DMA_IT_TCIF5,
//        };
/* TIMERS FOR MOTORS PWM OUTPUT:
    1: TIM8_CH1
    2: TIM3_CH3
    3: TIM1_CH3
    4: TIM1_CH1
*/
/* PINS FOR MOTORS:
    1: PB0
    2: PC6
    3: PA10
    4: PA8
 */
// 4: PA8


//const Timer_dev Motor1 =
//        {
//                GPIO               = GPIOB,
//                GPIO_Pin           = GPIO_Pin_0,
//                GPIO_PinSource     = GPIO_PinSource0,
//                TIM                = TIM8,
//                TIM_Channel        = TIM_Channel_1,
//                GIPO_AF_TIM        = GPIO_AF_TIM8
//                TIM_IRQn           = ,
//                TIM_IT_CC          =
//        };
//const Timer_dev Motor2 =
//        {
//
//                GPIO            = ,
//                GPIO_Pin        = ,
//                GPIO_PinSource  = ,
//                TIM             = ,
//                TIM_Channel     = ,
//                GIPO_AF_TIM     = ,
//                TIM_IRQn        = ,
//                TIM_IT_CC       =
//
//        };
//const Timer_dev Motor3 =
//        {
//                GPIO               = ,
//                GPIO_Pin           = ,
//                GPIO_PinSource     = ,
//                TIM                = ,
//                TIM_Channel        = ,
//                GIPO_AF_TIM        = ,
//                TIM_IRQn           = ,
//                TIM_IT_CC          =
//        };
//const Timer_dev Motor4 =
//        {
//                GPIO                 = ,
//                GPIO_Pin             = ,
//                GPIO_PinSource       = ,
//                TIM                  = ,
//                TIM_Channel          = ,
//                GIPO_AF_TIM          = ,
//                TIM_IRQn             = ,
//                TIM_IT_CC            =
//        };


#endif //FC_SOFT_BOARD_H
