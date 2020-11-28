//
// Created by pablito on 24.05.2020.
//

#include "system_init.h"


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;



/* TIMERS FOR MOTORS PWM OUTPUT:
    1: TIM8_CH1
    2: TIM3_CH3
    3: TIM1_CH3
    4: TIM1_CH1

    MOTORS POSITION:

    FRONT
   4     1
    \   /
     \ /
      |
     / \
    /   \
   3     2
     BACK
*/


__IO uint32_t *output_channels[] = {&(TIM8->CCR1), // motor 1
                                    &(TIM3->CCR3), // motor 2
                                    &(TIM1->CCR3), // motor 3
                                    &(TIM1->CCR1)  // motor 4
                                        };

PIDControl pid_angle[3];
PIDControl pid_z_velocity;
PIDControl pid_altitude;

TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

float KP = 3.0f, KI =  0.005f, KD = 0.15f;
float KP_vel = 2.0f, KI_vel =  0.5f, KD_vel = 0.7f;
float KP_alt = 10.0f, KI_alt =  0.05f, KD_alt = 3.0f;


void system_init(void){
    init_system_clock();
    init_clocks();
    init_gpio();
//    init_timers();

    /*SPI periphs init*/

//    chip_select_init(&MPU6000_CS, GPIOA, GPIO_Pin_4);
//    spi_init(&MPU6000, SPI1, SPI_CPOL_High, &MPU6000_CS, DMA2_Stream3_IRQn,
//             DMA_Channel_3, DMA2_Stream3,
//             DMA2_Stream2, DMA_FLAG_TCIF3, DMA_FLAG_TCIF2, 0x02);

    chip_select_init(&M25P16_CS, GPIOB, GPIO_Pin_12);
    spi_init(&M25P16, SPI2, SPI_CPOL_Low, &M25P16_CS, DMA1_Stream4_IRQn,
            DMA_Channel_0, DMA1_Stream4,
            DMA1_Stream3, DMA_FLAG_TCIF4, DMA_FLAG_TCIF3, 0x03);

    /*I2C periph init*/

    i2c_init(&BMP180, I2C1, GPIO_Pin_8, GPIO_Pin_9, 400000, I2C1_EV_IRQn, I2C1_ER_IRQn,
            DMA1_Stream0, DMA_Channel_1, DMA1_Stream0_IRQn,
             DMA_FLAG_TCIF0, true);
    delay_ms(100);

    /*UART periphs init*/

    // uart_init(&RC);

    /*USB Init*/

    USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID, &USR_desc,
              &USBD_CDC_cb, &USR_cb);
    delay_ms(100);

    init_motors();
//    delay_ms(1000);
//    init_mpu();
    delay_ms(100);
    init_m25p16();
    delay_ms(100);


    for(uint8_t i = 0; i < 3; i++) {
        pid_init(&pid_angle[i], KP, KI, KD, 0.01f, -150.0f, 150.0f,
                 AUTOMATIC, REVERSE);
    }
    pid_init(&pid_z_velocity, KP_vel, KI_vel, KD_vel, 0.01f, -150.0f, 150.0f,
             AUTOMATIC, DIRECT);

    pid_init(&pid_altitude, KP_alt, KI_alt, KD_alt, 0.01f, -80.0f, 80.0f,
             AUTOMATIC, DIRECT);


}

void init_clocks(void){
    /* SysTick end of count event each 10ms */
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


    //    RCC_AHB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

}

void init_gpio(void){

    /*INIT LED*/

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*SPI1*/

    // CS PA4

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIOC->BSRRL |= GPIO_Pin_4;

    // SCK PB3

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);

    // MISO PA6

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);

    // MOSI PA7

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*SPI2*/

    // CS PB12

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIOB->BSRRL |= GPIO_Pin_12;


    // SCK PB13

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);

    //MISO PB14

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);

    //MOSI PC3

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    GPIO_Init(GPIOC, &GPIO_InitStructure);


    /*I2C1*/

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //SCL PB8

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    //SDA PB9

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);



    // PWMs


    /* PINS FOR MOTORS:
        1: PB0
        2: PC6
        3: PA10
        4: PA8
     */
    // 4: PA8

//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
//
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//    // 3: PA10
//
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
//
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//    // 2: PC6
//
//    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
//
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//    // 1: PB0
//
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
//
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // EXT INT

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

//    /* Hc sr04 */
//    /* trig */
//
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//    /* echo */
//
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);


}

void init_timers(void){
    /* TIMERS FOR MOTORS PWM OUTPUT:
        1: TIM8_CH1
        2: TIM3_CH3
        3: TIM1_CH3
        4: TIM1_CH1
 */
//    TIM_TimeBaseInitStructure.TIM_Prescaler = 640 - 1; // 64MHz/640 = 100kHz
//    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInitStructure.TIM_Period = 6000-1;//okres przepelnienia 60ms (wczesniej bylo 60k)
//    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
//    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
//
//    //output compare PWM1 mode
//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = 1; //CCRx = 1 (0??), 10uS HIGH
//    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
//
//    TIM_Cmd(TIM2, ENABLE);

    uint16_t period = TIM_ARR;
    TIM_TimeBaseInitStructure.TIM_Period = period;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 168 - 1; // uses system clock
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    TIM_Cmd(TIM1, ENABLE);

    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);
    TIM_Cmd(TIM8, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = period;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1; // system clock divided by two, taken from docs
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    TIM_Cmd(TIM3, ENABLE);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse =  (uint16_t) (((uint32_t) 50 * (period - 1)) / 100);

    // 4: TIM1_CH1
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    // 3: TIM1_CH3
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    // 2: TIM3_CH3
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);

    // 1. TIM8_CH1

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    // TIM_OCInitStructure.TIM_Pulse =  (uint16_t) (((uint32_t) 50 * (period - 1)) / 100);

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);


    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);

    /*
     * at this stage, motors are initialized
     * and no need for extra range configuration
     * */
}


void init_motors(void){
//    NOT REALLY NECESSARY
//    delay_ms(25);
//    write_motor(3, 1860);
//    delay_ms(25);
//    write_motor(3, 1060);
}

void toggle_leds_on_start(void) {
    for(uint8_t i = 0; i < 10; i++ ){
        INFO_LED_ON;
        delay_ms(100);
        INFO_LED_OFF;
        delay_ms(100);
    }

}

void init_usart(void){
    RCC_APB2PeriphClockLPModeCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate              = 9600;
    USART_InitStructure.USART_WordLength            = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits              = USART_StopBits_1;
    USART_InitStructure.USART_Parity                = USART_Parity_No ;
    USART_InitStructure.USART_Mode                  = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}


void write_motor(uint8_t channel, uint16_t value) {
    *output_channels[channel] = value;
}
