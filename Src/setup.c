//
// Created by pablito on 22.03.2020.
//

//
// Created by pablito on 22.03.2020.
//
#include "setup.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


GPIO_InitTypeDef            GPIO_InitStructure;
TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
TIM_OCInitTypeDef           TIM_OCInitStructure;
SPI_InitTypeDef             SPI_InitStruct;

static __IO uint32_t TimingDelay = 0;
static __IO uint32_t elapsed_ms_since_start = 0;
__IO uint32_t sysTickCycleCounter = 0;
__IO uint32_t usTicks = 0;

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


float KP = 0.05f, KI =  0.005f, KD = 0.15f;
float KP_vel = 2.0f, KI_vel =  0.5f, KD_vel = 0.7f;

void setup(void){
    system_init();
    init_clocks();
    gpio_inits();
    spi_init();
    init_timers();
    toggle_leds_on_start();
    USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID, &USR_desc,
              &USBD_CDC_cb, &USR_cb);

    init_motors();
    initMPU6000();

    for(uint8_t i = 0; i < 3; i++)
        pid_init(&pid_angle[i], KP, KI, KD, 0.01f, -150.0f, 150.0f,
                 AUTOMATIC, REVERSE);
    pid_init(&pid_z_velocity, KP_vel, KI_vel, KD_vel, 0.01f, -150.0f, 150.0f,
             AUTOMATIC, DIRECT);
}



void SysTick_Handler(void)
{
    ++elapsed_ms_since_start;
    readMPU6000();
}


void system_init(void){
    SystemInit();
    RCC_GetClocksFreq(&RCC_Clocks);
    if (SysTick_Config(SystemCoreClock / 1000)) {
        /* Capture error */
        while (1);
    }
    usTicks = SystemCoreClock / 1000000L;
    // enable DWT access
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // enable the CPU cycle counter
    DWT_CTRL |= CYCCNTENA;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

void init_clocks(void){
    /* SysTick end of count event each 10ms */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


}

void init_timers(void){
    /* TIMERS FOR MOTORS PWM OUTPUT:
        1: TIM8_CH1
        2: TIM3_CH3
        3: TIM1_CH3
        4: TIM1_CH1
 */

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
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1; // system clock divided by two,
                                                      // taken from docs,
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


void write_motor(uint8_t channel, uint16_t value) {
    *output_channels[channel] = value;
}



void toggle_leds_on_start(void) {
    for(uint8_t i = 0; i < 10; i++ ){
        GPIO_ResetBits(GPIOC, GPIO_Pin_14);
        delay_ms(4);
        GPIO_SetBits(GPIOC, GPIO_Pin_14);
        delay_ms(4);
    }

}

void gpio_inits(void){

    /*INIT LED*/

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*SPI*/

    // CS

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIOA->BSRRL |= GPIO_Pin_4;

    // SCK

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);

    // MISO

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);

    // MOSI

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


    // GPIO_SetBits(GPIOA, GPIO_Pin_4);

    // PWMs


    /* PINS FOR MOTORS:
        1: PB0
        2: PC6
        3: PA10
        4: PA8
     */
    // 4: PA8

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3: PA10

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 2: PC6

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 1: PB0

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


}

void usart_inits(void){
    RCC_APB2PeriphClockLPModeCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}


int spi_init(void){

    uint8_t dummyread = 0;


    SPI_I2S_DeInit(SPI1);

    SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_High;
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  // 42/64 = 0.65625 MHz SPI Clock
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial     = 7;

    SPI_Init(SPI1, &SPI_InitStruct);

    SPI_CalculateCRC(SPI1, DISABLE);



    SPI_Cmd(SPI1, ENABLE);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    dummyread = SPI_I2S_ReceiveData(SPI1);

    return dummyread;
}


void spi_tx(uint8_t address, uint8_t data){
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);

    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, address);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
    SPI_I2S_ReceiveData(SPI1);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, data);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
    SPI_I2S_ReceiveData(SPI1);

    GPIO_SetBits(GPIOA, GPIO_Pin_4);
}


uint8_t spi_transfer(uint8_t data) {
    uint16_t spiTimeout = 0x1000;
    uint8_t received = 0;

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return(0);

    SPI_I2S_SendData(SPI1, data);
    spiTimeout = 0x1000;

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return(0);

    received = (uint8_t)SPI_I2S_ReceiveData(SPI1);
    return received;
}

void set_spi_divisor(uint16_t data)
{
#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;

    SPI_Cmd(SPI1, DISABLE);

    tempRegister = SPI1->CR1;

    switch (data)
    {
        case 2:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_2;
            break;

        case 4:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_4;
            break;

        case 8:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_8;
            break;

        case 16:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_16;
            break;

        case 32:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_32;
            break;

        case 64:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_64;
            break;

        case 128:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_128;
            break;

        case 256:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_256;
            break;
    }

    SPI1->CR1 = tempRegister;

    SPI_Cmd(SPI1, ENABLE);
}


uint32_t micros(void)
{
    register uint32_t oldCycle, cycle, timeMs;

    do
    {
        timeMs = __LDREXW(&elapsed_ms_since_start);
        cycle = DWT->CYCCNT;
        oldCycle = sysTickCycleCounter;
    }
    while ( __STREXW( timeMs , &elapsed_ms_since_start ) );

    return (timeMs * 1000) + (cycle - oldCycle) / usTicks;
}


uint32_t millis(void){
    return elapsed_ms_since_start;
}

void delay_us(uint32_t us) {
    __IO uint32_t cycles = usTicks * us;
    __IO uint32_t start = DWT->CYCCNT;
    while(DWT->CYCCNT - start < cycles);
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void delay_ms(uint32_t ms)
{
    delay_us(ms * 1000);
}

void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0)
    {
        TimingDelay--;
    }
}

/**
  * @brief  This function handles the test program fail.
  * @param  None
  * @retval None
  */
void Fail_Handler(void)
{
    /* Erase last sector */
    FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
    /* Write FAIL code at last word in the flash memory */

    while(1)
    {
        delay_ms(5);
    }
}