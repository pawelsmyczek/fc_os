//
// Created by pablito on 22.03.2020.
//

//
// Created by pablito on 22.03.2020.
//
#include "setup.h"

static volatile uint32_t elapsed_ms_since_start = 0;
static volatile uint32_t sysTickCycleCounter = 0;
static volatile uint32_t usTicks = 0;

RCC_ClocksTypeDef RCC_Clocks;

void SysTick_Handler(void)
{
    ++elapsed_ms_since_start;
}


void init_system_clock(void){
    SystemInit();
    RCC_GetClocksFreq(&RCC_Clocks);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    if (SysTick_Config(SystemCoreClock / 1000)) {
        /* Capture error */
        while (true);
    }
    usTicks = SystemCoreClock / 1000000L;
//    // enable DWT access
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // enable the CPU cycle counter
    DWT_CTRL |= CYCCNTENA;
    NVIC_SetPriority(SysTick_IRQn, 0);
}


uint64_t micros(void)
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
    uint32_t cycles = usTicks * us;
    uint32_t start = DWT->CYCCNT;
    while(DWT->CYCCNT - start < cycles);
}


void delay_ms(uint32_t ms)
{
    delay_us(ms * 1000);
}

void Fail_Handler(void)
{
    /* Erase last sector */
    FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
    /* Write FAIL code at last word in the flash memory */

    for( ; ; )
    {
        delay_ms(5);
    }
}