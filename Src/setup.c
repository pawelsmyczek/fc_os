//
// Created by pablito on 22.03.2020.
//

//
// Created by pablito on 22.03.2020.
//
#include "setup.h"

static __IO uint32_t elapsed_ms_since_start = 0;
volatile uint32_t sysTickCycleCounter = 0;
volatile uint32_t usTicks = 0;

void SysTick_Handler(void)
{
    ++elapsed_ms_since_start;
}

volatile uint64_t micros(void)
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


volatile uint32_t millis(void){
    return (volatile uint32_t)(elapsed_ms_since_start);
}

void delay_us(uint32_t us) {
    volatile uint32_t cycles = usTicks * us;
    volatile uint32_t start = DWT->CYCCNT;
    while(DWT->CYCCNT - start < cycles);
}


void delay_ms(uint32_t ms)
{
    delay_us(ms * 1000);
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