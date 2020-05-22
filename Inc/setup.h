//
// Created by pablito on 22.03.2020.
//

#ifndef FC_SOFT_SETUP_H
#define FC_SOFT_SETUP_H

#include "stm32f4xx.h"
#include "mpu6000.h"
#include "pid.h"
#include "spi.h"
#include <stdbool.h>
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "kalman_filter.h"

#define TIM_ARR                          (uint16_t)1999
#define TIM_CCR                          (uint16_t)1000

#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  ((volatile uint32_t *)0xE0001004)
#define CYCCNTENA   (1 << 0)

#define MOTOR_1     0
#define MOTOR_2     1
#define MOTOR_3     2
#define MOTOR_4     3

DMA_InitTypeDef             DMA_InitStructure;

RCC_ClocksTypeDef           RCC_Clocks;
GPIO_InitTypeDef            GPIO_InitStructure;
TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
TIM_OCInitTypeDef           TIM_OCInitStructure;
SPI_InitTypeDef             SPI_InitStruct;


__IO uint32_t elapsed_ms_since_start;
__IO uint32_t sysTickCycleCounter;
__IO uint32_t usTicks;

float KP, KD, KI;
PIDControl pid_angle[3];
PIDControl pid_z_velocity;
char serial_out[70];


void system_init(void);
void SysTick_Handler(void);
void system_clock_init(void);
void init_clocks(void);
void init_timers(void);
void init_motors(void);
void write_motor(uint8_t channel, uint16_t value);
void toggle_leds_on_start(void);
void gpio_inits(void);
void usart_inits(void);

uint32_t millis(void);
uint32_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void TimingDelay_Decrement(void);
void Fail_Handler(void);

#endif //FC_SOFT_SETUP_H
