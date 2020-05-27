//
// Created by pablito on 24.05.2020.
//

#ifndef FC_SOFT_SYSTEM_INIT_H
#define FC_SOFT_SYSTEM_INIT_H

#include "board.h"
// #include "setup.h"
#include "mpu6000.h"
#include "pid.h"
#include "spi.h"
#include "iic.h"
#include "m25p16.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "kalman_filter.h"


#define TIM_ARR     (uint16_t)1999
#define TIM_CCR     (uint16_t)1000

float KP, KD, KI;

PIDControl pid_angle[3];
PIDControl pid_z_velocity;


void system_init(void);
void init_system_clock(void);
void init_clocks(void);
void init_timers(void);
void init_gpio(void);
void toggle_leds_on_start(void);
void init_usart(void);
void init_motors(void);
void write_motor(uint8_t channel, uint16_t value);

#endif //FC_SOFT_SYSTEM_INIT_H
