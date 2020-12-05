//
// Created by pablito on 24.05.2020.
//

#ifndef FC_SOFT_SYSTEM_INIT_H
#define FC_SOFT_SYSTEM_INIT_H

// #include "setup.h"
#include "timer.h"
#include "mpu6000.h"
#include "m25p16.h"
#include "bmp180.h"
#include "bmp280.h"
#include "pid.h"
#include "spi.h"
#include "iic.h"
#include "uart.h"
#include "e32.h"
#include "hcsr04.h"
#include "board.h"

#ifdef __cplusplus
extern "C"
{
#endif
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "MadgwickAHRS.h"
#ifdef __cplusplus
}
#endif

#define TIM_ARR     (uint16_t)1999
#define TIM_CCR     (uint16_t)1000

extern float KP, KD, KI;

extern PIDControl pid_angle[3];
extern PIDControl pid_z_velocity;
extern PIDControl pid_altitude;

void system_init(void);
void init_clocks(void);
void init_timers(void);
void init_gpio(void);
void toggle_leds_on_start(void);
void init_usart(void);
void init_motors(void);
void write_motor(uint8_t channel, uint16_t value);

#endif //FC_SOFT_SYSTEM_INIT_H
