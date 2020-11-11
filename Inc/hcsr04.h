//
// Created by pablito on 08.11.2020.
//

#ifndef FC_SOFT_HCSR04_H
#define FC_SOFT_HCSR04_H

#include "board.h"

typedef struct{
    volatile uint16_t tim_value_rising[4];
    volatile uint16_t tim_value_falling[4];
    uint16_t distance[4];
} hcsr04_tim_vals;
void hcsr04_init(); // for exti
void hcsr04_calc_dist();

void EXTI15_10_IRQHandler();
void EXTI9_5_IRQHandler();
#endif //FC_SOFT_HCSR04_H
