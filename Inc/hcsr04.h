//
// Created by pablito on 08.11.2020.
//

#ifndef FC_SOFT_HCSR04_H
#define FC_SOFT_HCSR04_H

#include "board.h"
#include "timer.h"

typedef struct{
    volatile uint16_t tim_value_rising[4];
    volatile uint16_t tim_value_falling[4];
    uint16_t distance[4];
} hcsr04_tim_vals;

class HCSR04
{
    HCSR04(const HCSR04& ) = delete;
    const HCSR04& operator =(const HCSR04& ) = delete;
public:
    HCSR04(PWM* pwm) noexcept;
    ~HCSR04() noexcept;
    void hcsr04_calc_dist();
    volatile uint16_t& measurement_start();
    volatile uint16_t& measurement_end();
private:
    struct hcsr04_tim_vals{
        volatile uint16_t tim_value_rising;
        volatile uint16_t tim_value_falling;
        uint16_t distance;
    } hcsr04;
    PWM* pwm;
};

void hcsr04_init(); // for exti
void hcsr04_calc_dist();

extern "C"
{
void EXTI15_10_IRQHandler();
void EXTI9_5_IRQHandler();
}
#endif //FC_SOFT_HCSR04_H
