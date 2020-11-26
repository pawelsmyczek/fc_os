//
// Created by pablito on 12.11.2020.
//

#ifndef FC_SOFT_TIMER_H
#define FC_SOFT_TIMER_H
#include "board.h"

class PWM
{
    PWM(const PWM&) = delete;
    const PWM& operator=(const PWM&) = delete;
public:
    PWM(Timer_dev* dev, uint16_t freq, uint16_t min_ms, uint16_t max_ms) noexcept;
    ~PWM() noexcept;


private:
    Timer_dev* dev;
    uint16_t frequency;
};


#endif //FC_SOFT_TIMER_H
