//
// Created by pablito on 12.11.2020.
//

#include "timer.h"


PWM::PWM(Timer_dev* _dev,
                uint16_t freq, uint16_t min_ms, uint16_t max_ms) noexcept
: dev(_dev)
, frequency(freq)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    uint32_t prescaler_default = 42;
    uint32_t tim_prescaler = prescaler_default;

    if( dev->TIM == TIM1 ||
            dev->TIM == TIM8 ||
                dev->TIM == TIM9 ||
                    dev->TIM == TIM10 ||
                        dev->TIM == TIM11)
        tim_prescaler*=2;



    TIM_TimeBaseInitStructure.TIM_Period = max_ms;
    TIM_TimeBaseInitStructure.TIM_Prescaler = tim_prescaler - 1; //168 - 1; // uses system clock
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse =  min_ms;

    switch (dev->TIM_Channel)
    {
        case TIM_Channel_1:
        {
            TIM_OC1Init(dev->TIM, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(dev->TIM, TIM_OCPreload_Enable);
            break;
        }
        case TIM_Channel_2:
        {
            TIM_OC2Init(dev->TIM, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(dev->TIM, TIM_OCPreload_Enable);
            break;
        }
        case TIM_Channel_3:
        {
            TIM_OC3Init(dev->TIM, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(dev->TIM, TIM_OCPreload_Enable);
            break;
        }
        case TIM_Channel_4:
        {
            TIM_OC4Init(dev->TIM, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(dev->TIM, TIM_OCPreload_Enable);
            break;
        }
        default:
            break;
    }

    if(dev->TIM ==TIM1 || dev->TIM ==TIM8)
        TIM_CtrlPWMOutputs(dev->TIM, ENABLE);

    TIM_ARRPreloadConfig(dev->TIM, ENABLE);
    TIM_Cmd(dev->TIM, ENABLE);

}
PWM::~PWM() noexcept
{

}


