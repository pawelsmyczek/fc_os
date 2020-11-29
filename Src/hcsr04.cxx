//
// Created by pablito on 08.11.2020.
//

#include "hcsr04.h"

HCSR04* hcsr04 = nullptr;

HCSR04::HCSR04(PWM* _pwm) noexcept
:pwm(_pwm)
{
    EXTI_InitTypeDef exti;
    EXTI_StructInit(&exti);

    //INTERRUPT on RISING/FALLING edge - PC10, PC11, PC12, PC9 - ECHO PINs
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, GPIO_PinSource10);
    exti.EXTI_Line = EXTI_Line10;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
}
HCSR04::~HCSR04() noexcept
{

}
void HCSR04::hcsr04_calc_dist()
{
    int16_t differ;
    differ = hcsr04.tim_value_falling - hcsr04.tim_value_rising;
    //abs value
    if(differ<0) differ *= (-1);
    hcsr04.distance =  (differ * 10)/58;
}
volatile uint16_t& HCSR04::measurement_start()
{
    return hcsr04.tim_value_rising;
}
volatile uint16_t& HCSR04::measurement_end()
{
    return hcsr04.tim_value_falling;
}


//void EXTI15_10_IRQHandler()
//{
//    //odczyt licznika timera przy zboczu narastajacym i opadajacym
//    //pozwoli to wyliczyc czas trwania impulsu i odleglosc
//    if (EXTI_GetITStatus(EXTI_Line10))
//    {
//        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10) == SET)
//        {
//            hcsr04->measurement_start() = TIM_GetCounter(TIM2);
//        }
//        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10) == RESET)
//        {
//            hcsr04->measurement_end() = TIM_GetCounter(TIM2);
//        }
//        EXTI_ClearITPendingBit(EXTI_Line10);
//    }
//
//    if (EXTI_GetITStatus(EXTI_Line11))
//    {
//        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11) == SET)
//        {
//            //TIM_value_rising[1] = TIM_GetCounter(TIM2);
//        }
//        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11) == RESET)
//        {
//            //TIM_value_falling[1] = TIM_GetCounter(TIM2);
//        }
//        EXTI_ClearITPendingBit(EXTI_Line11);
//    }
//
//    if (EXTI_GetITStatus(EXTI_Line12))
//    {
//
//        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12) == SET)
//        {
//            //TIM_value_rising[2] = TIM_GetCounter(TIM2);
//        }
//        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12) == RESET)
//        {
//            //TIM_value_falling[2] = TIM_GetCounter(TIM2);
//        }
//        EXTI_ClearITPendingBit(EXTI_Line12);
//    }
//}

