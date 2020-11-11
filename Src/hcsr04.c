//
// Created by pablito on 08.11.2020.
//

#include "hcsr04.h"

void init_hcsr04()
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



void HCSR04_CalculateDist(hcsr04_tim_vals* vals)
{
    uint16_t differ;
    for(uint8_t i = 0 ; i<4 ; i++)
    {
        differ = vals->tim_value_falling[i] - vals->tim_value_rising[i];
        //abs value
        if(differ<0) differ *= (-1);
        vals->distance[i] =  (differ * 10)/58;
    }
}


void EXTI15_10_IRQHandler()
{
    //odczyt licznika timera przy zboczu narastajacym i opadajacym
    //pozwoli to wyliczyc czas trwania impulsu i odleglosc
    if (EXTI_GetITStatus(EXTI_Line10))
    {
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10) == SET)
        {
            //TIM_value_rising[0] = TIM_GetCounter(TIM2);
        }
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10) == RESET)
        {
            //TIM_value_falling[0] = TIM_GetCounter(TIM2);
        }
        EXTI_ClearITPendingBit(EXTI_Line10);
    }

    if (EXTI_GetITStatus(EXTI_Line11))
    {
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11) == SET)
        {
            //TIM_value_rising[1] = TIM_GetCounter(TIM2);
        }
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11) == RESET)
        {
            //TIM_value_falling[1] = TIM_GetCounter(TIM2);
        }
        EXTI_ClearITPendingBit(EXTI_Line11);
    }

    if (EXTI_GetITStatus(EXTI_Line12))
    {

        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12) == SET)
        {
            //TIM_value_rising[2] = TIM_GetCounter(TIM2);
        }
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12) == RESET)
        {
            //TIM_value_falling[2] = TIM_GetCounter(TIM2);
        }
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

void EXTI9_5_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line9))
    {
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9) == SET)
        {
            //TIM_value_rising[3] = TIM_GetCounter(TIM2);
        }
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9) == RESET)
        {
            //TIM_value_falling[3] = TIM_GetCounter(TIM2);
        }
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}


