//
// Created by pablito on 22.03.2020.
//
#include "stm32f4xx.h"
#include "stdint.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"

void usart_printf(char* message){
    uint16_t size = strlen(message);
    char data[size];
    size = sprintf(data, message);
    for(uint16_t i = 0; i < size; i++){
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE));
        USART_SendData(USART1, data[i]);
    }
}

void usb_printf(char* message, ...){
    uint16_t size = strlen(message);
    char* data=NULL;
    va_list args;
    va_start(args, message);
    vsprintf(data, message, args);
    for(uint16_t i = 0; i < size; i++){
        VCP_put_char(*(data+i));
    }

}
