//
// Created by pablito on 01.12.2020.
//

#include "e32.h"

E32* e;

static void rx_cb(uint8_t byte)
{
    e->read_cb(byte);
}

E32::E32(UART_* _uart)
: uart(_uart)
{
    e = this;
    uart->set_mode(9600, MODE_8N1);
    uart->uart_register_rx_callback(rx_cb);

    // set normal mode

    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);

}

void E32::write(const uint8_t* data, uint8_t len)
{
    uart->uart_write(data, len);
}

uint8_t E32::read()
{
    return prev_byte;
}

void E32::read_cb(uint8_t byte)
{
    buffer[head == 9? 0: ++head] = byte;
    prev_byte = byte;
}

