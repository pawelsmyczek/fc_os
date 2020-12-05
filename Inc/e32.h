//
// Created by pablito on 01.12.2020.
//

#ifndef FC_SOFT_E32_H
#define FC_SOFT_E32_H

#include "uart.h"


class E32
{
    E32(const E32&) = delete;
    const E32& operator =(const E32&) = delete;
public:
    E32(UART_* uart);
    void write(const uint8_t*, uint8_t);
    uint8_t read();
    void read_cb(uint8_t byte);

private:
    UART_* uart;
    uint8_t buffer[10];
    uint8_t head = 0;
    uint8_t prev_byte = 0;
};


#endif //FC_SOFT_E32_H
