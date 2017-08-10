#pragma once

#include <config/uart_def.h>

//void TERA_UART_write(UART_t* uart, uint8_t* data, uint16_t size);
//void TERA_UART_read(UART_t* uart, uint8_t* data, uint16_t size);
//void TERA_UART_readNB(UART_t uart, uint8_t* data, uint16_t size);

void TERA_UART_write(UartChannel_t* uart, uint8_t* data, uint16_t size);
void TERA_UART_read(UartChannel_t* uart, uint8_t* data, uint16_t size);
//void TERA_UART_readNB(UartChannel_t uart, uint8_t* data, uint16_t size);
