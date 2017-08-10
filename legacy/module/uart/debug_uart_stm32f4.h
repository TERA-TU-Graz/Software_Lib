#ifdef STM32F40_41xxx

#ifndef TERA_LIBRARY_DEBUG_UART_STM32F4_H_INCLUDED
#define TERA_LIBRARY_DEBUG_UART_STM32F4_H_INCLUDED

#include <module/uart/uart_stm32f4.h>

#define DEBUG_UART_TX_QUEUE_LENGTH 2048
#define DEBUG_UART_RX_QUEUE_LENGTH  512

#ifdef STM32F4TERA_V2
#	define  DEBUG_USART_irqHandler      USART3_IRQHandler
#endif

extern UART_Config debug_uart_;
extern uint8_t debug_uart_tx_buffer_[DEBUG_UART_TX_QUEUE_LENGTH];
extern uint8_t debug_uart_rx_buffer_[DEBUG_UART_RX_QUEUE_LENGTH];

void DEBUG_UART_init();

#endif // TERA_LIBRARY_DEBUG_UART_STM32F4_H_INCLUDED

#endif
