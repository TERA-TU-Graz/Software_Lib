#ifdef STM32F40_41xxx

#include <module/common/fifo.h>
#include <module/gpio/gpio.h>
#include <module/uart/debug_uart_stm32f4.h>
#include <stdbool.h>
#include <stdint.h>


uint8_t debug_uart_tx_buffer_[DEBUG_UART_TX_QUEUE_LENGTH];
uint8_t debug_uart_rx_buffer_[DEBUG_UART_RX_QUEUE_LENGTH];

#if defined(STM32F4TERA_V2)
UART_Config debug_uart_ = {                                  \
  .usart = USART3,                                           \
  .irq   = USART3_IRQn,                                      \
  .clock = RCC_APB1Periph_USART3,                            \
  .af = GPIO_AF_USART3,                                      \
  .pins = {PC11, PC10},                                      \
                                                             \
  .baudrate = 460800,                                        \
                                                             \
  .rxQueue = {.buffer = debug_uart_rx_buffer_,               \
              .mask = DEBUG_UART_RX_QUEUE_LENGTH-1,          \
              .read = 0,                                     \
              .write = 0},                                   \
                                                             \
  .txQueue = {.buffer = debug_uart_tx_buffer_,               \
              .mask = DEBUG_UART_TX_QUEUE_LENGTH-1,          \
              .read = 0,                                     \
              .write = 0},                                   \
                                                             \
  .buffer_overrun_protection = 0,                            \
  .start_isr = 0                                             \
};
#endif


void DEBUG_USART_irqHandler() {
  uint8_t value;

  if (USART_GetITStatus(debug_uart_.usart, USART_IT_RXNE) != RESET) {
    // Read one byte from the receive data register
    value = USART_ReceiveData(debug_uart_.usart);
    FIFO_append(&debug_uart_.rxQueue, value);
  }

  if (USART_GetITStatus(debug_uart_.usart, USART_IT_TXE) != RESET) {

    if (FIFO_QUEUE_isEmpty(debug_uart_.txQueue)) {
      USART_ITConfig(debug_uart_.usart, USART_IT_TXE, DISABLE);
      //usart_start_isr_ = true;
    } else {
      FIFO_fetch(&debug_uart_.txQueue, &value);
      USART_SendData(debug_uart_.usart, value);
    }
  } else if (USART_GetITStatus(debug_uart_.usart, USART_IT_TC) != RESET) {

    if (FIFO_QUEUE_isEmpty(debug_uart_.txQueue)) {
      debug_uart_.start_isr = true;
      USART_ClearITPendingBit(debug_uart_.usart, USART_IT_TC);
    } else {
      USART_ITConfig(debug_uart_.usart, USART_IT_TXE, ENABLE);
      FIFO_fetch(&debug_uart_.txQueue, &value);
      USART_SendData(debug_uart_.usart, value);
    }
  }
}


void DEBUG_UART_init() {
  UART_init(&debug_uart_);
}

#endif // STM32F40_41xxx
