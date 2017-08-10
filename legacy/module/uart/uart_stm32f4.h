#ifndef TERA_LIBRARY_UART_H_INCLUDED
#define TERA_LIBRARY_UART_H_INCLUDED

#include <module/common/fifo.h>
#include <module/gpio/gpio.h>
#include <module/common/modules_def.h>

typedef enum
{
  UINT8 = 8,
  UINT16 = 16,
  UINT32 = 32
} DataType;


typedef struct {
  USART_TypeDef *usart;  // used USART
  uint32_t baudrate;     // baudrate
  uint32_t clock;        // UASRT AHB clock
  IRQn_Type  irq;        // IRQ
  uint8_t af;            // alternate function
  GPIO_Pin pins[2];
  FifoQueue rxQueue, txQueue;
  bool buffer_overrun_protection;
  bool start_isr;
} UART_Config;

void UART_init(UART_Config* uart);

void UART_setBufferOverrunProtection(UART_Config* uart, bool b);
void UART_waitForBuffer(UART_Config* uart);

void UART_sendBinaryString(UART_Config* uart, uint32_t number, DataType type);
void UART_sendValue(UART_Config* uart, char *variable, int32_t value, char *dimension);
void UART_sendByte(UART_Config* uart, uint8_t ch);
void UART_sendString(UART_Config* uart, char *str);
void UART_sendHexString(UART_Config* uart, uint32_t number);
void UART_sendNumber(UART_Config* uart, int32_t number);
void UART_sendUnsignedNumber(UART_Config* uart, uint32_t number);

bool UART_receiveByte(UART_Config* uart, uint8_t *ch);
bool UART_isByteAvailable(UART_Config* uart);


#endif // TERA_LIBRARY_UART_H_INCLUDED
