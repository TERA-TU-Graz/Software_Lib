#ifdef STM32F40_41xxx

#include <module/uart/uart_stm32f4.h>
#include <stdbool.h>
#include <stdint.h>


const uint8_t hex_convert_str_[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};


void UART_init(UART_Config* uart) {
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // usart interrupt
  NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  uart->buffer_overrun_protection = false;

  if (uart->usart==USART1 || uart->usart==USART6)
    RCC_APB2PeriphClockCmd(uart->clock, ENABLE);
  else
    RCC_APB1PeriphClockCmd(uart->clock, ENABLE);

  GPIO_InitStruct gpio_init;
  gpio_init.mode = GPIO_MODE_AF_PP;
  gpio_init.speed = GPIO_SPEED_FASTEST;
  gpio_init.pull_mode = GPIO_PULL_DOWN;
  GPIO_init(&uart->pins[0], &gpio_init);
  GPIO_init(&uart->pins[1], &gpio_init);

  GPIO_PinAFConfig(uart->pins[0].gpio, uart->pins[0].gpio_pinsource, uart->af);
  GPIO_PinAFConfig(uart->pins[1].gpio, uart->pins[1].gpio_pinsource, uart->af);

  // USART configuration
  USART_InitStructure.USART_BaudRate = uart->baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(uart->usart, &USART_InitStructure);

  uart->start_isr = true;

  USART_ITConfig(uart->usart, USART_IT_RXNE, ENABLE); // enable USARTz Receive and Transmit interrupts
  USART_ITConfig(uart->usart, USART_IT_TC, ENABLE);

  USART_Cmd(uart->usart, ENABLE);
}

void UART_setBufferOverrunProtection(UART_Config* uart, bool b) {
  uart->buffer_overrun_protection = b;
}

void UART_waitForBuffer(UART_Config* uart) {
  while (!FIFO_QUEUE_isEmpty(uart->txQueue));
}

void UART_sendString(UART_Config* uart, char *str) {

  while (*str!=0) {
    UART_sendByte(uart, *str);
    ++str;
  }
}

void UART_sendBinaryString(UART_Config* uart, uint32_t number, DataType type)
{
    uint8_t counter = 1;
    number <<= (32-type);
    for (counter = 1; counter <= type; counter++)
    {
      // print the MSB by masking and moving it to the position of the LSB
      UART_sendByte(uart, '0'+(uint32_t)((number & 0x80000000) >> (32 - 1)));
      // move the whole bit sting one bit left
      number <<= 1;
      // print a " " after each 8 bits
      if (!(counter % 8))
        UART_sendByte(uart, ' ');
    }
}

void UART_sendValue(UART_Config* uart, char *variable, int32_t value, char *dimension)
{
  UART_sendString(uart, variable);
  UART_sendString(uart, ": ");
  UART_sendNumber(uart, value);
  UART_sendByte(uart, ' ');
  UART_sendString(uart, dimension);
}


void UART_sendByte(UART_Config* uart, uint8_t ch) {
  bool sent;
  NVIC_DisableIRQ(uart->irq);

  if (uart->buffer_overrun_protection==true) { //enable buffer overrun protection
    sent = FIFO_append2(&uart->txQueue, ch);
    while (!sent) { // not in buffer
      NVIC_EnableIRQ(uart->irq);
      while (!FIFO_QUEUE_isEmpty(uart->txQueue));   // wait
      NVIC_DisableIRQ(uart->irq);
      sent = FIFO_append2(&uart->txQueue, ch);   // try again
    }
    if (uart->start_isr) {
      uart->start_isr = false;
      USART_ITConfig(uart->usart, USART_IT_TXE, ENABLE);
    }
  } else {
    FIFO_append(&uart->txQueue, ch);
    if (uart->start_isr) {
      uart->start_isr = false;
      USART_ITConfig(uart->usart, USART_IT_TXE, ENABLE);
    }
  }

  NVIC_EnableIRQ(uart->irq);
}

bool UART_receiveByte(UART_Config* uart, uint8_t *ch) {
  bool ret = false;
  NVIC_DisableIRQ(uart->irq);

  ret = FIFO_fetch(&uart->rxQueue, ch);

  NVIC_EnableIRQ(uart->irq);
  return ret;
}

/**
 * returns true if a byte is available
 */
bool UART_isByteAvailable(UART_Config* uart) {
  return !FIFO_QUEUE_isEmpty(uart->rxQueue);
}


void UART_sendHexString(UART_Config* uart, uint32_t number) {
  uint8_t i, leading;
  uint32_t mask;
  uint32_t tmp;

  mask = 0xF0000000;
  leading = 1;
  UART_sendString(uart, "0x");
  for (i=0;i<8;++i) {
    tmp = number & mask;
    mask >>= 4;

    if (tmp==0 && leading)
      continue;
    leading = 0;

    tmp >>= (28-i*4);
    UART_sendByte(uart, hex_convert_str_[tmp&0xF]);
  }
}

void UART_sendNumber(UART_Config* uart, int32_t number) {
  if (number<0) {
    UART_sendByte(uart, '-');
    number = -number;
  }
  UART_sendUnsignedNumber(uart, number);
}

void UART_sendUnsignedNumber(UART_Config* uart, uint32_t number) {
  uint8_t tmp;
  uint32_t n;
  uint32_t reverse;

  if (number==0) {
    UART_sendByte(uart, '0');
    return;
  }

  n = 0;
  reverse = number;
  while ((reverse%10)==0) {
    ++n;
    reverse = reverse / 10;
  }

  reverse = 0;
  while (number>0) {
    reverse = reverse * 10 + number % 10;
    number = number / 10;
  }

  while (reverse>0) {
    tmp = reverse % 10;

    UART_sendByte(uart, tmp | 0x30);

    reverse = reverse / 10;
  }

  while (n>0) {
    UART_sendByte(uart, '0');
    --n;
  }
}

#endif // STM32F40_41xxx
