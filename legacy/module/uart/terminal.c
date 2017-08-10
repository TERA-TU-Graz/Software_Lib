#include <module/uart/debug_uart_stm32f4.h>
#include "terminal.h"
#include <stdint.h>

static UART_Config *terminal_uart_;

void TERM_init(UART_Config *usart) {
  terminal_uart_ = usart;
}

void TERM_putText(char *str) {
  UART_sendString(terminal_uart_, str);
}

void TERM_putChar(char ch) {
  UART_sendByte(terminal_uart_, ch);
}

void TERM_putNumber(int32_t number) {
  UART_sendNumber(terminal_uart_, number);
}
void TERM_putNumberAsBinary(uint8_t number) {
  uint8_t mask = 0x80;
  char erg[] = {'0','0','0','0',' ','0','0','0','0','\0'};
  uint8_t i;
  for(i = 0; i < 9; i++)
  {
    if(i==4)
      continue;

    if(number & mask)
      erg[i] = '1';
    mask >>= 1;
  }
  TERM_putText(erg);
}

void TERM_putUnsignedNumber(uint32_t number) {
  UART_sendUnsignedNumber(terminal_uart_, number);
}

void TERM_putDouble(double d, uint8_t precision) {
//  if(precision == 0)
//    return;

  TERM_putNumber((int32_t)d);
  TERM_putChar(',');

  int64_t temp = d;
  uint64_t j = 10;

  uint8_t i = 0;
  for(; i < precision && i < 17; i++, j*=10)
  {
    if(d >= 0)
      TERM_putUnsignedNumber((uint64_t)j*d-10*temp);
    else
      TERM_putUnsignedNumber((uint64_t)(j*d*(-1))-10*temp*-1);
    temp = ((uint64_t)j*d);
  }
  //(recursive) rounding without math library would be nice...
}

void TERM_putHex(uint8_t byte) {
  char letter[] = "0123456789ABCDEF";

  int i = 1;
  for(; i >= 0 ; i--, byte = byte >> 4){
    unsigned char half_byte = byte & 0xF;
    char ch = letter[half_byte];
    TERM_putChar(ch);
  }
}

void TERM_setCursor(int row, int column) {
  UART_sendByte(terminal_uart_, ESCAPE);
  UART_sendByte(terminal_uart_, '[');
  UART_sendNumber(terminal_uart_, row);
  UART_sendByte(terminal_uart_, ';');
  UART_sendNumber(terminal_uart_, column);
  UART_sendByte(terminal_uart_, 'H');
}

void TERM_moveCursor(int direction, int count) {
  uint8_t ch;

  TERM_putChar(ESCAPE);
  TERM_putChar('[');
  if (count>1)
    UART_sendNumber(terminal_uart_, count);

  switch (direction) {
  case CURSOR_UP:
    ch = 'A';
    break;
  case CURSOR_DOWN:
    ch = 'B';
    break;
  case CURSOR_LEFT:
    ch = 'D';
    break;
  case CURSOR_RIGHT:
    ch = 'C';
    break;
  default:
    ch = 'B';
    break;
  }
  TERM_putChar(ch);
}

void TERM_setCursorHome() {
  TERM_putChar(ESCAPE);
  TERM_putText("[H");
}

void TERM_cls() {
  TERM_putChar(ESCAPE);
  TERM_putText("[2J");
  TERM_putChar(ESCAPE);
  TERM_putText("[H");
}

void TERM_eraseBlock(uint8_t n)
{
  uint8_t i = 0;
  for (i = 0; i < n; i++)
    TERM_putChar(' ');
  TERM_moveCursor(CURSOR_LEFT,n);
}

void TERM_eraseLine() {
  TERM_putChar(ESCAPE);
  TERM_putText("[2K");
}

void TERM_eraseEndOfLine() {
  TERM_putChar(ESCAPE);
  TERM_putText("[K");
}

void TERM_eraseStartOfLine() {
  TERM_putChar(ESCAPE);
  TERM_putText("[1K");
}

void TERM_eraseUp() {
  TERM_putChar(ESCAPE);
  TERM_putText("[1J");
}

void TERM_eraseDown() {
  TERM_putChar(ESCAPE);
  TERM_putText("[J");
}

void TERM_scrolUp() {
  TERM_putChar(ESCAPE);
  TERM_putText("[M");
}

void TERM_scrolDown() {
  TERM_putChar(ESCAPE);
  TERM_putText("[D");
}

void TERM_scrollScreen() {
  TERM_putChar(ESCAPE);
  TERM_putText("[r");
}

void TERM_setColor(uint8_t layer, uint8_t color) {
  TERM_putChar(ESCAPE);
  TERM_putChar('[');

  TERM_putChar(layer);
  TERM_putChar(color);
  TERM_putChar('m');
}

void TERM_setAttribute(int attrib) {
  TERM_putChar(ESCAPE);
  TERM_putChar('[');
  TERM_putChar(attrib);
  TERM_putChar('m');
}
