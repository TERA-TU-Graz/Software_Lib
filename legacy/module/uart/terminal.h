#ifndef TERA_LIBRARY_TERMINAL_H_INCLUDED
#define TERA_LIBRARY_TERMINAL_H_INCLUDED

#include <module/uart/uart_stm32f4.h>

// ascii codes of important key's
#define BACKSPACE   0x08
#define TABULATOR   0x09
#define LINEFEED    0x0A
#define ENTER       0x0D
#define ESCAPE      0x1B
#define SPACE       0x20
#define TILDE       0x7E

/*
#define F1         11
#define F2         12
#define F3         13
#define F4         14
#define F5         15
#define F6         17
#define F7         18
#define F8         19
#define F9         20
#define F10        21
#define F11        23
#define F12        24
*/

#define F1          0
#define F2          1
#define F3          2
#define F4          3
#define F5          4
#define F6          5
#define F7          6
#define F8          7
#define F9          8
#define F10         9
#define F11        10
#define F12        11


#define POS1       0x31
#define ENDE       0x34
#define PAGE_UP    0x35
#define PAGE_DOWN  0x36

#define CURSOR_UP     1
#define CURSOR_DOWN   2
#define CURSOR_LEFT   3
#define CURSOR_RIGHT  4

#define ATTRIB_NONE         '0' // Reset all attributes
#define ATTRIB_BOLD         '1' // Bright
#define ATTRIB_DIM          '2' // Dim
#define ATTRIB_UNDERLINE    '4' // Underscore
#define ATTRIB_BLINK        '5' // Blink
#define ATTRIB_REVERSED     '7' // Reverse
#define ATTRIB_CONCEALED    '8' // Hidden


// text colors
#define COL_FOREGROUND    '3'
#define COL_BACKGROUND    '4'

#define COL_BLACK         '0'
#define COL_RED           '1'
#define COL_GREEN         '2'
#define COL_YELLOW        '3'
#define COL_BLUE          '4'
#define COL_MAGENTA       '5'
#define COL_CYAN          '6'
#define COL_WHITE         '7'


void TERM_init(UART_Config *usart);

void TERM_putText(char *str);
void TERM_putChar(char ch);
void TERM_putNumber(int32_t number);
void TERM_putNumberAsBinary(uint8_t number);
void TERM_putUnsignedNumber(uint32_t number);
void TERM_putDouble(double d, uint8_t precision);
void TERM_putHex(uint8_t byte);

  // moves the cursor to specified position
  // (1, 1) is in the top left corner
  //
void TERM_setCursor(int row, int column);
void TERM_moveCursor(int direction, int count);
void TERM_setCursorHome();


void TERM_cls();
void TERM_eraseLine();
void TERM_eraseEndOfLine();
void TERM_eraseStartOfLine();
void TERM_eraseUp();
void TERM_eraseDown();
void TERM_eraseBlock(uint8_t n);

void TERM_setColor(uint8_t layer, uint8_t color);
void TERM_setAttribute(int attrib);

void TERM_scrolUp();
void TERM_scrolDown();
void TERM_scrollScreen();

#endif // TERA_LIBRARY_TERMINAL_H_INCLUDED
