#include "stm32f4tera_v2.h"

Led_Pin LED1 = {.pin =  PB5, .active_high = 1};
Led_Pin LED2 = {.pin =  PD2, .active_high = 1};
Led_Pin LED3 = {.pin = PC12, .active_high = 1};


void Board_init() {
    LED_init(&LED1);
    LED_init(&LED2);
    LED_init(&LED3);

    // init buttons, switches, 7-segment digits, lcds, ...
    // everything that belongs to the board goes here
}
