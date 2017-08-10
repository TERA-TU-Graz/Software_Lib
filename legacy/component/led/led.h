#ifndef TERA_LIBRARY_LED_H_INCLUDED
#define TERA_LIBRARY_LED_H_INCLUDED

#include "module/common/modules_def.h"

typedef struct {
    GPIO_Pin pin;
    unsigned active_high;
} Led_Pin;

void LED_init(Led_Pin* led);
void LED_init2(Led_Pin* led, unsigned led_cnt);

void LED_on(Led_Pin* led);
void LED_off(Led_Pin* led);
void LED_toggle(Led_Pin* led);
void LED_set(Led_Pin* led, uint8_t val);

#endif // TERA_LIBRARY_LED_H_INCLUDED
