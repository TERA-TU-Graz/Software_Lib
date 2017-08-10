#pragma once

#include <gpio.h>

typedef struct Led {
  GpioChannel_t* gpio_channel;
} Led_t;

Led_t TERA_LED_init(GpioChannel_t* gpio_channel);
void TERA_LED_on(Led_t* led);
void TERA_LED_off(Led_t* led);
void TERA_LED_toggle(Led_t* led);

