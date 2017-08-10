#pragma once

#include <config/gpio_def.h>

//void TERA_GPIO_setInput(Gpio_t* gpio, uint8_t bitnr);
//void TERA_GPIO_setOutput(Gpio_t* gpio, uint8_t bitnr);
//void TERA_GPIO_toggle(Gpio_t* gpio, uint8_t bitnr);
//void TERA_GPIO_setBit(Gpio_t* gpio, uint8_t bitnr);
//void TERA_GPIO_clearBit(Gpio_t* gpio, uint8_t bitnr);


void TERA_GPIO_toggle(GpioChannel_t* channel);
void TERA_GPIO_setBit(GpioChannel_t* channel);
void TERA_GPIO_clearBit(GpioChannel_t* channel);
