#include "gpio.h"

//void TERA_GPIO_toggle(Gpio_t* gpio, uint8_t bitnr) {
//  gpio->ODR ^= (1 << bitnr);
//}
//void TERA_GPIO_setBit(Gpio_t* gpio, uint8_t bitnr) {
//  gpio->BSRR = (1 << bitnr);
//}
//void TERA_GPIO_clearBit(Gpio_t* gpio, uint8_t bitnr) {
//  gpio->BSRR = ((1 << 16) << bitnr);
//}


void TERA_GPIO_toggle(GpioChannel_t* channel) {
  channel->gpio->ODR ^= (1 << channel->pinnr);
}
void TERA_GPIO_setBit(GpioChannel_t* channel) {
  channel->gpio->BSRR = (1 << channel->pinnr);
}
void TERA_GPIO_clearBit(GpioChannel_t* channel) {
  channel->gpio->BSRR = ((1 << 16) << channel->pinnr);
}
