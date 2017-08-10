#include "led.h"

Led_t TERA_LED_init(GpioChannel_t* gpio_channel) {
  Led_t led = {
    .gpio_channel = gpio_channel,
  };
  return led;
}

void TERA_LED_on(Led_t* led) {
  TERA_GPIO_setBit(led->gpio_channel);
}

void TERA_LED_off(Led_t* led) {
  TERA_GPIO_clearBit(led->gpio_channel);
}

void TERA_LED_toggle(Led_t* led) {
  TERA_GPIO_toggle(led->gpio_channel);
}
