#ifndef TERALIB_DRIVERS_STM32F4_GPIO_DEF_H_INCLUDED
#define TERALIB_DRIVERS_STM32F4_GPIO_DEF_H_INCLUDED

#include <stm32f4xx_hal_gpio.h>

typedef GPIO_TypeDef Gpio_t;

typedef struct GpioChannel {
  Gpio_t* const gpio;
  uint8_t const pinnr;
}GpioChannel_t;

#endif // TERALIB_DRIVERS_STM32F4_GPIO_DEF_H_INCLUDED
