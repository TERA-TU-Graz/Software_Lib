#ifdef STM32F40_41xxx

#include "gpio.h"

#include <stdbool.h>

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"


static void stdStructInit(GPIO_Pin* pin, GPIO_InitStruct* init,
        GPIO_InitTypeDef* stm_init) {
    switch (init->mode) {
    case GPIO_MODE_AF_OD:
        stm_init->GPIO_Mode = GPIO_Mode_AF;
        stm_init->GPIO_OType = GPIO_OType_OD;
        break;
    case GPIO_MODE_AF_PP:
        stm_init->GPIO_Mode = GPIO_Mode_AF;
        stm_init->GPIO_OType = GPIO_OType_PP;
        break;
    case GPIO_MODE_AN:
        stm_init->GPIO_Mode = GPIO_Mode_AN;
        break;
    case GPIO_MODE_IN:
        stm_init->GPIO_Mode = GPIO_Mode_IN;
        break;
    case GPIO_MODE_OUT_OD:
        stm_init->GPIO_Mode = GPIO_Mode_OUT;
        stm_init->GPIO_OType = GPIO_OType_OD;
        break;
    case GPIO_MODE_OUT_PP:
        stm_init->GPIO_Mode = GPIO_Mode_OUT;
        stm_init->GPIO_OType = GPIO_OType_PP;
        break;
    default:
        while(1);
        break;
    }

    stm_init->GPIO_Pin = 0x01 << pin->pin_nr;

    switch (init->pull_mode) {
    case GPIO_PULL_NONE:
        stm_init->GPIO_PuPd = GPIO_PuPd_NOPULL;
        break;
    case GPIO_PULL_DOWN:
        stm_init->GPIO_PuPd = GPIO_PuPd_DOWN;
        break;
    case GPIO_PULL_UP:
        stm_init->GPIO_PuPd = GPIO_PuPd_UP;
        break;
    default:
        while(1);
        break;
    }

    switch (init->speed) {
    case GPIO_SPEED_SLOWEST:
        stm_init->GPIO_Speed = GPIO_Speed_2MHz;
        break;
    case GPIO_SPEED_SLOW:
        stm_init->GPIO_Speed = GPIO_Speed_2MHz;
        break;
    case GPIO_SPEED_MEDIUM:
        stm_init->GPIO_Speed = GPIO_Speed_25MHz;
        break;
    case GPIO_SPEED_FAST:
        stm_init->GPIO_Speed = GPIO_Speed_50MHz;
        break;
    case GPIO_SPEED_FASTEST:
        stm_init->GPIO_Speed = GPIO_Speed_100MHz;
        break;
    default:
        while(1);
        break;
    }
}

void GPIO_init2(GPIO_Pin* pin, GPIO_InitStruct* init, unsigned bitval) {
    // Convert generic definition to STM std peripheral library init definition
    GPIO_InitTypeDef stm_init;
    stdStructInit(pin, init, &stm_init);

    pin->rcc_clock_cmd(pin->rcc_clock, ENABLE);
    GPIO_Init(pin->gpio, &stm_init);
}

void GPIO_init(GPIO_Pin* pin, GPIO_InitStruct* init) {
    // Convert generic definition to STM std peripheral library init definition
    GPIO_InitTypeDef stm_init;
    stdStructInit(pin, init, &stm_init);

    pin->rcc_clock_cmd(pin->rcc_clock, ENABLE);
    GPIO_Init(pin->gpio, &stm_init);
}

uint8_t GPIO_read(GPIO_Periph* gpio) {
    return GPIO_ReadInputData(gpio);
}

uint8_t GPIO_readBit(GPIO_Pin* pin) {
    return GPIO_ReadInputDataBit(pin->gpio, 0x01 << pin->pin_nr);
}

uint8_t GPIO_readOutput(GPIO_Periph* gpio) {
    return GPIO_ReadOutputData(gpio);
}

uint8_t GPIO_readOutputBit(GPIO_Pin* pin) {
    return GPIO_ReadOutputDataBit(pin->gpio, 0x01 << pin->pin_nr);
}

void GPIO_write(GPIO_Periph* gpio, unsigned mask) {
    GPIO_Write(gpio, mask);
}

void GPIO_writeBit(GPIO_Pin *pin, unsigned bitval) {
    GPIO_WriteBit(pin->gpio, 0x01 << pin->pin_nr, bitval);
}

void GPIO_setBit(GPIO_Pin* pin) {
    GPIO_writeBit(pin, 1);
}

void GPIO_clearBit(GPIO_Pin* pin) {
    GPIO_writeBit(pin, 0);
}

void GPIO_toggleBit(GPIO_Pin *pin) {
    uint8_t current = GPIO_ReadOutputDataBit(pin->gpio, 0x01 << pin->pin_nr);
    GPIO_WriteBit(pin->gpio, 0x01 << pin->pin_nr, !current);
}

#endif //STM32F40_41xxx
