#include "led.h"
#include "module/gpio/gpio.h"

void LED_init(Led_Pin* led) {
    LED_init2(led, 1);
}

void LED_init2(Led_Pin* led, unsigned led_cnt) {
    GPIO_InitStruct init;
    init.mode      = GPIO_MODE_OUT_PP;
    init.pull_mode = GPIO_PULL_NONE;
    init.speed     = GPIO_SPEED_SLOWEST;

    uint8_t i = 0;
    for (; i < led_cnt; ++i){
        GPIO_init(&(led + i)->pin, &init);
        GPIO_writeBit(&(led + i)->pin, !(led + i)->active_high);
    }
}

void LED_on(Led_Pin* led) {
    GPIO_writeBit(&led->pin, led->active_high);
}

void LED_off(Led_Pin* led) {
    GPIO_writeBit(&led->pin, !led->active_high);
}

void LED_toggle(Led_Pin* led) {
    GPIO_toggleBit(&led->pin);
}

void LED_set(Led_Pin* led, uint8_t val) {
    GPIO_writeBit(&led->pin, val == led->active_high);
}
