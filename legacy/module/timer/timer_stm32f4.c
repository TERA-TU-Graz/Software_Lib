#ifdef STM32F40_41xxx

#include "timer.h"
#include <stdint.h>

#include "stm32f4xx_rcc.h"


#define IS_APB1_PERIPH(clock) (\
clock & \
( RCC_APB1Periph_TIM2  \
| RCC_APB1Periph_TIM3  \
| RCC_APB1Periph_TIM4  \
| RCC_APB1Periph_TIM5  \
| RCC_APB1Periph_TIM6  \
| RCC_APB1Periph_TIM7  \
| RCC_APB1Periph_TIM12 \
| RCC_APB1Periph_TIM13 \
| RCC_APB1Periph_TIM14 \
))

#define IS_APB2_PERIPH(clock) (\
clock & \
( RCC_APB2Periph_TIM1  \
| RCC_APB2Periph_TIM8  \
| RCC_APB2Periph_TIM9  \
| RCC_APB2Periph_TIM10 \
| RCC_APB2Periph_TIM11 \
))

static uint32_t getTimerFrequency(const Timer_Periph* timer) {
    //is there an easier/safer way?

    RCC_ClocksTypeDef rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);

    uint32_t frequency = 0;
    if (IS_APB1_PERIPH(timer->rcc_clock)) {
        frequency =
                rcc_clocks.PCLK1_Frequency / 4 <= rcc_clocks.HCLK_Frequency ?
                        rcc_clocks.PCLK1_Frequency * 2 :
                        rcc_clocks.PCLK1_Frequency;
    } else if (IS_APB2_PERIPH(timer->rcc_clock)) {
        frequency =
                rcc_clocks.PCLK2_Frequency / 2 <= rcc_clocks.HCLK_Frequency ?
                        rcc_clocks.PCLK2_Frequency * 2 :
                        rcc_clocks.PCLK1_Frequency;
    } else {
        while(1);
    }

    return frequency;
}

static void setupTimeBase(const Timer_Periph* timer, uint32_t frequency) {
    uint32_t timer_periph_freq = getTimerFrequency(timer);
    if (frequency > timer_periph_freq / 2)
        while (1)
            ; //error, frequency too high

    uint32_t max_period_reg_value =
            (timer->base_addr == TIM2 || timer->base_addr == TIM5) ? 0xFFFFFFFF : 0xFFFF;
    uint32_t max_prescale_reg_value = 0x0000FFFF;

    uint32_t theoretical_period = (timer_periph_freq / frequency) - 1;
    uint32_t theoretical_prescaler = theoretical_period / max_period_reg_value;
    uint8_t clock_divisior = theoretical_prescaler / max_prescale_reg_value;

    if (clock_divisior && (timer->base_addr == TIM6 || timer->base_addr == TIM7)) {
        while (1)
            ; //error, too large period - Basic timers can't do clock division
    }

    uint16_t clock_divisior_def = TIM_CKD_DIV1;
    switch (clock_divisior) {
    case 0:
        clock_divisior = 1;
        clock_divisior_def = TIM_CKD_DIV1;
        break;
    case 1:
        clock_divisior = 2;
        clock_divisior_def = TIM_CKD_DIV2;
        break;
    case 2:
    case 3:
        clock_divisior = 4;
        clock_divisior_def = TIM_CKD_DIV4;
        break;
    default:
        while (1)
            ; //error, too large period
    }

    uint32_t prescaler = theoretical_prescaler / clock_divisior;
    uint32_t period = (theoretical_period + 1) / (prescaler + 1) - 1;

// Time base configuration
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = clock_divisior_def;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer->base_addr, &TIM_TimeBaseStructure);
}

void TIMER_initPeriodicInterrupt(Timer_Periph* timer, uint32_t frequency,
        uint8_t irq_priority, uint8_t irq_subpriority) {
    timer->rcc_clock_cmd(timer->rcc_clock, ENABLE);

//Enable periodic interrupt
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = timer->irq_nr;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = irq_priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = irq_subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    setupTimeBase(timer, frequency);

    TIM_ITConfig(timer->base_addr, TIM_IT_Update, ENABLE);

// enable counter
    TIM_Cmd(timer->base_addr, ENABLE);
}

void TIMER_initPWMOutput(TimerPin* pin, uint32_t frequency) {
    TIMER_initPWMOutput2(pin, frequency, 0);
}

void TIMER_initPWMOutput2(TimerPin* pin, uint32_t frequency,
        float duty_factor) {
//  TIM_Cmd(timer->timx_, DISABLE);

    pin->timer.rcc_clock_cmd(pin->timer.rcc_clock, ENABLE);
    pin->gpio_pin.rcc_clock_cmd(pin->gpio_pin.rcc_clock, ENABLE);

    GPIO_InitStruct gpio_init;
    gpio_init.mode = GPIO_MODE_AF_PP;
    gpio_init.pull_mode = GPIO_PULL_NONE;
    gpio_init.speed = GPIO_SPEED_FASTEST;
    GPIO_init(&pin->gpio_pin, &gpio_init);

    GPIO_PinAFConfig(pin->gpio_pin.gpio, pin->gpio_pin.gpio_pinsource, pin->gpio_af_tim);

    setupTimeBase(&pin->timer, frequency);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pin->timer.base_addr->ARR / (float) 100.0
            * duty_factor;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    switch (pin->timer.channel) {
    case TIM_Channel_1:
        TIM_OC1Init(pin->timer.base_addr, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(pin->timer.base_addr, TIM_OCPreload_Enable);
        break;
    case TIM_Channel_2:
        TIM_OC2Init(pin->timer.base_addr, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(pin->timer.base_addr, TIM_OCPreload_Enable);
        break;
    case TIM_Channel_3:
        TIM_OC3Init(pin->timer.base_addr, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(pin->timer.base_addr, TIM_OCPreload_Enable);
        break;
    case TIM_Channel_4:
        TIM_OC4Init(pin->timer.base_addr, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(pin->timer.base_addr, TIM_OCPreload_Enable);
        break;
    default:
        while (1)
            ; //error
    }

    TIM_ARRPreloadConfig(pin->timer.base_addr, ENABLE);
    TIM_Cmd(pin->timer.base_addr, ENABLE);
}

void TIMER_setPWMOutputDutyCycle(Timer_Periph* timer, float percent) {
    switch (timer->channel) {
    case TIM_Channel_1:
        timer->base_addr->CCR1 = timer->base_addr->ARR / (float) 100 * percent;
        break;
    case TIM_Channel_2:
        timer->base_addr->CCR2 = timer->base_addr->ARR / (float) 100 * percent;
        break;
    case TIM_Channel_3:
        timer->base_addr->CCR3 = timer->base_addr->ARR / (float) 100 * percent;
        break;
    case TIM_Channel_4:
        timer->base_addr->CCR4 = timer->base_addr->ARR / (float) 100 * percent;
        break;
    default:
        while (1)
            ; //error
    }
}

void TIMER_getPWMOutputValues(Timer_Periph* timer, float* frequency,
        float* duty_cycle) {
    uint32_t ccr = 0;
    uint32_t arr = timer->base_addr->ARR;
    switch (timer->channel) {
    case TIM_Channel_1:
        ccr = timer->base_addr->CCR1;
        break;
    case TIM_Channel_2:
        ccr = timer->base_addr->CCR2;
        break;
    case TIM_Channel_3:
        ccr = timer->base_addr->CCR3;
        break;
    case TIM_Channel_4:
        ccr = timer->base_addr->CCR4;
        break;
    default:
        while (1)
            ; //error
    }

    *frequency = getTimerFrequency(timer)
            / (float) ((timer->base_addr->PSC + 1) * arr);
    *duty_cycle = (float) ccr / (float) arr * 100;
}

void TIMER_initPWMInput(TimerPin* pin) {
    if (pin->timer.channel != TIM_Channel_1
            && pin->timer.channel != TIM_Channel_2)
        while (1)
            ; //error, only possible with channel 1 and 2

    pin->timer.rcc_clock_cmd(pin->timer.rcc_clock, ENABLE);
    pin->gpio_pin.rcc_clock_cmd(pin->gpio_pin.rcc_clock, ENABLE);

    GPIO_InitStruct gpio_init;
    gpio_init.mode = GPIO_MODE_AF_PP;
    gpio_init.pull_mode = GPIO_PULL_NONE;
    gpio_init.speed = GPIO_SPEED_FASTEST; //probably not needed

    GPIO_init(&pin->gpio_pin, &gpio_init);

    GPIO_PinAFConfig(pin->gpio_pin.gpio, pin->gpio_pin.gpio_pinsource, pin->gpio_af_tim);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = pin->timer.channel;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x3;
    TIM_PWMIConfig(pin->timer.base_addr, &TIM_ICInitStructure);

    if (pin->timer.channel == TIM_Channel_1)
        TIM_SelectInputTrigger(pin->timer.base_addr, TIM_TS_TI1FP1);
    else
        TIM_SelectInputTrigger(pin->timer.base_addr, TIM_TS_TI2FP2);

// Select the slave Mode: Reset Mode
    TIM_SelectSlaveMode(pin->timer.base_addr, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(pin->timer.base_addr, TIM_MasterSlaveMode_Enable);

// TIM enable counter
    TIM_Cmd(pin->timer.base_addr, ENABLE);
}

void TIMER_getPWMInputValues(Timer_Periph* timer, float* frequency,
        float* duty_cycle) {
    float ccr1 = TIM_GetCapture1(timer->base_addr);
    float ccr2 = TIM_GetCapture2(timer->base_addr);
    if (timer->channel == TIM_Channel_1 && ccr1) {
        *frequency = getTimerFrequency(timer) / ccr1;
        *duty_cycle = ccr2 * 100 / ccr1;
    } else if (timer->channel == TIM_Channel_2 && ccr2) {
        *frequency = getTimerFrequency(timer) / ccr2;
        *duty_cycle = ccr1 * 100 / ccr2;
    }
}

void TIMER_initInputCapture(TimerPin* pin, uint8_t irq_priority,
        uint8_t irq_subpriority) {
    pin->timer.rcc_clock_cmd(pin->timer.rcc_clock, ENABLE);
    pin->gpio_pin.rcc_clock_cmd(pin->gpio_pin.rcc_clock, ENABLE);

    GPIO_InitStruct gpio_init;
    gpio_init.mode      = GPIO_MODE_AF_PP;
    gpio_init.pull_mode = GPIO_PULL_NONE;
    gpio_init.speed     = GPIO_SPEED_FASTEST; //probably not needed

    GPIO_init(&pin->gpio_pin, &gpio_init);
    GPIO_PinAFConfig(pin->gpio_pin.gpio, pin->gpio_pin.gpio_pinsource, pin->gpio_af_tim);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = pin->timer.irq_nr;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = irq_priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = irq_subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = pin->timer.channel;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x3;
    TIM_ICInit(pin->timer.base_addr, &TIM_ICInitStructure);

    if (pin->timer.channel == TIM_Channel_1)
        TIM_SelectInputTrigger(pin->timer.base_addr, TIM_TS_TI1FP1);
    else
        TIM_SelectInputTrigger(pin->timer.base_addr, TIM_TS_TI2FP2);

// TIM enable counter
    TIM_Cmd(pin->timer.base_addr, ENABLE);

// Enable the CC Interrupt Request
    if (pin->timer.channel == TIM_Channel_1)
        TIM_ITConfig(pin->timer.base_addr, TIM_IT_CC1, ENABLE);
    else
        TIM_ITConfig(pin->timer.base_addr, TIM_IT_CC2, ENABLE);
}

#endif // STM32F40_41xxx
