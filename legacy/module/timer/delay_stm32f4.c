#ifdef STM32F40_41xxx

#include "delay.h"
#include <stdint.h>

// TODO DELAY - make this timer default, but give option to choose a different one... then rewrite the whole module...
#define DELAY_TIMER                           TIM7
#define DELAY_TIMER_CLK                       RCC_APB1Periph_TIM7
#define DELAY_TIMER_UP_IRQ                    TIM7_IRQn
#define DELAY_TIMER_PREEMTION_PRIORITY        3
#define DELAY_TIMER_SUP_PRIORITY              3
#define DELAY_IRQ_updateHandler               TIM7_IRQHandler

static volatile uint32_t delay_timing_;


//------------------------------------------------------------------------------
/**
 * used for timer update irq handler
 */
void DELAY_IRQ_updateHandler() {
    DELAY_TIMER->SR = (uint16_t) ~TIM_IT_Update;

    if (delay_timing_ != 0)
        --delay_timing_;
    else
        DELAY_TIMER->CR1 &= ~TIM_CR1_CEN;
}

//------------------------------------------------------------------------------
void DELAY_init() {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DELAY_TIMER_UP_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
            DELAY_TIMER_PREEMTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = DELAY_TIMER_SUP_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(DELAY_TIMER_CLK, ENABLE);

    TIM_DeInit(DELAY_TIMER);

    // getaktet mit 84Mhz
    TIM_TimeBaseStructure.TIM_Prescaler = 83; // timer clock 1Mhz
    TIM_TimeBaseStructure.TIM_Period = 99;   // period 100us
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(DELAY_TIMER, &TIM_TimeBaseStructure);

    TIM_ClearITPendingBit(DELAY_TIMER, TIM_IT_Update);
    TIM_ITConfig(DELAY_TIMER, TIM_IT_Update, ENABLE);
}

/**
 * specifies the delay time length, in 100us
 */
void DELAY_waitMicro(uint32_t delay_100us) {
    delay_timing_ = delay_100us;
    DELAY_TIMER->CR1 |= TIM_CR1_CEN;
    while (delay_timing_ != 0)
        ;
}

/**
 * specifies the delay time length, in 1 ms.
 */
void DELAY_wait(uint32_t delay_ms) {
    DELAY_waitMicro(10 * delay_ms);
}

#endif // STM32F40_41xxx
