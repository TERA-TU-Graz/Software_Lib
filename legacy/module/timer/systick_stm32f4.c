#ifdef STM32F40_41xxx

#include "systick.h"
#include <stdint.h>


// div: AHB clock
//  * SysTick_CLKSource_HCLK_Div8:
//  * SysTick_CLKSource_HCLK:
// period [us]
uint32_t SYSTICK_init(uint32_t div, uint32_t period) {
  RCC_ClocksTypeDef rccClocks;
  uint32_t n;

  RCC_GetClocksFreq(&rccClocks);
  n = (rccClocks.SYSCLK_Frequency / 1000000UL) * period;

  SysTick_CLKSourceConfig(div);

  // needs number of ticks between interrupts
  return SysTick_Config(n);
}

void SYSTICK_enable() {
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void SYSTICK_disable() {
  SysTick->CTRL &= (~(SysTick_CTRL_ENABLE_Msk));
}

// priority: 0-15
void SYSTICK_setPriority(uint32_t priority) {
  NVIC_SetPriority(SysTick_IRQn, priority);
}

#endif // STM32F40_41xxx
