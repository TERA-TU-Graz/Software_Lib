#ifndef TERA_LIBRARY_SYSTICK_H_INCLUDED
#define TERA_LIBRARY_SYSTICK_H_INCLUDED

#include <module/common/modules_def.h>

// periode in us
// maximaler reload value 16777215 = 0xFFFFFF
// entspricht bei 168MHz = 99864us = 99ms
// resolution: 6ns
// period in us
// sysclock/8,
// maximaler wert bei 168MHz = 798915us = 798ms
// resolution: 111ns
//
//  *   div This parameter can be one of the following values:
//  *     @arg SysTick_CLKSource_HCLK_Div8: AHB clock divided by 8 selected as SysTick clock source.
//  *     @arg SysTick_CLKSource_HCLK: AHB clock selected as SysTick clock source.
uint32_t SYSTICK_init(uint32_t div, uint32_t period);

// priority hat 4bits
// 0 - 16, 0 ist die höchste
void SYSTICK_setPriority(uint32_t priority);

void SYSTICK_enable();
void SYSTICK_disable();

#endif // TERA_LIBRARY_SYSTICK_H_INCLUDED
