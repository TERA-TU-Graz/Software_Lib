#ifndef TERA_LIBRARY_TIMER_H_INCLUDED
#define TERA_LIBRARY_TIMER_H_INCLUDED

#include <module/common/modules_def.h>
#include <module/gpio/gpio.h>

//--------------------------------------------------------------------------------------------------
// Initializes a periodic occurring interrupt on the specified TIMx_IRQn.
// NOTE: Make sure to set irqn_ to the update value when using TIM1 or 8!
// @param Timer_Periph* tim    The structure containing all relevant initialization data, watch out for IRQn on TIM1 and 8!
// @param uint32_t frequency   The frequency of the interrupt event
// @param uint8_t priority     The priority of the interrupt request
// @param uint8_t subpriority  The subpriority of the interrupt request
//
void TIMER_initPeriodicInterrupt(Timer_Periph* tim, uint32_t frequency,
        uint8_t priority, uint8_t subpriority);

//------------------------------------------------------------------------------
// Initializes a PWM output signal with the specified frequency on the
// specified Timer/GPIO Pin. Initial duty cycle is 0.
// @param Timer_Pin* timer      The timer pin to use
// @param uint32_t frequency    The frequency of the PWM
// @param uint32_t duty_factor  Duty factor of the PWM. Is set to 0 if not given
//
void TIMER_initPWMOutput(TimerPin* pin, uint32_t frequency);
void TIMER_initPWMOutput2(TimerPin* pin, uint32_t frequency, float duty_factor);

//------------------------------------------------------------------------------
// Changes the duty cycle of a previously initialized PWM output.
// @param Timer_Periph* tim  Timer peripheral
// @param uint8_t percent    The new duty cycle in percent
//
void TIMER_setPWMOutputDutyCycle(Timer_Periph* tim, float percent);

//------------------------------------------------------------------------------
// Gets frequency and duty factor of the PWM output signal
// @param const Timer_Periph* timer A timer/channel which produces a PWM signal
// @param float* frequency          Address where the frequency will be stored
// @param float* duty_cycle         Address where the duty_factor will be stored
//
void TIMER_getPWMOutputValues(Timer_Periph* timer, float* frequency,
        float* duty_cycle);

//------------------------------------------------------------------------------
// The PWM input feature needs 2 capture compare registers (CCR). By default,
// CCRx is taken as the frequency capturing register, where x is the provided
// channel number. The second CC register is determined as follows:
// Channel 1 -> CC1 (frequency/master), CC2 (duty factor/slave)
// Channel 2 -> CC2 (frequency/master), CC1 (duty factor/slave)
// Channel 3 -> CC3 (frequency/master), CC4 (duty factor/slave)
// Channel 4 -> CC4 (frequency/master), CC3 (duty factor/slave)
//
// @param const Timer_Pin* timer  The pin to use
//
void TIMER_initPWMInput(TimerPin* timer);

//------------------------------------------------------------------------------
// Computes frequency and duty factor of the PWM signal on the specified
// timer/channel/port and stores them at the given addresses.
//
// @param const Timer_Periph* timer  A timer channel which monitors a PWM signal
// @param float* frequency           Address where frequency will be stored
// @param float* duty_cycle          Address where duty_factor will be stored
//
void TIMER_getPWMInputValues(Timer_Periph* timer, float* frequency,
        float* duty_cycle);

//------------------------------------------------------------------------------
// ATTENTION! Not tested yet!
__attribute__((warning("untested function")))
void TIMER_initInputCapture(TimerPin* timer, uint8_t irq_priority,
        uint8_t irq_subpriority);

#endif // TERA_LIBRARY_TIMER_H_INCLUDED
