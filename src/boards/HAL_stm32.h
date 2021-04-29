#pragma once

#include <stdint.h>
#include <HardwareTimer.h>
#include "macros.h"

#define NUM_HARDWARE_TIMERS 2

#define CPU_32_BIT

#define hal_timer_t uint32_t
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFF // Timers can be 16 or 32 bit

#define FORCE_INLINE __attribute__((always_inline)) inline

#define HAL_STEP_TIMER_ISR void Step_Handler(HardwareTimer *htim)

#define CRITICAL_SECTION_START()  uint32_t primaskV = __get_PRIMASK(); __disable_irq()
#define CRITICAL_SECTION_END()    if(!primaskV) __enable_irq()
#define ENABLE_ISRS() __enable_irq()
#define DISABLE_ISRS() __disable_irq()

#define __TIMER_DEV(X) TIM##X
#define _TIMER_DEV(X) __TIMER_DEV(X)
#define STEP_TIMER_DEV _TIMER_DEV(STEP_TIMER)

#define __TIMER_IRQ_NAME(X) TIM##X##_IRQn
#define _TIMER_IRQ_NAME(X) __TIMER_IRQ_NAME(X)
#define STEP_TIMER_IRQ_NAME _TIMER_IRQ_NAME(STEP_TIMER)

#define STEP_TIMER_IRQ_PRIO_DEFAULT      2
#define STEP_TIMER_IRQ_PRIO STEP_TIMER_IRQ_PRIO_DEFAULT

#define STEP_TIMER_NUM 0

extern void Step_Handler(HardwareTimer *htim);

extern HardwareTimer *timer_instance[NUM_HARDWARE_TIMERS];

FORCE_INLINE void HAL_timer_start(const uint8_t timerIndex) {
  timer_instance[timerIndex] = new HardwareTimer(STEP_TIMER_DEV);
  timer_instance[timerIndex]->setPrescaleFactor(STEPPER_TIMER_PRESCALE); //the -1 is done internally
  timer_instance[timerIndex]->setOverflow(_MIN(hal_timer_t(HAL_TIMER_TYPE_MAX), (HAL_TIMER_RATE) / (STEPPER_TIMER_PRESCALE) /* /frequency */), TICK_FORMAT);
  timer_instance[timerIndex]->attachInterrupt(Step_Handler);
  timer_instance[timerIndex]->resume(); // First call to resume() MUST follow the attachInterrupt()
  HAL_NVIC_SetPriority(STEP_TIMER_IRQ_NAME, STEP_TIMER_IRQ_PRIO, 0);
}

FORCE_INLINE bool HAL_timer_initialized(const uint8_t timerIndex) {
  return timer_instance[timerIndex] != NULL;
}

// NOTE: Method name may be misleading.
// STM32 has an Auto-Reload Register (ARR) as opposed to a "compare" register
FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timerIndex,hal_timer_t value) {
  if (HAL_timer_initialized(timerIndex)) {
    timer_instance[timerIndex]->setOverflow(value + 1, TICK_FORMAT); // Value decremented by setOverflow()
    // wiki: "force all registers (Autoreload, prescaler, compare) to be taken into account"
    // So, if the new overflow value is less than the count it will trigger a rollover interrupt.
    if (value < timer_instance[timerIndex]->getCount())  // Added 'if' here because reports say it won't boot without it
      timer_instance[timerIndex]->refresh();
  }
}

FORCE_INLINE hal_timer_t HAL_timer_get_count(const uint8_t timerIndex) {
  return HAL_timer_initialized(timerIndex) ? timer_instance[timerIndex]->getCount() : 0;
}