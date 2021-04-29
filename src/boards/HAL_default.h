#pragma once

#include <Arduino.h>

#ifndef CLOCK_FREQ
#  define CLOCK_FREQ (16000000L)
#endif

#define CPU_16_BIT

#define hal_timer_t uint32_t
#define HAL_TIMER_TYPE_MAX 0xFFFF

// TODO: get rid of manual rate/prescale/ticks/cycles taken for procedures in stepper.cpp
#define STEPPER_TIMER_RATE 2000000 // 2 Mhz
#define STEPPER_TIMER_PRESCALE ((HAL_TIMER_RATE)/(STEPPER_TIMER_RATE))
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START()  unsigned char _sreg = SREG; cli()
  #define CRITICAL_SECTION_END()    SREG = _sreg
#endif
#define ENABLE_ISRS()  sei()
#define DISABLE_ISRS() cli()

#define FORCE_INLINE __attribute__((always_inline)) inline

#define STEP_TIMER_NUM 0 

FORCE_INLINE void HAL_timer_start(const uint8_t timerIndex) {
  Serial.println(F("timer start"));
#ifndef DEBUG_STEPPING
  // disable global interrupts
  DISABLE_ISRS();

  // set entire TCCR1A register to 0
  TCCR1A = 0;
  // set the overflow clock to 0
  TCNT1 = 0;
  // set compare match register to desired timer count
  OCR1A = 2000;  // 1ms
  // turn on CTC mode
  TCCR1B = (1 << WGM12);
  // Set 8x prescaler
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  ENABLE_ISRS();
#endif  // DEBUG_STEPPING
}

FORCE_INLINE bool HAL_timer_initialized(const uint8_t timerIndex) {
    return true;
}

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timerIndex,hal_timer_t value) {
  OCR1A = (value);
}

FORCE_INLINE hal_timer_t HAL_timer_get_count(const uint8_t timerIndex) {
  return OCR1A;
}