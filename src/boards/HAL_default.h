#pragma once

#ifdef TARGET_DEFAULT

#include <Arduino.h>
#include "macros.h"

#define CPU_16_BIT

#define hal_timer_t uint16_t
#define HAL_TIMER_TYPE_MAX 0xFFFF
#define HAL_TIMER_RATE ((F_CPU) / 8)

// TODO: get rid of manual rate/prescale/ticks/cycles taken for procedures in stepper.cpp
#define STEPPER_TIMER_RATE 2000000 // 2 Mhz
#define STEPPER_TIMER_PRESCALE 8
//#define STEPPER_TIMER_RATE HAL_TIMER_RATE // 2 Mhz
//#define STEPPER_TIMER_PRESCALE ((HAL_TIMER_RATE)/(STEPPER_TIMER_RATE))

#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START()  unsigned char _sreg = SREG; cli()
  #define CRITICAL_SECTION_END()    SREG = _sreg
#endif
#define ENABLE_ISRS()  sei()
#define DISABLE_ISRS() cli()

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  SBI(TIMSK1, OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() CBI(TIMSK1, OCIE1A)
#define STEPPER_ISR_ENABLED()              TEST(TIMSK1, OCIE1A)

#ifndef HAL_STEP_TIMER_ISR
#define HAL_STEP_TIMER_ISR ISR(TIMER1_COMPA_vect)
#endif

// The hardware clock id
#define STEP_TIMER 1 
// the software clock id
#define STEP_TIMER_NUM 0

#define SERVO0  0
#define SERVO_ANGLE(index,angle)  servos[index].write(angle)

FORCE_INLINE void HAL_init() {}

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


#endif // TARGET_DEFAULT