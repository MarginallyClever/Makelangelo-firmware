#pragma once

#ifdef TARGET_DEFAULT

#include <Arduino.h>
#include "macros.h"

#define CPU_16_BIT

#define NUM_SERIAL 1
#define SERIAL_PORT 0
#include "boards/MarlinSerial.h"
#include "serial.h"


#define hal_timer_t uint16_t
#define HAL_TIMER_TYPE_MAX 0xFFFF
#define HAL_TIMER_RATE ((F_CPU) / 8)

#define STEPPER_TIMER_RATE HAL_TIMER_RATE // 2 Mhz
#define STEPPER_TIMER_PRESCALE 8
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs

#define PGMSTR(NAM,STR) const char NAM[] PROGMEM = STR

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START()  unsigned char _sreg = SREG; cli()
  #define CRITICAL_SECTION_END()    SREG = _sreg
#endif
#define ISRS_ENABLED() TEST(SREG, SREG_I)
#define ENABLE_ISRS()  sei()
#define DISABLE_ISRS() cli()

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  SBI(TIMSK1, OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() CBI(TIMSK1, OCIE1A)
#define STEPPER_ISR_ENABLED()              TEST(TIMSK1, OCIE1A)

#ifndef HAL_STEP_TIMER_ISR

#define HAL_STEP_TIMER_ISR() \
extern "C" void TIMER1_COMPA_vect() __attribute__ ((signal, naked, used, externally_visible)); \
extern "C" void TIMER1_COMPA_vect_bottom() asm ("TIMER1_COMPA_vect_bottom") __attribute__ ((used, externally_visible, noinline)); \
void TIMER1_COMPA_vect() { \
  __asm__ __volatile__ ( \
    A("push r16")                      /* 2 Save R16 */ \
    A("in r16, __SREG__")              /* 1 Get SREG */ \
    A("push r16")                      /* 2 Save SREG into stack */ \
    A("lds r16, %[timsk0]")            /* 2 Load into R0 the Temperature timer Interrupt mask register */ \
    A("push r16")                      /* 2 Save TIMSK0 into the stack */ \
    A("andi r16,~%[msk0]")             /* 1 Disable the temperature ISR */ \
    A("sts %[timsk0], r16")            /* 2 And set the new value */ \
    A("lds r16, %[timsk1]")            /* 2 Load into R0 the stepper timer Interrupt mask register [TIMSK1] */ \
    A("andi r16,~%[msk1]")             /* 1 Disable the stepper ISR */ \
    A("sts %[timsk1], r16")            /* 2 And set the new value */ \
    A("push r16")                      /* 2 Save TIMSK1 into stack */ \
    A("in r16, 0x3B")                  /* 1 Get RAMPZ register */ \
    A("push r16")                      /* 2 Save RAMPZ into stack */ \
    A("in r16, 0x3C")                  /* 1 Get EIND register */ \
    A("push r0")                       /* C runtime can modify all the following registers without restoring them */ \
    A("push r1")                       \
    A("push r18")                      \
    A("push r19")                      \
    A("push r20")                      \
    A("push r21")                      \
    A("push r22")                      \
    A("push r23")                      \
    A("push r24")                      \
    A("push r25")                      \
    A("push r26")                      \
    A("push r27")                      \
    A("push r30")                      \
    A("push r31")                      \
    A("clr r1")                        /* C runtime expects this register to be 0 */ \
    A("call TIMER1_COMPA_vect_bottom") /* Call the bottom handler - No inlining allowed, otherwise registers used are not saved */   \
    A("pop r31")                       \
    A("pop r30")                       \
    A("pop r27")                       \
    A("pop r26")                       \
    A("pop r25")                       \
    A("pop r24")                       \
    A("pop r23")                       \
    A("pop r22")                       \
    A("pop r21")                       \
    A("pop r20")                       \
    A("pop r19")                       \
    A("pop r18")                       \
    A("pop r1")                        \
    A("pop r0")                        \
    A("out 0x3C, r16")                 /* 1 Restore EIND register */ \
    A("pop r16")                       /* 2 Get the original RAMPZ register value */ \
    A("out 0x3B, r16")                 /* 1 Restore RAMPZ register to its original value */ \
    A("pop r16")                       /* 2 Get the original TIMSK1 value but with stepper ISR disabled */ \
    A("ori r16,%[msk1]")               /* 1 Reenable the stepper ISR */ \
    A("cli")                           /* 1 Disable global interrupts - Reenabling Stepper ISR can reenter amd temperature can reenter, and we want that, if it happens, after this ISR has ended */ \
    A("sts %[timsk1], r16")            /* 2 And restore the old value - This reenables the stepper ISR */ \
    A("pop r16")                       /* 2 Get the temperature timer Interrupt mask register [TIMSK0] */ \
    A("sts %[timsk0], r16")            /* 2 And restore the old value - This reenables the temperature ISR */ \
    A("pop r16")                       /* 2 Get the old SREG value */ \
    A("out __SREG__, r16")             /* 1 And restore the SREG value */ \
    A("pop r16")                       /* 2 Restore R16 value */ \
    A("reti")                          /* 4 Return from interrupt */ \
    :                                   \
    : [timsk0] "i" ((uint16_t)&TIMSK0), \
      [timsk1] "i" ((uint16_t)&TIMSK1), \
      [msk0] "M" ((uint8_t)(1<<OCIE0B)),\
      [msk1] "M" ((uint8_t)(1<<OCIE1A)) \
    : \
  ); \
} \
void TIMER1_COMPA_vect_bottom()
#endif

// The hardware clock id
#define STEP_TIMER 1 
// the software clock id
#define STEP_TIMER_NUM 0

#define SERVO0  0
#define SERVO_ANGLE(index,angle)  servos[index].write(angle)

FORCE_INLINE void HAL_init() {}

// TCNT* - Timer/Counter Register.  The actual timer value is stored here.
// OCR*  - Output Compare Register

FORCE_INLINE void HAL_timer_start(const uint8_t timerIndex) {
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

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timerIndex) {
  return OCR1A;
}

extern void HAL_timer_enable_interrupt(const uint8_t timerIndex);
extern void HAL_timer_disable_interrupt(const uint8_t timerIndex);

#endif // TARGET_DEFAULT