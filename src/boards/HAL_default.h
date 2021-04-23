#pragma once

#if MOTHERBOARD 

#ifndef CLOCK_FREQ
#  define CLOCK_FREQ (16000000L)
#endif

#ifndef MAX_OCR1A_VALUE
#  define MAX_OCR1A_VALUE (0xFFFF)
#endif

extern void CRITICAL_SECTION_START();
extern void CRITICAL_SECTION_END();


FORCE_INLINE void HAL_start_timer(const uint8_t timerIndex) {

#ifndef DEBUG_STEPPING
  // disable global interrupts
  CRITICAL_SECTION_START();

#  ifdef ESP8266
  timer0_isr_init();
  timer0_attachInterrupt(itr);
  CLOCK_ADJUST(2000);
#  else
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
#  endif  // ESP8266

  CRITICAL_SECTION_END();
#endif  // DEBUG_STEPPING
}

FORCE_INLINE bool HAL_timer_initialized(const uint8_t timerIndex) {
    return true;
}