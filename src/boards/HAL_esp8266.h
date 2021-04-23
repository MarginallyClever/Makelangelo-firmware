#pragma once

// clock stuff
#  define CRITICAL_SECTION_START noInterrupts();
#  define CRITICAL_SECTION_END   interrupts();

#ifndef CLOCK_FREQ
#  define CLOCK_FREQ (16000000L)
#endif

#ifndef MAX_OCR1A_VALUE
#  define MAX_OCR1A_VALUE (0xFFFF)
#endif

#define hal_timer_t uint32_t
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFF // Timers can be 16 or 32 bit

#ifndef CLOCK_ADJUST
#  define CLOCK_ADJUST(x) { timer0_write(ESP.getCycleCount() + (long)(80000L * (x))); }  // microseconds
#endif

FORCE_INLINE void CRITICAL_SECTION_START() {}
FORCE_INLINE void CRITICAL_SECTION_END() {}


FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timerIndex,uint32_t value) {
  CLOCK_ADJUST(value);
}