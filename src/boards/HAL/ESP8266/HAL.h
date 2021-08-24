#pragma once

#ifdef ESP8266

// clock stuff
#  define CRITICAL_SECTION_START noInterrupts();
#  define CRITICAL_SECTION_END   interrupts();

#ifndef MAX_OCR1A_VALUE
#  define MAX_OCR1A_VALUE (0xFFFF)
#endif

#define hal_timer_t uint32_t
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFF // Timers can be 16 or 32 bit

#ifndef CLOCK_ADJUST
#  define CLOCK_ADJUST(x) { timer0_write(ESP.getCycleCount() + (long)(80000L * (x))); }  // microseconds
#endif

#define FORCE_INLINE __attribute__((always_inline)) inline

#define SERVO0  SERVO0_PIN
#define SERVO_ANGLE(index,angle)  analogWrite(index, arg0)

FORCE_INLINE void HAL_timer_start(const uint8_t timerIndex) {
#ifndef DEBUG_STEPPING
  // disable global interrupts
  DISABLE_ISRS();

  timer0_isr_init();
  timer0_attachInterrupt(itr);
  CLOCK_ADJUST(2000);

  ENABLE_ISRS();
#endif  // DEBUG_STEPPING
}

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timerIndex,uint32_t value) {
  CLOCK_ADJUST(value);
}

#endif