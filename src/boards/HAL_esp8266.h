#pragma once

// clock stuff
#  define MAX_COUNTER            (4294967295L)  // 32 bits
#  define CRITICAL_SECTION_START noInterrupts();
#  define CRITICAL_SECTION_END   interrupts();

#ifndef CLOCK_FREQ
#  define CLOCK_FREQ (16000000L)
#endif

extern void CRITICAL_SECTION_START();
extern void CRITICAL_SECTION_END();

#ifndef MAX_OCR1A_VALUE
#  define MAX_OCR1A_VALUE (0xFFFF)
#endif

#ifndef CLOCK_ADJUST
#  define CLOCK_ADJUST(x) \
    { timer0_write(ESP.getCycleCount() + (long)(80000L * (x))); }  // microseconds
#endif

inline void CRITICAL_SECTION_START() {}
inline void CRITICAL_SECTION_END() {}
