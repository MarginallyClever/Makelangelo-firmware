#pragma once

#include "macros.h"

#ifdef CPU_32_BIT
static FORCE_INLINE uint32_t MultiU32X24toH32(uint32_t longIn1, uint32_t longIn2) {
  return ((uint64_t)longIn1 * longIn2 + 0x00800000) >> 24;
}

#else

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
static FORCE_INLINE uint16_t MultiU16X8toH16(uint8_t charIn1, uint16_t intIn2) {
  register uint8_t tmp;
  register uint16_t intRes;
  __asm__ __volatile__(
    A("clr %[tmp]")
    A("mul %[charIn1], %B[intIn2]") 
    A("movw %A[intRes], r0")
    A("mul %[charIn1], %A[intIn2]") 
    A("add %A[intRes], r1") 
    A("adc %B[intRes], %[tmp]")
    A("lsr r0") 
    A("adc %A[intRes], %[tmp]") 
    A("adc %B[intRes], %[tmp]") 
    A("clr r1")
    : [ intRes ] "=&r"(intRes), [ tmp ] "=&r"(tmp)
    : [ charIn1 ] "d"(charIn1), [ intIn2 ] "d"(intIn2)
    : "cc"
  );
  return intRes;
}

// intRes = longIn1 * longIn2 >> 24
// uses:
// A[tmp] to store 0
// B[tmp] to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B A are bits 24-39 and are the returned value
// C B A is longIn1
// D C B A is longIn2
//
static FORCE_INLINE uint16_t MultiU24X32toH16(uint32_t longIn1, uint32_t longIn2) {
#ifdef ESP8266
  uint16_t intRes = longIn1 * longIn2 >> 24;
#else
  register uint8_t tmp1;
  register uint8_t tmp2;
  register uint16_t intRes;
  __asm__ __volatile__(
    A("clr %[tmp1]") 
    A("mul %A[longIn1], %B[longIn2]") 
    A("mov %[tmp2], r1") 
    A("mul %B[longIn1], %C[longIn2]")
    A("movw %A[intRes], r0") 
    A("mul %C[longIn1], %C[longIn2]") 
    A("add %B[intRes], r0")
    A("mul %C[longIn1], %B[longIn2]") 
    A("add %A[intRes], r0") 
    A("adc %B[intRes], r1")
    A("mul %A[longIn1], %C[longIn2]") 
    A("add %[tmp2], r0") 
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]") 
    A("mul %B[longIn1], %B[longIn2]") 
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1") 
    A("adc %B[intRes], %[tmp1]") 
    A("mul %C[longIn1], %A[longIn2]")
    A("add %[tmp2], r0") 
    A("adc %A[intRes], r1") 
    A("adc %B[intRes], %[tmp1]")
    A("mul %B[longIn1], %A[longIn2]") 
    A("add %[tmp2], r1") 
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]") 
    A("lsr %[tmp2]") 
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]") 
    A("mul %D[longIn2], %A[longIn1]")
    A("add %A[intRes], r0")
    A("adc %B[intRes], r1")
    A("mul %D[longIn2], %B[longIn1]") 
    A("add %B[intRes], r0") 
    A("clr r1")
    : [ intRes ] "=&r"(intRes), [ tmp1 ] "=&r"(tmp1), [ tmp2 ] "=&r"(tmp2)
    : [ longIn1 ] "d"(longIn1), [ longIn2 ] "d"(longIn2)
    : "cc"
  );
#endif  // ESP8266
  return intRes;
}

#endif
