#pragma once

#include <Arduino.h>

#define RADIANS(A) ((A)*180.0f/PI)
#define DEGREES(A) ((A)*PI/180.0f)


// for assembly in isr inner loop
#define A(CODE) " " CODE "\n\t"

// optimize code, please
#define FORCE_INLINE __attribute__((always_inline)) inline

// convenience
#define PENDING(NOW, SOON) ((uint32_t)(NOW - (SOON)) < 0)
#define ELAPSED(NOW, SOON) (!PENDING(NOW, SOON))

#define REPORT(AA,BB) {  Serial.print(AA);  Serial.println(BB);  }

// set bit on
#ifndef SBI
#  define SBI(NN, BB) (NN |= (1 << BB))
#endif

// set bit off
#ifndef CBI
#  define CBI(NN, BB) (NN &= ~(1 << BB))
#endif

#undef SET_BIT_ON
#define SET_BIT_ON(NN, BB)  SBI(NN, BB)

#undef SET_BIT_OFF
#define SET_BIT_OFF(NN, BB) CBI(NN, BB)

#undef TEST
#define TEST(NN, BB)        (((NN >> BB) & 0x1) == 0x1)

#undef SET_BIT
#define SET_BIT(NN, BB, TF) \
  do {                      \
    if (TF)                 \
      SBI(NN, BB);          \
    else                    \
      CBI(NN, BB);          \
  } while (0);

#undef FLIP_BIT
#define FLIP_BIT(NN, BB) (NN ^= (1 << BB))

// wrap all degrees to within -180...180.
FORCE_INLINE float WRAP_DEGREES(float n) {
  n = fmod(n, 360);
  n += 360;
  n = fmod(n, 360);
  if (n > 180) n -= 360;
  return n;
}

// wrap all radians within -PI...PI
FORCE_INLINE float WRAP_RADIANS(float n) {
  n = fmod(n, TWO_PI);
  n += TWO_PI;
  n = fmod(n, TWO_PI);
  if (n > PI) n -= TWO_PI;
  return n;
}

// use in for(ALL_AXIES(i)) { //i will be rising
#define ALL_AXIES(NN) \
  int NN = 0;         \
  NN < NUM_AXIES;     \
  ++NN

// use in for(ALL_MOTORS(i)) { //i will be rising
#define ALL_MOTORS(NN) \
  int NN = 0;          \
  NN < NUM_MOTORS;     \
  ++NN

// use in for(ALL_MUSCLES(i)) { //i will be rising
#define ALL_MUSCLES(NN) \
  int NN = 0;           \
  NN < NUM_MUSCLES;     \
  ++NN

#define USE_CACHED_SQRT
#ifdef USE_CACHED_SQRT
#  define CACHED_SQRT(N, V) \
    static float saved_V, N; \
    if (V != saved_V) { N = sqrtf(V); saved_V = V; }
#else
  #define CACHED_SQRT(N, V) const float N = sqrtf(V)
#endif


#define _NUM_ARGS(_,Z,Y,X,W,V,U,T,S,R,Q,P,O,N,M,L,K,J,I,H,G,F,E,D,C,B,A,OUT,...) OUT
#define NUM_ARGS(V...) _NUM_ARGS(0,V,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)

/*
#ifdef __cplusplus

  #ifndef _MINMAX_H_
  #define _MINMAX_H_

    extern "C++" {

      // C++11 solution that is standards compliant. Return type is deduced automatically
      template <class L, class R> static inline constexpr auto _MIN(const L lhs, const R rhs) -> decltype(lhs + rhs) {
        return lhs < rhs ? lhs : rhs;
      }
      template <class L, class R> static inline constexpr auto _MAX(const L lhs, const R rhs) -> decltype(lhs + rhs) {
        return lhs > rhs ? lhs : rhs;
      }
      template<class T, class ... Ts> static inline constexpr const T _MIN(T V, Ts... Vs) { return _MIN(V, _MIN(Vs...)); }
      template<class T, class ... Ts> static inline constexpr const T _MAX(T V, Ts... Vs) { return _MAX(V, _MAX(Vs...)); }

    }

  #endif

#else*/

  #define MIN_2(a,b)      ((a)<(b)?(a):(b))
  #define MIN_3(a,V...)   MIN_2(a,MIN_2(V))
  #define MIN_4(a,V...)   MIN_2(a,MIN_3(V))
  #define MIN_5(a,V...)   MIN_2(a,MIN_4(V))
  #define MIN_6(a,V...)   MIN_2(a,MIN_5(V))
  #define MIN_7(a,V...)   MIN_2(a,MIN_6(V))
  #define MIN_8(a,V...)   MIN_2(a,MIN_7(V))
  #define MIN_9(a,V...)   MIN_2(a,MIN_8(V))
  #define MIN_10(a,V...)  MIN_2(a,MIN_9(V))
  #define __MIN_N(N,V...) MIN_##N(V)
  #define _MIN_N(N,V...)  __MIN_N(N,V)
  #define _MIN(V...)      _MIN_N(NUM_ARGS(V), V)

  #define MAX_2(a,b)      ((a)>(b)?(a):(b))
  #define MAX_3(a,V...)   MAX_2(a,MAX_2(V))
  #define MAX_4(a,V...)   MAX_2(a,MAX_3(V))
  #define MAX_5(a,V...)   MAX_2(a,MAX_4(V))
  #define MAX_6(a,V...)   MAX_2(a,MAX_5(V))
  #define MAX_7(a,V...)   MAX_2(a,MAX_6(V))
  #define MAX_8(a,V...)   MAX_2(a,MAX_7(V))
  #define MAX_9(a,V...)   MAX_2(a,MAX_8(V))
  #define MAX_10(a,V...)  MAX_2(a,MAX_9(V))
  #define __MAX_N(N,V...) MAX_##N(V)
  #define _MAX_N(N,V...)  __MAX_N(N,V)
  #define _MAX(V...)      _MAX_N(NUM_ARGS(V), V)

//#endif

#define NUMERIC(A) ((A)>='0'&&(A)<='9')
