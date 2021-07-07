#pragma once

#include <Arduino.h>

#define RADIANS(A) ((A)*180.0f/PI)
#define DEGREES(A) ((A)*PI/180.0f)


// for assembly in isr inner loop
#define A(CODE) " " CODE "\n\t"
#define L(CODE) CODE ":\n\t"

#define FORCE_INLINE __attribute__((always_inline)) inline
#define NO_INLINE      __attribute__((noinline))
#define _UNUSED      __attribute__((unused))

// convenience
#define PENDING(NOW, SOON) ((uint32_t)(NOW - (SOON)) < 0)
#define ELAPSED(NOW, SOON) (!PENDING(NOW, SOON))

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

  // Allow manipulating enumeration value like flags without ugly cast everywhere
  #define ENUM_FLAGS(T) \
    FORCE_INLINE constexpr T operator&(T x, T y) { return static_cast<T>(static_cast<int>(x) & static_cast<int>(y)); } \
    FORCE_INLINE constexpr T operator|(T x, T y) { return static_cast<T>(static_cast<int>(x) | static_cast<int>(y)); } \
    FORCE_INLINE constexpr T operator^(T x, T y) { return static_cast<T>(static_cast<int>(x) ^ static_cast<int>(y)); } \
    FORCE_INLINE constexpr T operator~(T x)      { return static_cast<T>(~static_cast<int>(x)); } \
    FORCE_INLINE T & operator&=(T &x, T y) { return x &= y; } \
    FORCE_INLINE T & operator|=(T &x, T y) { return x |= y; } \
    FORCE_INLINE T & operator^=(T &x, T y) { return x ^= y; }

  // C++11 solution that is standard compliant. <type_traits> is not available on all platform
  namespace Private {
    template<bool, typename _Tp = void> struct enable_if { };
    template<typename _Tp>              struct enable_if<true, _Tp> { typedef _Tp type; };

    template<typename T, typename U> struct is_same { enum { value = false }; };
    template<typename T> struct is_same<T, T> { enum { value = true }; };

    template <typename T, typename ... Args> struct first_type_of { typedef T type; };
    template <typename T> struct first_type_of<T> { typedef T type; };
  }

  // C++11 solution using SFINAE to detect the existance of a member in a class at compile time.
  // It creates a HasMember<Type> structure containing 'value' set to true if the member exists
  #define HAS_MEMBER_IMPL(Member) \
    namespace Private { \
      template <typename Type, typename Yes=char, typename No=long> struct HasMember_ ## Member { \
        template <typename C> static Yes& test( decltype(&C::Member) ) ; \
        template <typename C> static No& test(...); \
        enum { value = sizeof(test<Type>(0)) == sizeof(Yes) }; }; \
    }

  // Call the method if it exists, but do nothing if it does not. The method is detected at compile time.
  // If the method exists, this is inlined and does not cost anything. Else, an "empty" wrapper is created, returning a default value
  #define CALL_IF_EXISTS_IMPL(Return, Method, ...) \
    HAS_MEMBER_IMPL(Method) \
    namespace Private { \
      template <typename T, typename ... Args> FORCE_INLINE typename enable_if<HasMember_ ## Method <T>::value, Return>::type Call_ ## Method(T * t, Args... a) { return static_cast<Return>(t->Method(a...)); } \
                                                      _UNUSED static                                                  Return  Call_ ## Method(...) { return __VA_ARGS__; } \
    }
  #define CALL_IF_EXISTS(Return, That, Method, ...) \
    static_cast<Return>(Private::Call_ ## Method(That, ##__VA_ARGS__))

#else

  #define __MIN_N(N,V...) MIN_##N(V)
  #define _MIN_N(N,V...)  __MIN_N(N,V)
  #define _MIN_N_REF()    _MIN_N
  #define _MIN(V...)      EVAL(_MIN_N(TWO_ARGS(V),V))
  #define MIN_2(a,b)      ((a)<(b)?(a):(b))
  #define MIN_3(a,V...)   MIN_2(a,DEFER2(_MIN_N_REF)()(TWO_ARGS(V),V))

  #define __MAX_N(N,V...) MAX_##N(V)
  #define _MAX_N(N,V...)  __MAX_N(N,V)
  #define _MAX_N_REF()    _MAX_N
  #define _MAX(V...)      EVAL(_MAX_N(TWO_ARGS(V),V))
  #define MAX_2(a,b)      ((a)>(b)?(a):(b))
  #define MAX_3(a,V...)   MAX_2(a,DEFER2(_MAX_N_REF)()(TWO_ARGS(V),V))

#endif

#define NUMERIC(A) ((A)>='0'&&(A)<='9')

#define LOOP_S_LE_N(VAR, S, N) for (uint8_t VAR=(S); VAR<=(N); VAR++)
#define LOOP_S_L_N(VAR, S, N) for (uint8_t VAR=(S); VAR<(N); VAR++)
#define LOOP_LE_N(VAR, N) LOOP_S_LE_N(VAR, 0, N)
#define LOOP_L_N(VAR, N) LOOP_S_L_N(VAR, 0, N)


// Define macros for compatibility

// Use NUM_ARGS(__VA_ARGS__) to get the number of variadic arguments
#define _NUM_ARGS(_,Z,Y,X,W,V,U,T,S,R,Q,P,O,N,M,L,K,J,I,H,G,F,E,D,C,B,A,OUT,...) OUT
#define NUM_ARGS(V...) _NUM_ARGS(0,V,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)

// Use TWO_ARGS(__VA_ARGS__) to get whether there are 1, 2, or >2 arguments
#define _TWO_ARGS(_,n,m,l,k,j,i,h,g,f,e,d,c,b,a,Z,Y,X,W,V,U,T,S,R,Q,P,O,N,M,L,K,J,I,H,G,F,E,D,C,B,A,OUT,...) OUT
#define TWO_ARGS(V...) _TWO_ARGS(0,V,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,1,0)

// Macros to constrain values
#ifdef __cplusplus

  // C++11 solution that is standards compliant.
  template <class V, class N> static inline constexpr void NOLESS(V& v, const N n) {
    if (n > v) v = n;
  }
  template <class V, class N> static inline constexpr void NOMORE(V& v, const N n) {
    if (n < v) v = n;
  }
  template <class V, class N1, class N2> static inline constexpr void LIMIT(V& v, const N1 n1, const N2 n2) {
    if (n1 > v) v = n1;
    else if (n2 < v) v = n2;
  }

#else

  #define NOLESS(v, n) \
    do{ \
      __typeof__(v) _n = (n); \
      if (_n > v) v = _n; \
    }while(0)

  #define NOMORE(v, n) \
    do{ \
      __typeof__(v) _n = (n); \
      if (_n < v) v = _n; \
    }while(0)

  #define LIMIT(v, n1, n2) \
    do{ \
      __typeof__(v) _n1 = (n1); \
      __typeof__(v) _n2 = (n2); \
      if (_n1 > v) v = _n1; \
      else if (_n2 < v) v = _n2; \
    }while(0)

#endif

#define _DO_1(W,C,A)       (_##W##_1(A))
#define _DO_2(W,C,A,B)     (_##W##_1(A) C _##W##_1(B))
#define _DO_3(W,C,A,V...)  (_##W##_1(A) C _DO_2(W,C,V))
#define _DO_4(W,C,A,V...)  (_##W##_1(A) C _DO_3(W,C,V))
#define _DO_5(W,C,A,V...)  (_##W##_1(A) C _DO_4(W,C,V))
#define _DO_6(W,C,A,V...)  (_##W##_1(A) C _DO_5(W,C,V))
#define _DO_7(W,C,A,V...)  (_##W##_1(A) C _DO_6(W,C,V))
#define _DO_8(W,C,A,V...)  (_##W##_1(A) C _DO_7(W,C,V))
#define _DO_9(W,C,A,V...)  (_##W##_1(A) C _DO_8(W,C,V))
#define _DO_10(W,C,A,V...) (_##W##_1(A) C _DO_9(W,C,V))
#define _DO_11(W,C,A,V...) (_##W##_1(A) C _DO_10(W,C,V))
#define _DO_12(W,C,A,V...) (_##W##_1(A) C _DO_11(W,C,V))
#define __DO_N(W,C,N,V...) _DO_##N(W,C,V)
#define _DO_N(W,C,N,V...)  __DO_N(W,C,N,V)
#define DO(W,C,V...)       _DO_N(W,C,NUM_ARGS(V),V)

#define _CAT(a,V...) a##V
#define CAT(a,V...) _CAT(a,V)

// Macros for adding
#define INC_0   1
#define INC_1   2
#define INC_2   3
#define INC_3   4
#define INC_4   5
#define INC_5   6
#define INC_6   7
#define INC_7   8
#define INC_8   9
#define INC_9  10
#define INC_10 11
#define INC_11 12
#define INC_12 13
#define INC_13 14
#define INC_14 15
#define INC_15 16
#define INCREMENT_(n) INC_##n
#define INCREMENT(n) INCREMENT_(n)

#define ADD0(N)  N
#define ADD1(N)  INCREMENT_(N)
#define ADD2(N)  ADD1(ADD1(N))
#define ADD3(N)  ADD1(ADD2(N))
#define ADD4(N)  ADD2(ADD2(N))
#define ADD5(N)  ADD2(ADD3(N))
#define ADD6(N)  ADD3(ADD3(N))
#define ADD7(N)  ADD3(ADD4(N))
#define ADD8(N)  ADD4(ADD4(N))
#define ADD9(N)  ADD4(ADD5(N))
#define ADD10(N) ADD5(ADD5(N))
#define SUM(A,B) _CAT(ADD,A)(B)
#define DOUBLE_(n) ADD##n(n)
#define DOUBLE(n) DOUBLE_(n)

// Macros for subtracting
#define DEC_0   0
#define DEC_1   0
#define DEC_2   1
#define DEC_3   2
#define DEC_4   3
#define DEC_5   4
#define DEC_6   5
#define DEC_7   6
#define DEC_8   7
#define DEC_9   8
#define DEC_10  9
#define DEC_11 10
#define DEC_12 11
#define DEC_13 12
#define DEC_14 13
#define DEC_15 14
#define DECREMENT_(n) DEC_##n
#define DECREMENT(n) DECREMENT_(n)

#define SUB0(N)  N
#define SUB1(N)  DECREMENT_(N)
#define SUB2(N)  SUB1(SUB1(N))
#define SUB3(N)  SUB1(SUB2(N))
#define SUB4(N)  SUB2(SUB2(N))
#define SUB5(N)  SUB2(SUB3(N))
#define SUB6(N)  SUB3(SUB3(N))
#define SUB7(N)  SUB3(SUB4(N))
#define SUB8(N)  SUB4(SUB4(N))
#define SUB9(N)  SUB4(SUB5(N))
#define SUB10(N) SUB5(SUB5(N))

#define FIRST(a,...)     a
#define SECOND(a,b,...)  b
#define THIRD(a,b,c,...) c

// Defer expansion
#define EMPTY()
#define DEFER(M)  M EMPTY()
#define DEFER2(M) M EMPTY EMPTY()()
#define DEFER3(M) M EMPTY EMPTY EMPTY()()()
#define DEFER4(M) M EMPTY EMPTY EMPTY EMPTY()()()()

// Force define expansion
#define EVAL(V...)     EVAL16(V)
#define EVAL1024(V...) EVAL512(EVAL512(V))
#define EVAL512(V...)  EVAL256(EVAL256(V))
#define EVAL256(V...)  EVAL128(EVAL128(V))
#define EVAL128(V...)  EVAL64(EVAL64(V))
#define EVAL64(V...)   EVAL32(EVAL32(V))
#define EVAL32(V...)   EVAL16(EVAL16(V))
#define EVAL16(V...)   EVAL8(EVAL8(V))
#define EVAL8(V...)    EVAL4(EVAL4(V))
#define EVAL4(V...)    EVAL2(EVAL2(V))
#define EVAL2(V...)    EVAL1(EVAL1(V))
#define EVAL1(V...)    V

#define IS_PROBE(V...) SECOND(V, 0)     // Get the second item passed, or 0
#define PROBE() ~, 1                    // Second item will be 1 if this is passed
#define _NOT_0 PROBE()
#define NOT(x) IS_PROBE(_CAT(_NOT_, x)) // NOT('0') gets '1'. Anything else gets '0'.
#define _BOOL(x) NOT(NOT(x))            // NOT('0') gets '0'. Anything else gets '1'.

#define IF_ELSE(TF) _IF_ELSE(_BOOL(TF))
#define _IF_ELSE(TF) _CAT(_IF_, TF)

#define _IF_1(V...) V _IF_1_ELSE
#define _IF_0(...)    _IF_0_ELSE

#define _IF_1_ELSE(...)
#define _IF_0_ELSE(V...) V

#define _ISENA_     ~,1
#define _ISENA_1    ~,1
#define _ISENA_0x1  ~,1
#define _ISENA_true ~,1
#define _ISENA(V...)        IS_PROBE(V)

#define _ENA_1(O)           _ISENA(CAT(_IS,CAT(ENA_, O)))
#define _DIS_1(O)           NOT(_ENA_1(O))
#define ENABLED(V...)       DO(ENA,&&,V)
#define DISABLED(V...)      DO(DIS,&&,V)
#define COUNT_ENABLED(V...) DO(ENA,+,V)

#define TERN(O,A,B)         _TERN(_ENA_1(O),B,A)    // OPTION converted to '0' or '1'
#define TERN0(O,A)          _TERN(_ENA_1(O),0,A)    // OPTION converted to A or '0'
#define TERN1(O,A)          _TERN(_ENA_1(O),1,A)    // OPTION converted to A or '1'
#define TERN_(O,A)          _TERN(_ENA_1(O),,A)     // OPTION converted to A or '<nul>'
#define _TERN(E,V...)       __TERN(_CAT(T_,E),V)    // Prepend 'T_' to get 'T_0' or 'T_1'
#define __TERN(T,V...)      ___TERN(_CAT(_NO,T),V)  // Prepend '_NO' to get '_NOT_0' or '_NOT_1'
#define ___TERN(P,V...)     THIRD(P,V)              // If first argument has a comma, A. Else B.

#define WITHIN(N,L,H)       ((N) >= (L) && (N) <= (H))

#define ANY(V...)          !DISABLED(V)
#define NONE(V...)          DISABLED(V)
#define ALL(V...)           ENABLED(V)
#define BOTH(V1,V2)         ALL(V1,V2)
#define EITHER(V1,V2)       ANY(V1,V2)
#define MANY(V...)          (COUNT_ENABLED(V) > 1)



// Call OP(I) N times with ascending counter.
#define _REPEAT(_RPT_I,_RPT_N,_RPT_OP)                        \
  _RPT_OP(_RPT_I)                                             \
  IF_ELSE(SUB1(_RPT_N))                                       \
    ( DEFER2(__REPEAT)()(ADD1(_RPT_I),SUB1(_RPT_N),_RPT_OP) ) \
    ( /* Do nothing */ )
#define __REPEAT() _REPEAT

// Call OP(I, ...) N times with ascending counter.
#define _REPEAT2(_RPT_I,_RPT_N,_RPT_OP,V...)                     \
  _RPT_OP(_RPT_I,V)                                              \
  IF_ELSE(SUB1(_RPT_N))                                          \
    ( DEFER2(__REPEAT2)()(ADD1(_RPT_I),SUB1(_RPT_N),_RPT_OP,V) ) \
    ( /* Do nothing */ )
#define __REPEAT2() _REPEAT2

// Repeat a macro passing S...N-1.
#define REPEAT_S(S,N,OP)        EVAL(_REPEAT(S,SUB##S(N),OP))
#define REPEAT(N,OP)            REPEAT_S(0,N,OP)
#define REPEAT_1(N,OP)          REPEAT_S(1,INCREMENT(N),OP)

// Repeat a macro passing 0...N-1 plus additional arguments.
#define REPEAT2_S(S,N,OP,V...)  EVAL(_REPEAT2(S,SUB##S(N),OP,V))
#define REPEAT2(N,OP,V...)      REPEAT2_S(0,N,OP,V)

// Use RREPEAT macros with REPEAT macros for nesting
#define _RREPEAT(_RPT_I,_RPT_N,_RPT_OP)                           \
  _RPT_OP(_RPT_I)                                                 \
  IF_ELSE(SUB1(_RPT_N))                                           \
    ( DEFER2(__RREPEAT)()(ADD1(_RPT_I),SUB1(_RPT_N),_RPT_OP) )    \
    ( /* Do nothing */ )
#define __RREPEAT() _RREPEAT
#define _RREPEAT2(_RPT_I,_RPT_N,_RPT_OP,V...)                     \
  _RPT_OP(_RPT_I,V)                                               \
  IF_ELSE(SUB1(_RPT_N))                                           \
    ( DEFER2(__RREPEAT2)()(ADD1(_RPT_I),SUB1(_RPT_N),_RPT_OP,V) ) \
    ( /* Do nothing */ )
#define __RREPEAT2() _RREPEAT2
#define RREPEAT_S(S,N,OP)        EVAL1024(_RREPEAT(S,SUB##S(N),OP))
#define RREPEAT(N,OP)            RREPEAT_S(0,N,OP)
#define RREPEAT2_S(S,N,OP,V...)  EVAL1024(_RREPEAT2(S,SUB##S(N),OP,V))
#define RREPEAT2(N,OP,V...)      RREPEAT2_S(0,N,OP,V)
