#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MOTHERBOARD == BOARD_SKRPRO1_2

// wrong board type set
#  ifndef HEADER_SKRPRO
#    error "Oops!  Make sure you build this project using PlatformIO"
#  endif

#  define CPU_32_BIT
#  define CLOCK_FREQ (168000000L)

#  define MAX_MOTORS (6)

#  define MOTOR_0_LETTER           'X'
#  define MOTOR_0_DIR_PIN          (16)
#  define MOTOR_0_STEP_PIN         (17)
#  define MOTOR_0_ENABLE_PIN       (48)
#  define MOTOR_0_LIMIT_SWITCH_PIN (37)

#  define MOTOR_1_LETTER           'Y'
#  define MOTOR_1_DIR_PIN          (47)
#  define MOTOR_1_STEP_PIN         (54)
#  define MOTOR_1_ENABLE_PIN       (55)
#  define MOTOR_1_LIMIT_SWITCH_PIN (36)

#  define MOTOR_2_LETTER           'Z'
#  define MOTOR_2_DIR_PIN          (56)
#  define MOTOR_2_STEP_PIN         (57)
#  define MOTOR_2_ENABLE_PIN       (62)
#  define MOTOR_2_LIMIT_SWITCH_PIN (35)

#  define MOTOR_3_LETTER           'U'
#  define MOTOR_3_DIR_PIN          (22)
#  define MOTOR_3_STEP_PIN         (23)
#  define MOTOR_3_ENABLE_PIN       (24)
#  define MOTOR_3_LIMIT_SWITCH_PIN (34)

#  define MOTOR_4_LETTER           'V'
#  define MOTOR_4_DIR_PIN          (25)
#  define MOTOR_4_STEP_PIN         (26)
#  define MOTOR_4_ENABLE_PIN       (27)
#  define MOTOR_4_LIMIT_SWITCH_PIN (33)

#  define MOTOR_5_LETTER           'W'
#  define MOTOR_5_DIR_PIN          (28)
#  define MOTOR_5_STEP_PIN         (29)
#  define MOTOR_5_ENABLE_PIN       (39)
#  define MOTOR_5_LIMIT_SWITCH_PIN (32)

#  define MAX_BOARD_SERVOS (1)
#  define SERVO0_PIN       (5)

#  define LIMIT_SWITCH_PIN_LEFT  (MOTOR_0_LIMIT_SWITCH_PIN)
#  define LIMIT_SWITCH_PIN_RIGHT (MOTOR_1_LIMIT_SWITCH_PIN)

// LCD pins
#  if LCD_TYPE == LCD_IS_128X64
#    define BEEPER          44
#    define LCD_PINS_RS     19
#    define LCD_PINS_ENABLE 42
#    define LCD_PINS_D4     18
#    define LCD_PINS_D5     38
#    define LCD_PINS_D6     41
#    define LCD_PINS_D7     40

// Encoder rotation values
#    define BTN_EN1 11
#    define BTN_EN2 12
#    define BTN_ENC 43

// SD card settings
#    define SDPOWER      -1
#    define SDSS         53
#    define SDCARDDETECT 49
#  endif

#  if LCD_TYPE == LCD_IS_SMART
#    define BEEPER          44
#    define LCD_PINS_RS     19
#    define LCD_PINS_ENABLE 42
#    define LCD_PINS_D4     18
#    define LCD_PINS_D5     38
#    define LCD_PINS_D6     41
#    define LCD_PINS_D7     40

// Encoder rotation values
#    define BTN_EN1 11
#    define BTN_EN2 12
#    define BTN_ENC 43

// SD card settings
#    define SDPOWER      -1
#    define SDSS         53
#    define SDCARDDETECT 49
#  endif

#  ifdef HAS_TMC2130

#    define CS_PIN_0 30  // chip select
#    define CS_PIN_1 31  // chip select

#  endif  // HAS_TMC2130

// HAL stuff ---------------------------------------------


#  include <stdint.h>
#  include <HardwareTimer.h>

#define NUM_HARDWARE_TIMERS 2

#define FORCE_INLINE __attribute__((always_inline)) inline
#define STEPPER_TIMER_RATE 2000000 // 2 Mhz

#define HAL_STEP_TIMER_ISR void Step_Handler(HardwareTimer *htim)

#  define CRITICAL_SECTION_START()  uint32_t primaskV = __get_PRIMASK(); __disable_irq()
#  define CRITICAL_SECTION_END()    if(!primaskV) __enable_irq()
#  define cli() __enable_irq()
#  define sei() __disable_irq()


extern void Step_Handler(HardwareTimer *htim);

extern HardwareTimer *timer_instance[NUM_HARDWARE_TIMERS];

FORCE_INLINE void HAL_timer_start(const uint8_t timerIndex) {
}

FORCE_INLINE bool HAL_timer_initialized(const uint8_t timerIndex) {
  return timer_instance[timerIndex] != NULL;
}

FORCE_INLINE static void CLOCK_ADJUST(uint32_t overflow) {
  if (HAL_timer_initialized(0)) {
    timer_instance[0]->setOverflow(overflow + 1, TICK_FORMAT); // Value decremented by setOverflow()
    // wiki: "force all registers (Autoreload, prescaler, compare) to be taken into account"
    // So, if the new overflow value is less than the count it will trigger a rollover interrupt.
    if (overflow < timer_instance[0]->getCount())  // Added 'if' here because reports say it won't boot without it
      timer_instance[0]->refresh();
  }
}
// HAL stuff ---------------------------------------------


#endif
