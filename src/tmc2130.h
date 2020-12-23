#pragma once

#include "macros.h"


#ifdef HAS_TMC2130

#include <TMC2130Stepper.h>
#include <TMC2130Stepper_REGDEFS.h>

#if TMC2130STEPPER_VERSION < 0x020201
  #error "Update TMC2130Stepper library to 2.2.1 or newer."
#endif


#define STEPPER_DIR_HIGH   LOW
#define STEPPER_DIR_LOW    HIGH

#define HOMING_OCR1A                 450 //776
// define this only after you have measured your desired TSTEP
#define MEASURED_TSTEP              169 //295
#define MEASURED_TSTEP_MARGIN_PCT   15  // +/-% for stall warning
  
#define CURRENT                 219  // 310ma / sqrt(2)
#define R_SENSE                 0.11
#define HOLD_MULTIPLIER         0.5
  
#define STALL_VALUE             -64//-24
//#define HYBRID_THRESHOLD        100


extern bool homing;
extern TMC2130Stepper driver_0;
extern TMC2130Stepper driver_1;



FORCE_INLINE void homing_sequence() {
  if (en0 == true) {
    digitalWrite(MOTOR_0_STEP_PIN, HIGH);
    digitalWrite(MOTOR_0_STEP_PIN, LOW);
    if (digitalRead(MOTOR_0_LIMIT_SWITCH_PIN) == LOW) {
      digitalWrite( MOTOR_0_ENABLE_PIN,  HIGH );
      en0 = false;
    }
  }
  if (en1 == true) {
    digitalWrite(MOTOR_1_STEP_PIN, HIGH);
    digitalWrite(MOTOR_1_STEP_PIN, LOW);
    if (digitalRead(MOTOR_1_LIMIT_SWITCH_PIN) == LOW) {
      digitalWrite( MOTOR_1_ENABLE_PIN,  HIGH );
      en1 = false;
    }
  }
  // make homing false when en0 and en1 are both false at the same time.
  homing = en0 | en1;
}



extern void tmc_setup(TMC2130Stepper &driver);
extern void disable_stealthChop();
extern void enable_stealthChop();
extern void motor_home();


#endif  // HAS_TMC2130
