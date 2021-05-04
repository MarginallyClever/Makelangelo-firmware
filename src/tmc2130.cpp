
#include "configure.h"

#ifdef HAS_TMC2130

TMC2130Stepper *drivers[NUM_MOTORS];

bool en[NUM_MOTORS];
bool homing = false;
// bool vsense;

uint16_t tmc2130_rms_current(uint8_t CS, bool vsense, float Rsense = 0.11) {
  return (float)(CS + 1) / 32.0 * (vsense ? 0.180 : 0.325) / (Rsense + 0.02) / 1.41421 * 1000;
}

void tmc2130_setup_single(TMC2130Stepper *driver) {
  driver->begin();
  driver->setCurrent(CURRENT, R_SENSE, HOLD_MULTIPLIER);
  driver->microsteps(MICROSTEPS);
  driver->blank_time(24);
  driver->off_time(2);
  driver->interpolate(true);
  driver->power_down_delay(128);  // ~2s until driver lowers to hold current
  driver->hysteresis_start(5);
  driver->hysteresis_end(2);

  driver->diag1_stall(0);        // use diag1 to signal stall event
  driver->diag1_active_high(0);  // I want active low
  driver->sg_stall_value(STALL_VALUE);

  driver->coolstep_min_speed(0);
  driver->THIGH(0);
  driver->vsense(CURRENT >= 540 ? 1 : 0);

#  if defined(STEALTHCHOP)
  driver->stealth_freq(1);  // f_pwm = 2/683 f_clk
  driver->stealth_autoscale(1);
  driver->stealth_gradient(5);
  driver->stealth_amplitude(255);
  driver->stealthChop(1);
  //#if defined(HYBRID_THRESHOLD)
  //  driver->stealth_max_speed(12650000UL*MICROSTEPS/(256*HYBRID_THRESHOLD*STEPS_PER_MM));
  //#endif
#  endif
  driver->GSTAT(0);  // Clear GSTAT
}

void tmc2130_setup_all() {
  SPI.begin();
  pinMode(MISO, INPUT_PULLUP);

  uint16_t pins[NUM_MOTORS] = {
    CS_PIN_0,
#if NUM_MOTORS>1
    CS_PIN_1,
#endif
#if NUM_MOTORS>2
    CS_PIN_2,
#endif
#if NUM_MOTORS>3
    CS_PIN_3,
#endif
#if NUM_MOTORS>4
    CS_PIN_4,
#endif
#if NUM_MOTORS>5
    CS_PIN_5,
#endif
  };

  for(ALL_MOTORS(i)) {
    pinMode(pins[i],OUTPUT);
    digitalWrite(pins[i],HIGH);
    drivers[i] = new TMC2130Stepper(pins[i]);
    tmc2130_setup_single(drivers[i]);
  }
}

void tmc2130_disable_stealthChop() {
  // disable stealthchop
  for(ALL_MOTORS(j)) {
    drivers[j]->coolstep_min_speed(0xFFFFF);
    drivers[j]->diag1_stall(1);
  }

#  ifdef MEASURED_TSTEP
  Serial.println("measured_tstep is inputting");
  for(ALL_MOTORS(j)) {
    drivers[j]->TCOOLTHRS((float)MEASURED_TSTEP * (100 + MEASURED_TSTEP_MARGIN_PCT) / 100.0f);  // + margin %
    drivers[j]->THIGH((float)MEASURED_TSTEP * (100 - MEASURED_TSTEP_MARGIN_PCT) / 100.0f);      // - margin %
  }
#  endif // MEASURED_TSTEP

#  ifdef STEALTHCHOP
  Serial.println(F("Disabling StealthChop"));
  for(ALL_MOTORS(j)) {
    drivers[j]->stealthChop(0);
  }
#  endif  // STEALTHCHOP
}

void tmc2130_enable_stealthChop() {
  Serial.println(F("enabling StealthChop"));

  HAL_timer_disable_interrupt(0);
  HAL_timer_set_compare(0,2000);
  HAL_timer_enable_interrupt(0);

  // re-enable stealthchop
  for(ALL_MOTORS(j)) {
    drivers[j]->coolstep_min_speed(0);
    drivers[j]->diag1_stall(0);
#ifdef MEASURED_TSTEP
    drivers[j]->THIGH(0);
#endif
#ifdef STEALTHCHOP
    drivers[j]->stealthChop(1);
#endif
  }

  motor_engage();
}

void tmc2130_motor_home() {
  // Backoff
  for (uint32_t i = 0; i < STEPS_PER_MM * 25; ++i) {
    for(ALL_MOTORS(j)) {
      digitalWrite(motors[j].step_pin, HIGH);
      digitalWrite(motors[j].step_pin, LOW);
    }
    delayMicroseconds(45);
  }

  HAL_timer_disable_interrupt(0);
  HAL_timer_set_compare(0,HOMING_OCR1A);

  homing = true;

  motor_disengage();
  tmc2130_disable_stealthChop();
  motor_engage();

  for(ALL_MOTORS(j)) {
    digitalWrite(motors[j].dir_pin, STEPPER_DIR_LOW);
    en[j]=true;
  }

  HAL_timer_enable_interrupt(0);

  while (homing == true) {
    Serial.print(drivers[0]->TSTEP());
    for(ALL_MOTORS(j)) {
      Serial.print('\t');
      Serial.print(digitalRead(motors[j].limit_switch_pin));
    }
    Serial.print("\tstill homing");
  }
  Serial.println("BOTH EN false");
  enable_stealthChop();
}

void tmc2130_homing_sequence() {
  // make homing false when en* are all false at the same time.
  homing = false;

  for(ALL_MOTORS(j)) {
    if(en[j]) {
      digitalWrite(motors[j].step_pin, HIGH);
      digitalWrite(motors[j].step_pin, LOW);
      if (digitalRead(motors[j].limit_switch_pin) == LOW) {
        digitalWrite(motors[j].enable_pin, HIGH);
        en[j] = false;
      } else {
        homing = true;
      }
    }
  }
}

void tmc2130_status() {
   for(ALL_MOTORS(i)) {
      uint32_t drv_status = drivers[i]->DRV_STATUS();
      uint32_t stallValue = (drv_status & SG_RESULT_bm) >> SG_RESULT_bp;
      Serial.print(stallValue, DEC);
      Serial.print('\t');
    }
    Serial.println();
}
#endif  // HAS_TMC2130
