
#include "tmc2130.h"

#ifdef HAS_TMC2130

TMC2130Stepper driver_0 = TMC2130Stepper(CS_PIN_0);
TMC2130Stepper driver_1 = TMC2130Stepper(CS_PIN_1);

bool en0, en1 = false;
bool homing = false;
// bool vsense;


uint16_t rms_current(uint8_t CS, float Rsense = 0.11) {
  return (float)(CS+1)/32.0 * (vsense?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}


void tmc_setup(TMC2130Stepper &driver) {
  driver.begin();
  driver.setCurrent(CURRENT, R_SENSE, HOLD_MULTIPLIER);
  driver.microsteps(MICROSTEPS);
  driver.blank_time(24);
  driver.off_time(2);
  driver.interpolate(true);
  driver.power_down_delay(128); // ~2s until driver lowers to hold current
  driver.hysteresis_start(5);
  driver.hysteresis_end(2);
  
  driver.diag1_stall(0);  // use diag1 to signal stall event
  driver.diag1_active_high(0);  // I want active low
  driver.sg_stall_value(STALL_VALUE);
  
  driver.coolstep_min_speed(0);
  driver.THIGH(0);
  driver.vsense(CURRENT >= 540 ? 1 : 0);
  
  #if defined(STEALTHCHOP)
    driver.stealth_freq(1); // f_pwm = 2/683 f_clk
    driver.stealth_autoscale(1);
    driver.stealth_gradient(5);
    driver.stealth_amplitude(255);
    driver.stealthChop(1);
    //#if defined(HYBRID_THRESHOLD)
    //  driver.stealth_max_speed(12650000UL*MICROSTEPS/(256*HYBRID_THRESHOLD*STEPS_PER_MM));
    //#endif
  #endif
  driver.GSTAT(0); // Clear GSTAT
}


void disable_stealthChop() {
  // disable stealthchop
  driver_0.coolstep_min_speed(0xFFFFF);
  driver_1.coolstep_min_speed(0xFFFFF);
  driver_0.diag1_stall(1);
  driver_1.diag1_stall(1);
  
  #ifdef MEASURED_TSTEP
  Serial.println("measured_tstep is inputting");
  driver_0.TCOOLTHRS((float)MEASURED_TSTEP*(100+MEASURED_TSTEP_MARGIN_PCT)/100.0f);  // + margin %
  driver_1.TCOOLTHRS((float)MEASURED_TSTEP*(100+MEASURED_TSTEP_MARGIN_PCT)/100.0f);  // + margin %
  driver_0.THIGH    ((float)MEASURED_TSTEP*(100-MEASURED_TSTEP_MARGIN_PCT)/100.0f);  // - margin %
  driver_1.THIGH    ((float)MEASURED_TSTEP*(100-MEASURED_TSTEP_MARGIN_PCT)/100.0f);  // - margin %
  #endif MEASURED_TSTEP
  
  #ifdef STEALTHCHOP
  Serial.println("Disabling StealthChop");
  driver_0.stealthChop(0);
  driver_1.stealthChop(0);
  #endif // STEALTHCHOP
}


void enable_stealthChop() {  
  cli();
  TCCR1A = 0;   // set entire TCCR1A register to 0
  TCNT1  = 0;   // set the overflow clock to 0
  // set compare match register to desired timer count
  OCR1A = 2000;  // 1ms
  TCCR1B = (1 << WGM12);  // turn on CTC mode
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);    // Set 8x prescaler
  TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt
  sei();
  
  // re-enable stealthchop
  driver_0.coolstep_min_speed(0);
  driver_1.coolstep_min_speed(0);
  driver_0.diag1_stall(0);
  driver_1.diag1_stall(0);
  
  #ifdef MEASURED_TSTEP
    driver_0.THIGH(0);
    driver_1.THIGH(0);
  #endif MEASURED_TSTEP
  
  #ifdef STEALTHCHOP
    Serial.println("enabling StealthChop");
    driver_0.stealthChop(1);
    driver_1.stealthChop(1);
  #endif // STEALTHCHOP
  
  motor_engage();
}

void motor_home() {
  //Backoff
  for (uint32_t i = 0; i < STEPS_PER_MM * 25; ++i) {
    digitalWrite(MOTOR_0_STEP_PIN, HIGH);
    digitalWrite(MOTOR_1_STEP_PIN, HIGH);
    digitalWrite(MOTOR_0_STEP_PIN, LOW);
    digitalWrite(MOTOR_1_STEP_PIN, LOW);
    delayMicroseconds(45);
  }
  
  cli();
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = HOMING_OCR1A; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B = (1 << WGM12);    // turn on CTC mode
  TCCR1B |= (1 << CS11);    // Set CS11 bits for 8 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  homing = true;
  
  motor_disengage();
  disable_stealthChop();
  motor_engage();
  
  digitalWrite(MOTOR_0_DIR_PIN, STEPPER_DIR_LOW);
  digitalWrite(MOTOR_1_DIR_PIN, STEPPER_DIR_LOW);
  en1 = true;
  en0 = true;
  sei();  
}


#endif  // HAS_TMC2130
