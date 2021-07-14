#include "../../configure.h"

#ifdef TARGET_STM32F4

HardwareTimer *timer_instance[NUM_HARDWARE_TIMERS] = { NULL };
bool timer_enabled[NUM_HARDWARE_TIMERS] = { false };


void HAL_init() {
  pinMode(LED_BUILTIN,OUTPUT);
}

void HAL_timer_enable_interrupt(const uint8_t timerIndex) {
  if (HAL_timer_initialized(timerIndex) && !timer_enabled[timerIndex]) {
    //MYSERIAL1.println("HAL_timer_enable_interrupt");
    timer_enabled[timerIndex] = true;
    timer_instance[timerIndex]->attachInterrupt(Step_Handler);
  }
}

bool HAL_timer_interrupt_enabled(const uint8_t timerIndex) {
  return HAL_timer_initialized(timerIndex) && timer_enabled[timerIndex];
}

void HAL_timer_disable_interrupt(const uint8_t timerIndex) {
  if (HAL_timer_interrupt_enabled(timerIndex)) {
    //MYSERIAL1.println("HAL_timer_disable_interrupt");
    timer_instance[timerIndex]->detachInterrupt();
    timer_enabled[timerIndex] = false;
  }
}

void HAL_timer_start(const uint8_t timerIndex) {
  if(HAL_timer_initialized(timerIndex)) return;

  MYSERIAL1.println("HAL_timer_start");
  timer_instance[timerIndex] = new HardwareTimer(STEP_TIMER_DEV);
  timer_instance[timerIndex]->pause();
  timer_instance[timerIndex]->setMode(2,TIMER_OUTPUT_COMPARE);
  timer_instance[timerIndex]->setPrescaleFactor(STEPPER_TIMER_PRESCALE); //the -1 is done internally
  timer_instance[timerIndex]->setOverflow(_MIN(hal_timer_t(HAL_TIMER_TYPE_MAX), (HAL_TIMER_RATE) / (STEPPER_TIMER_PRESCALE) /* /frequency */), TICK_FORMAT);
  HAL_timer_enable_interrupt(timerIndex);
  timer_instance[timerIndex]->refresh();
  timer_instance[timerIndex]->resume(); // First call to resume() MUST follow the attachInterrupt()
  HAL_NVIC_SetPriority(STEP_TIMER_IRQ_NAME, STEP_TIMER_IRQ_PRIO, 0);
}

#endif // TARGET_STM32F4