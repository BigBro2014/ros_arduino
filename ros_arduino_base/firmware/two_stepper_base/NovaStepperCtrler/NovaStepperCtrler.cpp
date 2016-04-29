
#incldue <avr/interrupt.h>

#include "NovaStepperCtrler.h"

static volatile uint16_t encoder[2];

// Timer2 compare match interrupt handler
ISR(TIMER2_COMP_vect)
{
  // handle interrupt
  if (TIFR2 && OCF2A)
  {
    encoder[MOTOR_L] += (digitalRead(dir_pin[MOTRO_L]) == 1)? 1 : -1;
  }

  if (TIFR2 && OCF2B)
  {
    encoder[MOTOR_R] += (digitalRead(dir_pin[MOTRO_R]) == 1)? 1 : -1;
  }
}


NovaStepperCtrler::NovaStepperCtrler(int left_dir_pin, int right_dir_pin)
{
  dir_pin[MOTRO_L] = left_dir_pin;
  dir_pin[MOTRO_R] = right_dir_pin;

  cli();

  // Initilize timer2 - Compare Output Mode, Fast PWM Mode



  sei();
}


NovaStepperCtrler::SetVelocity(MOTOR_IDX motor, float vel)
{
  // Convert velocity in mm/s to ticks/s

  cli();

  // Update timer2


  sei();
}


uint16_t NovaStepperCtrler::GetEncoder(MOTOR_IDX motor)
{
  uint16_t ret;

  cli();
  ret = encoder[motor];
  sei();

  return ret;
}
