#pragma once

#include <stdint.h>

class NovaStepperCtrler
{
public:
  enum MOTOR_IDX
  {
    MOTOR_L = 0;
    MOTOR_R = 1;
  }

  enum MOTOR_DIR
  {
    DIR_L = 0;
    DIR_R = 1;
  }

  // Constructor
  NovaStepperCtrler(int left_dir_pin, int right_dir_pin);

  // Set speed of specfic motor
  void SetVelocity(MOTOR_IDX motor, float vel);

  // Get virtual encoder value
  uint16_t GetEncoder(MOTOR_IDX motor);

private:
   uint8_t dir_pin[2];  // dir pin for stepper motors
}

