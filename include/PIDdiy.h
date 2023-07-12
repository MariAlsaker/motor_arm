#ifndef PIDdiy_H
#define PIDdiy_H

void setMotor(int dir, int pwmVal, int pwmPin, int dirPin);
float pidController(int target, int32_t pos, int pwm_pin, int dir_pin);

#endif