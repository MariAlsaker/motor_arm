#include <Arduino.h>

// Initial PID values
long prevT = 0;
float eprev = 0;
float eintegral = 0;


// Set motor direction and speed
void setMotor(int dir, int pwmVal, int pwmPin, int dirPin){
  analogWrite(pwmPin,pwmVal);
  if(dir == 1){
    digitalWrite(dirPin,LOW);
  }
  else if (dir == -1){
    digitalWrite(dirPin,HIGH);
  }
}

float pidController(int target, int32_t pos, int pwm_pin, int dir_pin){

  // PID constants
  float kp = 2;
  float kd = 0.05;
  float ki = 0;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;
  
  // error
  int e = target - pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // Integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if(pwr>255){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }

  // Signal the motor
  setMotor(dir,pwr,pwm_pin,dir_pin);

  // store previus error
  eprev = e;

  return eprev;

}