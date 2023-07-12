#include <stdint.h>
#include <Arduino.h>
#include "ArmState.h"
#include "PIDdiy.h"

#define ENCA 2 // Encoder A
#define ENCB 4 // Encoder B
#define TOP_SW 6 // Register top pos - Connect to pull down
#define PRESSURE_SW 10 // Pain stimuli - signal from mother board
#define DIR_PIN 12 // To motor
#define PWM_PIN 3 // To motor
#define CURRENT_SENSING A0 

enum ArmState arm_state = ARM_INIT; // Set first state

// Subroutine variables
uint32_t button_subroutine_pressure = 0;
uint32_t motor_stall_timer = 0;
uint32_t regulator_subroutine_timer = 0;
uint32_t arm_timer = 0;

// Position variables
int16_t target;         // Declear target value
int32_t pos = 0;        // Position of arm/motor
int16_t highPos = -600; // Arm in high position
int16_t lowPos = -3800; // Arm in low position

// Current variables
uint16_t currentRaw;             // the raw analogRead ranging from 0-1023
uint16_t samples[256];           // Store curruent reading
uint16_t samples_idx = 0;        // Indexing for current array
uint16_t currentThreshold = 200; // Limit for motor stall 

// Calculate avarage of array
float average (uint16_t * array, int len)
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

// Interrupt service routine
void readEncoder(){ 
  int b = digitalRead(ENCB);
  if (b>0){
    pos++;
  }
  else {
    pos--;
  }
}

void setup() {

  Serial.begin(9600);

  // Encoder
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  // Motor
  pinMode(DIR_PIN,OUTPUT);
  pinMode(PWM_PIN,OUTPUT);

  // Switches
  pinMode(TOP_SW,INPUT);
  pinMode(PRESSURE_SW,INPUT);

  // Read encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

  // Set PWM frequence of 31372.55 Hz
  TCCR2B = (TCCR2B & B11111000) | B00000001;

  // Set ininital relative time
  arm_timer = millis();                    
  button_subroutine_pressure = millis();
  motor_stall_timer = millis();
  regulator_subroutine_timer = millis();
}

void loop() {

// Current sensing subroutine
if (millis() - motor_stall_timer > 10)
{
  motor_stall_timer = millis();
  currentRaw = analogRead(CURRENT_SENSING);
  samples[samples_idx] = currentRaw;
  samples_idx++;
  if (samples_idx == 100)
  {
    uint16_t avg = average(samples,100);
    samples_idx = 0;
    if (avg >= currentThreshold)
    {
      Serial.println("Motor Stall");
      motor_stall_timer = millis();
      target = pos;
      arm_state = ARM_IDLE;
    }
    
  }
}

// Subroutine for force sensor
if(millis() - button_subroutine_pressure > 10){
  button_subroutine_pressure = millis();
  if(digitalRead(PRESSURE_SW)){
    arm_state = ARM_TWITCH_UP;
  }
}

// Regulator -- FSM
if(millis() - regulator_subroutine_timer > 10){
  regulator_subroutine_timer = millis();

  switch(arm_state){
    
    case ARM_INIT: // Wait for 1 second 
      if (millis() - arm_timer > 1000){
          arm_timer = millis();
          arm_state = ARM_SET_TOP_POS;
      }
      break;

    case ARM_SET_TOP_POS: // Calibrating position
      digitalWrite(DIR_PIN,LOW);
      analogWrite(PWM_PIN,180);
      if (digitalRead(TOP_SW)){
        noInterrupts();
        pos = 0;
        interrupts();
        arm_timer = millis();
        arm_state = ARM_IDLE;
      }    
      break;

    case ARM_IDLE:
      pidController(lowPos, pos, PWM_PIN, DIR_PIN);
      if (digitalRead(PRESSURE_SW)){
        arm_timer = millis();
        arm_state = ARM_TWITCH_UP;
      }
      break;

    case ARM_TWITCH_UP:
      pidController(highPos, pos, PWM_PIN, DIR_PIN);
      if (millis() - arm_timer > 3000) {
        arm_timer = millis();
        arm_state = ARM_IDLE;
      }
      break;
  }
}
}