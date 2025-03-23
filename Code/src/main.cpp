#include <Arduino.h>
#include <IBusBM.h>

IBusBM Ibus;

// Motor PWM Pins
#define LEFT_FWD_PIN   18
#define LEFT_REV_PIN   22
#define RIGHT_FWD_PIN  19
#define RIGHT_REV_PIN  23

//default values to be used
const uint16_t MID_VAL = 1500;
const uint16_t LOW_VAL = 1000;
const uint16_t HIGH_VAL = 2000;

//defining global controller values
uint16_t ppm1 = MID_VAL;
uint16_t ppm2 = MID_VAL;

void channelUpdate() {
  ppm1 = Ibus.readChannel(1);
  ppm2 = Ibus.readChannel(2);
}

//Controls speed of the robot
void speedControl() {
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  
  Ibus.begin(Serial2);

  // Motor pins
  pinMode(LEFT_FWD_PIN, OUTPUT);
  pinMode(LEFT_REV_PIN, OUTPUT);
  pinMode(RIGHT_FWD_PIN, OUTPUT);
  pinMode(RIGHT_REV_PIN, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  channelUpdate();
}

