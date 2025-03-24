#include <Arduino.h>
#include <IBusBM.h>

IBusBM Ibus;

// Motor PWM Pins
#define LEFT_FWD_PIN   18
#define LEFT_REV_PIN   22
#define RIGHT_FWD_PIN  19
#define RIGHT_REV_PIN  23


// Configuration (adjustable percentages)
#define CENTER_THRESHOLD_PCT  10    // ±10% from center considered "neutral"
#define STROKE_THRESHOLD_PCT 20    // 20% from center required for stroke
#define CALIBRATION_TIME     3000  // 5-second calibration
#define SEQUENCE_COOLDOWN    600   // delay between strokes
#define HISTORY_SIZE         7     // Number of strokes in activation sequence
#define THROTLLE_DEADZONE    8    // ± from center considered "neutral"
#define STEERING_DEADZONE    10    // ± from center considered "neutral"
#define MAXZONE              15    // ± from max val to go full power


//default values to be used
const uint16_t MID_VAL = 1500;
const uint16_t LOW_VAL = 1000;
const uint16_t HIGH_VAL = 2000;

//defining global controller values
uint16_t ppm1 = MID_VAL;
uint16_t ppm2 = MID_VAL;

//defining global speed value
uint8_t speed = 0;

void channelUpdate() {
  ppm1 = Ibus.readChannel(1);
  ppm2 = Ibus.readChannel(2);
}


struct Calibration {
  uint16_t min1, max1, mid1;
  uint16_t min2, max2, mid2;
} calibration = { 
  LOW_VALUE, HIGH_VALUE, CENTER_VALUE,
  LOW_VALUE, HIGH_VALUE, CENTER_VALUE 
};

Stroke strokeHistory[HISTORY_SIZE];
uint8_t historyIndex = 0;
uint32_t lastStrokeTime = 0;
bool calibrating = false;
uint32_t calibrationStart = 0;

4
Stroke detectStroke() {
  static uint16_t lastMid1 = calibration.mid1;
  static uint16_t lastMid2 = calibration.mid2;
  
  const int16_t diff1 = ppm1_value - lastMid1;
  const int16_t diff2 = ppm2_value - lastMid2;

  if ( ((uint16_t) abs(diff1)) < CENTER_THRESHOLD && ((uint16_t) abs(diff2)) < CENTER_THRESHOLD) return NONE;
  
  if (abs(diff1) > abs(diff2)) {
    return (diff1 > 0) ? UP : DOWN;
  }
  return (diff2 > 0) ? RIGHT : LEFT;
}

void updateStrokeHistory(Stroke stroke) {
  if (stroke == NONE || millis() - lastStrokeTime < SEQUENCE_COOLDOWN) return;
  
  // Shift history
  for (uint8_t i = 0; i < HISTORY_SIZE-1; i++) {
    strokeHistory[i] = strokeHistory[i+1];
  }
  strokeHistory[HISTORY_SIZE-1] = stroke;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  lastStrokeTime = millis();
}

bool checkCalibSequence() {
  for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
    if (strokeHistory[i] != CALIBRATION_SEQUENCE[i]) return false;
  }
  return true;
}

bool checkTankSequence() {
  for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
    if (strokeHistory[i] != TANK_SEQUENCE[i]) return false;
  }
  return true;
}

void startCalibration() {
  calibration.min1 = calibration.max1 = ppm1_value;
  calibration.min2 = calibration.max2 = ppm2_value;
  calibrationStart = millis();
  calibrating = true;
  digitalWrite(LED_PIN, HIGH);
}

void updateCalibration() {
  calibration.min1 = min(calibration.min1, ppm1_value);
  calibration.max1 = max(calibration.max1, ppm1_value);
  calibration.min2 = min(calibration.min2, ppm2_value);
  calibration.max2 = max(calibration.max2, ppm2_value);
}

void finalizeCalibration() {
  // Capture new center after 2 seconds of stability

  Serial.println("Min Max done");
  Serial.print("Min1: ");
  Serial.print(calibration.min1);
  Serial.print(" Max1: ");
  Serial.println(calibration.max1);
  Serial.print("Min2: ");
  Serial.print(calibration.min2);
  Serial.print("Max2: ");
  Serial.println(calibration.max2);

  uint32_t start = millis();
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize Boyer-Moore variables for both channels
  uint16_t major1 = calibration.mid1, major2 = calibration.mid2;
  int count1 = 1, count2 = 1;
  
  while (millis() - start < 2000) {
      // Update channel 1
      if (ppm1_value == major1) {
          count1++;
      } else {
          count1--;
          if (count1 == 0) {
              major1 = ppm1_value;
              count1 = 1;
          }
      }
  
      // Update channel 2
      if (ppm2_value == major2) {
          count2++;
      } else {
          count2--;
          if (count2 == 0) {
              major2 = ppm2_value;
              count2 = 1;
          }
      }
      delay(5);
  }
  
  // Final verification pass (optional but recommended)
  // You might want to add a second pass to confirm the majority
  // if your data might not have a true majority
  
  calibration.mid1 = major1;
  calibration.mid2 = major2;
  digitalWrite(LED_PIN, LOW);
  
}





//Controls speed of the robot
void speedControl(ppm) {
  if ppm < MID_VAL {
  return map(ppm, MID_VAL, LOW_VAL, 0, 255);
  }
  else {
  return map(ppm, MID_VAL, HIGH_VAL, 0, 255);
  }
}

void motorControl(uint8_t pin, int speed, ppm) {
  speed = speedControl(ppm);
  speed = constrain(speed, 0, 255);
  analogWrite(pin, speed);
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

