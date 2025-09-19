// /*
// Reading MB1030 using interrupts
// */

#include <Arduino.h>

#define pwPin 14
volatile unsigned long pulseStart = 0;
volatile float distanceCm = 0;
volatile bool US_ready = false;

void US1_ISR() {
  // Called when pwPin changes
  if (digitalRead(pwPin) == HIGH) {
    // Rising edge
    pulseStart = micros();
    US_ready = false;
  } else {
    // Falling edge
    unsigned long pulseWidth = micros() - pulseStart;
    distanceCm = pulseWidth / 57.87;
    US_ready = true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(pwPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pwPin), US1_ISR, CHANGE);
  Serial.println("MB1030 pulse width distance measurement");
}

void loop() {
  if (US_ready) {
    Serial.print("Distance: ");
    Serial.print(distanceCm, 2);
    Serial.println(" cm");
    US_ready = false;
  } 
}

// #include <Arduino.h>

// #define PW_PIN 14
// #define MAX_SAMPLES 50

// volatile unsigned long pulseStart = 0;
// volatile unsigned long pulseBuffer[MAX_SAMPLES];
// volatile uint8_t pulseWriteIndex = 0;
// volatile bool newPulse = false;

// void IRAM_ATTR pwISR() {
//   if (digitalRead(PW_PIN)) {       // Rising edge
//     pulseStart = micros();
//   } else {                          // Falling edge
//     unsigned long pw = micros() - pulseStart;
//     pulseBuffer[pulseWriteIndex] = pw;
//     pulseWriteIndex = (pulseWriteIndex + 1) % MAX_SAMPLES;
//     newPulse = true;
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   pinMode(PW_PIN, INPUT);
//   attachInterrupt(digitalPinToInterrupt(PW_PIN), pwISR, CHANGE);
// }

// void loop() {
//   if (newPulse) {
//     noInterrupts();                 // safely read buffer
//     uint8_t index = (pulseWriteIndex + MAX_SAMPLES - 1) % MAX_SAMPLES;
//     unsigned long pw = pulseBuffer[index];
//     newPulse = false;
//     interrupts();

//     float distanceCm = pw / 58.0;  // MB1030 datasheet formula
//     Serial.print("Distance: ");
//     Serial.print(distanceCm);
//     Serial.println(" cm");
//   }
// }
