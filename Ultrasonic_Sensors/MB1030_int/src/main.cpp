// /*
// Reading MB1030 using interrupts
// */

#include <Arduino.h>

#define LOOP_ACTIVITY 22

// US 1
#define pwPin1 14
volatile unsigned long pulseStart1 = 0;
volatile float distanceCm1 = 0;
volatile bool US_ready1 = false;

// US 2
#define pwPin2 23
volatile unsigned long pulseStart2 = 0;
volatile float distanceCm2 = 0;
volatile bool US2_ready = false;

void US1_ISR() {
  // Called when pwPin changes
  if (digitalRead(pwPin1) == HIGH) {
    // Rising edge
    pulseStart1 = micros();
    US_ready1 = false;
  } else {
    // Falling edge
    unsigned long pulseWidth = micros() - pulseStart1;
    distanceCm1 = pulseWidth / 57.87;
    US_ready1 = true;
  }
}

// ISR for sensor 2
void IRAM_ATTR US2_ISR() {
  if (digitalRead(pwPin2) == HIGH) {
    pulseStart2 = micros();
    US2_ready = false;
  } else {
    unsigned long pulseWidth = micros() - pulseStart2;
    distanceCm2 = pulseWidth / 57.87;
    US2_ready = true;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LOOP_ACTIVITY, OUTPUT);
  digitalWrite(LOOP_ACTIVITY, LOW);

  pinMode(pwPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pwPin1), US1_ISR, CHANGE);

  pinMode(pwPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pwPin2), US2_ISR, CHANGE);

  Serial.println("MB1030 pulse width distance measurement");
}

void loop() {
  
  digitalWrite(LOOP_ACTIVITY, HIGH);
  if (US_ready1) {
    Serial.print("US1 Distance: ");
    Serial.print(distanceCm1, 2);
    Serial.println(" cm");
    US_ready1 = false;
  } 
  digitalWrite(LOOP_ACTIVITY, LOW);

  digitalWrite(LOOP_ACTIVITY, HIGH);
  if (US2_ready) {
    Serial.print("US2 Dist: ");
    Serial.print(distanceCm2, 2);
    Serial.println(" cm");
    US2_ready = false;
  }
  digitalWrite(LOOP_ACTIVITY, LOW);

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
