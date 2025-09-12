// MB1030/src/main.cpp
// 

// Similar to the MB1040
// The same code should work for both. 


#include <Arduino.h>

const int pwPin = 20;  // Connect MB1030 PW output to this pin
const int MAX_SAMPLES  = 50; // Sample size

struct Sample {
  unsigned long timestamp; //millis()
  float distanceCm; 
};

Sample samples[MAX_SAMPLES];
int sampleIndex = 0;
bool bufferFull = false;

void setup() {
  Serial.begin(115200);
  pinMode(pwPin, INPUT);
  Serial.println("MB1040 pulse width distance measurement fast sampling");
}

void loop() {
  unsigned long currentMillis = millis();

  unsigned long duration = pulseIn(pwPin, HIGH, 30000);  // 30 ms timeout

  float distanceCm;
  if (duration > 0) {
    distanceCm = duration / 57.87;
  } else {
    distanceCm = 0.00;
  }

  // Only store if reading > 0
  if (distanceCm > 0.0) {
    samples[sampleIndex].timestamp = currentMillis;
    samples[sampleIndex].distanceCm = distanceCm;

    sampleIndex++;

    if (sampleIndex >= MAX_SAMPLES) {
      sampleIndex = 0;

      // Print all samples at once
      Serial.println("Samples collected: [timestamp(ms), distance(cm), ...]");
      for (int i = 0; i < MAX_SAMPLES; i++) {
        Serial.print(samples[i].timestamp);
        Serial.print(", ");
        Serial.print(samples[i].distanceCm, 2);
        if (i < MAX_SAMPLES - 1) Serial.print(", ");
      }
      Serial.println("DONE!");
    }
  }
  // No delay here for maximum sampling speed
}

