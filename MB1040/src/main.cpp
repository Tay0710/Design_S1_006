// MB1040/src/main.cpp
// 

// Pinout assignment for MB1040
// Ground, Voltage in, Transmit, Receive, Analog, Pulse width, BW
// GND; VCC; Tx; Rx; AN; PW; BW

// AN: (Vcc/512) per inch. E.g. 5V/512 = 0.009765625V per inch
// PW: range is 147uS per inch

// The MB1040 has a 250mS delay after power-up. 
// Max reading rate is once per 49mS, which is about 20Hz.


#include <Arduino.h>

const int pwPin = 4;  // Connect MB1040 PW output to this pin
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
