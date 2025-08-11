// Analog reading from MB1030

// Note that esp32 uses 12-bit ADC resolution, so the raw value will be between 0 and 4095.
// This ADC can handle a max of 3.3V, so DONT USE 5V. Otherwise risking damaging the ESP32.

#include <Arduino.h>

const int anPin = 34;  // ADC-capable pin
const float Vcc = 3.316; // Sensor supply voltage (change to 3.3 if powered from 3.3V)
const int SAMPLE_COUNT = 200; // sample size. 

int rawValues[SAMPLE_COUNT];  // Array to store raw samples

void setup() {
  Serial.begin(115200);
  pinMode(anPin, INPUT);
  Serial.println("MB1030 analog distance measurement (cm)");
}


// More testing is required. The returned raw reading does not seems to go higher than 1.2V. About 10cm on the oscilliscope is reading at 50mV. 

void loop() {

  for (int i = 0; i < SAMPLE_COUNT; i++){
    rawValues[i] = analogRead(anPin);   // Read raw ADC value (0–4095 on ESP32)
  }

  unsigned long sum = 0; 
  // Taking samples
  for (int i = 0; i < SAMPLE_COUNT; i++){
    Serial.print(rawValues[i]);
    Serial.print(",");
    sum += rawValues[i];
  }

  // float avgRaw = (float)sum / SAMPLE_COUNT; // Average raw value

  // float voltage = (avgRaw / 4095.0) * Vcc*10^3;   // Convert raw ADC to voltage

  // float distanceCm = voltage / (Vcc / (512.0 * 2.54));   // Convert voltage directly to centimeters

  // Serial.println("Raw values Complete!");
  // Serial.print(avgRaw);
  // Serial.print(",");
  // Serial.print(voltage);
  // Serial.print(",");
  // Serial.print(distanceCm);
  // Serial.println(" cm");

  delay(100); // small delay for readability
}
