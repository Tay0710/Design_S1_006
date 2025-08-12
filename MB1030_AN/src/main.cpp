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

  // Ensure ADC1 usage and best sensitivity
  analogSetPinAttenuation(anPin, ADC_0db); // 0dB -> best sensitivity (~1.1V FS); This is not accurate for a 3.3V supply, however seems to work better for objects that are 40-60cm away. Otherwise objects are read as closer for onjects that are closer (e.g. 30cm is read as 10cm); or further for objects that are further away (80cm is read as 300cm). 
  analogSetWidth(12);                      // 12-bit -> 0..4095
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
 // collect raw data for different attenuations levels at different distances. Then can attempt to find a relationship between raw value and distance.

 // Also maybe look at getting a better ADC chip. This would mean the project is getting a bit too complex than required. This is still the 2nd mapping option, so does not need to be perfect. I think PW will be good enough. 
 
 
  float avgRaw = (float)sum / SAMPLE_COUNT; // Average raw value

  float voltage = (avgRaw / 4095.0) * Vcc;   // Convert raw ADC to voltage

  float distanceCm = voltage / (Vcc / (512.0 * 2.54));   // Convert voltage directly to centimeters

  Serial.println("Raw values Complete!");
  Serial.print(avgRaw);
  Serial.print(",");
  Serial.print(voltage);
  Serial.print(",");
  Serial.print(distanceCm);
  Serial.println(" cm");

  delay(500); // small delay for readability
}
