// // MB1030/src/main.cpp
// // 

// // Similar to the MB1040
// // The same code should work for both. 


// #include <Arduino.h>

// const int pwPin = 20;  // Connect MB1030 PW output to this pin
// const int MAX_SAMPLES  = 50; // Sample size

// struct Sample {
//   unsigned long timestamp; //millis()
//   float distanceCm; 
// };

// Sample samples[MAX_SAMPLES];
// int sampleIndex = 0;
// bool bufferFull = false;

// void setup() {
//   Serial.begin(115200);
//   pinMode(pwPin, INPUT);
//   Serial.println("MB1040 pulse width distance measurement fast sampling");
// }

// void loop() {
//   unsigned long currentMillis = millis();

//   unsigned long duration = pulseIn(pwPin, HIGH, 30000);  // 30 ms timeout

//   float distanceCm;
//   if (duration > 0) {
//     distanceCm = duration / 57.87;
//   } else {
//     distanceCm = 0.00;
//   }

//   // Only store if reading > 0
//   if (distanceCm > 0.0) {
//     samples[sampleIndex].timestamp = currentMillis;
//     samples[sampleIndex].distanceCm = distanceCm;

//     sampleIndex++;

//     if (sampleIndex >= MAX_SAMPLES) {
//       sampleIndex = 0;

//       // Print all samples at once
//       Serial.println("Samples collected: [timestamp(ms), distance(cm), ...]");
//       for (int i = 0; i < MAX_SAMPLES; i++) {
//         Serial.print(samples[i].timestamp);
//         Serial.print(", ");
//         Serial.print(samples[i].distanceCm, 2);
//         if (i < MAX_SAMPLES - 1) Serial.print(", ");
//       }
//       Serial.println("DONE!");
//     }
//   }
//   // No delay here for maximum sampling speed
// }

#include <Arduino.h>

void setup() {
  Serial.begin(115200);                  // Debug
  Serial1.begin(9600, SERIAL_8N1, 21);   // RX on GPIO21
  Serial.println("Reading MB1030 TX output...");
}

void loop() {
  static String buffer = "";

  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\r') { // End of reading
      if (buffer.length() == 4 && buffer[0] == 'R') {
        int distanceInches = buffer.substring(1).toInt();
        float distanceCm = distanceInches * 2.54;

        Serial.print("Distance: ");
        Serial.print(distanceInches);
        Serial.print(" in  |  ");
        Serial.print(distanceCm, 2);
        Serial.println(" cm");
      }
      buffer = ""; // Reset buffer
    } else {
      buffer += c;
    }
  }
}




