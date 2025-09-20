// // MB1030/src/main.cpp
// // 

// // Similar to the MB1040
// // The same code should work for both. 


#include <Arduino.h>

const int pwPin = 14;  // MB1030 PW output

void setup() {
  Serial.begin(115200);
  pinMode(pwPin, INPUT);
  Serial.println("MB1040 pulse width distance measurement");
}

void loop() {
  // Measure pulse width (timeout 30 ms)
  unsigned long duration = pulseIn(pwPin, HIGH, 30000);

  if (duration > 0) {
    // Convert to cm (per sensor datasheet)
    float distanceCm = duration / 57.87;
    unsigned long now = millis();

    Serial.print("Time: ");
    Serial.print(now);
    Serial.print(" ms | Distance: ");
    Serial.print(distanceCm, 2);
    Serial.println(" cm");
  } else {
    Serial.println("No echo");
  }

  // Optional small delay so the serial output is readable
  delay(20);   // Adjust or remove for fastest sampling
}


// #include <Arduino.h>

// void setup() {
//   Serial.begin(115200);                  // Debug
//   Serial1.begin(9600, SERIAL_8N1, 21);   // RX on GPIO21
//   Serial.println("Reading MB1030 TX output...");
// }

// void loop() {
//   static String buffer = "";

//   while (Serial1.available()) {
//     char c = Serial1.read();

//     if (c == '\r') { // End of reading
//       if (buffer.length() == 4 && buffer[0] == 'R') {
//         int distanceInches = buffer.substring(1).toInt();
//         float distanceCm = distanceInches * 2.54;

//         Serial.print("Distance: ");
//         Serial.print(distanceInches);
//         Serial.print(" in  |  ");
//         Serial.print(distanceCm, 2);
//         Serial.println(" cm");
//       }
//       buffer = ""; // Reset buffer
//     } else {
//       buffer += c;
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




