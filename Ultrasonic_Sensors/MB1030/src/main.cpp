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

// #include <Arduino.h>

// void setup() {
//   Serial.begin(115200);                  // Debug
//   Serial1.begin(9600, SERIAL_8N1, 21, -1);   // RX on GPIO21
//   Serial.println("Reading MB1030 TX output...");
  
// }

// void loop() {
//   static String buffer = "";

//   while (Serial1.available()) {
//     Serial.println("in While");
//     char c = Serial1.read();
//     Serial.println(c);
//     Serial.println(buffer);
//     Serial.println("After Read");
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
// }






#include <Arduino.h>

// int pwPin1 = 21;  // MB1030 #1 pulse width output
// int pwPin2 = 22;  // MB1030 #2 pulse width output

const int pwPins[] = {21, 22};  // Array of MB1030 PW pins
const int numSensors = sizeof(pwPins) / sizeof(pwPins[0]);

void ultrasonicTask(void *pvParameters) {
  // int pwPin = *((int*)pvParameters);
    (void)pvParameters;  // unused


  for (;;) {  // infinite loop for the task
    unsigned long currentMillis = millis();
    for (int i = 0; i < numSensors; i++) {
      Serial.print("Reading sensor on pin ");
      Serial.println(pwPins[i]);
      unsigned long duration = pulseIn(pwPins[i], HIGH, 37000);  // 30 ms timeout

      Serial.print("Duration: ");
      Serial.println(duration);
      
      float distanceCm = 0.0;
      if (duration > 0) {
        distanceCm = duration / 57.87;
      }

      if (distanceCm > 0.0) {
        Serial.print("Sensor on pin ");
        Serial.print(pwPins[i]);
        Serial.print(" -> ");
        Serial.print(currentMillis);
        Serial.print(" ms, ");
        Serial.println(distanceCm, 2);
      }

    }
  vTaskDelay(1 / portTICK_PERIOD_MS); // tiny delay to yield CPU

  }

}

void setup() {
  Serial.begin(115200);
Serial.println(numSensors);


  // pinMode(pwPin1, INPUT);
  // pinMode(pwPin2, INPUT);
  // Set all sensor pins as input
  for (int i = 0; i < numSensors; i++) {
    pinMode(pwPins[i], INPUT);
    Serial.print("Configured pin ");
    Serial.print(pwPins[i]);
  }

  Serial.println("Starting ultrasonic FreeRTOS tasks...");

  // // Create a FreeRTOS task for ultrasonic #1
  // xTaskCreatePinnedToCore(
  //   ultrasonicTask,   // Task function
  //   "Ultrasonic1",    // Task name
  //   2048,             // Stack size
  //   &pwPin1,          // Parameter (pin number)
  //   1,                // Priority
  //   NULL,              // Task handle
  //   1                 // Run on core 1
  // );

  //     // Create task for second sensor
  //   int* pin2Ptr = new int(pwPin2);
  //   xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic2", 2048, pin2Ptr, 1, NULL, 1);

  // Single FreeRTOS task handles all sensors
  xTaskCreatePinnedToCore(
    ultrasonicTask,
    "Ultrasonics",
    4096,
    NULL,
    1,
    NULL,
    1  // Run on core 1
  );

}

void loop() {
  // Nothing here, FreeRTOS tasks handle everything
}
