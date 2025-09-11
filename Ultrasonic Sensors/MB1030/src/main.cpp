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



/*
FreeRTOS using RMT to read multiple ultrasonic sensors in parallel
- Uses RMT peripheral to capture pulse widths from multiple ultrasonic sensors simultaneously
*/
// #include "driver/rmt_rx.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "driver/gpio.h"
// #include <stdio.h>

// #define NUM_ULTRASONICS 5

// // Example GPIO pins for PW outputs
// int ultrasonic_pins[NUM_ULTRASONICS] = {18, 19, 21, 22, 23};
// rmt_channel_handle_t rmt_channels[NUM_ULTRASONICS];

// QueueHandle_t ultrasonic_queue;

// typedef struct {
//     int sensor_id;
//     uint32_t distance_cm;
// } ultrasonic_data_t;

// // Convert pulse width to distance (datasheet: ~147uS per inch ≈ 58uS per cm)
// static inline uint32_t pulse_to_cm(uint32_t pulse_width_us) {
//     return pulse_width_us / 58;
// }

// // ISR-like callback from RMT
// static bool rmt_rx_done_callback(rmt_channel_handle_t channel,
//                                  const rmt_rx_done_event_data_t *edata,
//                                  void *user_ctx)
// {
//     int sensor_id = (int)user_ctx; // sensor index
//     uint32_t pulse_width_us = edata->received_symbols[0].duration0; // high pulse width

//     ultrasonic_data_t data = {
//         .sensor_id = sensor_id,
//         .distance_cm = pulse_to_cm(pulse_width_us)
//     };

//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     xQueueSendFromISR(ultrasonic_queue, &data, &xHigherPriorityTaskWoken);
//     return xHigherPriorityTaskWoken == pdTRUE;
// }

// // Task to process ultrasonic data
// void ultrasonic_ctrl_task(void *arg)
// {
//     ultrasonic_data_t data;
//     while (1) {
//         if (xQueueReceive(ultrasonic_queue, &data, portMAX_DELAY)) {
//             printf("Sensor %d: %lu cm\n", data.sensor_id, data.distance_cm);

//             // TODO:
//             // 1. Log to CSV (only mapping sensors)
//             // 2. Process obstacle avoidance decision
//             // 3. Send SBUS command to STM
//         }
//     }
// }

// void app_main(void)
// {
//     ultrasonic_queue = xQueueCreate(20, sizeof(ultrasonic_data_t));

//     for (int i = 0; i < NUM_ULTRASONICS; i++) {
//         rmt_rx_channel_config_t rx_cfg = {
//             .clk_src = RMT_CLK_SRC_DEFAULT,
//             .resolution_hz = 1 * 1000 * 1000, // 1MHz → 1 tick = 1us
//             .mem_block_symbols = 64,
//             .gpio_num = ultrasonic_pins[i],
//             .flags.invert_in = false,
//         };

//         rmt_new_rx_channel(&rx_cfg, &rmt_channels[i]);

//         rmt_rx_event_callbacks_t cbs = {
//             .on_recv_done = rmt_rx_done_callback,
//         };
//         rmt_rx_register_event_callbacks(rmt_channels[i], &cbs, (void *)i);

//         rmt_enable(rmt_channels[i]);
//         rmt_receive(rmt_channels[i], NULL, 0, NULL); // start receiving
//     }

//     xTaskCreatePinnedToCore(ultrasonic_ctrl_task, "ultrasonic_ctrl", 4096, NULL, 5, NULL, 0);
// }
