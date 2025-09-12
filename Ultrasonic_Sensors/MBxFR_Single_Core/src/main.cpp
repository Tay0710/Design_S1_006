/*
FreeRTOS using RMT to read multiple ultrasonic sensors in parallel
- Uses RMT peripheral to capture pulse widths from multiple ultrasonic sensors simultaneously
- Stores distances in a buffer, then prints them at the end of each loop
- Prints FreeRTOS runtime stats
*/

#define configGENERATE_RUN_TIME_STATS 1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  // optional
#define portGET_RUN_TIME_COUNTER_VALUE() esp_cpu_get_ccount()

#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_system.h"

#define NUM_ULTRASONICS 5

// Example GPIO pins for PWM outputs
int ultrasonic_pins[NUM_ULTRASONICS] = {18, 19, 21, 22, 23};
rmt_channel_handle_t rmt_channels[NUM_ULTRASONICS];

QueueHandle_t ultrasonic_queue;

typedef struct {
    int sensor_id;
    uint32_t distance_cm;
} ultrasonic_data_t;

// Convert pulse width to distance
static inline uint32_t pulse_to_cm(uint32_t pulse_width_us) {
    return pulse_width_us / 58;
}

// RMT callback
static bool rmt_rx_done_callback(rmt_channel_handle_t channel,
                                 const rmt_rx_done_event_data_t *edata,
                                 void *user_ctx)
{
    int sensor_id = (int)user_ctx;
    if (edata->num_symbols == 0) return false;

    uint32_t pulse_width_us = edata->received_symbols[0].duration0;
    ultrasonic_data_t data = { .sensor_id = sensor_id, .distance_cm = pulse_to_cm(pulse_width_us) };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(ultrasonic_queue, &data, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}

// Task to process ultrasonic data and print at the end of each loop
void ultrasonic_ctrl_task(void *arg)
{
    ultrasonic_data_t data;
    char stats_buffer[512];
    char distance_buffer[256];
    uint32_t distances[NUM_ULTRASONICS] = {0};
    int received_count = 0;
    int counter = 0;

    while (1) {
        // Wait for each sensor's data
        if (xQueueReceive(ultrasonic_queue, &data, portMAX_DELAY)) {
            distances[data.sensor_id] = data.distance_cm;
            received_count++;

            // Once we've received all sensor data, print it
            if (received_count >= NUM_ULTRASONICS) {
                // Build distance buffer
                int len = 0;
                for (int i = 0; i < NUM_ULTRASONICS; i++) {
                    len += snprintf(distance_buffer + len, sizeof(distance_buffer) - len,
                                    "Sensor %d: %lu cm  ", i, distances[i]);
                }
                distance_buffer[len] = '\0';  // null-terminate
                printf("%s\n", distance_buffer);

                received_count = 0;  // reset for next cycle
            }

            // Print runtime stats every 50 measurements
            counter++;
            if (counter >= 50) {
                vTaskGetRunTimeStats(stats_buffer);
                printf("\n--- FreeRTOS Task Runtime Stats ---\n%s\n", stats_buffer);
                counter = 0;
            }
        }
    }
}

void app_main(void)
{
    // Create queue
    ultrasonic_queue = xQueueCreate(20, sizeof(ultrasonic_data_t));

    // Initialize RMT channels
    for (int i = 0; i < NUM_ULTRASONICS; i++) {
        rmt_rx_channel_config_t rx_cfg = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 1 * 1000 * 1000, // 1 tick = 1us
            .mem_block_symbols = 64,
            .gpio_num = ultrasonic_pins[i],
            .flags.invert_in = false,
        };

        rmt_new_rx_channel(&rx_cfg, &rmt_channels[i]);

        rmt_rx_event_callbacks_t cbs = { .on_recv_done = rmt_rx_done_callback };
        rmt_rx_register_event_callbacks(rmt_channels[i], &cbs, (void *)i);

        rmt_enable(rmt_channels[i]);
        rmt_receive(rmt_channels[i], NULL, 0, NULL);
    }

    // Create task on Core 1
    xTaskCreatePinnedToCore(ultrasonic_ctrl_task, "ultrasonic_ctrl", 4096, NULL, 5, NULL, 1);
}
