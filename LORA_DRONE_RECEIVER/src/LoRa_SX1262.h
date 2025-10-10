#include <SX126x-Arduino.h>
#include <SPI.h>

// ESP32-S3 - SX126x pin configuration
int PIN_LORA_RESET = 1;	 // LORA RESET
int PIN_LORA_DIO_1 = 2; // LORA DIO_1
int PIN_LORA_BUSY = 3;	 // LORA SPI BUSY
int PIN_LORA_NSS = 10;	 // LORA SPI CS
int PIN_LORA_SCLK = 12;	 // LORA SPI CLK
int PIN_LORA_MISO = 11;	 // LORA SPI MISO
int PIN_LORA_MOSI = 13;	 // LORA SPI MOSI

// Define LoRa parameters
#define RF_FREQUENCY 915000000	// Hz 
#define TX_OUTPUT_POWER 22		// dBm (Note: maximum power of SX1262 is 22 dBm)
#define LORA_BANDWIDTH 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12], SF7 is fastest, SF12 is slowest but greatest range and more robust
#define LORA_CODINGRATE 3		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 16	// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0	// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 5000 
#define TX_TIMEOUT_VALUE 5000 


