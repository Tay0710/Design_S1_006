// Adapted from PingPong Example from SX126x-Arduino Library
// https://github.com/beegee-tokyo/SX126x-Arduino/blob/master/examples/PingPong/PingPong.ino 

#include <Arduino.h>
#include <Ticker.h>

#include "LoRa_SX1262.h"

static RadioEvents_t RadioEvents;
hw_config hwConfig;
time_t lastMessage;

Ticker loraTimer;

#define FAILSAFE_LORA_TIMEOUT 5000 // ms
#define LORA_TIMER_UDPATE_RATE 1000 // check every 1000 ms 

// TODO: make flag system

void triggerFailsafe() {
  Serial.println("Failsafe triggered. Land drone.");
  // rcChannels[AUX3] = 1800;

}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
  Serial.println("On Rx Timeout CB");
  Serial.printf("OnRxTimeout: No message received in last %d ms \n", RX_TIMEOUT_VALUE);

  // TODO: check if failsafe is required if LORA never connects in the first place

}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Serial.println("OnRxDone:");

  // Time from last message
  time_t now = millis();
  double time_between_message = (now - lastMessage) / 1000.0;

  Serial.print("Message: ");
  Serial.println((const char *)payload);
  Serial.printf("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);
  Serial.printf("Time from last message=%.2f seconds \n", time_between_message);

  String payload_str = (const char *)payload;

  if (payload_str == "STOP") {
    triggerFailsafe();
  }

  lastMessage = millis();
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnRxError(void)
{
  Serial.println("OnRxError CB");
}

void setupLoRaModule()
{
  // Define the HW configuration between MCU and SX126x
  hwConfig.CHIP_TYPE = SX1262_CHIP;         // Example uses an eByte E22 module with an SX1262
  hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET // Note: times out way more often...
  hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;     // LORA SPI CS
  hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;   // LORA SPI CLK
  hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;   // LORA SPI MISO
  hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
  hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;   // LORA SPI BUSY
  hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;   // LORA SPI MOSI
  hwConfig.USE_DIO2_ANT_SWITCH = false;     // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
  hwConfig.USE_DIO3_TCXO = false;           // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
  hwConfig.USE_DIO3_ANT_SWITCH = false;     // Only Insight ISP4520 module uses DIO3 as antenna control
  hwConfig.USE_RXEN_ANT_PWR = false;        // RXEN is used as power for antenna switch

  // Initialize the LoRa chip
  Serial.println("Starting lora_hardware_init");
  uint32_t init_result = 0;

  init_result = lora_hardware_init(hwConfig);
  Serial.printf("LoRa init %s\r\n", init_result == 0 ? "success" : "failed");

  // Initialize the Radio callbacks
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  // Initialize the Radio
  Radio.Init(&RadioEvents);

  // SX126xGetSyncWord

  // Serial.println(Radio.GetSyncWord()); // 5156 is default?

  Radio.SetCustomSyncWord(0x1424);


  // Set radio to sleep for next setup
  Radio.Sleep();

  // Set Radio channel
  Radio.SetChannel(RF_FREQUENCY);

  // Set Radio TX configuration
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  // Set Radio RX configuration
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  // Start LoRa
  Serial.println("Starting Radio.Rx");
  Radio.Rx(0); // Note: timeout only works for initial packet
  // Receiver is non-blocking
}



void loraTimerCallback()
{
  time_t currentTime = millis();

  // TODO: check for 5 seconds and 30 seconds (not that it can hover anyway....)

  // TODO: if less than 5 seconds, turn off hover flag

  if (currentTime - lastMessage > FAILSAFE_LORA_TIMEOUT)
  {
    Serial.printf("Communications lost to failsafe transceiver for %d ms. \n", currentTime - lastMessage);
    triggerFailsafe();
  }
}

void setup()
{
  // Initialize Serial for debug output
  Serial.begin(115200);

  delay(1000); // Wait for serial to begin

  Serial.println("=====================================");
  Serial.println("LoRa Drone Receiver");
  Serial.println("=====================================");

  setupLoRaModule();

  loraTimer.attach_ms(LORA_TIMER_UDPATE_RATE, loraTimerCallback); // TODO: only attach timer after first packet is received

  lastMessage = millis();
}

void loop()
{
  // Radio.Rx(0); // Note: this is non-blocking
  // Serial.println("Wait for message...");

  delay(5000);
}
