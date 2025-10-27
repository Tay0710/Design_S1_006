// Adapted from PingPong Example from SX126x-Arduino Library
// https://github.com/beegee-tokyo/SX126x-Arduino/blob/master/examples/PingPong/PingPong.ino

#include <Arduino.h>
#include <Ticker.h>

#include "SBUS.h"
#include "LoRa_SX1262.h"

static RadioEvents_t RadioEvents;
hw_config hwConfig;
time_t lastMessage;

Ticker loraTimer;
Ticker SBUSTimer;

#define FAILSAFE_LORA_TIMEOUT 30000 // 30 secs
#define HOVER_LORA_TIMEOUT 5000     // 5 secs
#define LORA_TIMER_UDPATE_RATE 1000 // check every 1000 ms

// Using UART0 on ESP32-S3
#define RX_PIN 44
#define TX_PIN 43

#define THROTTLE_MIN 890
#define THROTTLE_MID 1150
#define THROTTLE_MAX 1410 // shrink range of throttle

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;

uint32_t currentMillis;
uint32_t armingMillis;

bool armingSequenceFlag = false;

boolean start_flag = false;

void triggerFailsafe()
{
  Serial.println("Failsafe triggered. Land drone.");
  armingSequenceFlag = false;
  rcChannels[AUX3] = 1800;
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
  Serial.println("On Rx Timeout CB");
  Serial.printf("OnRxTimeout: No message received in last %d ms \n", RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  if (!start_flag)
  {
    start_flag = true; // For first packet
    armingSequenceFlag = true;
    armingMillis = millis();
  }

  Serial.println("OnRxDone:");

  // Time from last message
  time_t now = millis();
  double time_between_message = (now - lastMessage) / 1000.0;

  Serial.print("Message: ");
  Serial.println((const char *)payload);
  Serial.printf("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);
  Serial.printf("Time from last message=%.2f seconds \n", time_between_message);

  String payload_str = (const char *)payload;

  if (payload_str == "STOP")
  {
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
  Radio.Rx(0);
  // Receiver is non-blocking
  while (!start_flag)
  {
    delay(1000);
    Serial.println("Wait for first packet...");
  }
}

void loraTimerCallback()
{
  time_t currentTime = millis();

  if (currentTime - lastMessage > HOVER_LORA_TIMEOUT)
  {
    Serial.printf("Communications lost to failsafe transceiver for %d ms. \n", currentTime - lastMessage);
    // triggerHoverMode();
  }
  else if (currentTime - lastMessage > FAILSAFE_LORA_TIMEOUT)
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

  Serial.println(" --- Setup SBUS --- ");

  // Initialise all channels to 1500
  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++)
  {
    rcChannels[i] = 1500;
  }
  rcChannels[THROTTLE] = THROTTLE_MIN;

  Serial1.begin(100000, SERIAL_8E2, RX_PIN, TX_PIN, true); // Initialize Serial1 with 100000 baud rate

  setupLoRaModule();

  loraTimer.attach_ms(LORA_TIMER_UDPATE_RATE, loraTimerCallback);

  lastMessage = millis();

  Serial.println(" --- Setup Complete --- ");
}

void loop()
{
  currentMillis = millis();

  // Arming sequence hard coded
  if (armingSequenceFlag)
  {
    // wait 5 seconds then arm
    if (currentMillis > 5000 + armingMillis && currentMillis < 10000 + armingMillis)
    {
      rcChannels[THROTTLE] = THROTTLE_MIN;
      rcChannels[AUX1] = 1800;
      // Serial.println("Arm drone.");
    }
    else if (currentMillis > 10000 + armingMillis && currentMillis < 15000 + armingMillis)
    { // Wait another 5 seconds before turning on throttle and leave on for 10 seconds
      rcChannels[THROTTLE] = 1350;
      // Serial.println("Throttle 1350.");
      // rcChannels[AUX1] = 1800;
    }
    else if (currentMillis > 15000 + armingMillis)
    {
      // Serial.println("Arming sequence finished");

      rcChannels[THROTTLE] = THROTTLE_MIN;
      rcChannels[AUX1] = 1500;

      armingSequenceFlag = false;
    }
  }

  if (currentMillis > sbusTime)
  {
    sbusPreparePacket(sbusPacket, rcChannels, false, false);
    Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
    // printSBUSChannel(rcChannels);
    // printSBUSPacket(sbusPacket);
    sbusTime = currentMillis + SBUS_UPDATE_RATE;
  }

  // delay(5000);
}
