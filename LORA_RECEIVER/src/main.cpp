#include <Arduino.h>

#include <SX126x-Arduino.h>
#include <SPI.h>


hw_config hwConfig;

// ESP32 - SX126x pin configuration
// int PIN_LORA_RESET = 1;	 // LORA RESET
int PIN_LORA_DIO_1 = 2; // LORA DIO_1
int PIN_LORA_BUSY = 3;	 // LORA SPI BUSY
int PIN_LORA_NSS = 10;	 // LORA SPI CS
int PIN_LORA_SCLK = 12;	 // LORA SPI CLK
int PIN_LORA_MISO = 11;	 // LORA SPI MISO
int PIN_LORA_MOSI = 13;	 // LORA SPI MOSI

// Not actually connected
// int RADIO_TXEN = 43;	 // LORA ANTENNA TX ENABLE
// int RADIO_RXEN = 44;	 // LORA ANTENNA RX ENABLE


// Function declarations
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void OnCadDone(bool cadResult); // Cad = Channel Activity Detection

#define LED_BUILTIN 2

// Define LoRa parameters
#define RF_FREQUENCY 915000000	// Hz (Note: should I change this to 915 MHz) - YES (must match exact frequency)
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12], SF7 is fastest, SF12 is slowest but greatest range and more robust
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8	// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0	// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000  // originally 3000
#define TX_TIMEOUT_VALUE 5000 // originally 5000

#define BUFFER_SIZE 64 // Define the payload size here

static RadioEvents_t RadioEvents;
static uint16_t BufferSize = BUFFER_SIZE;
static uint8_t RcvBuffer[BUFFER_SIZE];
static uint8_t TxdBuffer[BUFFER_SIZE];
static bool isMaster = true;
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

time_t timeToSend;

time_t cadTime;

time_t lastMessage;

uint8_t pingCnt = 0;
uint8_t pongCnt = 0;

#define NO_EVENT 0b0000000000000000
#define TX_FIN 0b0000000000000001
#define N_TX_FIN 0b1111111111111110
#define TX_ERR 0b0000000000000010
#define N_TX_ERR 0b1111111111111101
#define RX_FIN 0b0000000000000100
#define N_RX_FIN 0b1111111111111011
#define RX_ERR 0b0000000000010000
#define N_RX_ERR 0b1111111111101111
#define CAD_FIN 0b0000000000100000
#define N_CAD_FIN 0b1111111111011111

/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t g_task_sem = NULL;

/** Flag for the event type */
volatile uint16_t g_task_event_type = NO_EVENT;

// RX data
uint8_t *rx_payload;
uint16_t rx_size;
int16_t rx_rssi;
int8_t rx_snr;

// CAD result
bool tx_cadResult;

void setup()
{
	// Define the HW configuration between MCU and SX126x
	hwConfig.CHIP_TYPE = SX1262_CHIP;		  // Example uses an eByte E22 module with an SX1262
	// hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET
	hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;	  // LORA SPI CS
	hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;	  // LORA SPI CLK
	hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;	  // LORA SPI MISO
	hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
	hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;	  // LORA SPI BUSY
	hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;	  // LORA SPI MOSI
	// hwConfig.RADIO_TXEN = RADIO_TXEN;		  // LORA ANTENNA TX ENABLE (Note: if this is commented out, serial will print gpio error but code still works)
	// hwConfig.RADIO_RXEN = RADIO_RXEN;		  // LORA ANTENNA RX ENABLE (Note: if this is commented out, serial will print gpio error but code still works)
	hwConfig.USE_DIO2_ANT_SWITCH = false;	  // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
	hwConfig.USE_DIO3_TCXO = false;			  // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
	hwConfig.USE_DIO3_ANT_SWITCH = false;	  // Only Insight ISP4520 module uses DIO3 as antenna control
	hwConfig.USE_RXEN_ANT_PWR = false;		  // RXEN is used as power for antenna switch

  pinMode(LED_BUILTIN, OUTPUT); // For ESP32 Dev Module
	digitalWrite(LED_BUILTIN, LOW);

	// Initialize Serial for debug output
	Serial.begin(115200);

	time_t serial_timeout = millis();
	// On nRF52840 the USB serial is not available immediately
	while (!Serial)
	{
		if ((millis() - serial_timeout) < 5000)
		{
			delay(100);
		}
		else
		{
			break;
		}
	}

	Serial.println("=====================================");
	Serial.println("SX126x Receiver test");
	Serial.println("=====================================");

	Serial.println("MCU Espressif ESP32");
	
	uint8_t deviceId[8];

	BoardGetUniqueId(deviceId);
	Serial.printf("BoardId: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n",
				  deviceId[7],
				  deviceId[6],
				  deviceId[5],
				  deviceId[4],
				  deviceId[3],
				  deviceId[2],
				  deviceId[1],
				  deviceId[0]);

	// Initialize the LoRa chip
	Serial.println("Starting lora_hardware_init");
	uint32_t init_result = 0;

	init_result = lora_hardware_init(hwConfig);
	Serial.printf("LoRa init %s\r\n", init_result == 0 ? "success" : "failed");

	// Initialize the Radio callbacks
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;
	RadioEvents.CadDone = OnCadDone;

	// Initialize the Radio
	Radio.Init(&RadioEvents);

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

	// Create the task event semaphore
	// g_task_sem = xSemaphoreCreateBinary();
	// // Initialize semaphore
	// xSemaphoreGive(g_task_sem); // Controls access to common resource by threads

	// // Take the semaphore so the loop will go to sleep until an event happens
	// xSemaphoreTake(g_task_sem, 10);

	// Start LoRa
	Serial.println("Starting Radio.Rx");
	Radio.Rx(RX_TIMEOUT_VALUE);

  lastMessage = millis();
}

void loop() {
  Radio.Rx(0); // Note: this is non-blocking
  // Serial.println("Wait for message...");
  delay(2000);

  // add logic to check if no message received in last 30 seconds then stop
}


/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
	Serial.println("OnTxDone CB");
	g_task_event_type |= TX_FIN;
	// Wake up task to send initial packet
	//xSemaphoreGive(g_task_sem);
}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	Serial.println("OnRxDone:");
	rx_payload = payload;
	rx_size = size;
	rx_rssi = rssi;
	rx_snr = snr;
	g_task_event_type |= RX_FIN;

	memcpy(RcvBuffer, payload, 64);

  // Time from last message
  time_t now = millis();
  double time_between_message = (now - lastMessage)/1000.0;


  Serial.print("Message: ");
  Serial.println((const char *)RcvBuffer);
  Serial.printf("RssiValue=%d dBm, SnrValue=%d\n", rx_rssi, rx_snr);
  Serial.printf("Time from last message=%.2f seconds \n", time_between_message);

  lastMessage = millis();


	// Wake up task to send initial packet
	// xSemaphoreGive(g_task_sem);
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
	Serial.println("OnTxTimeout CB");
	g_task_event_type |= TX_ERR;

	// Wake up task to send initial packet
	// xSemaphoreGive(g_task_sem);
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
	Serial.println("OnRxTimeout CB");
	g_task_event_type |= RX_ERR;

	// Wake up task to send initial packet
	// xSemaphoreGive(g_task_sem);
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnRxError(void)
{
	Serial.println("OnRxError CB");
	g_task_event_type |= RX_ERR;

	// Wake up task to send initial packet
	// xSemaphoreGive(g_task_sem);
}

/**@brief Function to be executed on CAD Done event
 */
void OnCadDone(bool cadResult)
{
	Serial.println("OnCadDone CB");
	tx_cadResult = cadResult;
	g_task_event_type |= CAD_FIN;

	// Wake up task to send initial packet
	// xSemaphoreGive(g_task_sem);
}
