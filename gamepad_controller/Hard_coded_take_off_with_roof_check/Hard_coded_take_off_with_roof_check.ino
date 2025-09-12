#include <Arduino.h>
#include <Ticker.h>


#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010
#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0F
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 7

// SBUS channels
#define ROLL 0      // A
#define PITCH 1     // E
#define THROTTLE 2  // T
#define YAW 3       // R
#define AUX1 4      // ARM
#define AUX2 5      // ANGLE
#define AUX3 6      // Failsafe

// Using UART1
#define RX_PIN 18
#define TX_PIN 17

uint32_t currentMillis;
uint32_t startMillis;

Ticker sbusTimer;


// for ultrasonic sensor
const int trigPin = 23;
const int echoPin = 22;

float duration, distance;


uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;

long getHeight() {
  // Send a 10µs pulse to trigger the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the time for echo pulse
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * .0343) / 2;
  delay(100);

  return distance;
}


void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe) {
  static int output[SBUS_CHANNEL_NUMBER] = { 0 };
  /*
  * Map 1000-2000 with middle at 1500 chanel values to
  * 173-1811 with middle at 992 S.BUS protocol requires
  */
  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
    output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
  }

  uint8_t stateByte = 0x00;  // modified this from 0x00 (order of bits: failsafe, lost, 18, 17)

  if (isSignalLoss) {
    stateByte |= SBUS_STATE_SIGNALLOSS;  // NOTE: is signal loss represented by a 1
  }

  if (isFailsafe) {
    stateByte |= SBUS_STATE_FAILSAFE;
  }

  packet[0] = SBUS_FRAME_HEADER;
  //Header
  packet[1] = (uint8_t)(output[0] & 0x07FF);
  packet[2] = (uint8_t)((output[0] & 0x07FF) >> 8 | (output[1] & 0x07FF) << 3);
  packet[3] = (uint8_t)((output[1] & 0x07FF) >> 5 | (output[2] & 0x07FF) << 6);
  packet[4] = (uint8_t)((output[2] & 0x07FF) >> 2);
  packet[5] = (uint8_t)((output[2] & 0x07FF) >> 10 | (output[3] & 0x07FF) << 1);
  packet[6] = (uint8_t)((output[3] & 0x07FF) >> 7 | (output[4] & 0x07FF) << 4);
  packet[7] = (uint8_t)((output[4] & 0x07FF) >> 4 | (output[5] & 0x07FF) << 7);
  packet[8] = (uint8_t)((output[5] & 0x07FF) >> 1);
  packet[9] = (uint8_t)((output[5] & 0x07FF) >> 9 | (output[6] & 0x07FF) << 2);
  packet[10] = (uint8_t)((output[6] & 0x07FF) >> 6 | (output[7] & 0x07FF) << 5);
  packet[11] = (uint8_t)((output[7] & 0x07FF) >> 3);
  packet[12] = (uint8_t)((output[8] & 0x07FF));
  packet[13] = (uint8_t)((output[8] & 0x07FF) >> 8 | (output[9] & 0x07FF) << 3);
  packet[14] = (uint8_t)((output[9] & 0x07FF) >> 5 | (output[10] & 0x07FF) << 6);
  packet[15] = (uint8_t)((output[10] & 0x07FF) >> 2);
  packet[16] = (uint8_t)((output[10] & 0x07FF) >> 10 | (output[11] & 0x07FF) << 1);
  packet[17] = (uint8_t)((output[11] & 0x07FF) >> 7 | (output[12] & 0x07FF) << 4);
  packet[18] = (uint8_t)((output[12] & 0x07FF) >> 4 | (output[13] & 0x07FF) << 7);
  packet[19] = (uint8_t)((output[13] & 0x07FF) >> 1);
  packet[20] = (uint8_t)((output[13] & 0x07FF) >> 9 | (output[14] & 0x07FF) << 2);
  packet[21] = (uint8_t)((output[14] & 0x07FF) >> 6 | (output[15] & 0x07FF) << 5);
  packet[22] = (uint8_t)((output[15] & 0x07FF) >> 3);
  packet[23] = stateByte;

  //Flags byte
  packet[24] = SBUS_FRAME_FOOTER;  //Footer
}


void printSBUSData(uint8_t packet[]) {
  for (int i = 0; i < 25; i++) {
    Serial.print(packet[i], OCT);
    Serial.print('\t');
  }
  Serial.print('\n');
}


void SBUSCallback() {

  sbusPreparePacket(sbusPacket, rcChannels, false, false);
  Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
  // printSBUSChannel(rcChannels);
  // printSBUSPacket(sbusPacket);
}

void setup() {
  Serial.begin(115200);

  Serial.println("get ready: ...");

  delay(10000);

  Serial.println("Begin: ...");

  // Initialise all channels to 1500
  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
    rcChannels[i] = 1500;
  }

  // Stuff that Aidan wanted lmao
  rcChannels[AUX2] = 1200;
  rcChannels[THROTTLE] = 890;  // must be below min_check = 1050 when arming

  Serial1.begin(100000, SERIAL_8E2, RX_PIN, TX_PIN, true);  // Initialize Serial1 with 100000 baud rate
  // false = univerted, true = inverted

  // Setup SBUS sending timer
  sbusTimer.attach_ms(SBUS_UPDATE_RATE, SBUSCallback);

  startMillis = millis();
}

void loop() {
  currentMillis = millis() - startMillis;

  // Roof check
  long height = getHeight();
  if (height <= 80) {
      rcChannels[AUX3] = 1800; // trigger failsafe
      Serial.println("Roof. Trigger failsafe.");
      Serial.print("Distance: ");
      Serial.println(distance);
  }

  /*
    * Here you can modify values of rcChannels while keeping it in 1000:2000 range
    */
  // Wait 70 seconds and then arm
  if (currentMillis > 70 * 1000 && currentMillis < 80 * 1000) {
    rcChannels[AUX1] = 1800;
    Serial.println("Arm drone.");
  } else if (currentMillis > 80 * 1000 && currentMillis < 85 * 1000)  // Wait another 10 seconds before turning throttle on
  {
    rcChannels[THROTTLE] = 1075;
    Serial.println("Throttle 1075.");
  } else if (currentMillis > 85 * 1000) {
    rcChannels[AUX1] = 1500;
    rcChannels[THROTTLE] = 890;
    rcChannels[AUX3] = 1500;
    Serial.println("Disarm. stop drone. Failsafe.");
  }

  // if (currentMillis > sbusTime) {
  //   sbusPreparePacket(sbusPacket, rcChannels, false, false);
  //   Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
  //   //printSBUSData(sbusPacket);
  //   sbusTime = currentMillis + SBUS_UPDATE_RATE;
  // }
}