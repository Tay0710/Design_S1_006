// Adapted from PingPong Example from SX126x-Arduino Library
// https://github.com/beegee-tokyo/SX126x-Arduino/blob/master/examples/PingPong/PingPong.ino

#include <Arduino.h>
#include <Ticker.h>

#include "SBUS.h"
#include "LoRa_SX1262.h"

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX


#define pwPinTop 15  // Top US
volatile unsigned long pulseStart1 = 0;
volatile double distanceCm1 = 0;
volatile bool US_ready1 = false;
volatile double CurrentDistance = 0; // read from ultrasonic

// #define pwPinFront 4  // Front US
#define pwPinFront 5 // Note: this is the one originally located on the left side
volatile unsigned long pulseStartF = 0;
volatile double distanceCmF = 0;
volatile bool US_readyF = false;
volatile double CurrentDistanceF = 0; // read from ultrasonic



// Sensor objects and measurement data
SparkFun_VL53L5CX sensor1;
VL53L5CX_ResultsData measurementData1; // Result data class structure, 1356 byes of RAM
int imageResolution1 = 0;
int imageWidth1 = 0;
int frontAvg = -1;
int backAvg = -1;
int totalAvg = -1;

// TOF RIGHT
#define SDA_PIN 48
#define SCL_PIN 47


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

// Timing variables
uint32_t currentMillis;
uint32_t armingMillis;
uint32_t endofpathtime = 0;
uint32_t TurnLefttimecomplete = 0;
uint32_t brakingtime = 0;
uint32_t adjustingYAWtime = 0;

// Logic Booleans for Naviagtion
bool armingSequenceFlag = false;
volatile bool endofpath = false; // Used to decide when to turn drone
volatile bool turningsoon = false; // used to activate endofpath
volatile bool busychnagingYAW = false;
volatile int turningYAW = 1500;
bool armsequencecomplete = false;
bool runoncePITCH = false;
volatile bool sbusmeesagesent = false;

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
    triggerFailsafe();
  }
  else if (currentTime - lastMessage > FAILSAFE_LORA_TIMEOUT)
  {
    Serial.printf("Communications lost to failsafe transceiver for %d ms. \n", currentTime - lastMessage);
    triggerFailsafe();
  }
}

// Ultrasonic Top Interupt Function
void US1_ISR() {
  // Called when pwPin changes
  if (digitalRead(pwPinTop) == HIGH) {
    // Rising edge
    pulseStart1 = micros();
    US_ready1 = false;
  } else {
    // Falling edge
    unsigned long pulseWidthT = micros() - pulseStart1;
    distanceCm1 = pulseWidthT / 57.87;
    US_ready1 = true;
  }
}

// Ultrasonic Front Interupt Function
void USFront_ISR() {
  // Called when pwPin changes
  if (digitalRead(pwPinFront) == HIGH) {
    // Rising edge
    pulseStartF = micros();
    US_readyF = false;
  } else {
    // Falling edge
    unsigned long pulseWidthF = micros() - pulseStartF;
    distanceCmF = pulseWidthF / 57.87;
    US_readyF = true;
  }
}


/// Reading RIGHT TOF sensor. 
void readCenterAverage(SparkFun_VL53L5CX &sensor, VL53L5CX_ResultsData &measurementData) {
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&measurementData)) {

      int frontSum = 0;
      int backSum = 0;
      int totalSum = 0;

      int frontCount = 0;
      int backCount = 0;
      int totalCount = 0;

      frontAvg = 0;
      backAvg = 0;
      totalAvg = 0;

      for (int i = 0; i < 64; i++) {
        uint8_t status = measurementData.target_status[i];
        int distance = measurementData.distance_mm[i];
        if (status == 5) { // valid return
          totalSum +=  distance;
          totalCount++;
          if (i < 24) {
            // Front 3 columns: D0 to D23
            frontSum += distance;
            frontCount++;
          } else if (i > 39) {
            // Back 3 columns: D40 to D63
            backSum += distance;
            backCount++;
          }
        }
      }

      if (totalCount > 0) {
        totalAvg = totalSum / totalCount;
        Serial.print("Average total distance (mm): ");
        Serial.println(totalAvg);
      } else {
        totalAvg = -1;
        Serial.println("No valid pixels.");
      }

      if (frontCount > 0) {
        frontAvg = frontSum / frontCount;
        Serial.print("Average front distance (mm): ");
        Serial.println(frontAvg);
      } else {
        frontAvg = -1;
        Serial.println("No valid front pixels.");
      }

    
      if (backCount > 0) {
        backAvg = backSum / backCount;
        Serial.print("Average back distance (mm): ");
        Serial.println(backAvg);
      } else {
        backAvg = -1;
        Serial.println("No valid back pixels.");
      }

    } else{
      Serial.println("Failed to get Data!");
    }
  } else{
    // Serial.println("Data was not ready");
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

  // Ultrasonic Interrupt Setup
  pinMode(pwPinTop, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pwPinTop), US1_ISR, CHANGE); // USFront_ISR
  attachInterrupt(digitalPinToInterrupt(pwPinFront), USFront_ISR, CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus
  Wire.setClock(100000); // Optional: 400 kHz I2C
  delay(50);

  // Initialize TOF Sensor 1
  while (!sensor1.begin()) {
    Serial.println("Sensor 1 not found at 0x29! Retrying...");
    delay(50); // small delay to avoid spamming I2C or Serial
  }
  sensor1.setResolution(8*8);
  imageResolution1 = sensor1.getResolution();
  imageWidth1 = sqrt(imageResolution1);
  Serial.println("Sensor 1 initialized successfully at 0x29");
  delay(50);
  // Start ranging on sensor. 
  Serial.println("Starting ranging on tof sensors...");
  sensor1.setRangingFrequency(15);
  sensor1.startRanging();
  Serial.println("sensor tof is now ranging.");

  setupLoRaModule();
  loraTimer.attach_ms(LORA_TIMER_UDPATE_RATE, loraTimerCallback);
  lastMessage = millis();

  Serial.println(" --- Setup Complete --- ");
}

void loop()
{
  currentMillis = millis();

  // Arming sequence hard coded
  if (armingSequenceFlag){ // armingSequenceFlag
    // wait 5 seconds then arm
    if (currentMillis > 5000 + armingMillis && currentMillis < 10000 + armingMillis){
      rcChannels[THROTTLE] = THROTTLE_MIN;
      rcChannels[AUX1] = 1800;
      Serial.println("Arm drone.");
    } else if (currentMillis > 10000 + armingMillis && currentMillis < 11500 + armingMillis){ // Wait another 5 seconds before turning on throttle and leave on for 10 seconds
      rcChannels[THROTTLE] = 1360;
      rcChannels[AUX1] = 1800;
      Serial.println("Throttle 1360.");
    } else if (currentMillis > 11500 + armingMillis){
      Serial.println("Arming sequence finished");
      rcChannels[THROTTLE] = 1360;
      rcChannels[AUX1] = 1800; 
      armingSequenceFlag = false;
      armsequencecomplete = true;
    }
  }

  if(armsequencecomplete){ //armsequencecomplete

    // THROTTLE CHANGES - TOP side Ultrasonic Sensor
    if (US_ready1) {
      // Serial.print("US1 Distance: ");
      CurrentDistance = round(distanceCm1 * 100) / 100;
      Serial.print("Top US:                   ");
      Serial.print(distanceCm1, 2);
      Serial.println(" cm");
      US_ready1 = false;  // Current distance of ultrasonic is saved in: distanceCm1
      if (CurrentDistance <  156.70 ){ 
        rcChannels[THROTTLE] = 1333; // 156.70588 
      } else if (CurrentDistance >  156.70 && CurrentDistance < 159.41){ 
        rcChannels[THROTTLE] = 8.5*CurrentDistance; // if height is lowered then add C = 8.5*loweredheight
      } else if (CurrentDistance > 159.41){ // 159.41176 
        rcChannels[THROTTLE] = 1355; 
      }
    }

    // ROLL Changes
    // Serial.println("READ CENTER AVG");
    if(sbusmeesagesent){
      totalAvg = -1;
      frontAvg = -1;
      backAvg = -1;
    }
    readCenterAverage(sensor1, measurementData1);
    // Serial.print("Total roll-TOF: "); Serial.println(totalAvg);    
    // Serial.print("FRONT-TOF: "); Serial.println(frontAvg);   
    // Serial.print("BACK aERY -TOF: "); Serial.println(backAvg);   
    // Serial.print("TLTC: "); Serial.println(currentMillis - TurnLefttimecomplete);    
    if(currentMillis - TurnLefttimecomplete > 500){   
      // Serial.println("IN ROLL if statement");
        if(totalAvg > 0 && totalAvg < 1000){ // centerAvg returns middle 4 tof average ranges in mm. 
          rcChannels[ROLL] = 1470; 
        } else if(totalAvg > 1050){ // 1200mm = 1.2m
          rcChannels[ROLL] = 1530;
        } else{
          rcChannels[ROLL] = 1500;
        }
    } else{
      rcChannels[ROLL] = 1500;
      // Serial.println("IN ROLL ELSE");
    }
  
  
    //YAW angle fixes
    if(!endofpath && currentMillis - TurnLefttimecomplete > 500 && !busychnagingYAW){   
      if(frontAvg > 30 && backAvg > 30 && frontAvg < 2000 && backAvg < 2000 && frontAvg - backAvg < 1000 && frontAvg - backAvg > 1000){ // prevent changing when too close, too far or too greater difference
        if(frontAvg - backAvg > 100){ // turn left slightly
          turningYAW = 1700; 
          adjustingYAWtime = currentMillis;
        } else if(frontAvg - backAvg < 100){ // turn right
          turningYAW = 1300;
          adjustingYAWtime = currentMillis;
        } else{
          turningYAW = 1500;
          adjustingYAWtime = currentMillis;
        }
      } else{
        turningYAW = 1500;
        adjustingYAWtime = currentMillis;
      }
    } 

    // turning for time
    if(currentMillis - adjustingYAWtime < 40 && currentMillis - TurnLefttimecomplete > 500 && !endofpath){ // change YAW for 40ms
      rcChannels[YAW] = turningYAW;
      busychnagingYAW = true; 
    } else if(currentMillis - adjustingYAWtime > 40 && busychnagingYAW){
      busychnagingYAW = false;
    }


    // // FRONT Ultrasonic Sensor (MB1000) - CHECKING for upcoming obstacle infront (assume obstacle is wall)
    if (US_readyF) { 
      // Serial.print("US1 Distance: ");
      CurrentDistanceF = round(distanceCmF * 100) / 100;
      // Serial.print("CurrentMillis: "); Serial.println(currentMillis);
      Serial.print("LOOKY HERE AT THIS PART US FRONT :   ");
      Serial.print(distanceCmF, 2);
      Serial.println(" cm");
      US_readyF = false;  // Current distance of ultrasonic is saved in: distanceCm1
      
      // PITCH CONTROL.
      if(currentMillis - TurnLefttimecomplete < 500 && !endofpath){ // go for after turning for 1.0s
        rcChannels[PITCH] = 1500;
      } else if(currentMillis > 11500 + armingMillis) { // Could remove - to check. 
          if(CurrentDistanceF > 10.00 && CurrentDistanceF <= 200.00 && currentMillis - brakingtime < 1000){
            // rcChannels[PITCH] = 1390;
            rcChannels[PITCH] = 1500; // Replace with 1500 so it won't go backwards when distance is wrong?
            CurrentDistanceF = 0.00;
            Serial.println("Pitch backwards");
          } else if(CurrentDistanceF > 10.00 && CurrentDistanceF <= 200.00 && currentMillis - brakingtime > 1000){
            rcChannels[PITCH] = 1500;
            CurrentDistanceF = 0.00;
            endofpath = true;
            endofpathtime = currentMillis;
          } else if(CurrentDistanceF > 200.00){
            rcChannels[PITCH] = 1550; // apply pitch brakes and prepare to turn. 
            CurrentDistanceF = 0.00;
            brakingtime = currentMillis;
            Serial.print("Pitch forward");
            // Serial.print("IN EOP PITCH: "); Serial.println(rcChannels[PITCH]);
          } else{
            rcChannels[PITCH] = 1500;
            CurrentDistanceF = 0.00;
            brakingtime = currentMillis;
          }
        }
    }

    // // YAW CORNERING
    if(currentMillis - endofpathtime < 360 && endofpath){ // (endofpath && ) endofpath is assumed to be true if it is not false?
      // Serial.println("TURNING NOW!!");
      rcChannels[PITCH] = 1500; // Override the Front Ultrasonic PITCH commands
      rcChannels[YAW] =  1300;
      TurnLefttimecomplete = currentMillis;  // used to block Wall following
    } 
    else if(currentMillis - endofpathtime > 360 && endofpath){ // Only allow turning YAW to run for 300ms. 
      endofpath = false; // Path is no longer at an end.
      turningsoon = false; // not doing anything. 
      TurnLefttimecomplete = currentMillis; 
    }
  
  // End of Arming Complete Sequence. If statement. 
  }

  // Serial.println(" --- SBUS OUTPUT --- ");
  if (currentMillis > sbusTime)
  {
    sbusPreparePacket(sbusPacket, rcChannels, false, false);
    Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
    // printSBUSChannel(rcChannels);
    // printSBUSPacket(sbusPacket);
    sbusTime = currentMillis + SBUS_UPDATE_RATE;
    sbusmeesagesent = true;

    // Serial.print("PITCH: "); Serial.println(rcChannels[PITCH]);
    // Serial.print("SBUS ROLL:                 "); Serial.println(rcChannels[ROLL]);
    // Serial.print("SBUS YAW:  "); Serial.println(rcChannels[YAW]);
    // Serial.print("SBUS THROTTLE: "); Serial.println(rcChannels[THROTTLE]);
  } 
  else{
    sbusmeesagesent = false;
  }

}


// To TEST:

// // TO ADD: (HAVE to move into if(armsequencecomplete) statement.)
// // CHANGE in THROTTLE AS BATTERY DIES
// // Code description:
// // as the drone battery lowers, it will eventually stay in the THROTTLE = 1325 zone. If the drone stays in this throttle zone for longer than 5 seconds,
// // then both the lower and upper throttle limits should increase by 5 throttle values. 

// if(lowmode){  
//   if(firsttime){
//     lowmodetime = currentMillis;
//     firsttime = false;
//   }
//   if(currentMillis - lowmodetime > 5000 && runOnce){
//     LowThrottle += 5;
//     UpThrottle += 5;
//     runOnce = false; 
//   }
// } else{
//   lowmodetime = 0;
//   firsttime = true; 
//   runOnce = true; 
// }

// // To enable:
// // add lowmode = false (in low throttle and transistion throttle periods)
// // add lowmode = true (in high throttle period)

// // define lowmode, runOnce, firsttime as booleans. 
// // define lowmodetime as unit32 (timestamp)
// // define low and upper throttle period as variables (check requirements for transition period)



// // Turn Side Ways Ultrasonic
// // When it is not turning or just turned and is flying straight
  // if(leftUS > 250.00 && !endofpath && currentMillis - TurnLefttimecomplete > 1000 && !hallwayturn){
  //   // brake then turn into the empty hallway
  //   hallwayturn = true;
  // } else if(leftUS < 250.00 && hallwayturn){
  //   // do not allow turn until it is in the new hallway. 
  //   hallwayturn = false;
  // }
