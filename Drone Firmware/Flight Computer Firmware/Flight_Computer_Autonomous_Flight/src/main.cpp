#include <Arduino.h>
#include <Ticker.h>

#include "SBUS.h"
#include "LoRa_SX1262.h"

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

#define Ultrasonic_Rx_initialiser 40 // Rx Front US to initialise ultrasonic sensors'

#define pwPinTop 15  // Top US
volatile unsigned long pulseStart1 = 0;
volatile double distanceCm1 = 0;
volatile bool US_ready1 = false;
volatile double CurrentDistance = 0; // read from ultrasonic

#define pwPinFront 4  // Front US
volatile unsigned long pulseStartF = 0;
volatile double distanceCmF = 0;
volatile bool US_readyF = false;
volatile double CurrentDistanceF = 0; // read from ultrasonic


#define pwPinLeftUS  5 // LEFT US
volatile unsigned long pulseStartL = 0;
volatile double distanceCmL = 0;
volatile bool US_readyL = false;
volatile double CurrentDistance_US_L = 0; // read from ultrasonic

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
#define HOVER_LORA_TIMEOUT 10000     // 5 secs
#define LORA_TIMER_UDPATE_RATE 1000 // check every 1000 ms

// Using UART0 on ESP32-S3
#define RX_PIN 44
#define TX_PIN 43

#define THROTTLE_MIN 890
#define THROTTLE_MID 1445
#define THROTTLE_MAX 2000 // shrink range of throttle

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;

// Timing variables
uint32_t currentMillis;
uint32_t armingMillis;
uint32_t failsafeMillis;
uint32_t endofpathtime = 0;
uint32_t TurnLefttimecomplete = 0; // not used anymore. 
uint32_t brakingtime = 0;
uint32_t adjustingYAWtime = 0;
uint32_t BatteryCompensationtimer;
uint32_t lastvelmillis;
uint32_t turningtime = 0;
uint32_t lastvelmillisl;
uint32_t lastforwardvelmillis;

// Logic Booleans for Naviagtion
bool armingSequenceFlag = false;
volatile bool endofpath = false; // Used to decide when to turn drone
volatile bool turningsoon = false; // used to activate endofpath
volatile bool busychnagingYAW = false;
volatile int turningYAW = 1500;
bool armsequencecomplete = false;
bool runoncePITCH = false;
volatile bool sbusmeesagesent = false;
float BatteryCompensation = 0; // increase by increments of 5. 
bool BatteryCompensationBol = true;
bool armingthrottledone = true;
bool beginpitching = false;

volatile float throttlevelocity = 0;
volatile float olddistance = 0;
bool goingup = false;
bool goingdown = false;

volatile float rollvelocity = 0;
volatile float olddistancel = 0;

volatile float forwardvelocity = 0;
volatile float oldforwarddistance = 0;


boolean start_flag = false;
boolean failsafe_flag = false;

void triggerFailsafe()
{
  Serial.println("Failsafe triggered. Land drone.");
  armingSequenceFlag = false;
  failsafeMillis = millis();
  failsafe_flag = true;
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

// Ultrasonic LEFT Interupt Function
void US_LEFT() {
  // Called when pwPin changes
  if (digitalRead(pwPinLeftUS) == HIGH) {
    // Rising edge
    pulseStartL = micros();
    US_readyL = false;
  } else {
    // Falling edge
    unsigned long pulseWidthL = micros() - pulseStartL;
    CurrentDistance_US_L = pulseWidthL / 57.87;
    US_readyL = true;
  }
}

/// Reading RIGHT TOF sensor. 
void readCenterAverage(SparkFun_VL53L5CX &sensor, VL53L5CX_ResultsData &measurementData) {
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&measurementData)) {
      // total is acutally center average value. 

      int totalSum = 0;
      int totalCount = 0;
      totalAvg = 0;
      int indcenter[] = {5,6,9,10}; // center pixels

      for (int i = 0; i < 4; i++) {
        int index = indcenter[i];
        uint8_t status = measurementData.target_status[index];
        int distance = measurementData.distance_mm[index];
        if (status == 5) { // valid return
          totalSum +=  distance;
          totalCount++;
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
  attachInterrupt(digitalPinToInterrupt(pwPinLeftUS),US_LEFT , CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus
  Wire.setClock(100000); // Optional: 400 kHz I2C
  delay(50);

  // Initialize TOF Sensor 1
  while (!sensor1.begin()) {
    Serial.println("Sensor 1 not found at 0x29! Retrying...");
    delay(50); // small delay to avoid spamming I2C or Serial
  }
  sensor1.setResolution(4*4);
  imageResolution1 = sensor1.getResolution();
  imageWidth1 = sqrt(imageResolution1);
  Serial.println("Sensor 1 initialized successfully at 0x29");
  delay(50);
  // Start ranging on sensor. 
  Serial.println("Starting ranging on tof sensors...");
  sensor1.setRangingFrequency(60);
  sensor1.startRanging();
  Serial.println("sensor tof is now ranging.");

  // Ultrasonic Begin sensing. 
  pinMode(Ultrasonic_Rx_initialiser, OUTPUT);
  digitalWrite(Ultrasonic_Rx_initialiser, HIGH);
  delay(1);
  digitalWrite(Ultrasonic_Rx_initialiser, LOW);
  delay(1);
  pinMode(Ultrasonic_Rx_initialiser, INPUT);

  delay(100); 
  // Ensure that LoRa is at the END!. 

  setupLoRaModule();
  loraTimer.attach_ms(LORA_TIMER_UDPATE_RATE, loraTimerCallback);
  lastMessage = millis();

  Serial.println(" --- Setup Complete --- ");
}

void loop()
{
  currentMillis = millis();

  if (failsafe_flag) {
    // Reset orientation to 1500
    rcChannels[PITCH] = 1500;
    rcChannels[YAW] = 1500;
    rcChannels[ROLL] = 1500;

    if (currentMillis - failsafeMillis < 3000) {
      rcChannels[THROTTLE] = 1360; // Throttle for 3 secs
    } else {
      rcChannels[AUX3] = 1800; // then kill everything
    }
  }

  // Arming sequence hard coded
  else if (armingSequenceFlag){ // armingSequenceFlag
    // wait 5 seconds then arm
    if (currentMillis > 5000 + armingMillis && currentMillis < 10000 + armingMillis){
      rcChannels[THROTTLE] = THROTTLE_MIN;
      rcChannels[AUX1] = 1800;
      Serial.println("Arm drone.");
    } else if (currentMillis > 10000 + armingMillis && currentMillis < 11000 + armingMillis){ // Wait another 5 seconds before turning on throttle and leave on for 10 seconds
      rcChannels[THROTTLE] = 1380;
      rcChannels[AUX1] = 1800;
      Serial.println("Throttle 1360.");
      BatteryCompensation = 0; // reset battery compensation
      BatteryCompensationBol = true;
    } else if (currentMillis > 11000 + armingMillis){ // 11000 - <11300
      Serial.println("Arming sequence finished");
      rcChannels[THROTTLE] = 1380;
      rcChannels[AUX1] = 1800; 
      armingSequenceFlag = false;
      armsequencecomplete = true;
      armingthrottledone = true;
    }
  }

 //  25/10/2025
 // THe last attempt ended with the drone hitting the ceiling and then riding it. 

  else if(armsequencecomplete){ //armsequencecomplete

        // THROTTLE CHANGES - TOP side Ultrasonic Sensor
    if (US_ready1) {
      // Serial.print("US1 Distance: ");
      CurrentDistance = round(distanceCm1 * 100) / 100;
      Serial.print("Top US:                   ");
      Serial.print(distanceCm1, 2);
      Serial.println(" cm");
      US_ready1 = false;  // Current distance of ultrasonic is saved in: distanceCm1

      // Velocity determination
      throttlevelocity = (CurrentDistance - olddistance) / (millis() - lastvelmillis) * 1000; // cm/s
      lastvelmillis = millis();
      olddistance = CurrentDistance;
      Serial.print("THROTTLE VELOCITY: ");
      Serial.println(throttlevelocity, 2);
      if(throttlevelocity < -5.00){ // falling towards the roof so distance decreases and velocity is negative. 
        goingdown = false;
        goingup = true;
        throttlevelocity = 0.00;
      } else{
        goingdown = true;
        goingup = false;
        throttlevelocity = 0.00;
      }
      // Initial luanch control
      if(armingthrottledone){ // hold throttle until it passes upper limit then commence flight navigation.
        rcChannels[THROTTLE] = 1392; // 1370> ?? <1375
        BatteryCompensation = 0; // reset battery compensation
        if(CurrentDistance < 100.00 ){  
          beginpitching = true; //
          armingthrottledone = false; //
        }
      }
      // Flight Control
      if(!armingthrottledone){
        if(goingup && !goingdown){
          if(CurrentDistance < 50.00){
            rcChannels[THROTTLE] = 1360 + BatteryCompensation; // descend quickly
            // if(BatteryCompensationBol || millis() - BatteryCompensationtimer > 500){
            //   BatteryCompensationBol = false;
            //   BatteryCompensation = BatteryCompensation - 2;
            //   BatteryCompensationtimer = millis();
            // }
          } else if(CurrentDistance > 50.00 && CurrentDistance <  95.00 ){  // 118.23
            rcChannels[THROTTLE] = ((5*CurrentDistance) / 9) + 1332.2 + BatteryCompensation; 
            BatteryCompensationBol = true;
          } else if (CurrentDistance > 95.00 && CurrentDistance <  110.00){ 
            rcChannels[THROTTLE] = 1385 + BatteryCompensation; 
            BatteryCompensationBol = true;
          } else if (CurrentDistance > 110.00 && CurrentDistance <  150.00){  // 122.35
            rcChannels[THROTTLE] = 0.875*CurrentDistance + 1288.75 + BatteryCompensation; 
            BatteryCompensationBol = true;
          } else if(CurrentDistance > 150.00){
            rcChannels[THROTTLE] = 1420 + BatteryCompensation;
            BatteryCompensationBol = true; 
          }
        }
        else if(goingdown && !goingup){
          if(CurrentDistance < 50.00){
            rcChannels[THROTTLE] = 1370 + BatteryCompensation; // descend quickly
          } else if(CurrentDistance > 50.00 && CurrentDistance <  95.00 ){  // 118.23
            rcChannels[THROTTLE] = ((5*CurrentDistance)/9) + 1342.20 + BatteryCompensation; 
            BatteryCompensationBol = true;
          } else if (CurrentDistance > 95.00 && CurrentDistance <  110.00){ 
            rcChannels[THROTTLE] = 1395 + BatteryCompensation; // change to 1395?
            BatteryCompensationBol = true;
          } else if (CurrentDistance > 110.00 && CurrentDistance <  150.00){  // 122.35
            rcChannels[THROTTLE] = 1.125*CurrentDistance + 1271.25 + BatteryCompensation; 
            BatteryCompensationBol = true;
          } else if(CurrentDistance > 150.00){
            rcChannels[THROTTLE] = 1440 + BatteryCompensation; 
            // if(BatteryCompensationBol || millis() - BatteryCompensationtimer > 500){
            //   BatteryCompensationBol = false;
            //   BatteryCompensation = BatteryCompensation + 5;
            //   BatteryCompensationtimer = millis();
            // }
          } 
        }
      }

      if(beginpitching && !endofpath && !turningsoon){
        if(CurrentDistance < 150.00){  
          rcChannels[PITCH] = 1540; // pitch backwards
        } else{  
          rcChannels[PITCH] = 1500; // neutral
        } 
      }
    }


    // // ROLL Changes
    if (US_readyL) {
      // Serial.print("US1 Distance: ");
      Serial.print("LEFT US:      ");
      Serial.print(CurrentDistance_US_L, 2);
      Serial.println(" cm");
      US_readyL = false;  // Current distance of ultrasonic is saved in: distanceCm1

      // Roll Velocity
      rollvelocity = (CurrentDistance_US_L - olddistancel) / (millis() - lastvelmillisl) * 1000; // cm/s
      lastvelmillisl = millis();
      olddistancel = CurrentDistance_US_L;
      Serial.print("ROLL VELOCITY: ");
      Serial.println(rollvelocity, 2);
      // if velocity is negative, drone is flying towards left wall
      // if velocity is positive, drone is flying towards right wall


      if(millis() - turningtime > 360 && !turningsoon){   // currentMillis - TurnLefttimecomplete > 500 
      // Serial.println("IN ROLL if statement");
        if(CurrentDistance_US_L < 65.00 && rollvelocity < 3){ 
          rcChannels[ROLL] = 1525;
        } else if(CurrentDistance_US_L > 65.00 && CurrentDistance_US_L < 75.00 && rollvelocity < 0){ 
            rcChannels[ROLL] = -2.5*CurrentDistance_US_L + 1687.5; // Dampen the turn
        } else if(CurrentDistance_US_L > 75.00 && CurrentDistance_US_L < 85.00 && rollvelocity > 0){ 
            rcChannels[ROLL] = -2.5*CurrentDistance_US_L + 1687.5; // Dampen the turn
        } else if(CurrentDistance_US_L > 85.00 && rollvelocity > -3){ 
          rcChannels[ROLL] = 1475; // left
        } else{  
          rcChannels[ROLL] = 1500;
        }
    } else{
      rcChannels[ROLL] = 1500;
    }
  }

    readCenterAverage(sensor1, measurementData1);

    if (totalAvg > 0){
    forwardvelocity = (totalAvg - oldforwarddistance) / (millis() - lastforwardvelmillis) * 100; // cm/s
    lastforwardvelmillis = millis();
    oldforwarddistance = totalAvg;
    Serial.print("FORWARD VELOCITY: ");
    Serial.println(forwardvelocity, 2);
    }
    // target distance 1000 mm
    // if vel is positivie drone is moving forawrd
    // if vel is negative drone is moving backwards
    // vel should never really be bigger or less than 250 cm/s and -250 cm/s respectively.

    if(totalAvg < 2000 && totalAvg > 0 && !turningsoon){
      endofpath = true; // Path is not at an end.
      if(totalAvg > 1700 && totalAvg < 2200){
        endofpathtime = millis(); // timestamp when end of path is detected. 
        if(forwardvelocity > 250.00){
          rcChannels[PITCH] = 1400; // slow down harshly
      } else if(forwardvelocity > 100.00){
          rcChannels[PITCH] = 1450; // slow down
        } else if(forwardvelocity > 20.00){
          rcChannels[PITCH] = 1500; // go neutral if going forward (for vel between: 100 to 20)
        } else if(forwardvelocity < -150.00){
          rcChannels[PITCH] = 1540; // speed up
        } else{
          rcChannels[PITCH] = 1520; // go forward (for vel between: 20 to -150)
        }
      } else if(totalAvg > 1300 && totalAvg < 1700){
        if(forwardvelocity > 150.00){
          rcChannels[PITCH] = 1450; // slow down
        } else if(forwardvelocity > 80.00) {
          rcChannels[PITCH] = 1470; // slow down gently
        } else if(forwardvelocity > 8.00){
          rcChannels[PITCH] = 1500; // if going forward go neutral (for vel between: 80 to 8)
        } else if(forwardvelocity < -150.00){
          rcChannels[PITCH] = 1540; // speed up
        } else{
          rcChannels[PITCH] = (totalAvg/20) + 1435; // go forward slowly (for vel between: 8 to -150)
        }
      } else if(totalAvg > 1100 && totalAvg < 1300){
        if(forwardvelocity > 150.00){
          rcChannels[PITCH] = 1470; // slow down 
        } else if(forwardvelocity > 40.00){
          rcChannels[PITCH] = 1490; // slow down very gently
        } else if(forwardvelocity < -150.00){
          rcChannels[PITCH] = 1530; // speed up
        } else if(forwardvelocity < -40.00){
          rcChannels[PITCH] = 1510; // speed up slowly
        } else{
          rcChannels[PITCH] = 1500; // neutral P control (for vel between: 40 to -40)
        }
      } else if(totalAvg > 600 && totalAvg < 1100){
        if(forwardvelocity > 200.00){
          rcChannels[PITCH] = 1400; // slow down harshly
        } else if(forwardvelocity > 100.00) {
          rcChannels[PITCH] = 1470; // slow down 
        } else if (forwardvelocity < -100.00){
          rcChannels[PITCH] = 1520; // speed up 
        } else if (forwardvelocity < -8.00){
          rcChannels[PITCH] = 1500; // if going backwards go neutral (for vel between: -8 to -100)
        } else{
          rcChannels[PITCH] =  (totalAvg/20) + 1445; // slowly reverse (for vel between: 100 to -8)
        }
      } else if(totalAvg < 600){
        endofpathtime = millis(); // timestamp when end of path is detected. 
        if(forwardvelocity > 150.00){
          rcChannels[PITCH] = 1400; // slow down harshly
        } else if(forwardvelocity > 80.00) {
          rcChannels[PITCH] = 1450; // slow down 
        } else if (forwardvelocity < -250.00){
          rcChannels[PITCH] = 1540; // speed up 
        } else if (forwardvelocity < -50.00){
          rcChannels[PITCH] = 1520; // speed up 
        } else {
          rcChannels[PITCH] =  1475; // reverse (for vel between: 80 to -50)
        }
      }
    } else{
      endofpath = false; // Path is at an end.
      endofpathtime = millis(); // timestamp when end of path is detected. 
    }
  
    // PITCH Braking
    if(endofpath && millis() - endofpathtime > 2000){
      // TurnLefttimecomplete = millis(); 
      turningsoon = true; //
      turningtime = millis();
      endofpath = false;
    }
    // // YAW CORNERING
    if(millis() - turningtime < 360 && turningsoon){ // (endofpath && ) endofpath is assumed to be true if it is not false?
      // Serial.println("TURNING NOW!!");
      rcChannels[PITCH] = 1500; // Override the Front Ultrasonic PITCH commands
      rcChannels[YAW] =  1700; // THIS IS FOR LEFT TURN; use 1700 for RIGHT TURN (1300 for LEFT)
    } 
    else if(currentMillis - endofpathtime > 360 && turningsoon){ // Only allow turning YAW to run for 300ms. 
      turningsoon = false; // not doing anything. 
      rcChannels[YAW] =  1500;
    }
  
  // End of Arming Complete Sequence. If statement. 
  }


  // Serial.println(" --- SBUS OUTPUT --- ");
  if (currentMillis > sbusTime)
  {
    sbusPreparePacket(sbusPacket, rcChannels, false, false);
    Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
    printSBUSChannel(rcChannels);
    printSBUSPacket(sbusPacket);
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