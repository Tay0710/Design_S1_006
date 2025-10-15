/*

  Adapted using example 'Controller.ino' from Bluepad32 library for Arduino (https://github.com/ricardoquesada/bluepad32) following documentation from 
  https://bluepad32.readthedocs.io/en/latest/plat_arduino/
  
*/

#include <Bluepad32.h>
#include "SBUS.h"

#include <AsyncTCP.h>
#include <WiFi.h>
#include <stdio.h>
#include <Ticker.h>

#include "soc/soc.h"           // disable brownout problems
#include "soc/rtc_cntl_reg.h"  // disable brownout problems

#include <Arduino.h>
#include "QuickPID.h"

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX


// Top Ultrasonic Variables
#define pwPin1 25  // GPIO pin
volatile unsigned long pulseStart1 = 0;
volatile double distanceCm1 = 0;
volatile bool US_ready1 = false;
volatile double CurrentDistance = 0; // read from ultrasonic
float targetheight = 200; // in cm
volatile uint32_t lastmillis1 = 0; 
volatile uint32_t lastmillis2 = 0; 

// Front Ultrasonic Variables
#define pwPinFront 4  // GPIO pin
volatile unsigned long pulseStartF = 0;
volatile double distanceCmF = 0;
volatile bool US_readyF = false;
volatile double CurrentDistanceF = 0; // read from ultrasonic
volatile bool endofpath = false; // Used to decide when to turn drone

volatile int TurnLeft = 3;

// TOF RIGHT
#define SDA_PIN 26
#define SCL_PIN 27
 // AVDD - needs 3v3 supply
 // IOVDD - needs 3v3 supply
 // GND - GND
//  #define P1 19 // LPn - not required for single sensor

// Sensor objects and measurement data
SparkFun_VL53L5CX sensor1;
VL53L5CX_ResultsData measurementData1; // Result data class structure, 1356 byes of RAM
int imageResolution1 = 0;
int imageWidth1 = 0;
volatile int centerAvg = -1;  // Global variable to hold the latest average


// Using UART2 on ESP32
#define RX_PIN 16
#define TX_PIN 17

// Using UART0 on ESP32
// #define RX_PIN 3
// #define TX_PIN 1

// Initialise global variables
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
AsyncClient* client = new AsyncClient;  // Initialise TCP client
Ticker sendTimer;

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;

int timestamp = 0;

uint32_t currentMillis;
uint32_t armingMillis;

uint32_t lastYAWtime = 0;
uint32_t endofpathtime = 0;

uint32_t LastDownPress = 0;
uint32_t LastUpPress = 0;

bool armingSequenceFlag = false;
bool armsequencecomplete = false;
bool runoncePITCH = false;


// Constant for joystick
#define AXIS_L_NEUTRAL_X 4
#define AXIS_L_NEUTRAL_Y -3
#define AXIS_R_NEUTRAL_X 3
#define AXIS_R_NEUTRAL_Y 0
#define AXIS_UP -512
#define AXIS_DOWN 512
#define AXIS_LEFT -512
#define AXIS_RIGHT 512

// Extra SBUS constants
#define SBUS_MIN 1400  // 885 to 890
#define SBUS_MID 1500
#define SBUS_MAX 1600  // 2115, switched to 2110

// YAW Limits
#define YAW_MIN 890  // 885 to 890
#define YAW_MID 1500
#define YAW_MAX 2110  // 2115, switched to 2110
// CHange angle gradually 

#define THROTTLE_MIN 890
#define THROTTLE_MID 1150 //1150
#define THROTTLE_MAX 1410  // shrink range of throttle, was 1410

#define DPAD_INCREMENT 5 // YAW: 188 = ~ 22.56 

// Constants for Wifi
#define SSID "ESP-TEST-DJ"
#define PASSWORD "123456789"

#define SERVER_HOST_NAME "192.168.4.2"  // IP address of TCP server (on PC)

#define TCP_PORT 7050
#define DNS_PORT 53

const int debounceDelay = 100;

int currentThrottle = 0;
bool hoverFlag = false;

int lastPress = 0;


// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }

  // TODO: trigger failsafe mode on AUX2
  // TODO: setup failsafe mode to land drone on betaflight
}

/*

Using 8BitDo Controller:

Dpad: up = 0x01, down = 0x02, left = 0x08, right = 0x04
Buttons: A = 0x0002, B = 0x0001, Y = 0x0004, X = 0x0008, R1 = 0x0020, R2 = 0x0080, L1 = 0x0010, L2 = 0x0040
axis L (x,y) : 
  neutral = 4,-3
  up: X, -512
  down: X, 512
  left: -512, Y
  right: 512, Y

axis R (x,y) :
  neutral = 3, 0
  others = same as above
*/

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}

void processGamepad(ControllerPtr ctl) {
  // When R1 is pressed
  if (ctl->r1()) {
    dumpGamepad(ctl);  // Prints everything for debug
  }
  // Press L2 + R2 to arm (AUX1)
  if (ctl->l2() and ctl->r2()) {
    Serial.println("ARM DRONE");
    rcChannels[AUX1] = 1800;
  } else {
    rcChannels[AUX1] = 1500;
    // Unsure if it needs to be set back to 1500 otherwise... probably
  }

  // Press X to trigger failsafe (Note: X button is labelled as Y on controller)
  if (ctl->y()) {
    Serial.println("FAILSAFE");
    rcChannels[AUX3] = 1800;
  }

  // Press B to reset roll and pitch to 1500
  if (ctl->buttons() & 0x0001) {
    Serial.println("Reset pitch and roll");
    rcChannels[ROLL] = SBUS_MID;
    // rcChannels[PITCH] = SBUS_MID;
  }

  // else {
  //   // rcChannels[AUX3] = 1500;
  // }

  // Map throttle values
  // NOTE: may require non-linear mapping
  // Press A to keep throttle value constant
  // if ((ctl->buttons() & 0x0002) && (millis() - lastPress) > debounceDelay)
  // {
  //   lastPress = millis();
  //   rcChannels[THROTTLE] = currentThrottle;
  //   hoverFlag = !hoverFlag;

  // }
  // else if (ctl->axisY() <= AXIS_L_NEUTRAL_Y && !hoverFlag) {
  //   currentThrottle = (int)map(ctl->axisY(), AXIS_L_NEUTRAL_Y, AXIS_UP, THROTTLE_MID, THROTTLE_MAX);
  //   rcChannels[THROTTLE] = currentThrottle;
  // } else if (!hoverFlag) {
  //   currentThrottle = (int)map(ctl->axisY(), AXIS_L_NEUTRAL_Y, AXIS_DOWN, THROTTLE_MID, THROTTLE_MIN);
  //   rcChannels[THROTTLE] = currentThrottle;
  // }


  // Serial.print("axis l - y: ");
  // Serial.print(ctl->axisY());
  // Serial.print("\t throttle: ");
  // Serial.println(rcChannels[THROTTLE]);

  // Map yaw values
  // NOTE: disable for now
  // if (ctl->axisX() <= AXIS_L_NEUTRAL_X) {
  //   rcChannels[YAW] = (int)map(ctl->axisX(), AXIS_L_NEUTRAL_X, AXIS_LEFT, SBUS_MID, SBUS_MIN);
  // } else {
  //   rcChannels[YAW] = (int)map(ctl->axisX(), AXIS_L_NEUTRAL_X, AXIS_RIGHT, SBUS_MID, SBUS_MAX);
  // }
  if (ctl->axisRX() <= AXIS_R_NEUTRAL_X) {
    rcChannels[YAW] = (int)map(ctl->axisRX(), AXIS_R_NEUTRAL_X, AXIS_LEFT, YAW_MID, YAW_MIN);
  } else {
    rcChannels[YAW] = (int)map(ctl->axisRX(), AXIS_R_NEUTRAL_X, AXIS_RIGHT, YAW_MID, YAW_MAX);
  }
  // Serial.print("axis l - x: ");
  // Serial.print(ctl->axisX());
  // Serial.print("\t yaw: ");
  // Serial.println(rcChannels[YAW]);

  // Map pitch values
  // Note: this currently assumes pitching down is > 1500 and pitching up is < 1500, if not simply swap SBUS_MAX and SBUS_MIN
  // TODO: change this to button
  // if (ctl->axisRY() <= AXIS_R_NEUTRAL_Y) {
  //   rcChannels[PITCH] = (int)map(ctl->axisRY(), AXIS_R_NEUTRAL_Y, AXIS_UP, SBUS_MID, SBUS_MAX);
  // } else {
  //   rcChannels[PITCH] = (int)map(ctl->axisRY(), AXIS_R_NEUTRAL_Y, AXIS_DOWN, SBUS_MID, SBUS_MIN);
  // }
  // Serial.print("axis r - y: ");
  // Serial.print(ctl->axisY());
  // Serial.print("\t pitch: ");
  // Serial.println(rcChannels[PITCH]);

  // Map roll values
  // if (ctl->axisRX() <= AXIS_R_NEUTRAL_X) {
  //   rcChannels[ROLL] = (int)map(ctl->axisRX(), AXIS_R_NEUTRAL_X, AXIS_LEFT, SBUS_MID, SBUS_MIN);
  // } else {
  //   rcChannels[ROLL] = (int)map(ctl->axisRX(), AXIS_R_NEUTRAL_X, AXIS_RIGHT, SBUS_MID, SBUS_MAX);
  // }
  // Serial.print("axis r - x: ");
  // Serial.print(ctl->axisRX());
  // Serial.print("\t yaw: ");
  // Serial.println(rcChannels[ROLL]);


  // Dpad: up = 0x01 = 0b00000001, down = 0x02 = 0b00000010, left = 0x08 = 0b00001000, right = 0x04 = 0b00000100
  // Map pitch to Dpad

  // currentMillis
  if (ctl->dpad() & 0x02 && rcChannels[THROTTLE] > THROTTLE_MIN && (currentMillis - LastDownPress) > debounceDelay) {
    // rcChannels[PITCH] = rcChannels[PITCH] - DPAD_INCREMENT;
    rcChannels[THROTTLE] = rcChannels[THROTTLE] - DPAD_INCREMENT;
    LastDownPress = currentMillis; 
  }
  if (ctl->dpad() & 0x01 && rcChannels[THROTTLE] < THROTTLE_MAX && (currentMillis - LastUpPress) > debounceDelay) {
    // rcChannels[PITCH] = rcChannels[PITCH] + DPAD_INCREMENT;
    rcChannels[THROTTLE] = rcChannels[THROTTLE] + DPAD_INCREMENT;
    LastUpPress = currentMillis; 
  }
  if (ctl->dpad() & 0x08 && rcChannels[ROLL] > SBUS_MIN) {
    rcChannels[ROLL] = rcChannels[ROLL] - DPAD_INCREMENT;
  }
  if (ctl->dpad() & 0x04 && rcChannels[ROLL] < SBUS_MAX) {
    rcChannels[ROLL] = rcChannels[ROLL] + DPAD_INCREMENT;
  }


  // Arming sequence code
  // Start arming sequence if Y button is pressed
  // Note: only start after it is let go
  if (ctl->buttons() & 0x0004) {
    Serial.println("Y button is pressed");
    Serial.println("Start arming sequence");

    armingMillis = millis();    // store starting time of arming sequence
    armingSequenceFlag = true;  // set flag to true
  }

  // if(ctl->dpad()) {
  //   Serial.print("pitch: ");
  //   Serial.println(rcChannels[PITCH]);
  //   Serial.print("roll: ");
  //   Serial.println(rcChannels[ROLL]);
  // }


  // dumpGamepad(ctl);  // Prints everything for debug
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void sendData() {
  char buffer[200];                                                                                                                                                                                 // arbitrarily large size
  sprintf(buffer, "(%d, %d, %d, %d, %d, %d, %d, %d)", millis(), rcChannels[ROLL], rcChannels[PITCH], rcChannels[THROTTLE], rcChannels[YAW], rcChannels[AUX1], rcChannels[AUX2], rcChannels[AUX3]);  // automatically trims buffer[]
  // Send data
  if (client->canSend()) {
    Serial.println(buffer);
    client->add(buffer, strlen(buffer));
    client->send();
  }
}

/* event callbacks */
static void handleData(void* arg, AsyncClient* client, void* data, size_t len) {
  Serial.printf("\n data received from %s \n", client->remoteIP().toString().c_str());
  Serial.write((uint8_t*)data, len);
}

void onConnect(void* arg, AsyncClient* client) {
  Serial.printf("\n client has been connected to %s on port %d \n", SERVER_HOST_NAME, TCP_PORT);
  sendTimer.attach(0.2, sendData);  // interval in seconds
}

void onDisconnect(void* arg, AsyncClient* client) {
  Serial.printf("\n client has been disconnected from %s on port %d \n", SERVER_HOST_NAME, TCP_PORT);
  sendTimer.detach();  // Pause timer
}

// Ultrasonic Top Interupt Function
void US1_ISR() {
  // Called when pwPin changes
  if (digitalRead(pwPin1) == HIGH) {
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


// Arduino setup function. Runs in CPU 1
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //disable brownout detector

  Serial.begin(115200);

  Serial.println(" --- Setup Bluetooth Controller --- ");
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  BP32.enableNewBluetoothConnections(true);

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);

  Serial.println(" --- Setup SBUS --- ");

  // Initialise all channels to 1500
  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
    rcChannels[i] = 1500;
  }
  // Initialise throttle to 890 (failsafe is triggered if mapping maps 885 below 885)
  rcChannels[THROTTLE] = 890;  // must be below min_check = 1050 when arming
  rcChannels[AUX2] = 1200;     // For angle mode
  // TODO: might map this to button instead

  Serial1.begin(100000, SERIAL_8E2, RX_PIN, TX_PIN, true);  // Initialize Serial1 with 100000 baud rate
  // false = univerted, true = inverted

  // Serial.println(" --- Setup WIFI/TCP Connection --- ");
  // // Setup ESP32 as the access point
  // // https://randomnerdtutorials.com/esp32-access-point-ap-web-server/
  // Serial.println("Setting Access Point...");
  // WiFi.softAP(SSID, PASSWORD);

  // // Print ESP Local IP Address
  // IPAddress IP = WiFi.softAPIP();
  // Serial.print("AP IP Address: ");
  // Serial.println(IP);

  // // Setup ESP32 as AsyncTCP Client
  // client->onConnect(&onConnect, client);  // on successful connect

  // client->connect(SERVER_HOST_NAME, TCP_PORT);  // attempt to connect
  // client->onData(&handleData, client);          // when data is received

  // Serial.println("Connecting to TCP server");

  // // Wait until ESP32 is connected to the TCP Server on PC
  // // while (!client->connected()) {
  // //   Serial.print(".");
  // //   delay(1000);
  // // }

  // client->onDisconnect(&onDisconnect, client);  // when disconnected

  // Ultrasonic Interrupt Setup
  pinMode(pwPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pwPin1), US1_ISR, CHANGE); // USFront_ISR
  attachInterrupt(digitalPinToInterrupt(pwPinFront), USFront_ISR, CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus
  Wire.setClock(400000); // Optional: 400 kHz I2C
  delay(50);

  while (!sensor1.begin()) {
    Serial.println("Sensor 1 not found at 0x29! Retrying...");
    delay(50); // small delay to avoid spamming I2C or Serial
  }

  sensor1.setResolution(4 * 4);
  imageResolution1 = sensor1.getResolution();
  imageWidth1 = sqrt(imageResolution1);
  Serial.println("Sensor 1 initialized successfully at 0x29");

  delay(50);

  // Start ranging on both sensors. 
  Serial.println("Starting ranging on both sensors...");
  sensor1.setRangingFrequency(60);
  sensor1.startRanging();
  Serial.println("Both sensors are now ranging.");

  Serial.println(" --- Setup Complete --- ");
}


/// Reading RIGHT TOF sensor. 
void readCenterAverage(SparkFun_VL53L5CX &sensor, VL53L5CX_ResultsData &measurementData) {
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&measurementData)) {

      // Middle 4 indices in a 4x4 grid: 5, 6, 9, 10
      int middleIdx[4] = {5, 6, 9, 10};
      int sum = 0;
      int count = 0;

      for (int i = 0; i < 4; i++) {
        int idxPixel = middleIdx[i];
        uint8_t status = measurementData.target_status[idxPixel];
        if (status == 5) { // valid return
          sum += measurementData.distance_mm[idxPixel];
          count++;
        }
      }

      if (count > 0) {
        centerAvg = sum / count;
        // Serial.print("Average center distance (mm): ");
        // Serial.println(centerAvg);
      } else {
        centerAvg = -1;
        // Serial.println("No valid center pixels.");
      }
    }
  }
}

// Need a sensor to detect the roof (keep drone in air)
// one for right wall, one for left wall, one for upcoming wall infront. 
// if wall infront turn to side where TOF data is of greater distance? (atleast 150cm)
//    else if both less than 150cm then do a 180 degree turn. 

// Arduino loop function. Runs in CPU 1.
void loop() {
  currentMillis = millis();

  // // This call fetches all the controllers' data.
  // // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();
  

  // Arming sequence hard coded
  if (armingSequenceFlag) {
    rcChannels[THROTTLE] = THROTTLE_MIN; // set to min throttle despite controller being connected

    // wait 10 seconds then arm
    if (currentMillis > 5000 + armingMillis && currentMillis < 10000 + armingMillis) {
      rcChannels[AUX1] = 1800;
      // armsequencecomplete = false;
      Serial.println("Arm drone.");
    } else if (currentMillis > 10000 + armingMillis && currentMillis < 11700 + armingMillis) { // Wait another 10 seconds before turning on throttle and leave on for 5 seconds
      rcChannels[THROTTLE] = 1336;     
      Serial.println("Throttle 1320.");
      rcChannels[AUX1] = 1800;
    } else if (currentMillis > 11700 + armingMillis){
      rcChannels[THROTTLE] = 1336;
      armsequencecomplete = true;
      // runoncePITCH = true;
      armingSequenceFlag = false;

    }
  }

    // Serial.print("PITCH: "); Serial.println(rcChannels[PITCH]);
    // Serial.println("BEFORE THE TRUE LOOP");
  if(armsequencecomplete){ // armsequencecomplete
    // THROTTLE CHANGES 
    // TOP side Ultrasonic Sensor
    // Serial.print("US READY");
    if (US_ready1) {
      // Serial.print("US1 Distance: ");
      CurrentDistance = round(distanceCm1 * 100) / 100;
      // Serial.print("Top US:                   ");
      // Serial.print(distanceCm1, 2);
      // Serial.println(" cm");
      US_ready1 = false;  // Current distance of ultrasonic is saved in: distanceCm1
      if (CurrentDistance < 155.41 ){ //  && currentMillis - lastmillis1 > 1000
        rcChannels[THROTTLE] = 1321; // 155.41176
        // lastmillis1 = currentMillis;
      } else if (CurrentDistance > 155.41 && CurrentDistance < 158.24){
      rcChannels[THROTTLE] = 8.5*CurrentDistance;    
      } else if (CurrentDistance > 158.24){ // 158.23529
      rcChannels[THROTTLE] = 1345;    
      }
    }    

    // Serial.println("BEFORE YAW CHANGES");
    // // YAW Changes
    readCenterAverage(sensor1, measurementData1);
    Serial.print("EOP: "); Serial.println(endofpath);    
    if(!endofpath){
      Serial.println("Inside YAW IF "); 
      Serial.print("Time Delay:: "); Serial.println(currentMillis - lastYAWtime);    
      Serial.print("TOF: "); Serial.println(centerAvg);    
      if(currentMillis - lastYAWtime > 800){ // 800ms 
      // Serial.println("IN YAW if statement");
        if(centerAvg > 0 && centerAvg < 1000){ // centerAvg returns middle 4 tof average ranges in mm. 
          rcChannels[YAW] = 1312;  // 125 ~ 15 degrees change. 188 ~ 22.56 degrees. 94 ~11.28
          lastYAWtime = currentMillis;
        } else if(centerAvg > 1100){ // 1200mm = 1.2m
          rcChannels[YAW] = 1688;
          lastYAWtime = currentMillis;
        }
        Serial.print("YAW: "); Serial.println(rcChannels[YAW]);    
        // Serial.print("TurnLeft: "); Serial.println(TurnLeft);
      } 
    }
    // TURN DRONE but wait 500ms before turning. 
    else if(currentMillis - endofpathtime > 500 && !(TurnLeft == 3)){ // (endofpath && ) endofpath is assumed to be true if it is not false?
      rcChannels[YAW] =  1250; // Left Turn 90 degrees ~ 750, 45* ~ 375, 30* =250
      TurnLeft += 1;
      endofpathtime = currentMillis; // reset timer, Drone is to rotate slowly. 
      // Serial.print("EOP YAW Value: "); Serial.println(rcChannels[YAW]);
      // Serial.print("TurnLeft value: "); Serial.println(TurnLeft);
    }
    // Serial.print("lastYAWtime: "); Serial.println(lastYAWtime);
    // Same process for turning right. However the direction the drone turns must be decided by the tof? US? or always follow left wall? or hardcode turns?
    // expected path (assuming we are starting from the window area):
    //  Go straight, 180, turn left, 180, turn left, turn right, turn left, end. 

    // PITCH Changes
    if (endofpath){
    rcChannels[PITCH] = 1500;
    } else{ 
      rcChannels[PITCH] = 1550; // appply pitch brakes and prepare to turn. 
      // Serial.print("IN EOP PITCH: "); Serial.println(rcChannels[PITCH]);
    }

    // FRONT Ultrasonic Sensor (MB1000) - CHECKING for upcoming obstacle infront (assume obstacle is wall)
    if (US_readyF) { 
      // Serial.print("US1 Distance: ");
      CurrentDistanceF = round(distanceCmF * 100) / 100;
      // Serial.print("CurrentMillis: "); Serial.println(currentMillis);
      Serial.print("LOOKY HERE AT THIS PART US FRONT :   ");
      Serial.print(distanceCmF, 2);
      Serial.println(" cm");
      US_readyF = false;  // Current distance of ultrasonic is saved in: distanceCm1
      
      if(TurnLeft == 3 && CurrentDistanceF < 30.00){
        endofpath = true; // Drone should be turning LEFT
        TurnLeft = 0;
      }  
      // else{
      //   // nothing
      // }
      Serial.print("END OF PATH: "); Serial.println(endofpath);
    }

    // Turn Left complete, reset endofpath boolean to false.   
    if(TurnLeft == 3){
      endofpath = false;
    }      

    // END of if(armsequencecomplete) statement. 
  }

// Future
// Add ROLL? testing 1525 right, 1425 left, hold for 100ms then go back to 1500. (DPAD first) - remove wall follow for this. 
// TurnLeft once complete go forward blindly for 2 seconds. 
// TurnLeft increase to 45 degrees per iteration (Drone seems to undershoot). 



// To TEST:
// TILT to prevent side wall crash (minor correction left and right)

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



  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);

  if (currentMillis > sbusTime) {
    rcChannels[THROTTLE] = min(THROTTLE_MAX, rcChannels[THROTTLE]);
    rcChannels[THROTTLE] = max(THROTTLE_MIN, rcChannels[THROTTLE]);

    sbusPreparePacket(sbusPacket, rcChannels, false, false);
    Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
    // printSBUSChannel(rcChannels);
    // printSBUSPacket(sbusPacket);
    sbusTime = currentMillis + SBUS_UPDATE_RATE;
    // Serial.print("Current Throttle: "); Serial.println(rcChannels[THROTTLE]);
    Serial.print("PITCH: "); Serial.println(rcChannels[PITCH]);
    Serial.print("SBUS TOF: "); Serial.println(centerAvg);    
    Serial.print("SBUS YAW:                 "); Serial.println(rcChannels[YAW]);
  }
}
