/*

  Adapted using example 'Controller.ino' from Bluepad32 library for Arduino (https://github.com/ricardoquesada/bluepad32) following documentation from 
  https://bluepad32.readthedocs.io/en/latest/plat_arduino/
  
*/

#include <Bluepad32.h>
#include "SBUS.h"

// Using UART2 on ESP32
#define RX_PIN 16
#define TX_PIN 17

// Initialise global variables
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;

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
#define SBUS_MIN 885
#define SBUS_MID 1500
#define SBUS_MAX 2115

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
      dumpGamepad(ctl); // Prints everything for debug
    }
    // Press L2 + R2 to arm (AUX1)
    if (ctl->l2() and ctl->r2()) {
      Serial.println("ARM DRONE");
      rcChannels[AUX1] = 1800;
    } else {
      rcChannels[AUX1] = 1500;
      // Unsure if it needs to be set back to 1500 otherwise... probably
    }

    // Map throttle values
    if (ctl->axisY() <= AXIS_L_NEUTRAL_Y) {
      rcChannels[THROTTLE] =(int)map(ctl->axisY(), AXIS_L_NEUTRAL_Y, AXIS_UP, SBUS_MID, SBUS_MAX);
    } else {
      rcChannels[THROTTLE] =(int)map(ctl->axisY(), AXIS_L_NEUTRAL_Y, AXIS_DOWN, SBUS_MID, SBUS_MIN);
    }
    // Serial.print("axis l - y: ");
    // Serial.print(ctl->axisY());
    // Serial.print("\t throttle: ");
    // Serial.println(rcChannels[THROTTLE]);

    // Map yaw values
    if (ctl->axisX() <= AXIS_L_NEUTRAL_X) {
      rcChannels[YAW] =(int)map(ctl->axisX(), AXIS_L_NEUTRAL_X, AXIS_LEFT, SBUS_MID, SBUS_MIN);
    } else {
      rcChannels[YAW] =(int)map(ctl->axisX(), AXIS_L_NEUTRAL_X, AXIS_RIGHT, SBUS_MID, SBUS_MAX);
    }
    // Serial.print("axis l - x: ");
    // Serial.print(ctl->axisX());
    // Serial.print("\t yaw: ");
    // Serial.println(rcChannels[YAW]);

    // Map pitch values
    // Note: this currently assumes pitching down is > 1500 and pitching up is < 1500, if not simply swap SBUS_MAX and SBUS_MIN
    if (ctl->axisRY() <= AXIS_R_NEUTRAL_Y) {
      rcChannels[PITCH] =(int)map(ctl->axisRY(), AXIS_R_NEUTRAL_Y, AXIS_UP, SBUS_MID, SBUS_MAX);
    } else {
      rcChannels[PITCH] =(int)map(ctl->axisRY(), AXIS_R_NEUTRAL_Y, AXIS_DOWN, SBUS_MID, SBUS_MIN);
    }
    // Serial.print("axis r - y: ");
    // Serial.print(ctl->axisY());
    // Serial.print("\t pitch: ");
    // Serial.println(rcChannels[PITCH]);

    // Map roll values
    if (ctl->axisRX() <= AXIS_R_NEUTRAL_X) {
      rcChannels[ROLL] =(int)map(ctl->axisRX(), AXIS_R_NEUTRAL_X, AXIS_LEFT, SBUS_MID, SBUS_MIN);
    } else {
      rcChannels[ROLL] =(int)map(ctl->axisRX(), AXIS_R_NEUTRAL_X, AXIS_RIGHT, SBUS_MID, SBUS_MAX);
    }
    // Serial.print("axis r - x: ");
    // Serial.print(ctl->axisRX());
    // Serial.print("\t yaw: ");
    // Serial.println(rcChannels[ROLL]);

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

// Arduino setup function. Runs in CPU 1
void setup() {
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
    // Initialise throttle to 885
    rcChannels[THROTTLE] = 885; // must be below min_check = 1050 when arming
 
    // Stuff that Aidan wanted lmao
    rcChannels[AUX2] = 1200; // For angle mode?
    // TODO: might map this to button instead

    Serial1.begin(100000, SERIAL_8E2, RX_PIN, TX_PIN, true); // Initialize Serial1 with 100000 baud rate
    // false = univerted, true = inverted

    Serial.println(" --- Setup Complete --- ");
}

// Arduino loop function. Runs in CPU 1.
void loop() {

    uint32_t currentMillis = millis();

    // This call fetches all the controllers' data.
    // Call this function in your main loop.


    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);

    if (currentMillis > sbusTime) {
      sbusPreparePacket(sbusPacket, rcChannels, false, false);
      Serial1.write(sbusPacket, SBUS_PACKET_LENGTH);
      //printSBUSChannel(rcChannels);
      //printSBUSData(sbusPacket);
      sbusTime = currentMillis + SBUS_UPDATE_RATE;
    }
}
