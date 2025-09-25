// Adapted from RadioLib Transmit with Interrupts Example from RadioLib Library
// https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/tree/master/examples/RadioLibExamples/Transmit_Interrupt


#include "LoRaBoards.h"
#include <RadioLib.h>

#define CONFIG_RADIO_FREQ           915.0
#define CONFIG_RADIO_OUTPUT_POWER   22
#define CONFIG_RADIO_BW             125.0

#define BOOT_BUTTON 0
#define RED_BUTTON 42

// Define radio object
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

void drawMain(); // Function declaration
void drawInstructions();


// save transmission state between loops
static int transmissionState = RADIOLIB_ERR_NONE;
// flag to indicate that a packet was sent
static volatile bool transmittedFlag = false;
static volatile bool failsafeFlag = false;
static uint32_t counter = 0;
static String payload;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void)
{
    // we sent a packet, set the flag
    transmittedFlag = true;
}

void interruptCallback() {

    // if failsafe button was pressed, set the flag
    failsafeFlag = true;
}

void setup()
{
    setupBoards();

    // When the power is turned on, a delay is required.
    delay(1500);

    // initialize radio with default settings
    int state = radio.begin();

    printResult(state == RADIOLIB_ERR_NONE);

    Serial.printf("[%s]:", RADIO_TYPE_STR);
    Serial.print(F("Radio Initializing ... "));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // set the function that will be called
    // when packet transmission is finished
    radio.setPacketSentAction(setFlag);

    /*
    *   Sets carrier frequency.
    *   SX1268/SX1262 : Allowed values are in range from 150.0 to 960.0 MHz.
    * * * */
    if (radio.setFrequency(CONFIG_RADIO_FREQ) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        Serial.println(F("Selected frequency is invalid for this module!"));
        while (true);
    }

    /*
    *   Sets LoRa link bandwidth.
    *   SX1268/SX1262 : Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
    * * * */
    if (radio.setBandwidth(CONFIG_RADIO_BW) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        Serial.println(F("Selected bandwidth is invalid for this module!"));
        while (true);
    }


    /*
    * Sets LoRa link spreading factor.
    * SX1262        :  Allowed values range from 5 to 12.
    * * * */
    if (radio.setSpreadingFactor(7) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        Serial.println(F("Selected spreading factor is invalid for this module!"));
        while (true);
    }

    /*
    * Sets LoRa coding rate denominator.
    * SX1278/SX1276/SX1268/SX1262 : Allowed values range from 5 to 8. Only available in LoRa mode.
    * * * */
    if (radio.setCodingRate(5) == RADIOLIB_ERR_INVALID_CODING_RATE) {
        Serial.println(F("Selected coding rate is invalid for this module!"));
        while (true);
    }

    /*
    * Sets transmission output power.
    * SX1262        :  Allowed values are in range from -9 to 22 dBm. This method is virtual to allow override from the SX1261 class.
    * * * */
    if (radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        Serial.println(F("Selected output power is invalid for this module!"));
        while (true);
    }

    /*
    * Sets current limit for over current protection at transmitter amplifier.
    * SX1262/SX1268 : Allowed values range from 45 to 120 mA in 2.5 mA steps and 120 to 240 mA in 10 mA steps.
    * NOTE: set value to 0 to disable overcurrent protection
    * * * */
    if (radio.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        Serial.println(F("Selected current limit is invalid for this module!"));
        while (true);
    }

    /*
    * Sets preamble length for LoRa or FSK modem.
    * SX1262/SX1268 : Allowed values range from 1 to 65535.
    * * */
    if (radio.setPreambleLength(8) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
        Serial.println(F("Selected preamble length is invalid for this module!"));
        while (true);
    }

    // Enables or disables CRC check of received packets.
    if (radio.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        Serial.println(F("Selected CRC is invalid for this module!"));
        while (true);
    }


    // Setup up failsafe button
    pinMode(RED_BUTTON, INPUT); // Pulled down on PCB


    
    // Print instructions to screen
    drawInstructions();

    // Wait for start button to be pressed
    while(!digitalRead(RED_BUTTON)) {
    }

    attachInterrupt(digitalPinToInterrupt(RED_BUTTON), interruptCallback, RISING);

    // start transmitting the first packet
    Serial.print(F("Radio Sending first packet ... "));

    // you can transmit C-string or Arduino string up to
    // 256 characters long
    transmissionState = radio.startTransmit("#0");

}

void loop()
{
    // check if the previous transmission finished
    if (transmittedFlag) {

        // reset flag
        transmittedFlag = false;

        flashLed();


        if (transmissionState == RADIOLIB_ERR_NONE) {
            // packet was successfully sent
            Serial.println(F("transmission finished!"));
            // NOTE: when using interrupt-driven transmit method,
            //       it is not possible to automatically measure
            //       transmission data rate using getDataRate()
        } else {
            Serial.print(F("failed, code "));
            Serial.println(transmissionState);
        }


        drawMain();
        // wait a second before transmitting again
        delay(1000);

        // send another one
        Serial.print(F("Radio Sending another packet ... "));

        // you can transmit C-string or Arduino string up to
        // 256 characters long

        if (failsafeFlag) {
            payload = "STOP";
        } else {
            payload = "#" + String(counter++);
        }
        transmissionState = radio.startTransmit(payload);
    }
}

void drawInstructions()
{
    if (u8g2) {
        u8g2->clearBuffer();
        u8g2->drawRFrame(0, 0, 128, 64, 5);

        u8g2->setFont(u8g2_font_pxplusibmvga9_mr);
        u8g2->setCursor(15, 20);
        u8g2->print("Press the");
        u8g2->setCursor(15, 35);
        u8g2->print("button to");
        u8g2->setCursor(15, 50);
        u8g2->print("start!");

        u8g2->sendBuffer();
    }
}

void drawMain()
{
    if (u8g2) {
        u8g2->clearBuffer();
        u8g2->drawRFrame(0, 0, 128, 64, 5);

        u8g2->setFont(u8g2_font_pxplusibmvga9_mr);
        u8g2->setCursor(22, 37);
        u8g2->print("TX:");

        u8g2->setFont(u8g2_font_crox2h_tr);
        u8g2->setCursor( U8G2_HOR_ALIGN_RIGHT(payload.c_str()) - 21, 37);
        u8g2->print(payload);
        u8g2->sendBuffer();
    }
}