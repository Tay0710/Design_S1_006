#include <Arduino.h>

/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "sbus.h"

#define RX_PIN 18
#define TX_PIN 17

#define SBUS_MIN 885
#define SBUS_MAX 2115
#define SBUS_MID 1500
#define SBUS_CHANNELS 16

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

#define MID_RC 1500


/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1, RX_PIN, TX_PIN, false, false);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1, RX_PIN, TX_PIN, false, false);
/* SBUS data */
bfs::SbusData data;

// static float sbusChannelsReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
// {
//     // Linear fitting values read from OpenTX-ppmus and comparing with values received by X4R
//     // http://www.wolframalpha.com/input/?i=linear+fit+%7B173%2C+988%7D%2C+%7B1812%2C+2012%7D%2C+%7B993%2C+1500%7D
//     return (5 * (float)rxRuntimeState->channelData[chan] / 8) + 880;
//     // if channel = 992 --> 1500
// }

// void sbusChannelsInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
// {
//     rxRuntimeState->rcReadRawFn = sbusChannelsReadRawRC;
//     for (int b = 0; b < SBUS_CHANNELS; b++) {
//         rxRuntimeState->channelData[b] = (16 * rxConfig->midrc) / 10 - 1408;
//         // maps mid_rc = 1500 to 992
//     }
// }

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  delay(500);
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();

  // Initialise data as mid values
  for (int i = 0; i < SBUS_CHANNELS; i++) {
    data.ch[i] = 992;
  }
}

void loop () {
    Serial.println("Sending data...");
    /* Set the SBUS TX data to the received data */
    sbus_tx.data(data);
    /* Write the data to the servos */
    sbus_tx.Write();
    delay(100);
}