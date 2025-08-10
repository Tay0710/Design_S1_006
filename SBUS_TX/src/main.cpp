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

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1, RX_PIN, TX_PIN, false, false);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1, RX_PIN, TX_PIN, false, false);
/* SBUS data */
bfs::SbusData data;

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
    data.ch[i] = SBUS_MID;
  }
}

void loop () {
    Serial.println("Sending data...");
    /* Set the SBUS TX data to the received data */
    sbus_tx.data(data);
    /* Write the data to the servos */
    sbus_tx.Write();
    //delay(1000);
}