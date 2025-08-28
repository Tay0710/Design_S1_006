#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Bitcraze_PMW3901.h>

// Defining SPI pins
#define SPI_CS 10
#define SPI_MOSI 11
#define SPI_CLK 12
#define SPI_MISO 13
#define RST 3

Bitcraze_PMW3901 flow(SPI_CS);

char frame[35*35]; //array to hold the framebuffer

int PixelScale = 4;

void setup() {
  SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);
  Serial.begin(115200);
  flow.begin();
  delay(100);

  flow.enableFrameBuffer(); 
}

void loop() {
  flow.readFrameBuffer(frame);
  for (int i = 0; i < 1225; i++) { // 1 frame of 1225 pixels (35*35)
    Serial.printf("%d, ", frame[i]);
  }
  Serial.println(" ");
}