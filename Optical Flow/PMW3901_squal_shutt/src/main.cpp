#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Bitcraze_PMW3901.h>

// Defining SPI pins
#define OF_CS 15 // Optical Flow CS pin
#define SPI_MOSI 13
#define SPI_CLK 14
#define SPI_MISO 33
#define RST 3
#define RST 3

Bitcraze_PMW3901 flow(OF_CS);

char frame[35*35]; //array to hold the framebuffer

int PixelScale = 4;

void setup() {
  SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, OF_CS);
  Serial.begin(115200);
  flow.begin();
  delay(100);

  flow.enableFrameBuffer(); 
}

uint8_t readRegister(uint8_t reg) {
  digitalWrite(OF_CS, LOW);
  SPI.transfer(reg & 0x7F);  // MSB=0 -> read
  uint8_t val = SPI.transfer(0);
  digitalWrite(OF_CS, HIGH);
  return val;
}

uint16_t readShutter() {
  uint8_t hi = readRegister(0x0C);   // shutter high byte
  uint8_t lo = readRegister(0x0B);   // shutter low byte
  return (uint16_t(hi) << 8) | lo;
}

void loop() {
  flow.readFrameBuffer(frame);
  for (int i = 0; i < 1225; i++) { // 1 frame of 1225 pixels (35*35)
    Serial.printf("%d, ", frame[i]);
  }
  Serial.println(" ");
  
  uint8_t squal = readRegister(0x07);
  Serial.print("SQUAL = ");
  Serial.println(squal);
  uint8_t shutt = readShutter();
  Serial.print("SHUTTER = ");
  Serial.println(shutt);
  delay(100);
}