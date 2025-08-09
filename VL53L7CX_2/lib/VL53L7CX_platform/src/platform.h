#pragma once
#include <Arduino.h>
#include <Wire.h>

typedef struct {
  TwoWire* wire;
  uint8_t  address;
} VL53L7CX_Platform;

extern "C" {
  int32_t VL53L7CX_WriteMulti(
    VL53L7CX_Platform* p,
    uint16_t reg,
    uint8_t* data,
    uint32_t count);

  int32_t VL53L7CX_ReadMulti(
    VL53L7CX_Platform* p,
    uint16_t reg,
    uint8_t* data,
    uint32_t count);

  void VL53L7CX_WaitMs(VL53L7CX_Platform* p, int32_t ms);
}