#include "platform.h"

int32_t VL53L7CX_WriteMulti(
  VL53L7CX_Platform* p,
  uint16_t reg,
  uint8_t* data,
  uint32_t count)
{
  p->wire->beginTransmission(p->address);
  p->wire->write(uint8_t(reg >> 8));
  p->wire->write(uint8_t(reg & 0xFF));
  p->wire->write(data, count);
  int err = p->wire->endTransmission();
  return (err == 0) ? 0 : -1;
}

int32_t VL53L7CX_ReadMulti(
  VL53L7CX_Platform* p,
  uint16_t reg,
  uint8_t* data,
  uint32_t count)
{
  p->wire->beginTransmission(p->address);
  p->wire->write(uint8_t(reg >> 8));
  p->wire->write(uint8_t(reg & 0xFF));
  p->wire->endTransmission(false);  

  uint32_t rec = p->wire->requestFrom(p->address, count);
  if (rec != count) return -1;
  for (uint32_t i = 0; i < count; i++) {
    data[i] = p->wire->read();
  }
  return 0;
}

void VL53L7CX_WaitMs(VL53L7CX_Platform* p, int32_t ms) {
  delay(ms);
}