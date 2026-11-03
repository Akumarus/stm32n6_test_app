#pragma once
#include "i2c.h"

class I2c {
public:
    I2c(I2C_HandleTypeDef& i2c, uint32_t timeout) : i2c(i2c), timeout(timeout) {}
    void Init();
    bool Write(uint16_t addr, uint8_t* data, uint16_t size);
    bool Read(uint16_t addr, uint8_t* data, uint16_t size);
    bool WriteMem(uint16_t addr, uint16_t reg, uint8_t* data, uint16_t size);
    bool ReadMem(uint16_t addr, uint16_t reg, uint8_t* data, uint16_t size);
    
    void SetTimeout(uint32_t time);

private:
    I2C_HandleTypeDef& i2c;
    uint32_t timeout;
};