#pragma once
#include "i2c.hpp"

class Sccb {
public:
    Sccb(I2c& i2c, uint16_t addr) : i2c(i2c), addr(addr) {}
    bool WriteReg(uint16_t reg, uint8_t& data);
    bool ReadReg(uint16_t reg, uint8_t& data);

private:
    I2c& i2c;
    uint16_t addr;
};