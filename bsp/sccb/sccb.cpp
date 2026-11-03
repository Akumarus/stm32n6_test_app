#include "sccb.hpp"



bool Sccb::WriteReg(uint16_t reg, uint8_t& data)
{
    return i2c.WriteMem(addr, reg, &data, 1);
}

bool Sccb::ReadReg(uint16_t reg, uint8_t& data)
{
    return i2c.ReadMem(addr, reg, &data, 1);
}