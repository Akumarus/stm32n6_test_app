#include "imx335.hpp"

uint32_t Imx335::ReadId()
{
    uint8_t data = 0xFF;
    sccb.ReadReg(RegId, data);
    return data;
}