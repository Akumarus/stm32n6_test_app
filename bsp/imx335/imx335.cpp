#include "imx335.hpp"

bool Imx335::Init()
{
    bool res;
    uint16_t count1 = sizeof(imx335::Res2592x1944) / sizeof(imx335::Res2592x1944[0]);
    uint16_t count2 = sizeof(imx335::Mode2Lane10bit) / sizeof(imx335::Mode2Lane10bit[0]);
    res = WriteTable(imx335::Res2592x1944, sizeof(imx335::Res2592x1944) / sizeof(imx335::Res2592x1944[0]));
    res = WriteTable(imx335::Mode2Lane10bit, sizeof(imx335::Mode2Lane10bit) / sizeof(imx335::Mode2Lane10bit[0]));
    // res = SetStream(true);
    HAL_Delay(20);
    return res;
}

uint32_t Imx335::ReadId()
{
    uint8_t data = 0xFF;
    sccb.ReadReg(RegId, data);
    return data;
}

bool Imx335::SetStream(bool on)
{
    uint8_t data = on ? 0x00 : 0x01;
    return sccb.WriteReg(RegMode, data);
}

bool Imx335::WriteTable(const imx335::Reg *regs, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        uint8_t v = regs[i].val;
        if (!sccb.WriteReg(regs[i].addr, v))
            return false;
    }
    return true;
}

bool Imx335::EnableTestPattern(uint8_t mode)
{
    sccb.WriteReg(0x329E, mode);
    static constexpr imx335::Reg en[] = {
        {0x3148, 0x10}, {0x3280, 0x00}, {0x329C, 0x01},
        {0x32A0, 0x11}, {0x3302, 0x00}, {0x3303, 0x00}, {0x336C, 0x00},
    };
    return WriteTable(en, sizeof(en)/sizeof(en[0]));
}