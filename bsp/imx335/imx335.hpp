#pragma once
#include "sccb.hpp"
#include "imx335_reg.hpp"

class Imx335 {
public:
    Imx335(Sccb& sccb) : sccb(sccb) {}
    bool Init();
    uint32_t ReadId();
    bool SetStream(bool on);
    bool EnableTestPattern(uint8_t mode);

private:
    bool WriteTable(const imx335::Reg* regs, uint16_t n);

    static constexpr uint16_t RegId = 0x3912;
    static constexpr uint16_t RegMode = 0x3000;
    Sccb& sccb;
};