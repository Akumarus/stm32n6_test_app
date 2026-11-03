#pragma once
#include "sccb.hpp"

class Imx335 {
public:
    Imx335(Sccb& sccb) : sccb(sccb) {}
    uint32_t ReadId();

private:
    static constexpr uint16_t RegId = 0x3912;
    Sccb& sccb;
};