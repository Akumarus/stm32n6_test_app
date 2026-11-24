#pragma once
#include <cstdint>
#include "jpeg.h"

class Jpeg {
public:
    bool     Init(uint32_t w, uint32_t h, uint32_t quality);
    uint32_t Encode(const uint16_t *inp, uint8_t *out, uint32_t outSize);

    // Последний код ошибки HAL (0 — без ошибки)
    uint32_t LastError() const { return lastError; }

    // вызывается из колбэка HAL
    void     NextChunk();

private:
    void ConvertRow(uint32_t row);
    void Recover();
    static void rgb565ToRgb(uint16_t v, int &r, int &g, int &b);

    static constexpr uint32_t MaxWidth = 640;
    static constexpr uint32_t RowBytesMax = (MaxWidth / 16) * 384;

    JPEG_ConfTypeDef conf = {};
    uint32_t width = 0, height = 0;
    uint32_t rowBytes = 0, row = 0, rows = 0;
    uint32_t lastError = 0;
    const uint16_t *src = nullptr;
    static inline uint8_t chunk[RowBytesMax] __attribute__((aligned(32)));
};
