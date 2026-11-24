#include "jpeg.hpp"

static Jpeg *g_self = nullptr;
static uint32_t g_outLen = 0;      // сколько байт JPEG уже принято
static uint32_t g_outCap = 0;      // размер выходного буфера
static volatile bool g_overflow = false;
static volatile uint32_t g_halErr = 0;

extern "C" void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *h, uint32_t n)
{
    (void)h; (void)n;
    if (g_self) g_self->NextChunk();
}

/* HAL вызывает этот колбэк, когда выходной буфер заполнен или кодирование
   завершено. Суммируем фактическую длину JPEG. */
extern "C" void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *h, uint8_t *p, uint32_t len)
{
    (void)h; (void)p;
    if (g_outLen + len > g_outCap) {
        g_overflow = true;          // кадр не влез в буфер — потом отбросим
    } else {
        g_outLen += len;
    }
}

extern "C" void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *h)
{
    g_halErr = h->ErrorCode;
}

bool Jpeg::Init(uint32_t w, uint32_t h, uint32_t q)
{
    if ((w % 16) || (h % 16) || w > MaxWidth || w == 0 || h == 0) return false;

    width = w; height = h;
    rowBytes = (w / 16) * 384;
    rows = h / 16;
    g_self = this;
    lastError = 0;

    conf.ColorSpace        = JPEG_YCBCR_COLORSPACE;
    conf.ChromaSubsampling = JPEG_420_SUBSAMPLING;
    conf.ImageWidth        = w;
    conf.ImageHeight       = h;
    conf.ImageQuality      = q;

    hjpeg.Instance = JPEG;
    if (HAL_JPEG_Init(&hjpeg) != HAL_OK) return false;
    if (HAL_JPEG_ConfigEncoding(&hjpeg, &conf) != HAL_OK) return false;
    return true;
}

/* Возвращает фактический размер JPEG или 0 при ошибке. */
uint32_t Jpeg::Encode(const uint16_t *inp, uint8_t *out, uint32_t outSize)
{
    if (!inp || !out || rows == 0 || outSize < 16) {
        lastError = 0xFFFFFFFFu;
        return 0;
    }

    src = inp;
    row = 0;
    g_outLen = 0;
    g_outCap = outSize;
    g_overflow = false;
    g_halErr = hjpeg.ErrorCode = HAL_JPEG_ERROR_NONE;

    ConvertRow(0);

    HAL_StatusTypeDef st = HAL_JPEG_Encode(&hjpeg, chunk, rowBytes, out, outSize, 5000);
    lastError = hjpeg.ErrorCode;

    if (st != HAL_OK || g_overflow || g_halErr != 0 || hjpeg.ErrorCode != HAL_JPEG_ERROR_NONE) {
        if (!lastError) lastError = (st == HAL_TIMEOUT) ? HAL_JPEG_ERROR_TIMEOUT : 0xFFFFFFFFu;
        Recover();
        return 0;
    }
    return g_outLen;
}

/* После ошибки/таймаута периферия остаётся в BUSY/ERROR, поэтому переинициализируем её. */
void Jpeg::Recover()
{
    HAL_JPEG_Abort(&hjpeg);
    hjpeg.ErrorCode = HAL_JPEG_ERROR_NONE;
    hjpeg.State = HAL_JPEG_STATE_READY;
    if (HAL_JPEG_Init(&hjpeg) == HAL_OK) {
        (void)HAL_JPEG_ConfigEncoding(&hjpeg, &conf);
    }
}

void Jpeg::NextChunk()
{
    row++;
    if (row < rows) {
        ConvertRow(row);
        HAL_JPEG_ConfigInputBuffer(&hjpeg, chunk, rowBytes);
    } else {
        HAL_JPEG_ConfigInputBuffer(&hjpeg, chunk, 0);   // данных больше нет
    }
}

void Jpeg::rgb565ToRgb(uint16_t v, int &r, int &g, int &b)
{
    r = (v >> 11) & 0x1F; r = (r << 3) | (r >> 2);
    g = (v >> 5)  & 0x3F; g = (g << 2) | (g >> 4);
    b =  v        & 0x1F; b = (b << 3) | (b >> 2);
}

/* Одна строка MCU 16xN: RGB565 -> YCbCr 4:2:0 (Y00 Y01 Y10 Y11 Cb Cr, 384 байта на MCU) */
void Jpeg::ConvertRow(uint32_t rowIdx)
{
    uint8_t *dst = chunk;
    const uint32_t my = rowIdx * 16;
    for (uint32_t mx = 0; mx < width; mx += 16) {
        uint8_t *Y = dst, *Cb = dst + 256, *Cr = dst + 320;
        int sumCb[64] = {0}, sumCr[64] = {0};
        for (int by = 0; by < 2; by++)
        for (int bx = 0; bx < 2; bx++) {
            uint8_t *yb = Y + (by * 2 + bx) * 64;
            for (int y = 0; y < 8; y++)
            for (int x = 0; x < 8; x++) {
                int px = mx + bx * 8 + x, py = my + by * 8 + y;
                int r, g, b;
                rgb565ToRgb(src[py * width + px], r, g, b);
                yb[y * 8 + x] = (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
                int cb = ((-43 * r - 85 * g + 128 * b) >> 8) + 128;
                int cr = ((128 * r - 107 * g - 21 * b) >> 8) + 128;
                int cx = (bx * 8 + x) >> 1, cy = (by * 8 + y) >> 1;
                sumCb[cy * 8 + cx] += cb;
                sumCr[cy * 8 + cx] += cr;
            }
        }
        for (int i = 0; i < 64; i++) {
            int a = sumCb[i] / 4, c = sumCr[i] / 4;
            Cb[i] = (uint8_t)(a < 0 ? 0 : a > 255 ? 255 : a);
            Cr[i] = (uint8_t)(c < 0 ? 0 : c > 255 ? 255 : c);
        }
        dst += 384;
    }
}
