#include "main.h"
#include "usb_otg.h"
extern "C" {
    #include "ux_api.h"
    #include "ux_device_class_cdc_acm.h"
}
#include "gpio.h"
#include "i2c.h"

extern "C" {
    UINT MX_USBX_Init(void);
}

extern PCD_HandleTypeDef           hpcd_USB_OTG_HS1;
extern UX_SLAVE_CLASS_CDC_ACM *cdc;
static uint8_t rx_buf[64];
static ULONG   rx_len;

/* Чтение команд: возвращает число принятых байт (0 — пока нет данных) */
uint32_t CdcAcm_Poll(uint8_t **data)
{
    if (cdc == UX_NULL) return 0;
    UINT s = ux_device_class_cdc_acm_read_run(cdc, rx_buf, sizeof(rx_buf), &rx_len);
    if (s == UX_STATE_NEXT && rx_len > 0) { *data = rx_buf; return rx_len; }
    return 0;
}

/* Запись: вызывать, пока не вернёт 1 (всё отправлено) или -1 (ошибка) */
int CdcAcm_WriteStep(uint8_t *buf, uint32_t len)
{
    static ULONG actual;
    if (cdc == UX_NULL) return -1;
    UINT s = ux_device_class_cdc_acm_write_run(cdc, buf, len, &actual);
    if (s == UX_STATE_NEXT)  return 1;
    if (s == UX_STATE_WAIT)  return 0;
    return -1;
}

#include "sccb.hpp"
#include "i2c.hpp"
#include "imx335.hpp"
#include "dcmipp.h"

/* Кадр: 640 * 480 * 2 (RGB565) = 614400 байт, AXISRAM1.
   Проверьте, что линкер не использует 0x34000000..0x34096000 */
#define FRAME_W     640
#define FRAME_H     480
#define FRAME_SIZE  (FRAME_W * FRAME_H * 2)
static uint8_t * const frame_buf = (uint8_t*)0x34000000;
// static uint8_t * const frame_buf = (uint8_t*)0x24000000;

/* Диагностика */
volatile bool     frame_ready;
volatile uint32_t cnt_vsync, cnt_frame, cnt_err, cnt_pipe_err, cnt_sof, cnt_eof, cnt_lerr;
volatile uint32_t last_err;

extern "C" {
void HAL_DCMIPP_PIPE_VsyncEventCallback(DCMIPP_HandleTypeDef *h, uint32_t pipe) { cnt_vsync++; }
void HAL_DCMIPP_PIPE_FrameEventCallback(DCMIPP_HandleTypeDef *h, uint32_t pipe)
{
    cnt_frame++;
    if (pipe == DCMIPP_PIPE1) frame_ready = true;
}
void HAL_DCMIPP_PIPE_ErrorCallback(DCMIPP_HandleTypeDef *h, uint32_t pipe) { cnt_pipe_err++; last_err = h->ErrorCode; }
void HAL_DCMIPP_ErrorCallback(DCMIPP_HandleTypeDef *h) { cnt_err++; last_err = h->ErrorCode; }
void HAL_DCMIPP_CSI_StartOfFrameEventCallback(DCMIPP_HandleTypeDef *h, uint32_t vc) { cnt_sof++; }
void HAL_DCMIPP_CSI_EndOfFrameEventCallback(DCMIPP_HandleTypeDef *h, uint32_t vc) { cnt_eof++; }
void HAL_DCMIPP_CSI_LineErrorCallback(DCMIPP_HandleTypeDef *h, uint32_t lane) { cnt_lerr++; last_err = h->ErrorCode; }
}
static void delay_usb(uint32_t ms)
{
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() - t < ms) ux_system_tasks_run();
}

/* Питание и сброс камеры (последовательность из BSP_CAMERA_HwReset примера ST):
   PC8 = питание 2V8, PD2 = reset */
static void camera_hw_reset(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_8;  HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_2;  HAL_GPIO_Init(GPIOD, &g);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    delay_usb(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    delay_usb(100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    delay_usb(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    delay_usb(100);
}

extern "C" int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB1_OTG_HS_PCD_Init();
  MX_USBX_Init();
  
  HAL_PCD_Start(&hpcd_USB_OTG_HS1);
  delay_usb(2000);
  /* 1) питание и reset камеры */
  camera_hw_reset();
  SystemIsolation_Config();

  MX_DCMIPP_Init();
  __HAL_DCMIPP_CSI_ENABLE_IT(CSI, DCMIPP_CSI_IT_SOF0 | DCMIPP_CSI_IT_EOF0);

  uint32_t data = 0xFF;
  static I2c i2c1(hi2c1, 1000);
  i2c1.Init();
  static Sccb sccb(i2c1, 0x34);
  static Imx335 imx335(sccb);
  data = imx335.ReadId();
  auto res = imx335.Init();
  (void)data; (void)res;
  // imx335.EnableTestPattern(2);

  memset(frame_buf, 0x55, FRAME_SIZE);
  if (HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0,
                                (uint32_t)frame_buf, DCMIPP_MODE_CONTINUOUS) != HAL_OK)
  {
      Error_Handler();
  }

  imx335.SetStream(true);
  delay_usb(20);

  /* Протокол по USB CDC: заголовок 16 байт + кадр RGB565 (little-endian)
     magic 'FRM1' (4) | width u16 | height u16 | size u32 | frame_id u32 */
  struct __attribute__((packed)) FrameHdr {
      char     magic[4];
      uint16_t w, h;
      uint32_t size;
      uint32_t id;
  };
  static FrameHdr hdr;
  uint32_t frame_id = 0;
  enum { IDLE, SEND_HDR, SEND_DATA } tx = IDLE;

  while (1)
  {
    ux_system_tasks_run();

    uint8_t *p;
    uint32_t n = CdcAcm_Poll(&p);
    (void)n;

    if (frame_ready && tx == IDLE)
    {
        frame_ready = false;
        /* если включён D-Cache: SCB_InvalidateDCache_by_Addr(frame_buf, FRAME_SIZE); */
        hdr = { {'F','R','M','1'}, FRAME_W, FRAME_H, FRAME_SIZE, frame_id++ };
        tx = SEND_HDR;
    }
    if (tx == SEND_HDR)
    {
        int r = CdcAcm_WriteStep((uint8_t*)&hdr, sizeof(hdr));
        if (r == 1) tx = SEND_DATA;
        else if (r < 0) tx = IDLE;
    }
    else if (tx == SEND_DATA)
    {
        int r = CdcAcm_WriteStep(frame_buf, FRAME_SIZE);
        if (r != 0)
        {
            tx = IDLE;
            /* следующий кадр */
            HAL_DCMIPP_CSI_PIPE_Stop(&hdcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
            HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0,
                                      (uint32_t)frame_buf, DCMIPP_MODE_SNAPSHOT);
        }
    }
  }
}