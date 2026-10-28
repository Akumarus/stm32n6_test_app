#include "main.h"
#include "usb_otg.h"
extern "C" {
    #include "ux_api.h"
    #include "ux_device_class_cdc_acm.h"
}
#include "gpio.h"

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

extern "C" int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB1_OTG_HS_PCD_Init();
  MX_USBX_Init();
  /* USER CODE BEGIN 2 */
  HAL_PCD_Start(&hpcd_USB_OTG_HS1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ux_system_tasks_run();
    uint8_t *p;
    uint32_t n = CdcAcm_Poll(&p);
    if (n) {
      n = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}