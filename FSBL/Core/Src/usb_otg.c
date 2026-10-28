/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usb_otg.c
  * @brief   This file provides code for the configuration
  *          of the USB_OTG instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usb_otg.h"

/* USER CODE BEGIN 0 */
#define RESET_USB_MACRO() do { \
    LL_AHB5_GRP1_ForceReset(0x00800000); \
    __HAL_RCC_USB1_OTG_HS_FORCE_RESET(); \
    __HAL_RCC_USB1_OTG_HS_PHY_FORCE_RESET(); \
    LL_RCC_HSE_SelectHSEDiv2AsDiv2Clock(); \
    LL_AHB5_GRP1_ReleaseReset(0x00800000); \
} while(0)
/* USER CODE END 0 */

PCD_HandleTypeDef hpcd_USB_OTG_HS1;

/* USB1_OTG_HS init function */

void MX_USB1_OTG_HS_PCD_Init(void)
{

  /* USER CODE BEGIN USB1_OTG_HS_Init 0 */

  /* USER CODE END USB1_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB1_OTG_HS_Init 1 */

  /* USER CODE END USB1_OTG_HS_Init 1 */
  hpcd_USB_OTG_HS1.Instance = USB1_OTG_HS;
  hpcd_USB_OTG_HS1.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS1.Init.speed = PCD_SPEED_HIGH;
  hpcd_USB_OTG_HS1.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
  hpcd_USB_OTG_HS1.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS1.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS1.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS1.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS1.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_HS1.Init.dma_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB1_OTG_HS_Init 2 */
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_HS1, 0x200);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS1, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS1, 1, 0x20);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS1, 2, 0x100);
  /* USER CODE END USB1_OTG_HS_Init 2 */

}

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(pcdHandle->Instance==USB1_OTG_HS)
  {
  /* USER CODE BEGIN USB1_OTG_HS_MspInit 0 */

  /* USER CODE END USB1_OTG_HS_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USBOTGHS1;
    PeriphClkInitStruct.UsbPhy1ClockSelection = RCC_USBPHY1CLKSOURCE_HSE_DIV2;
    PeriphClkInitStruct.UsbOtgHs1ClockSelection = RCC_USBOTGHS1CLKSOURCE_HSE_DIRECT;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Enable VDDUSB */
    HAL_PWREx_EnableVddUSB();
    /* USB1_OTG_HS clock enable */
    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();
    __HAL_RCC_USB1_OTG_HS_PHY_CLK_ENABLE();

    /* USB1_OTG_HS interrupt Init */
    HAL_NVIC_SetPriority(USB1_OTG_HS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);
  /* USER CODE BEGIN USB1_OTG_HS_MspInit 1 */
  RESET_USB_MACRO();
  HAL_Delay(1);
  USB1_HS_PHYC->USBPHYC_CR &= ~(0x7 << 0x4);
  USB1_HS_PHYC->USBPHYC_CR |= (0x1 << 16) |   /* OTGDISABLE0 */
                              (0x2 << 4)  |    /* FSEL = 2 (24 МГц) */
                              (0x1 << 2)  |    /* CMN */
                               0x1U;           /* RETENABLEN */
  __HAL_RCC_USB1_OTG_HS_PHY_RELEASE_RESET();
  HAL_Delay(1);
  __HAL_RCC_USB1_OTG_HS_RELEASE_RESET();
  /* USER CODE END USB1_OTG_HS_MspInit 1 */
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle)
{

  if(pcdHandle->Instance==USB1_OTG_HS)
  {
  /* USER CODE BEGIN USB1_OTG_HS_MspDeInit 0 */

  /* USER CODE END USB1_OTG_HS_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USB1_OTG_HS_CLK_DISABLE();
    __HAL_RCC_USB1_OTG_HS_PHY_CLK_DISABLE();

    /* Disable VDDUSB */
      HAL_PWREx_DisableVddUSB();

    /* USB1_OTG_HS interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB1_OTG_HS_IRQn);
  /* USER CODE BEGIN USB1_OTG_HS_MspDeInit 1 */

  /* USER CODE END USB1_OTG_HS_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

