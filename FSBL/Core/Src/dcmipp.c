/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dcmipp.c
  * @brief   DCMIPP + CSI configuration for IMX335 (2 lanes, RAW10),
  *          PIPE1: demosaic -> downsize 640x480 -> RGB565
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "dcmipp.h"

/* USER CODE BEGIN 0 */
#include <math.h>

#define CAM_W  2592
#define CAM_H  1944
#define OUT_W  640
#define OUT_H  480

/* Downsize как в примере ST (RM0486, таблица расчёта коэффициентов) */
static HAL_StatusTypeDef dcmipp_downsize(DCMIPP_HandleTypeDef *h, uint32_t pipe,
                                         int32_t camW, int32_t camH,
                                         int32_t capW, int32_t capH)
{
  DCMIPP_DownsizeTypeDef d = {0};

  d.HRatio = (uint32_t)(((float)camW / (float)capW) * 8192.F);
  d.VRatio = (uint32_t)(((float)camH / (float)capH) * 8192.F);
  d.HSize  = (uint32_t)capW;
  d.VSize  = (uint32_t)capH;
  d.HDivFactor = (uint32_t)floorf((1024.F * 8192.F - 1.F) / (float)d.HRatio);
  d.VDivFactor = (uint32_t)floorf((1024.F * 8192.F - 1.F) / (float)d.VRatio);

  if (HAL_DCMIPP_PIPE_SetDownsizeConfig(h, pipe, &d) != HAL_OK) return HAL_ERROR;
  return HAL_DCMIPP_PIPE_EnableDownsize(h, pipe);
}
/* USER CODE END 0 */

DCMIPP_HandleTypeDef hdcmipp;

/* DCMIPP init function */
void MX_DCMIPP_Init(void)
{

  /* USER CODE BEGIN DCMIPP_Init 0 */

  /* USER CODE END DCMIPP_Init 0 */

  DCMIPP_CSI_PIPE_ConfTypeDef pCSI_PipeConfig = {0};
  DCMIPP_CSI_ConfTypeDef pCSI_Config = {0};
  DCMIPP_PipeConfTypeDef pPipeConfig = {0};

  /* USER CODE BEGIN DCMIPP_Init 1 */

  /* USER CODE END DCMIPP_Init 1 */
  hdcmipp.Instance = DCMIPP;
  if (HAL_DCMIPP_Init(&hdcmipp) != HAL_OK)
  {
    Error_Handler();
  }

  /** Pipe 0 Config
  */
  pCSI_PipeConfig.DataTypeMode = DCMIPP_DTMODE_DTIDA;
  pCSI_PipeConfig.DataTypeIDA = DCMIPP_DT_RAW10;
  pCSI_PipeConfig.DataTypeIDB = DCMIPP_DT_RAW10;
  if (HAL_DCMIPP_CSI_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE0, &pCSI_PipeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  pCSI_Config.PHYBitrate = DCMIPP_CSI_PHY_BT_1600;
  pCSI_Config.DataLaneMapping = DCMIPP_CSI_PHYSICAL_DATA_LANES;
  pCSI_Config.NumberOfLanes = DCMIPP_CSI_TWO_DATA_LANES;
  if (HAL_DCMIPP_CSI_SetConfig(&hdcmipp, &pCSI_Config) != HAL_OK)
  {
    Error_Handler();
  }
  pPipeConfig.FrameRate = DCMIPP_FRAME_RATE_ALL;
  pPipeConfig.PixelPipePitch = 10;
  pPipeConfig.PixelPackerFormat = DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1;
  if (HAL_DCMIPP_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE0, &pPipeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_DCMIPP_CSI_SetVCConfig(&hdcmipp, 0U, DCMIPP_CSI_DT_BPP10);

  /** Pipe 1 Config
  */
  if (HAL_DCMIPP_CSI_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE1, &pCSI_PipeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  pPipeConfig.PixelPipePitch = 1280;
  pPipeConfig.PixelPackerFormat = DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1;
  if (HAL_DCMIPP_PIPE_SetConfig(&hdcmipp, DCMIPP_PIPE1, &pPipeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMIPP_Init 2 */

  /* Демозаика RAW Bayer -> RGB (если цвета перепутаны, поменяйте RawBayerType) */
  DCMIPP_RawBayer2RGBConfTypeDef bay = {0};
  bay.RawBayerType  = DCMIPP_RAWBAYER_RGGB;
  bay.VLineStrength = DCMIPP_RAWBAYER_ALGO_NONE;
  bay.HLineStrength = DCMIPP_RAWBAYER_ALGO_NONE;
  bay.PeakStrength  = DCMIPP_RAWBAYER_ALGO_NONE;
  bay.EdgeStrength  = DCMIPP_RAWBAYER_ALGO_NONE;
  if (HAL_DCMIPP_PIPE_SetISPRawBayer2RGBConfig(&hdcmipp, DCMIPP_PIPE1, &bay) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DCMIPP_PIPE_EnableISPRawBayer2RGB(&hdcmipp, DCMIPP_PIPE1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Уменьшение 2592x1944 -> 640x480 */
  if (dcmipp_downsize(&hdcmipp, DCMIPP_PIPE1, CAM_W, CAM_H, OUT_W, OUT_H) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END DCMIPP_Init 2 */

}

void HAL_DCMIPP_MspInit(DCMIPP_HandleTypeDef* dcmippHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(dcmippHandle->Instance==DCMIPP)
  {
  /* USER CODE BEGIN DCMIPP_MspInit 0 */

  /* USER CODE END DCMIPP_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_DCMIPP|RCC_PERIPHCLK_CSI;
    PeriphClkInitStruct.DcmippClockSelection = RCC_DCMIPPCLKSOURCE_IC17;
    PeriphClkInitStruct.ICSelection[RCC_IC17].ClockSelection = RCC_ICCLKSOURCE_PLL4;
    PeriphClkInitStruct.ICSelection[RCC_IC17].ClockDivider = 4;
    PeriphClkInitStruct.ICSelection[RCC_IC18].ClockSelection = RCC_ICCLKSOURCE_PLL4;
    PeriphClkInitStruct.ICSelection[RCC_IC18].ClockDivider = 60;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* DCMIPP clock enable */
    __HAL_RCC_DCMIPP_CLK_ENABLE();
    __HAL_RCC_CSI_CLK_ENABLE();
    __HAL_RCC_CSI_FORCE_RESET();
    __HAL_RCC_CSI_RELEASE_RESET();

    /* DCMIPP interrupt Init */
    HAL_NVIC_SetPriority(DCMIPP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DCMIPP_IRQn);
    HAL_NVIC_SetPriority(CSI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CSI_IRQn);
  /* USER CODE BEGIN DCMIPP_MspInit 1 */

  /* USER CODE END DCMIPP_MspInit 1 */
  }
}

void HAL_DCMIPP_MspDeInit(DCMIPP_HandleTypeDef* dcmippHandle)
{

  if(dcmippHandle->Instance==DCMIPP)
  {
  /* USER CODE BEGIN DCMIPP_MspDeInit 0 */

  /* USER CODE END DCMIPP_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CSI_CLK_DISABLE();
    __HAL_RCC_CSI_FORCE_RESET();
    __HAL_RCC_CSI_RELEASE_RESET();

    /* DCMIPP interrupt Deinit */
    HAL_NVIC_DisableIRQ(DCMIPP_IRQn);
    HAL_NVIC_DisableIRQ(CSI_IRQn);
  /* USER CODE BEGIN DCMIPP_MspDeInit 1 */

  /* USER CODE END DCMIPP_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

