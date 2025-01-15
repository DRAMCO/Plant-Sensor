/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "iwdg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */
  goto ownIWDG;
  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  hiwdg.Init.EWI = 4000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
  ownIWDG:
  hiwdg.Instance = IWDG;
#if PLANTSENSOR
  // IWDG kernel clock is 250Hz (LSI/128)
  // WDG_PRESCALER_128 = (1/(250/128)) x 4096 = (128/250) x 4096 = 2097.152s = 34min57s
  // WDG_PRESCALER_64  =  (1/(250/64)) x 4096 =  (64/250) x 4096 = 1048.576s = 17min28s
  // WDG_PRESCALER_32  =  (1/(250/32)) x 4096 =  (32/250) x 4096 =  524.288s =  8min44s
  // WDG_PRESCALER_16  =  (1/(250/16)) x 4096 =  (16/250) x 4096 =  262.144s =  4min22s
  // WDG_PRESCALER_8   =   (1/(250/8)) x 4096 =   (8/250) x 4096 =  131.072s =  2min11s
  // WDG_PRESCALER_4   =   (1/(250/4)) x 4096 =   (4/250) x 4096 =   65.536s =  1min5s

  //   87035 [app_supercap] Super capacitor loaded (load time = 1316ms),  SCAP = 1845mV, BATT = 3300mV, BATTIN = 3298mV, Core = 898mV, Core temp = 30°C.
  // 1002770 [app_supercap] No request received in last 60s to load SCAP, SCAP = 1598mV, BATT = 3300mV, BATTIN = 3300mV, Core = 898mV, Core temp = 24°C.
  //  --> The supercap voltage is decreased to 1598mV in 1002770 – 87035 = 915735ms = 15m16s

  // This means that the Early Warning Interrupt should be minimum at 15m16s. --> Take 17m
  // The plant sensor is measuring roughly every 5 minutes, so if the refresh is not done at 8m (some extra margin taken), something is wrong.
  // --> The Reload value should then be at 25m and the EWI at 17m:
  // --> Reload @ 25m --> WDG_PRESCALER_128 is needed --> Reload = 25 x 60 x 250 / 128 = 2929.6875 --> Reload = 2930 = 1500.16s --> 25m0s
  // -->    EWI @ 17m                                 -->    EWI = 17 x 60 x 250 / 128 = 1992.1875 -->    EWI = 1993 = 1020.43s --> 17m0s

  // during testing directly powered (no battery), it looks that the Supercap is downgrading much slower

  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
#else
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
#endif
  hiwdg.Init.Window = 2930; // max value = 4095, if no window, take same value as Reload
  hiwdg.Init.Reload = 2930; // max value = 4095
  hiwdg.Init.EWI = 1993;
//  hiwdg.Init.Window = 3930; // max value = 4095, if no window, take same value as Reload
//  hiwdg.Init.Reload = 3930; // max value = 4095
//  hiwdg.Init.EWI = 2993;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
