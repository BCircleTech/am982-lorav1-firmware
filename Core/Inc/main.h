/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC1_N_Pin GPIO_PIN_14
#define OSC1_N_GPIO_Port GPIOC
#define OSC1_P_Pin GPIO_PIN_15
#define OSC1_P_GPIO_Port GPIOC
#define IMU_SDA_Pin GPIO_PIN_0
#define IMU_SDA_GPIO_Port GPIOF
#define IMU_SCL_Pin GPIO_PIN_1
#define IMU_SCL_GPIO_Port GPIOF
#define OSC0_N_Pin GPIO_PIN_0
#define OSC0_N_GPIO_Port GPIOH
#define OSC0_P_Pin GPIO_PIN_1
#define OSC0_P_GPIO_Port GPIOH
#define RTK_TX0_Pin GPIO_PIN_14
#define RTK_TX0_GPIO_Port GPIOD
#define RTK_RX0_Pin GPIO_PIN_15
#define RTK_RX0_GPIO_Port GPIOD
#define LORA_AUX_Pin GPIO_PIN_8
#define LORA_AUX_GPIO_Port GPIOC
#define LORA_M1_Pin GPIO_PIN_9
#define LORA_M1_GPIO_Port GPIOC
#define LORA_M0_Pin GPIO_PIN_8
#define LORA_M0_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LED_RUN_Pin GPIO_PIN_11
#define LED_RUN_GPIO_Port GPIOC
#define RS422_DI_Pin GPIO_PIN_12
#define RS422_DI_GPIO_Port GPIOC
#define MCU_RX_Pin GPIO_PIN_0
#define MCU_RX_GPIO_Port GPIOD
#define MCU_TX_Pin GPIO_PIN_1
#define MCU_TX_GPIO_Port GPIOD
#define RS422_RO_Pin GPIO_PIN_2
#define RS422_RO_GPIO_Port GPIOD
#define LED_ERR_Pin GPIO_PIN_3
#define LED_ERR_GPIO_Port GPIOD
#define RTK_NRST_Pin GPIO_PIN_4
#define RTK_NRST_GPIO_Port GPIOD
#define RTK_RX2_Pin GPIO_PIN_5
#define RTK_RX2_GPIO_Port GPIOD
#define RTK_TX2_Pin GPIO_PIN_6
#define RTK_TX2_GPIO_Port GPIOD
#define RTK_PPS_Pin GPIO_PIN_7
#define RTK_PPS_GPIO_Port GPIOD
#define RS232_RO0_Pin GPIO_PIN_9
#define RS232_RO0_GPIO_Port GPIOG
#define IMU_RST_Pin GPIO_PIN_10
#define IMU_RST_GPIO_Port GPIOG
#define LORA_TX_Pin GPIO_PIN_11
#define LORA_TX_GPIO_Port GPIOG
#define LORA_RX_Pin GPIO_PIN_12
#define LORA_RX_GPIO_Port GPIOG
#define IMU_CLK_Pin GPIO_PIN_13
#define IMU_CLK_GPIO_Port GPIOG
#define RS232_TI0_Pin GPIO_PIN_14
#define RS232_TI0_GPIO_Port GPIOG
#define RS485_DE_Pin GPIO_PIN_15
#define RS485_DE_GPIO_Port GPIOG
#define RS485_RO_Pin GPIO_PIN_3
#define RS485_RO_GPIO_Port GPIOB
#define RS485_DI_Pin GPIO_PIN_4
#define RS485_DI_GPIO_Port GPIOB
#define CAN_RO_Pin GPIO_PIN_5
#define CAN_RO_GPIO_Port GPIOB
#define CAN_DI_Pin GPIO_PIN_6
#define CAN_DI_GPIO_Port GPIOB
#define PVC_V3_3_Pin GPIO_PIN_7
#define PVC_V3_3_GPIO_Port GPIOB
#define IMU_FSYNC_Pin GPIO_PIN_0
#define IMU_FSYNC_GPIO_Port GPIOE
#define IMU_INT_Pin GPIO_PIN_1
#define IMU_INT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
