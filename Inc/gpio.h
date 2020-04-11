/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
	 #define GPIOB2 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2)
	 	 #define GPIOB0 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0)
	 	 #define GPIOF5 HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_5)
	 	 #define GPIOB9_RESET 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET)
	 	 #define GPIOD12_RESET 	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET)
	  #define GPIOB9_SET 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET)
	  #define GPIOD12_SET 	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET)
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
