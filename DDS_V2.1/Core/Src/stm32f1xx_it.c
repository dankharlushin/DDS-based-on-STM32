/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint16_t phaseWord;
extern uint16_t * freqWords;
uint8_t clkImpulseCounter = 0;
uint8_t wordCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
GPIO_PinState Set_PinState(uint8_t bit) {

	if(bit == 1)
		return GPIO_PIN_SET;
	else
		return GPIO_PIN_RESET;

}

void Set_Data_DDS1(uint16_t word) {
	for (uint8_t i = 1, pin = 0; i < 256; i = i << 1, pin++) {
		uint8_t bit = word & i;
		if (bit != 0) {
			bit = 1;
		}
		else {
			bit = 0;
		}

		GPIO_PinState state = Set_PinState(bit);

		if (pin == 0) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, state); //D0
		}
		else if (pin == 1) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, state); //D1
		}
		else if (pin == 2) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, state); //D2
		}
		else if (pin == 3) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, state); //D3
		}
		else if (pin == 4) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, state); //D4
		}
		else if (pin == 5) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, state); //D5
		}
		else if (pin == 6) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, state); //D6
		}
		else if (pin == 7) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, state);//D7
			break;
		}

	}


}
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if (clkImpulseCounter < 15) {
		  switch(clkImpulseCounter % 3) {

		  case 0:
			  Set_Data_DDS1((uint16_t) freqWords[wordCounter]);
			  wordCounter++;
			  break;
		  case 1:
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
			  break;
		  case 2:
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
			  break;

		  }

		  clkImpulseCounter++;

	  }
	  else if (clkImpulseCounter == 15) {
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
		  clkImpulseCounter++;
	  }
	  else if (clkImpulseCounter < 19) {
		  clkImpulseCounter++;
	  }

	  else if (clkImpulseCounter == 19) {
	  		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
	  		  clkImpulseCounter++;
	  	  }
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */


  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/*GPIO_PinState Set_PinState(uint8_t bit) {

	if(bit == 1)
		return GPIO_PIN_SET;
	else
		return GPIO_PIN_RESET;

}

void Set_Data_DDS1(uint16_t word) {
	for (uint8_t i = 1, pin = 0; i <= 200; i <<= 1, pin++) {
		uint8_t bit = word & i;
		if (bit != 0) {
			bit = 1;
		}
		else {
			bit = 0;
		}

		if (pin == 0) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, Set_PinState(bit)); //D0
		}
		else if (pin == 1) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, Set_PinState(bit)); //D1
		}
		else if (pin == 2) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, Set_PinState(bit)); //D2
		}
		else if (pin == 3) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, Set_PinState(bit)); //D3
		}
		else if (pin == 4) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, Set_PinState(bit)); //D4
		}
		else if (pin == 5) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, Set_PinState(bit)); //D5
		}
		else if (pin == 6) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, Set_PinState(bit)); //D6
		}
		else if (pin == 7) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, Set_PinState(bit)) ;//D7
		}

	}


}*/
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
