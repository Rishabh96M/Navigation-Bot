/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
extern int32_t EN1_pulse_counter;
uint16_t EN1_CH1=0;
uint16_t EN1_CH2=1;
uint16_t EN1_State=0;
uint16_t EN1_Previous_State = 1;

extern int32_t EN2_pulse_counter;
uint16_t EN2_CH1=0;
uint16_t EN2_CH2=1;
uint16_t EN2_State=0;
uint16_t EN2_Previous_State = 1;


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

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
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/


/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  encoder1_calculation();
  encoder2_calculation();
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void encoder1_calculation(void)
{
	EN1_CH1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
	EN1_CH2 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);

	EN1_State = (EN1_CH1<<1)|EN1_CH2;

	if(EN1_Previous_State != EN1_State)
	{
	  switch((int)EN1_State)
	  	{
	  	case 0:
	  		if((int)EN1_Previous_State == 1 )  EN1_pulse_counter++;
	  		else EN1_pulse_counter--;
	  		break;

	  	case 1:
	  		if((int)EN1_Previous_State == 3) EN1_pulse_counter++;
	  		else EN1_pulse_counter--;
	  		break;

	  	case 2:
	  		if((int)EN1_Previous_State == 0) EN1_pulse_counter++;
	  		else EN1_pulse_counter--;
	  		break;

	  	case 3:
	  		if((int)EN1_Previous_State == 2) EN1_pulse_counter++;
	  		else EN1_pulse_counter--;
	  		break;
	  	}

	  EN1_Previous_State = EN1_State;
	}
}

void encoder2_calculation(void)
{
	EN2_CH1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
	EN2_CH2 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10);

	EN2_State = (EN2_CH1<<1)|EN2_CH2;

	if(EN2_Previous_State != EN2_State)
	 {
	  switch((int)EN2_State)
	  	{
	  	case 0:
	  		if((int)EN2_Previous_State == 1 )  EN2_pulse_counter++;
	  		else EN2_pulse_counter--;
	  		break;

	  	case 1:
	  		if((int)EN2_Previous_State == 3) EN2_pulse_counter++;
	  		else EN2_pulse_counter--;
	  		break;

	  	case 2:
	  		if((int)EN2_Previous_State == 0) EN2_pulse_counter++;
	  		else EN2_pulse_counter--;
	  		break;

	  	case 3:
	  		if((int)EN2_Previous_State == 2) EN2_pulse_counter++;
	  		else EN2_pulse_counter--;
	  		break;
	  	}

	  EN2_Previous_State = EN2_State;
	 }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
