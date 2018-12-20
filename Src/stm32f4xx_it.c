/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */

#include "freertostask.h"
#include "bsp_can.h"

void LL_CAN_Receive(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim7;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
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
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
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
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
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
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
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
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
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
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles CAN1 RX0 interrupts.
*/
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	LL_CAN_Receive(&hcan1);
#if 0
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
#endif
  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if(((USART1->SR)&(1<<4))!=0)
	{
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
		(void)USART1->SR;				//清除中断
		(void)USART1->DR;				//清除数据
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
		LL_DMA_ClearFlag_TC5(DMA2);
	}
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles CAN2 RX0 interrupts.
*/
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */
	LL_CAN_Receive(&hcan2);
#if 0
  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */
#endif
  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream5 global interrupt.
*/
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */
	if (LL_DMA_IsActiveFlag_TC5(DMA2))
	{
		DMAUsart1DataFinishedHandle();
  /* USER CODE END DMA2_Stream5_IRQn 0 */
  
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

	}
	LL_DMA_ClearFlag_HT5(DMA2);
	LL_DMA_ClearFlag_TC5(DMA2);
  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/* USER CODE BEGIN 1 */
#define CAN_RFxR_RFOMx_RELEASE (0x0020U)
#define CAN_RFxR_FULL_FLAG (0x0008U)
#define CAN_RFxR_OVERRUN_FLAG (0x0010U)
#define CAN_RFxR_FMPx_MASK (0x0003U)
#define CAN_IER_FMP0 (0x0002U)


void LL_CAN_Receive(CAN_HandleTypeDef* hcan)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	tmp1 = ((hcan->Instance->RF0R & CAN_RFxR_FULL_FLAG) == CAN_RFxR_FULL_FLAG);
	tmp2 = ((hcan->Instance->RF0R & CAN_RFxR_OVERRUN_FLAG) == CAN_RFxR_OVERRUN_FLAG);
  if(tmp1 || tmp2)
		hcan->Instance->RF0R = CAN_RFxR_FULL_FLAG | CAN_RFxR_OVERRUN_FLAG;
	
	tmp1 = ((hcan->Instance->RF0R & CAN_RFxR_FMPx_MASK) != 0U);
  tmp2 = ((hcan->Instance->IER & CAN_IER_FMP0) == CAN_IER_FMP0);
  
  if(tmp1 && tmp2)
  {
		CanRxMsgTypeDef* RxMessage;
		
		RxMessage = hcan->pRxMsg;
		
		RxMessage->IDE = (uint8_t)0x04 & hcan->Instance->sFIFOMailBox[CAN_FIFO0].RIR;
		if (RxMessage->IDE == CAN_ID_STD)
		{
			RxMessage->StdId = 0x000007FFU & (hcan->Instance->sFIFOMailBox[CAN_FIFO0].RIR >> 21U);
		}
		else
		{
			RxMessage->ExtId = 0x1FFFFFFFU & (hcan->Instance->sFIFOMailBox[CAN_FIFO0].RIR >> 3U);
		}
		RxMessage->RTR = (uint8_t)0x02 & hcan->Instance->sFIFOMailBox[CAN_FIFO0].RIR;
		RxMessage->DLC = (uint8_t)0x0F & hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDTR;
		RxMessage->FIFONumber = CAN_FIFO0;
		RxMessage->FMI = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDTR >> 8U);
		
		RxMessage->Data[0] = (uint8_t)0xFF & hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDLR;
		RxMessage->Data[1] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDLR >> 8U);
		RxMessage->Data[2] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDLR >> 16U);
		RxMessage->Data[3] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDLR >> 24U);
		RxMessage->Data[4] = (uint8_t)0xFF & hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDHR;
		RxMessage->Data[5] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDHR >> 8U);
		RxMessage->Data[6] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDHR >> 16U);
		RxMessage->Data[7] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_FIFO0].RDHR >> 24U);
		
		hcan->Instance->RF0R = CAN_RFxR_RFOMx_RELEASE;
		
		/*目前仍使用HAL库，保留CAN State*/
		switch(hcan->State)
		{
			case(HAL_CAN_STATE_BUSY_TX_RX0):
				hcan->State = HAL_CAN_STATE_BUSY_TX;
				break;
			case(HAL_CAN_STATE_BUSY_RX0_RX1):
				hcan->State = HAL_CAN_STATE_BUSY_RX1;
				break;
			case(HAL_CAN_STATE_BUSY_TX_RX0_RX1):
				hcan->State = HAL_CAN_STATE_BUSY_TX_RX1;
				break;
			default: /* HAL_CAN_STATE_BUSY_RX0 */
				hcan->State = HAL_CAN_STATE_READY;
				break;
		}
	}
	HAL_CAN_RxCpltCallback(hcan);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
