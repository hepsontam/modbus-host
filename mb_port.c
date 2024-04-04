/**
 ******************************************************************************
 * @file    mb_port.c
 * @author  Derrick Wang
 * @brief   modebus移植接口
 ******************************************************************************
 * @note
 * 该文件为modbus移植接口的实现，根据不同的MCU平台进行移植
 ******************************************************************************
 */

#include "mb_include.h"


#if (!CubeMX_Setting)
void mb_port_uartInit(uint32_t baud, uint8_t parity)
{
	/*串口部分初始化*/
	huart1.Instance = USART1;
	huart1.Init.BaudRate = baud;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	if (parity == MB_PARITY_ODD)
	{
		huart1.Init.StopBits = UART_STOPBITS_1;
		huart1.Init.Parity = UART_PARITY_ODD;
	}
	else if (parity == MB_PARITY_EVEN)
	{
		huart1.Init.StopBits = UART_STOPBITS_1;
		huart1.Init.Parity = UART_PARITY_EVEN;
	}
	else
	{
		huart1.Init.StopBits = UART_STOPBITS_2;
		huart1.Init.Parity = UART_PARITY_NONE;
	}
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
}

void mb_port_timerInit(uint32_t baud)
{
	/*定时器部分初始化*/
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 71;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	if (baud > 19200) // 波特率大于19200固定使用1800作为3.5T
	{
		htim3.Init.Period = 1800;
	}
	else // 其他波特率的需要根据计算
	{
		/*	us=1s/(baud/11)*1000000*3.5
		 *			=(11*1000000*3.5)/baud
		 *			=38500000/baud
		 */
		htim3.Init.Period = (uint32_t)38500000 / baud;
	}
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(htim) != HAL_OK)
	{
		Error_Handler();
	}
}
#endif

// Example
UART_HandleTypeDef *pUART[COM_UART_MAX];
void USART_ValInit(void)
{
    pUART[COM1] = &huart1;  //按需指定串口序号
    pUART[COM2] = &huart2;
}

UART_HandleTypeDef * mb_port_mbTrans_to_uart(uint8_t com_index)
{
    return (pUART[com_index]);
}


void mb_port_uartEnable(uint8_t com_index, uint8_t txen, uint8_t rxen)
{
	UART_HandleTypeDef *huart = mb_port_mbTrans_to_uart(com_index);
	if (huart)
	{
		#ifdef USE_HAL_DRIVER
		if (txen)
		{
			__HAL_UART_ENABLE_IT(pUART[com_index], UART_IT_TXE);
		}
		else
		{
			__HAL_UART_DISABLE_IT(pUART[com_index], UART_IT_TXE);
		}

		if (rxen)
		{
			__HAL_UART_ENABLE_IT(pUART[com_index], UART_IT_RXNE);
		}
		else
		{
			__HAL_UART_DISABLE_IT(pUART[com_index], UART_IT_RXNE);
		}
		#else
		#endif
	}
}
void mb_port_putchar(uint8_t com_index, uint8_t ch)
{
	// Instance->DR = ch; // 直接操作寄存器比HAL封装的更高效
	(pUART[com_index]->Instance)->DR = ch;
}

void mb_port_getchar(uint8_t com_index, uint8_t *ch)
{
	// *ch = (uint8_t)(Instance->DR & (uint8_t)0x00FF);
	*ch= (uint8_t)((pUART[com_index]->Instance)->DR & (uint8_t)0x00FF);
}
