#ifndef __MB_INCLUDE_H
#define __MB_INCLUDE_H

//========== Maybe Useful===========//
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "main.h"

#define COM_UART_MAX    2  // UART account in use
#define COM1    0
#define COM2    1
#define COM3    2
#define COM4    3
//==================================//

#define CubeMX_Setting 1

#ifndef USE_HAL_DRIVER
#define USE_HAL_DRIVER
#endif

#include "mb_crc.h"
#include "mb_host.h"
#include "mb_port.h"
#include "mb_hook.h"

#endif

