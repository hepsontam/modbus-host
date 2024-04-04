#define MB_HOST_GLOBAL 1
/**
 ******************************************************************************
 * @file    mb_host.c
 * @author  Derrick Wang
 * @brief   modebus主机实现代码
 ******************************************************************************
 * @note
 * 该文件无需修改
 ******************************************************************************
 */

#include "mb_include.h"

static U08 Flag_SetSend[COM_UART_MAX] = {0};
// modbus初始化
#if (!CubeMX_Setting)
void mbh_init(uint32_t baud, uint8_t parity)
{
	mb_port_uartInit(baud, parity);
	mb_port_timerInit(baud);
}
#endif

void mbh_ValInit(void)
{
    uint8_t i = 0;
    for (i = 0; i < COM_UART_MAX; i++)
    {
        Flag_SetSend[i] = 0;
    }
    mbTrans[COM1].com_index = COM1;  //按需指定串口序号
    mbTrans[COM2].com_index = COM2;
}
uint8_t mbh_getState(uint8_t com_index)
{
	return mbTrans[com_index].state;
}

uint16_t mbh_getDataAddr(uint8_t com_index)
{
    uint16_t dataAddr = MAKEWORD(mbTrans[com_index].txBuf[2], mbTrans[com_index].txBuf[3]);
    return dataAddr;
}
// 发送一帧命令
int8_t mbh_send(uint8_t com_index, uint8_t add, uint8_t cmd, uint8_t *data, uint8_t data_len)
{
	uint16_t crc;
    Flag_SetSend[com_index] = false;
    if (mbTrans[com_index].state != MBH_STATE_IDLE)
        return -1; //busy state

    mbTrans[com_index].txCounter = 0;
    mbTrans[com_index].rxCounter = 0;
    mbTrans[com_index].txBuf[0] = add;
    mbTrans[com_index].txBuf[1] = cmd;
    memcpy(&(mbTrans[com_index].txBuf[2]), data, data_len);
    mbTrans[com_index].txLen = data_len + 2; //data(n)+add(1)+cmd(1)
    crc = mb_crc16(mbTrans[com_index].txBuf, mbTrans[com_index].txLen);
    mbTrans[com_index].txBuf[mbTrans[com_index].txLen++] = (uint8_t) (crc & 0xff);
    mbTrans[com_index].txBuf[mbTrans[com_index].txLen++] = (uint8_t) (crc >> 8);

#if 1
    Flag_SetSend[mbTrans[com_index].com_index] = true;  //使用标志位做标记；尽可能在状态机中切换“状态”
#else
    mbTrans[com_index].state=MBH_STATE_TX;
#endif
    mb_port_uartEnable(com_index, 1, 0);  //enable tx,disable rx
	/*当打开TXE中断以后，立马就会触发一次，所以这里不用先发送一个byte*/
	// mb_port_putchar(mbHost->txBuf[mbHost->txCounter++]); //send first char,then enter tx isr
	return 0;
}

// 接收正确,进行解析处理
static void mbh_exec(uint8_t com_index)
{
    uint8_t *pframe = mbTrans[com_index].rxBuf;
	// uint8_t len = mbHost->rxCounter;
    uint8_t datalen = mbTrans[com_index].rxCounter - 2;
	switch (pframe[1]) // cmd
	{
	case 1:
		mbh_hook_rec01(pframe[0], (pframe + 2), datalen);
		break;
	case 2:
		mbh_hook_rec02(pframe[0], (pframe + 2), datalen);
		break;
	case 3:
		mbh_hook_rec03(pframe[0], (pframe + 2), datalen);
		break;
	case 4:
		mbh_hook_rec04(pframe[0], (pframe + 2), datalen);
		break;
	case 5:
		mbh_hook_rec05(pframe[0], (pframe + 2), datalen);
		break;
	case 6:
		mbh_hook_rec06(pframe[0], (pframe + 2), datalen);
		break;
	case 15:
		mbh_hook_rec15(pframe[0], (pframe + 2), datalen);
		break;
	case 16:
		mbh_hook_rec16(pframe[0], (pframe + 2), datalen);
		break;
	default:
		break;
	}
}

extern U16 COMMU_CNT[5];  //利用外部计时累加
void mbh_poll(uint8_t com_index)
{
    mbTransInfo *mbHost = &mbTrans[com_index];

	switch (mbHost->state)   // 一个状态机集中调度
	{
	case MBH_STATE_IDLE:
        if (Flag_SetSend[com_index])
        {
            Flag_SetSend[com_index] = false;
            mbHost->state = MBH_STATE_TX_END;
        }
		break;

	case MBH_STATE_TX:
        //if (mbHost->txCounter >= mbHost->txLen) // 全部发送完，转入接收
		{
			mbHost->state = MBH_STATE_TX_END;
		}

		//break;

	case MBH_STATE_TX_END:
        if (COMMU_CNT[com_index] >= COMMU_WAIT_MS) // 手动设置间隔5ms，通过则表示发送完毕
		{
            COMMU_CNT[com_index] = 0;
			mbHost->rxTimeOut = 0; // 清除接收超时计数
			mbHost->state = MBH_STATE_RX;
		}
		break;

	case MBH_STATE_RX:
		if (mbHost->rxCounter > 0) // 有收到信息
		{
            if (COMMU_CNT[com_index] >= COMMU_WAIT_MS) // 3.5T到,接收一帧完成
			{
				mbHost->state = MBH_STATE_RX_CHECK;
				mb_port_uartEnable(com_index, 0, 0); // 串口tx、rx都关闭
			}
		}
		else
		{
            if (COMMU_CNT[com_index] >= COMMU_WAIT_MS) // 手动设置间隔5ms
			{
                COMMU_CNT[com_index] = 0;
				mbHost->rxTimeOut++;
				if (mbHost->rxTimeOut >= MBH_REC_TIMEOUT) // 接收超时
				{
					mbHost->rxTimeOut = 0;
					mbHost->state = MBH_STATE_REC_ERR;
					mb_port_uartEnable(com_index, 0, 0); // 串口tx、rx都关闭
				}
			}
		}
		break;

	/*接收完一帧数据,开始进行校验*/
	case MBH_STATE_RX_CHECK:																			  // 接收完成，对一帧数据进行检查
        if ((mbHost->rxCounter >= MBH_RTU_MIN_SIZE) &&
            (mb_crc16(mbHost->rxBuf, mbHost->rxCounter) == 0)) // 接收的一帧数据正确
        {
            if ((mbHost->txBuf[0] == mbHost->rxBuf[0]) &&
                (mbHost->txBuf[1] == mbHost->rxBuf[1])) // 发送帧数据和接收到的帧数据地址和功能码一样
            {
                mbHost->state = MBH_STATE_EXEC;
                mbHost->errTimes = 0;
            }
            else
            {
                mbHost->state = MBH_STATE_REC_ERR;
            }
        }
        else
        {
            mbHost->state = MBH_STATE_REC_ERR;
        }
        break;

    /*接收一帧数据出错*/
    case MBH_STATE_REC_ERR:
        mbHost->errTimes++;
        if (mbHost->errTimes >= MBH_ERR_MAX_TIMES)
        {
            mbHost->state = MBH_STATE_TIMES_ERR;
        }
        else // 重新再启动一次传输
        {
            mbHost->txCounter = 0;
            mbHost->rxCounter = 0;
            mbHost->state = MBH_STATE_TX;
        }
        break;

    /*超过最大错误传输次数*/
    case MBH_STATE_TIMES_ERR:
        mbh_hook_timesErr(&(mbHost->state), mbHost->txBuf[0], mbHost->txBuf[1]);
        mbHost->txCounter = 0;
        mbHost->rxCounter = 0;
        break;

    /*确定接收正确执行回调*/
    case MBH_STATE_EXEC: // 主机发送接收完成，执行回调
        mbh_exec(com_index);
        mbHost->state = MBH_STATE_IDLE;
        break;
	}
}

#if 0
void mbh_timer3T5Isr()
{
    switch(mbHost.state)
    {
 		/*发送完但没有接收到数据*/
        case MBH_STATE_TX_END:
            mbHost.rxTimeOut++;
            if(mbHost.rxTimeOut>=MBH_REC_TIMEOUT) //接收超时
            {
                mbHost.rxTimeOut=0;
                mbHost.state=MBH_STATE_REC_ERR;
                mb_port_timerDisable();		//关闭定时器
                mb_port_uartEnable(0,0); 	//串口tx、rx都关闭
            }
            break;
        case MBH_STATE_RX:     	//3.5T到,接收一帧完成
            mbHost.state=MBH_STATE_RX_CHECK;
            mb_port_timerDisable();		//关闭定时器
            mb_port_uartEnable(0,0); 	//串口tx、rx都关闭
            break;
    }

}
#endif


/**
 * @brief  This function &#&
 * @param  None
 * @note:  串口接收中断调用
 */
void mbh_uartRxIsr(uint8_t com_index)
{
    uint8_t ch;
    mb_port_getchar(com_index, &ch);

    if (mbTrans[com_index].rxCounter < MBH_RTU_MAX_SIZE)
    {
        mbTrans[com_index].rxBuf[mbTrans[com_index].rxCounter++] = ch;
    }
    COMMU_CNT[com_index] = 0;
}


/**
 * @brief  This function &#&
 * @param  None
 * @note:  串口发送中断
 */
void mbh_uartTxIsr(uint8_t com_index)
{
	if (mbTrans[com_index].txCounter >= mbTrans[com_index].txLen) // 全部发送完，转入接收
	{
		mbTrans[com_index].rxCounter = 0;
		mb_port_uartEnable(com_index, 0, 1); // disable tx,enable rx
	}
	else
	{
		mb_port_putchar(com_index, mbTrans[com_index].txBuf[mbTrans[com_index].txCounter++]);
	}

}
