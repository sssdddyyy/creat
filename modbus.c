#include "Modbus.h"
#include "string.h"
#include "spi_flash.h"
#include "spi.h"

#define MODBUS_UART uart2			//将串口改成相应的变量即可，例如用到UART1，此处改为uart1
#define MODBUS_HUART huart2			//将串口改成相应的变量即可，例如用到UART1，此处改为huart1
#define  wRegTempAdd 0x42
#define  wRegAdd 0x46

uint8_t  CoilState[(MAX_DIS_NUM + 7) / 8] = {0x00};
uint8_t  DisState [(MAX_DIS_NUM + 7) / 8] = {0xAA};
uint16_t InputReg[MAX_INPUT_REG_NUM] = {0xAA55,0x55AA};
uint16_t HoldReg[MAX_HOLD_REG_NUM] = {0xAA55, 0x55AA};


Var_Reg  SamVarReg;


void MODBUS_ERRFunction(uint8_t uCmdCode, uint8_t uErrorCode);

/*******************************************************************************
函数名称 ： ModbusCRC16
功    能 ： CRC校验
参    数 ： ptr--校验数组指针  len--校验数据长度
返 回 值 ： CRC校验码，双字节
*******************************************************************************/
uint16_t ModbusCRC16(uint8_t *ptr, uint16_t len)
{
    uint8_t i;
    uint16_t crc = ~0x00;

    if((ptr == NULL) || (len == 0xFFFF)) return crc;

    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc & 0x01)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return(crc);
}

void GetOneCoilVal(uint16_t wCoilAddr, uint8_t *pCoilVal)
{
	uint8_t uVal;
	
	uVal = CoilState[(wCoilAddr - COIL_ADD_MIN) / 8 ];
	if(uVal & ( 1 << ((wCoilAddr - COIL_ADD_MIN) % 8 )))
	{
		*pCoilVal = 0x01;
	}
	else
	{
		*pCoilVal = 0x00;
	}
}


/*******************************************************************************
函数名称 ： ReadCoilStateFUNC
功    能 ： 功能码:0x01,读取线圈
参    数 ： 无
返 回 值 ： 无
*发送:[硬件地址][01][线圈起始地址高][线圈起始地址低][线圈数量高][线圈数量低][CRC低][CRC高]
*返回:[硬件地址][01][字节长度][线圈值][线圈值][线圈值][CRC低][CRC高]
*******************************************************************************/
void ReadCoilStateFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
	uint16_t wCoilStartAddr,wCoilNum,wTotalCoilNum,CRC16Temp;
    uint8_t  i,k,uCommIndexNum = 0,uByteCount,uCoilVal,uErrorCode,uExit = 0;
	uint8_t  upTxdbuf[50];

    wCoilStartAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //获取线圈起始地址
    wCoilNum        = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //获取线圈个数

	if((wCoilNum >= 0x0001) || (wCoilNum <= MAX_COIL_NUM))
	{
		if(((wCoilStartAddr >= COIL_ADD_MIN) && (wCoilStartAddr <= COIL_ADD_MAX)) &&
			(wCoilNum + wCoilStartAddr <= COIL_ADD_MAX + 1))
		{
			uByteCount = (wCoilNum + 7) / 8;    //返回数据字节个数
	
			upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;
            upTxdbuf[uCommIndexNum ++] = ReadCoilState;
			upTxdbuf[uCommIndexNum ++] = uByteCount;
			
			wTotalCoilNum = 0;
            for(k = 0; k < uByteCount; k++)
			{
				upTxdbuf[uCommIndexNum] = 0;
				for(i = 0; i < 8; i++)
				{
					GetOneCoilVal(wCoilStartAddr + wTotalCoilNum,&uCoilVal);
					upTxdbuf[uCommIndexNum] |= uCoilVal << i;
 
					wTotalCoilNum ++;
					if(wTotalCoilNum >= wCoilNum)
					{
						 uExit = 1;
						 break;
					}
				}
				
				uCommIndexNum ++;
				
				if(uExit == 1)
				{
					break;
				}
			}

            CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);

            MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
            memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
            HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);
			
			return;
		}
		else
		{
			uErrorCode = MB_EX_ILLEGAL_DATA_ADDRESS;
		}
	}
	else 
	{
		 uErrorCode = MB_EX_ILLEGAL_DATA_VALUE;
	}

    MODBUS_ERRFunction(ReadCoilState, uErrorCode);
    return;
}

void GetOneDisInputVal(uint16_t wDisInputAddr, uint8_t *pDisInputVal)
{
	uint8_t uVal;
	
	uVal = DisState[(wDisInputAddr - DIS_ADD_MIN) / 8 ];
	if(uVal & ( 1 << ((wDisInputAddr - DIS_ADD_MIN) % 8 )))
	{
		*pDisInputVal = 0x01;
	}
	else
	{
		*pDisInputVal = 0x00;
	}
}

 

/*******************************************************************************
函数名称 ： ReadDisInputStateFUNC
功    能 ： 功能码:0x02,读取离散量
参    数 ： 无
返 回 值 ： 无
*发送:[硬件地址][02][离散量起始地址高][离散量起始地址低][离散量数量高][离散量数量低][CRC低][CRC高]
*返回:[硬件地址][02][字节长度][离散量值][离散量值][离散量值][CRC低][CRC高]
*******************************************************************************/
void ReadDisInputStateFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t wDisStartAddr,wDisNum,wTotalDisNum,CRC16Temp;
    uint8_t  i,k,uCommIndexNum = 0,uByteCount,uDisVal,uErrorCode,uExit = 0;
	uint8_t  upTxdbuf[50];

    wDisStartAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //获取离散量起始地址
    wDisNum        = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //获取离散量个数

	if((wDisNum >= 0x0001) || (wDisNum <= MAX_DIS_NUM))
	{
		if(((wDisStartAddr >= DIS_ADD_MIN) && (wDisStartAddr <= DIS_ADD_MAX)) &&
			(wDisNum + wDisStartAddr <= DIS_ADD_MAX + 1))
		{
			uByteCount = (wDisNum + 7) / 8;    //返回数据字节个数
	
			upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;
            upTxdbuf[uCommIndexNum ++] = ReadDisInputState;
			upTxdbuf[uCommIndexNum ++] = uByteCount;
			
			wTotalDisNum = 0;
            for(k = 0; k < uByteCount; k++)
			{
				upTxdbuf[uCommIndexNum] = 0;
				for(i = 0; i < 8; i++)
				{
					GetOneDisInputVal(wDisStartAddr + wTotalDisNum,&uDisVal);
					upTxdbuf[uCommIndexNum] |= uDisVal << i;
 
					wTotalDisNum ++;
					if(wTotalDisNum >= wDisNum)
					{
						 uExit = 1;
						 break;
					}
				}
				
				uCommIndexNum ++;
				
				if(uExit == 1)
				{
					break;
				}
			}

            CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);

            MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
            memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
            HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);
			
			return;
		}
		else
		{
			uErrorCode = MB_EX_ILLEGAL_DATA_ADDRESS;
		}
	}
	else 
	{
		 uErrorCode = MB_EX_ILLEGAL_DATA_VALUE;
	}

    MODBUS_ERRFunction(ReadDisInputState, uErrorCode);
    return;	
}

/*******************************************************************************
函数名称 ： GetHoldRegData
功    能 ： 获取RegAdd的数据
参    数 ： u16 RegAdd
返 回 值 ： 两字节寄存器数据
*******************************************************************************/
uint16_t GetHoldRegData(uint16_t RegAdd)
{
    int16_t wRegValue;

    wRegValue = HoldReg[RegAdd - HOLD_REG_ADD_MIN];

    return wRegValue;
}

/*******************************************************************************
函数名称 ： ReadHoldRegFUNC
功    能 ： 功能码:0x03,读取保持寄存器
参    数 ： 无
返 回 值 ： 无
*发送:[硬件地址][03][起始地址高][起始地址低][总寄存器数高][总寄存器数低][CRC低][CRC高]
*返回:[硬件地址][03][字节数][寄存器0高][寄存器0低][寄存器1高][寄存器1低][寄存器n高][寄存器n低][CRC低][CRC高]
*******************************************************************************/
void ReadHoldRegFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
	  u8_u16_u32_typedef  buf;
	  uint32_t wRegStartAdd;
    uint16_t  wRegLen, i,wRegValue, CRC16Temp;
    uint8_t  uErrorCode, uCommIndexNum = 0;
    uint8_t  upTxdbuf[70];
	  uint8_t m_data1[70] = {0};

    if(upRxdbuf == NULL)  return;
    buf.u8[0] = upRxdbuf[3];
		buf.u8[1] = upRxdbuf[2];
		buf.u8[2] = upRxdbuf[1];
		buf.u8[3] = upRxdbuf[0];//获取寄存器起始地址
		wRegStartAdd = buf.u32;
    wRegLen      = MAKEWORD(upRxdbuf[5], upRxdbuf[4]); //获取读取寄存器长度
		wRegStartAdd = (wRegStartAdd/wRegTempAdd)*wRegTempAdd+wRegAdd;
  //判断需要读取数据的起始地址是否超过存储器地址和读取的数据是否超过了存储器的大小
    if((wRegLen >= 0x01) && (wRegLen <= MAX_HOLD_REG_NUM))
    {
        if(((wRegStartAdd >= Wright_Str_ADD_MIN) && (wRegStartAdd <= Wright_Str_ADD_MAX)) &&
                (((wRegStartAdd + wRegLen) >= Wright_Str_ADD_MIN) && ((wRegStartAdd + wRegLen) <= Wright_Str_ADD_MAX + 1)))  
        {
            upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;  //将本地地址存储转发
            upTxdbuf[uCommIndexNum ++] = ReadHoldReg;   //将起始地址存储转发
            upTxdbuf[uCommIndexNum ++] = wRegLen * 2;  //将数据大小存储转发
            SPI_FLASH_BufferRead(&hspi1,(uint8_t *)&m_data1,wRegStartAdd,wRegLen);   //用SPI读取指定地址，指定大小的数据并存入数组
            for(i = 0; i < wRegLen; i++)
            {
                //获取16位数据并返回
                wRegValue = m_data1[i];
                upTxdbuf[uCommIndexNum ++] = (uint8_t)(wRegValue);
 //             upTxdbuf[uCommIndexNum ++] = (uint8_t)(wRegValue & 0xFF);
            }

            CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF); // crc16低字节在前
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);   // crc16高字节在后

            MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
            memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
            HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);

            return;
        }
        else
        {
            uErrorCode = MB_EX_ILLEGAL_DATA_ADDRESS;
        }
    }
    else
    {
        uErrorCode = MB_EX_ILLEGAL_DATA_VALUE;
    }

    MODBUS_ERRFunction(ReadHoldReg, uErrorCode);
    return;
}


/*******************************************************************************
函数名称 ： GetInputRegData
功    能 ： 获取RegAdd的数据
参    数 ： u16 RegAdd
返 回 值 ： 两字节寄存器数据
*******************************************************************************/
uint16_t GetInputRegData(uint16_t RegAdd)
{
    int16_t wRegValue;

    wRegValue = InputReg[RegAdd - INPUT_REG_ADD_MIN];
	
    return wRegValue;
}
/*******************************************************************************
函数名称 ： ReadInputRegFUNC
功    能 ： 功能码:0x04,读取输入寄存器
参    数 ： 无
返 回 值 ： 无
*发送:[硬件地址][04][起始地址高][起始地址低][总寄存器数高][总寄存器数低][    CRC低   ][     CRC高  ]
*返回:[硬件地址][04][  字节数  ][寄存器0高 ][  寄存器0低 ][  寄存器1高 ][  寄存器1低 ][  寄存器n高 ][  寄存器n低 ][  CRC低 ][  CRC高 ]
*******************************************************************************/
void ReadInputRegFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t wRegStartAdd, wRegLen, i, wRegValue, CRC16Temp;
    uint8_t  uErrorCode, uCommIndexNum = 0;
	uint8_t  upTxdbuf[50];
   
    if(upRxdbuf == NULL)  return;

    wRegStartAdd = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //获取寄存器起始地址
    wRegLen      =  MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //获取读取寄存器长度

    if((wRegLen >= 0x01) && (wRegLen <= MAX_INPUT_REG_NUM))
    {
        if(((wRegStartAdd >= INPUT_REG_ADD_MIN) && (wRegStartAdd <= INPUT_REG_ADD_MAX)) &&
           ((wRegStartAdd + wRegLen) <= INPUT_REG_ADD_MAX + 1))
        {
            upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;
            upTxdbuf[uCommIndexNum ++] = ReadInputReg;
            upTxdbuf[uCommIndexNum ++] = wRegLen * 2;

            for(i = 0; i < wRegLen; i++)
            {
                //获取16位数据并返回
                wRegValue = GetInputRegData(wRegStartAdd + i);
                upTxdbuf[uCommIndexNum ++] = (uint8_t)(wRegValue >> 8);
                upTxdbuf[uCommIndexNum ++] = (uint8_t)(wRegValue & 0xFF);
            }

            CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF); // crc16低字节在前
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);   // crc16高字节在后

            MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
            memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
            HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);

            return;
         }
         else
        {
            uErrorCode = MB_EX_ILLEGAL_DATA_ADDRESS;
        }
    }
    else
    {
        uErrorCode = MB_EX_ILLEGAL_DATA_VALUE;
    }

    MODBUS_ERRFunction(ReadInputReg, uErrorCode);
    return;
}



/*******************************************************************************
函数名称 : WriteHoldRegData
功    能 : 保存写入的保持寄存器数据
参    数 : u16 StartAdd
返 回 值 : 无
*******************************************************************************/
void WriteHoldRegData(uint16_t wRegAddr, uint16_t RegData)
{
    HoldReg[wRegAddr - HOLD_REG_ADD_MIN] = RegData;
}

/*******************************************************************************
函数名称 ： WriteSingleRegFUNC
功    能 ： 功能码:0x06 预设(写)单寄存器
参    数 ： 无
返 回 值 ： 无
*发送:[硬件地址][06][寄存器地址高][寄存器地址低][寄存器值高][寄存器值低][CRC低][CRC高]
*返回:[硬件地址][06][寄存器地址高][寄存器地址低][寄存器值高][寄存器值低][CRC低][CRC高]
*******************************************************************************/
void WriteSingleRegFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t wRegAddr, wRegValue, CRC16Temp;
    uint8_t  uCommIndexNum = 0, uErrorCode;
    uint8_t  upTxdbuf[50];

    if(upRxdbuf == NULL)  return;

    wRegAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //获取寄存器地址
    wRegValue = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //获取数据

    if((wRegAddr >= HOLD_REG_ADD_MIN) && (wRegAddr <= HOLD_REG_ADD_MAX))
    {
        WriteHoldRegData(wRegAddr, wRegValue);

        upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;
        upTxdbuf[uCommIndexNum ++] = WriteSingleReg;
        memcpy(upTxdbuf + uCommIndexNum, upRxdbuf, 4);
        uCommIndexNum += 4;

        CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
        upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF); // crc16低字节在前
        upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);   // crc16高字节在后

        MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
        memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
        HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);
        return;
    }
    else
    {
        uErrorCode = MB_EX_ILLEGAL_DATA_ADDRESS;
    }

    MODBUS_ERRFunction(WriteSingleReg, uErrorCode);
    return;
}

/*******************************************************************************
函数名称 ： WriteMultiRegFUNC
功    能 ： 功能码:0x10 写多个保持寄存器
参    数 ： 无
返 回 值 ： 无
*******************************************************************************/
void WriteMultiRegFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t i, wRegStartAdd, wRegNum, CRC16Temp, uErrorCode, wRegValue;
    uint8_t  uCommIndexNum = 0, uByteNum;
    uint8_t  upTxdbuf[50];

    if(upRxdbuf == NULL)  return;

    wRegStartAdd = MAKEWORD(upRxdbuf[1], upRxdbuf[0]);     //获取寄存器地址
    wRegNum      = MAKEWORD(upRxdbuf[3], upRxdbuf[2]);     //获取寄存器数量
    uByteNum     = upRxdbuf[4];                            //获取字节数

    if((wRegNum >= 0x01) && (wRegNum <= MAX_HOLD_REG_NUM) && (uByteNum == wRegNum * 2))
    {
        if(((wRegStartAdd >= HOLD_REG_ADD_MIN) && (wRegStartAdd <= HOLD_REG_ADD_MAX) && 
			(wRegStartAdd + wRegNum <= HOLD_REG_ADD_MAX + 1)))
        {
            for(i = 0; i < wRegNum; i++)
            {
                wRegValue = MAKEWORD(upRxdbuf[6 + i * 2], upRxdbuf[5 + i * 2]);
                WriteHoldRegData(wRegStartAdd + i, wRegValue);
            }

            upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;
            upTxdbuf[uCommIndexNum ++] = WriteMultiReg;
            memcpy(upTxdbuf + uCommIndexNum, upRxdbuf, 4);
            uCommIndexNum += 4;

            CRC16Temp                 = ModbusCRC16(upTxdbuf, uCommIndexNum);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);

            MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
            memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
            HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);

            return;
        }
        else
        {
            uErrorCode = MB_EX_ILLEGAL_DATA_ADDRESS;
        }
    }
    else
    {
        uErrorCode = MB_EX_ILLEGAL_DATA_VALUE;
    }

    MODBUS_ERRFunction(WriteMultiReg, uErrorCode);
    return;
}


/*******************************************************************************
函数名称 : WriteOneCoilData
功    能 : 保存写入的线圈寄存器数据
参    数 : u16 StartAdd
返 回 值 : 无
*******************************************************************************/
void WriteOneCoilData(uint16_t wRegAddr, uint16_t RegData)
{
	if(RegData == 0xFF00)
    {
		CoilState[(wRegAddr - COIL_ADD_MIN) / 8] |= 1 << ((wRegAddr - COIL_ADD_MIN) % 8);
	}
	else if(RegData == 0x0000)
	{
		CoilState[(wRegAddr - COIL_ADD_MIN) / 8] &= ~(1 << ((wRegAddr - COIL_ADD_MIN) % 8));
	}
}
/*******************************************************************************
函数名称 ： WriteSingleCoilFUNC
功    能 ： 功能码:0x05 写单个线圈寄存器
参    数 ： 无
返 回 值 ： 无
*******************************************************************************/
void WriteSingleCoilFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t wCoilAddr, wCoilValue, CRC16Temp;
    uint8_t  uCommIndexNum = 0, uErrorCode;
    uint8_t  upTxdbuf[50];

    wCoilAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //获取线圈地址
    wCoilValue = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //获取线圈数据

    if((wCoilValue == 0x0000) || (wCoilValue == 0xFF00))
    {
        if((wCoilAddr >= DIS_ADD_MIN ) && (wCoilAddr <= DIS_ADD_MAX))
        {
            WriteOneCoilData(wCoilAddr, wCoilValue);

            upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;
            upTxdbuf[uCommIndexNum ++] = WriteSingleCoil;
            memcpy(upTxdbuf + uCommIndexNum, upRxdbuf, 4);
            uCommIndexNum += 4;

            CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);

            MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
            memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
            HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);
			
			return;
        }
        else
        {
            uErrorCode = MB_EX_ILLEGAL_DATA_ADDRESS;
        }
    }
    else
    {
        uErrorCode = MB_EX_ILLEGAL_DATA_VALUE;
    }

    MODBUS_ERRFunction(WriteSingleCoil, uErrorCode);
    return;
}

/*******************************************************************************
函数名称 ： WriteMultiCoilFUNC
功    能 ： 功能码:0x0F,写多个线圈
参    数 ： 无
返 回 值 ： 无
*发送:[硬件地址][0F][起始地址高][起始地址低][线圈数量高][线圈数量低][字节数][线圈值][CRC低][CRC高]
*返回:[硬件地址][0F][起始地址高][起始地址低][线圈数量高][线圈数量低][CRC低][CRC高]
*******************************************************************************/
void WriteMultiCoilFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
	uint16_t wCoilStartAddr,wCoilNum,wCoilVal,wTotalCoilNum,CRC16Temp;
    uint8_t  i,k,uCommIndexNum = 0,uByteNum,uByteVal,uExit = 0,uErrorCode;
	uint8_t  upTxdbuf[50];

    wCoilStartAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //获取线圈地址
    wCoilNum        = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //获取线圈个数
	uByteNum        = upRxdbuf[4];                        //获取字节数

	if((wCoilNum >= 0x01) && (wCoilNum <= MAX_COIL_NUM) && (wCoilNum  <= 8 * uByteNum))
	{
		if((wCoilStartAddr >= COIL_ADD_MIN) && (wCoilStartAddr <= COIL_ADD_MAX) && 
		   (wCoilStartAddr + wCoilNum <= COIL_ADD_MAX + 1))
		{
			wTotalCoilNum = 0;
            for(k = 0; k < uByteNum; k++)
			{
				uByteVal = upRxdbuf[5 + k];
				for(i = 0; i < 8; i++)
				{
					if(uByteVal & (1 << i)) wCoilVal = 0xFF00;
					else                    wCoilVal = 0x0000;
					
					WriteOneCoilData(wCoilStartAddr + wTotalCoilNum, wCoilVal);
					 
					wTotalCoilNum ++;
					if(wTotalCoilNum >= wCoilNum)
					{
						 uExit = 1;
						 break;
					}
				}
				
				if(uExit == 1)
				{
					break;
				}
			}
			
			upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;
            upTxdbuf[uCommIndexNum ++] = WriteMultiCoil;
            memcpy(upTxdbuf + uCommIndexNum, upRxdbuf, 4);
            uCommIndexNum += 4;

            CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);

            MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
            memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
            HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);
			
			return;
		}
		else
		{
			uErrorCode = MB_EX_ILLEGAL_DATA_ADDRESS;
		}
	}
	else
	{
        uErrorCode = MB_EX_ILLEGAL_DATA_VALUE;
	}
	
    MODBUS_ERRFunction(WriteMultiCoil, uErrorCode);
    return;
}

/********
***********************************************************************
函数名称 : MODBUS_ERRFunction
功     能: 错误回应指令
参     数: 无
返 回  值: 无
异常功能码 = 功能码+0x80
*******************************************************************************/
void MODBUS_ERRFunction(uint8_t uCmdCode, uint8_t uErrorCode)
{
    uint8_t  uCommIndexNum = 0;
    uint16_t CRC16Temp     = 0;
    uint8_t upTxdbuf[50];

    upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;
    upTxdbuf[uCommIndexNum ++] = uCmdCode | 0x80;
    upTxdbuf[uCommIndexNum ++] = uErrorCode;

    CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
    upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF); //crc16低字节在前
    upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);

    MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
    memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
    HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);
}

/*******************************************************************************
函数名称 ： Modbus_Analysis
功    能 ： CRC校验
参    数 ： ptr--校验数组指针  len--校验数据长度
返 回 值 ： CRC校验码，双字节
*******************************************************************************/
void Modbus_Analysis(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint8_t  uSlaveAdd, uCmdCode;

    if((upRxdbuf == NULL) || (wRxdLen < 2))  return;

    uSlaveAdd    = upRxdbuf[0];
    uCmdCode     = upRxdbuf[1];

    // 从机地址为本机地址或者是广播帧
    if((uSlaveAdd == LOCAL_ADDRESS) || (uSlaveAdd == BROADCAST_ADDRESS))
    {
      switch(uCmdCode)
      {
				case ReadCoilState:
					ReadCoilStateFUNC(upRxdbuf + 2, wRxdLen - 2);             // 读线圈状态
					break;
		
				case ReadDisInputState:
					ReadDisInputStateFUNC(upRxdbuf + 2, wRxdLen - 2);         // 读离散输入状态
					break;
		
        case ReadHoldReg:
            ReadHoldRegFUNC(upRxdbuf + 2, wRxdLen - 2);               // 读取保持寄存器
            break;
		
        case ReadInputReg:
            ReadInputRegFUNC(upRxdbuf + 2, wRxdLen - 2);              // 读取输入寄存器
            break;
		
        case WriteSingleReg:
            WriteSingleRegFUNC(upRxdbuf + 2, wRxdLen - 2);            // 写单个寄存器
            break;

				case WriteMultiCoil:
            WriteMultiCoilFUNC(upRxdbuf + 2, wRxdLen - 2);            // 写多个线圈
            break;
		
        case WriteMultiReg:
            WriteMultiRegFUNC(upRxdbuf + 2, wRxdLen - 2);             // 写多个寄存器
            break;

        case WriteSingleCoil:
            WriteSingleCoilFUNC(upRxdbuf + 2, wRxdLen - 2);           // 写单个线圈
            break;

        default:
            MODBUS_ERRFunction(upRxdbuf[1], 0x01);                    // 错误码处理
            break;
        }
    }
}



void Modbus_Process(void)
{
    uint8_t  *pFrame;
    uint16_t wFrameLen = 0;
    uint16_t wFrameCRC, wCalCRC;
	  wFrameCRC = 0;
	  wCalCRC = 0;

    pFrame    = MODBUS_UART.rx_buf;    // 接收数据起始地址
    wFrameLen = MODBUS_UART.rx_size;   // 接收数据长度

    if(wFrameLen < 2) return;    // 数据长度不是有效值

    // 获取接收数据帧中的校验和
    wFrameCRC = MAKEWORD(pFrame[wFrameLen - 2], pFrame[wFrameLen - 1]);
    // 计算接收到的数据的校验和
    wCalCRC   = ModbusCRC16(pFrame, wFrameLen - 2);
    if(wFrameCRC != wCalCRC) return;

    Modbus_Analysis(MODBUS_UART.rx_buf, MODBUS_UART.rx_size);                        // 协议处理
}
