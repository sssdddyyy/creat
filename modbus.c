#include "Modbus.h"
#include "string.h"
#include "spi_flash.h"
#include "spi.h"

#define MODBUS_UART uart2			//�����ڸĳ���Ӧ�ı������ɣ������õ�UART1���˴���Ϊuart1
#define MODBUS_HUART huart2			//�����ڸĳ���Ӧ�ı������ɣ������õ�UART1���˴���Ϊhuart1
#define  wRegTempAdd 0x42
#define  wRegAdd 0x46

uint8_t  CoilState[(MAX_DIS_NUM + 7) / 8] = {0x00};
uint8_t  DisState [(MAX_DIS_NUM + 7) / 8] = {0xAA};
uint16_t InputReg[MAX_INPUT_REG_NUM] = {0xAA55,0x55AA};
uint16_t HoldReg[MAX_HOLD_REG_NUM] = {0xAA55, 0x55AA};


Var_Reg  SamVarReg;


void MODBUS_ERRFunction(uint8_t uCmdCode, uint8_t uErrorCode);

/*******************************************************************************
�������� �� ModbusCRC16
��    �� �� CRCУ��
��    �� �� ptr--У������ָ��  len--У�����ݳ���
�� �� ֵ �� CRCУ���룬˫�ֽ�
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
�������� �� ReadCoilStateFUNC
��    �� �� ������:0x01,��ȡ��Ȧ
��    �� �� ��
�� �� ֵ �� ��
*����:[Ӳ����ַ][01][��Ȧ��ʼ��ַ��][��Ȧ��ʼ��ַ��][��Ȧ������][��Ȧ������][CRC��][CRC��]
*����:[Ӳ����ַ][01][�ֽڳ���][��Ȧֵ][��Ȧֵ][��Ȧֵ][CRC��][CRC��]
*******************************************************************************/
void ReadCoilStateFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
	uint16_t wCoilStartAddr,wCoilNum,wTotalCoilNum,CRC16Temp;
    uint8_t  i,k,uCommIndexNum = 0,uByteCount,uCoilVal,uErrorCode,uExit = 0;
	uint8_t  upTxdbuf[50];

    wCoilStartAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //��ȡ��Ȧ��ʼ��ַ
    wCoilNum        = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //��ȡ��Ȧ����

	if((wCoilNum >= 0x0001) || (wCoilNum <= MAX_COIL_NUM))
	{
		if(((wCoilStartAddr >= COIL_ADD_MIN) && (wCoilStartAddr <= COIL_ADD_MAX)) &&
			(wCoilNum + wCoilStartAddr <= COIL_ADD_MAX + 1))
		{
			uByteCount = (wCoilNum + 7) / 8;    //���������ֽڸ���
	
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
�������� �� ReadDisInputStateFUNC
��    �� �� ������:0x02,��ȡ��ɢ��
��    �� �� ��
�� �� ֵ �� ��
*����:[Ӳ����ַ][02][��ɢ����ʼ��ַ��][��ɢ����ʼ��ַ��][��ɢ��������][��ɢ��������][CRC��][CRC��]
*����:[Ӳ����ַ][02][�ֽڳ���][��ɢ��ֵ][��ɢ��ֵ][��ɢ��ֵ][CRC��][CRC��]
*******************************************************************************/
void ReadDisInputStateFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t wDisStartAddr,wDisNum,wTotalDisNum,CRC16Temp;
    uint8_t  i,k,uCommIndexNum = 0,uByteCount,uDisVal,uErrorCode,uExit = 0;
	uint8_t  upTxdbuf[50];

    wDisStartAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //��ȡ��ɢ����ʼ��ַ
    wDisNum        = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //��ȡ��ɢ������

	if((wDisNum >= 0x0001) || (wDisNum <= MAX_DIS_NUM))
	{
		if(((wDisStartAddr >= DIS_ADD_MIN) && (wDisStartAddr <= DIS_ADD_MAX)) &&
			(wDisNum + wDisStartAddr <= DIS_ADD_MAX + 1))
		{
			uByteCount = (wDisNum + 7) / 8;    //���������ֽڸ���
	
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
�������� �� GetHoldRegData
��    �� �� ��ȡRegAdd������
��    �� �� u16 RegAdd
�� �� ֵ �� ���ֽڼĴ�������
*******************************************************************************/
uint16_t GetHoldRegData(uint16_t RegAdd)
{
    int16_t wRegValue;

    wRegValue = HoldReg[RegAdd - HOLD_REG_ADD_MIN];

    return wRegValue;
}

/*******************************************************************************
�������� �� ReadHoldRegFUNC
��    �� �� ������:0x03,��ȡ���ּĴ���
��    �� �� ��
�� �� ֵ �� ��
*����:[Ӳ����ַ][03][��ʼ��ַ��][��ʼ��ַ��][�ܼĴ�������][�ܼĴ�������][CRC��][CRC��]
*����:[Ӳ����ַ][03][�ֽ���][�Ĵ���0��][�Ĵ���0��][�Ĵ���1��][�Ĵ���1��][�Ĵ���n��][�Ĵ���n��][CRC��][CRC��]
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
		buf.u8[3] = upRxdbuf[0];//��ȡ�Ĵ�����ʼ��ַ
		wRegStartAdd = buf.u32;
    wRegLen      = MAKEWORD(upRxdbuf[5], upRxdbuf[4]); //��ȡ��ȡ�Ĵ�������
		wRegStartAdd = (wRegStartAdd/wRegTempAdd)*wRegTempAdd+wRegAdd;
  //�ж���Ҫ��ȡ���ݵ���ʼ��ַ�Ƿ񳬹��洢����ַ�Ͷ�ȡ�������Ƿ񳬹��˴洢���Ĵ�С
    if((wRegLen >= 0x01) && (wRegLen <= MAX_HOLD_REG_NUM))
    {
        if(((wRegStartAdd >= Wright_Str_ADD_MIN) && (wRegStartAdd <= Wright_Str_ADD_MAX)) &&
                (((wRegStartAdd + wRegLen) >= Wright_Str_ADD_MIN) && ((wRegStartAdd + wRegLen) <= Wright_Str_ADD_MAX + 1)))  
        {
            upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;  //�����ص�ַ�洢ת��
            upTxdbuf[uCommIndexNum ++] = ReadHoldReg;   //����ʼ��ַ�洢ת��
            upTxdbuf[uCommIndexNum ++] = wRegLen * 2;  //�����ݴ�С�洢ת��
            SPI_FLASH_BufferRead(&hspi1,(uint8_t *)&m_data1,wRegStartAdd,wRegLen);   //��SPI��ȡָ����ַ��ָ����С�����ݲ���������
            for(i = 0; i < wRegLen; i++)
            {
                //��ȡ16λ���ݲ�����
                wRegValue = m_data1[i];
                upTxdbuf[uCommIndexNum ++] = (uint8_t)(wRegValue);
 //             upTxdbuf[uCommIndexNum ++] = (uint8_t)(wRegValue & 0xFF);
            }

            CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF); // crc16���ֽ���ǰ
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);   // crc16���ֽ��ں�

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
�������� �� GetInputRegData
��    �� �� ��ȡRegAdd������
��    �� �� u16 RegAdd
�� �� ֵ �� ���ֽڼĴ�������
*******************************************************************************/
uint16_t GetInputRegData(uint16_t RegAdd)
{
    int16_t wRegValue;

    wRegValue = InputReg[RegAdd - INPUT_REG_ADD_MIN];
	
    return wRegValue;
}
/*******************************************************************************
�������� �� ReadInputRegFUNC
��    �� �� ������:0x04,��ȡ����Ĵ���
��    �� �� ��
�� �� ֵ �� ��
*����:[Ӳ����ַ][04][��ʼ��ַ��][��ʼ��ַ��][�ܼĴ�������][�ܼĴ�������][    CRC��   ][     CRC��  ]
*����:[Ӳ����ַ][04][  �ֽ���  ][�Ĵ���0�� ][  �Ĵ���0�� ][  �Ĵ���1�� ][  �Ĵ���1�� ][  �Ĵ���n�� ][  �Ĵ���n�� ][  CRC�� ][  CRC�� ]
*******************************************************************************/
void ReadInputRegFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t wRegStartAdd, wRegLen, i, wRegValue, CRC16Temp;
    uint8_t  uErrorCode, uCommIndexNum = 0;
	uint8_t  upTxdbuf[50];
   
    if(upRxdbuf == NULL)  return;

    wRegStartAdd = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //��ȡ�Ĵ�����ʼ��ַ
    wRegLen      =  MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //��ȡ��ȡ�Ĵ�������

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
                //��ȡ16λ���ݲ�����
                wRegValue = GetInputRegData(wRegStartAdd + i);
                upTxdbuf[uCommIndexNum ++] = (uint8_t)(wRegValue >> 8);
                upTxdbuf[uCommIndexNum ++] = (uint8_t)(wRegValue & 0xFF);
            }

            CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF); // crc16���ֽ���ǰ
            upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);   // crc16���ֽ��ں�

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
�������� : WriteHoldRegData
��    �� : ����д��ı��ּĴ�������
��    �� : u16 StartAdd
�� �� ֵ : ��
*******************************************************************************/
void WriteHoldRegData(uint16_t wRegAddr, uint16_t RegData)
{
    HoldReg[wRegAddr - HOLD_REG_ADD_MIN] = RegData;
}

/*******************************************************************************
�������� �� WriteSingleRegFUNC
��    �� �� ������:0x06 Ԥ��(д)���Ĵ���
��    �� �� ��
�� �� ֵ �� ��
*����:[Ӳ����ַ][06][�Ĵ�����ַ��][�Ĵ�����ַ��][�Ĵ���ֵ��][�Ĵ���ֵ��][CRC��][CRC��]
*����:[Ӳ����ַ][06][�Ĵ�����ַ��][�Ĵ�����ַ��][�Ĵ���ֵ��][�Ĵ���ֵ��][CRC��][CRC��]
*******************************************************************************/
void WriteSingleRegFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t wRegAddr, wRegValue, CRC16Temp;
    uint8_t  uCommIndexNum = 0, uErrorCode;
    uint8_t  upTxdbuf[50];

    if(upRxdbuf == NULL)  return;

    wRegAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //��ȡ�Ĵ�����ַ
    wRegValue = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //��ȡ����

    if((wRegAddr >= HOLD_REG_ADD_MIN) && (wRegAddr <= HOLD_REG_ADD_MAX))
    {
        WriteHoldRegData(wRegAddr, wRegValue);

        upTxdbuf[uCommIndexNum ++] = LOCAL_ADDRESS;
        upTxdbuf[uCommIndexNum ++] = WriteSingleReg;
        memcpy(upTxdbuf + uCommIndexNum, upRxdbuf, 4);
        uCommIndexNum += 4;

        CRC16Temp = ModbusCRC16(upTxdbuf, uCommIndexNum);
        upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF); // crc16���ֽ���ǰ
        upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);   // crc16���ֽ��ں�

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
�������� �� WriteMultiRegFUNC
��    �� �� ������:0x10 д������ּĴ���
��    �� �� ��
�� �� ֵ �� ��
*******************************************************************************/
void WriteMultiRegFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t i, wRegStartAdd, wRegNum, CRC16Temp, uErrorCode, wRegValue;
    uint8_t  uCommIndexNum = 0, uByteNum;
    uint8_t  upTxdbuf[50];

    if(upRxdbuf == NULL)  return;

    wRegStartAdd = MAKEWORD(upRxdbuf[1], upRxdbuf[0]);     //��ȡ�Ĵ�����ַ
    wRegNum      = MAKEWORD(upRxdbuf[3], upRxdbuf[2]);     //��ȡ�Ĵ�������
    uByteNum     = upRxdbuf[4];                            //��ȡ�ֽ���

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
�������� : WriteOneCoilData
��    �� : ����д�����Ȧ�Ĵ�������
��    �� : u16 StartAdd
�� �� ֵ : ��
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
�������� �� WriteSingleCoilFUNC
��    �� �� ������:0x05 д������Ȧ�Ĵ���
��    �� �� ��
�� �� ֵ �� ��
*******************************************************************************/
void WriteSingleCoilFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint16_t wCoilAddr, wCoilValue, CRC16Temp;
    uint8_t  uCommIndexNum = 0, uErrorCode;
    uint8_t  upTxdbuf[50];

    wCoilAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //��ȡ��Ȧ��ַ
    wCoilValue = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //��ȡ��Ȧ����

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
�������� �� WriteMultiCoilFUNC
��    �� �� ������:0x0F,д�����Ȧ
��    �� �� ��
�� �� ֵ �� ��
*����:[Ӳ����ַ][0F][��ʼ��ַ��][��ʼ��ַ��][��Ȧ������][��Ȧ������][�ֽ���][��Ȧֵ][CRC��][CRC��]
*����:[Ӳ����ַ][0F][��ʼ��ַ��][��ʼ��ַ��][��Ȧ������][��Ȧ������][CRC��][CRC��]
*******************************************************************************/
void WriteMultiCoilFUNC(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
	uint16_t wCoilStartAddr,wCoilNum,wCoilVal,wTotalCoilNum,CRC16Temp;
    uint8_t  i,k,uCommIndexNum = 0,uByteNum,uByteVal,uExit = 0,uErrorCode;
	uint8_t  upTxdbuf[50];

    wCoilStartAddr  = MAKEWORD(upRxdbuf[1], upRxdbuf[0]); //��ȡ��Ȧ��ַ
    wCoilNum        = MAKEWORD(upRxdbuf[3], upRxdbuf[2]); //��ȡ��Ȧ����
	uByteNum        = upRxdbuf[4];                        //��ȡ�ֽ���

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
�������� : MODBUS_ERRFunction
��     ��: �����Ӧָ��
��     ��: ��
�� ��  ֵ: ��
�쳣������ = ������+0x80
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
    upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp & 0xFF); //crc16���ֽ���ǰ
    upTxdbuf[uCommIndexNum ++] = (uint8_t)(CRC16Temp >> 8);

    MODBUS_UART.tx_size = (uCommIndexNum <= MODBUS_UART.tx_buf_cnt ? uCommIndexNum : MODBUS_UART.tx_buf_cnt);
    memcpy(MODBUS_UART.tx_buf, upTxdbuf, MODBUS_UART.tx_size);
    HAL_UART_Transmit_IT(&MODBUS_HUART, MODBUS_UART.tx_buf, MODBUS_UART.tx_size);
}

/*******************************************************************************
�������� �� Modbus_Analysis
��    �� �� CRCУ��
��    �� �� ptr--У������ָ��  len--У�����ݳ���
�� �� ֵ �� CRCУ���룬˫�ֽ�
*******************************************************************************/
void Modbus_Analysis(uint8_t * upRxdbuf, uint16_t wRxdLen)
{
    uint8_t  uSlaveAdd, uCmdCode;

    if((upRxdbuf == NULL) || (wRxdLen < 2))  return;

    uSlaveAdd    = upRxdbuf[0];
    uCmdCode     = upRxdbuf[1];

    // �ӻ���ַΪ������ַ�����ǹ㲥֡
    if((uSlaveAdd == LOCAL_ADDRESS) || (uSlaveAdd == BROADCAST_ADDRESS))
    {
      switch(uCmdCode)
      {
				case ReadCoilState:
					ReadCoilStateFUNC(upRxdbuf + 2, wRxdLen - 2);             // ����Ȧ״̬
					break;
		
				case ReadDisInputState:
					ReadDisInputStateFUNC(upRxdbuf + 2, wRxdLen - 2);         // ����ɢ����״̬
					break;
		
        case ReadHoldReg:
            ReadHoldRegFUNC(upRxdbuf + 2, wRxdLen - 2);               // ��ȡ���ּĴ���
            break;
		
        case ReadInputReg:
            ReadInputRegFUNC(upRxdbuf + 2, wRxdLen - 2);              // ��ȡ����Ĵ���
            break;
		
        case WriteSingleReg:
            WriteSingleRegFUNC(upRxdbuf + 2, wRxdLen - 2);            // д�����Ĵ���
            break;

				case WriteMultiCoil:
            WriteMultiCoilFUNC(upRxdbuf + 2, wRxdLen - 2);            // д�����Ȧ
            break;
		
        case WriteMultiReg:
            WriteMultiRegFUNC(upRxdbuf + 2, wRxdLen - 2);             // д����Ĵ���
            break;

        case WriteSingleCoil:
            WriteSingleCoilFUNC(upRxdbuf + 2, wRxdLen - 2);           // д������Ȧ
            break;

        default:
            MODBUS_ERRFunction(upRxdbuf[1], 0x01);                    // �����봦��
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

    pFrame    = MODBUS_UART.rx_buf;    // ����������ʼ��ַ
    wFrameLen = MODBUS_UART.rx_size;   // �������ݳ���

    if(wFrameLen < 2) return;    // ���ݳ��Ȳ�����Чֵ

    // ��ȡ��������֡�е�У���
    wFrameCRC = MAKEWORD(pFrame[wFrameLen - 2], pFrame[wFrameLen - 1]);
    // ������յ������ݵ�У���
    wCalCRC   = ModbusCRC16(pFrame, wFrameLen - 2);
    if(wFrameCRC != wCalCRC) return;

    Modbus_Analysis(MODBUS_UART.rx_buf, MODBUS_UART.rx_size);                        // Э�鴦��
}
