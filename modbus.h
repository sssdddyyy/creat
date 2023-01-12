#ifndef __modbus_H
#define __modbus_H

#include "tim.h"
#include "stdint.h"
#include "usart.h"


/*--------------------------------- �궨�� -----------------------------------*/
//������
#define   ReadCoilState                 0x01         //��ȡ��Ȧ״̬
#define   ReadDisInputState             0x02         //��ȡ����״̬
#define   ReadHoldReg                   0x03         //��ȡ���ּĴ���
#define   ReadInputReg            		  0x04         //��ȡ����Ĵ���
#define   WriteSingleCoil               0x05         //ǿ��д����Ȧ���״̬
#define   WriteSingleReg                0x06         //Ԥ��(д)���Ĵ���
#define   WriteMultiCoil                0x0F         //ǿ��д����Ȧ���״̬
#define   WriteMultiReg                 0x10         //Ԥ��(д)��Ĵ���
 


#define COIL_ADD_MIN       0x10
#define COIL_ADD_MAX       0x1F
#define MAX_COIL_NUM       ((COIL_ADD_MAX) - (COIL_ADD_MIN) + 1)

#define DIS_ADD_MIN       0x00
#define DIS_ADD_MAX       0xFF
#define MAX_DIS_NUM       ((DIS_ADD_MAX) - (DIS_ADD_MIN) + 1)

#define INPUT_REG_ADD_MIN        0x00
#define INPUT_REG_ADD_MAX        0x0F
#define MAX_INPUT_REG_NUM        ((INPUT_REG_ADD_MAX) - (INPUT_REG_ADD_MIN) + 1)

#define HOLD_REG_ADD_MIN        0x00
#define HOLD_REG_ADD_MAX        0xFF
#define MAX_HOLD_REG_NUM        ((HOLD_REG_ADD_MAX) - (HOLD_REG_ADD_MIN) + 1)

#define Wright_Str_ADD_MIN        0x00000F
#define Wright_Str_ADD_MAX        0x7FFFF0
#define MAX_Wright_NUM        ((HOLD_REG_ADD_MAX) - (HOLD_REG_ADD_MIN) + 1)



#define BROADCAST_ADDRESS   0x00
#define LOCAL_ADDRESS       0x01

#define MAKEWORD(a,b)      ((uint16_t)(((uint8_t)(a)) | ((uint16_t)((uint8_t)(b))) << 8))

typedef enum
{
    MB_EX_NONE = 0x00,
    MB_EX_ILLEGAL_FUNCTION = 0x01,
    MB_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    MB_EX_ILLEGAL_DATA_VALUE = 0x03,
    MB_EX_SLAVE_DEVICE_FAILURE = 0x04,
    MB_EX_ACKNOWLEDGE = 0x05,
    MB_EX_SLAVE_BUSY = 0x06,
    MB_EX_MEMORY_PARITY_ERROR = 0x08,
    MB_EX_GATEWAY_PATH_FAILED = 0x0A,
    MB_EX_GATEWAY_TGT_FAILED = 0x0B
} eMBException;





typedef union var_reg
{
   uint32_t    SamData [MAX_INPUT_REG_NUM];         // ��������
   uint32_t    InputReg[MAX_INPUT_REG_NUM];         // Reg �� SamData
}Var_Reg;
extern Var_Reg  SamVarReg;

typedef union
{
	uint8_t u8[4];
	uint16_t u16[2];
	uint32_t u32;
}u8_u16_u32_typedef;


extern void Modbus_Process(void);


#endif
