
#include "main.h"
#define SL_ADDR  0x01
#define MODBUS_UART huart3
#define MODBUS_REG_COUNT      64
#define u8 uint8_t
#define u16 uint16_t

// 接收缓冲区
uint8_t TX485_Buffer[256];
uint8_t RX_Count = 0;

// 定时器用于帧间隔检测
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart3;



//// 主函数初始化部分
//int main(void) {
//    HAL_Init();
//    SystemClock_Config();
//    MX_GPIO_Init();
//    MX_USART2_UART_Init();
//    MX_TIM2_Init();
//    

//    
//    //RS485_SetDir(false);
//    HAL_UART_Receive_IT(&MODBUS_UART, &TX485_Buffer[0], 1);
//    
//    while (1) {
//        // 主循环
//    }
//}



// UART接收中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == MODBUS_UART.Instance) {
        TX485_Buffer[RX_Count++] = MODBUS_UART.Instance->DR;
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_Base_Start_IT(&htim2);
        HAL_UART_Receive_IT(&MODBUS_UART, &TX485_Buffer[RX_Count], 1);
    }
}

// 定时器中断回调（3.5字符时间）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim2.Instance) {
        HAL_TIM_Base_Stop_IT(&htim2);
        RS485_MODBUS_RTU_req_pdu(TX485_Buffer, RX_Count);
        RX_Count = 0;
    }
}



u16 MODBUS_CRC16(u8 *pBytes, u8 Length)//MODBUS_RTU CRC16校验
{
	u8 i;
	u16 crc16;

	crc16 = 0xffff; // 预置16位CRC寄存器为0xffff（即全为1）
	do
	{
		crc16 ^= (u16)*pBytes;		// 把8位数据与16位CRC寄存器的低位相异或，把结果放于CRC寄存器
		for (i = 0; i < 8; i++) // 8位数据
		{
			if (crc16 & 1)
				crc16 = (crc16 >> 1) ^ 0xA001; // 如果最低位为0，把CRC寄存器的内容右移一位(朝低位)，用0填补最高位，
											   // 再异或多项式0xA001
			else
				crc16 >>= 1; // 如果最低位为0，把CRC寄存器的内容右移一位(朝低位)，用0填补最高位
		}
		pBytes++;
	} while (--Length != 0);
	return (crc16);
}



void RS485_MODBUS_RTU_req_pdu()//解析MODBUS_RTU功能码
{
	if (length < 4) return;  // 最小有效请求长度
	if (MODBUS_CRC16(RX485_Buffer, RX_Count) != 0)	 return;//首先判断CRC16是否正确, 不正确则忽略, 不处理也不返回信息
	if (RX485_Buffer[0] != SL_ADDR) return;	//然后判断站号地址是否正确, 或者是否广播地址(不返回信息)

	switch (RX485_Buffer[1]) /* 第2个字节 功能码 */
	{
			/*case 0x01: /* 读取线圈状态/
					MODS_01H();
					break;

			case 0x02: /* 读取输入状态 
					MODS_02H();
					break;*/

			case 0x03: /* 读取1个或多个参数保持寄存器 在一个或多个保持寄存器中取得当前的二进制值*/
					MODS_03H();
					break;

			/* case 0x04: /* 读取1个或多个模拟量输入寄存器 /
					MODS_04H();
					break;

			case 0x05: /* 强制单线圈（） 
					MODS_05H();
					break;*/

			case 0x06: /* 写单个参数保持寄存器 (存储在CPU的FLASH中，或EEPROM中的参数)*/
					MODS_06H();
					break;

			/* case 0x10: 写多个参数保持寄存器 (存储在CPU的FLASH中，或EEPROM中的参数)/
					MODS_10H();
					break;

			case 0x0F:
					MODS_0FH(); /* 强制多个线圈（对应D01/D02/D03） /
					break;

			case 0x64: /* 文件下载 /
					MODS_64H();
					break;

			case 0x65:  /* 临时执行小程序-废弃 /
					MODS_65H();
					break;

			case 0x66:   /* SWD操作指令(读内存，写内存等) /
					MODS_66H();
					break;

			case 0x70:   /* PC控制帧，无需应答。发送虚拟按键消息用 
					MODS_70H();
					break;*/
							
			default: 
				// 不支持的功能码
					MODS_SendAckErr(); /* 告诉主机命令错误 */
					break;


		}
	HAL_UART_Transmit(&MODBUS_UART, TX485_Buffer, TX_Length, 100);
}

void	MODS_03H()//读MODBUS_RTU寄存器
{
	u8 i;
	u16	crc;
	reg_addr = ((u16)RX485_Buffer[2] << 8) + RX485_Buffer[3]; // 寄存器地址
	 if (reg_addr ==0x0001)//工作模式
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=0;
				TX485_Buffer[4]=工作模式;
				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//要发送的字节数
				
			}
			else if(reg_addr ==0x0002)//输入电压
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=输入电压;
				TX485_Buffer[4]=输入电压;

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}	
			else if(reg_addr ==0x0003)//输入电流
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)输入电流;
				TX485_Buffer[4]=(u8)(输入电流>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
				
			else if(reg_addr ==0x0004)//输出电压
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)输出电压;
				TX485_Buffer[4]=(u8)(输出电压>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else if(reg_addr ==0x0005)//输出电流
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)输出电流;
				TX485_Buffer[4]=(u8)(输出电流>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else if(reg_addr ==0x0006)//启动停止信号
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)启动停止信号;
				TX485_Buffer[4]=(u8)(启动停止信号>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else if(reg_addr ==0x0007)//Kp
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)Kp;
				TX485_Buffer[4]=(u8)(Kp>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else if(reg_addr ==0x0008)//Ki
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)Ki;
				TX485_Buffer[4]=(u8)(Ki>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else if(reg_addr ==0x0009)//Kd
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)Kd;
				TX485_Buffer[4]=(u8)(Kd>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else
			{
				TX485_Buffer[0] = RX485_Buffer[0];	//要返回的应答

				TX485_Buffer[1]=0x83;
				TX485_Buffer[2]=0x03;
				
				crc = MODBUS_CRC16(TX485_Buffer, 3);
				TX485_Buffer[3] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[4] = (u8)(crc>>8);
				TX_Length = 5;		//要发送的字节数
			}
			
}

void	MODS_06H()// 写MODBUS_RTU寄存器
{
	u8 i;
	u16	crc;
	reg_addr = ((u16)RX485_Buffer[2] << 8) + RX485_Buffer[3]; // 寄存器地址
	 if (reg_addr ==0x0001)//工作模式
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

//操作

				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}
		else if(reg_addr ==0x0002)//输入电压
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答


				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			
		else if(reg_addr ==0x0003)//输入电流
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}
			else if(reg_addr ==0x0004)//输出电压
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}
			else if(reg_addr ==0x0005)//输出电流
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			else if(reg_addr ==0x0006)//启动停止信号
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			else if(reg_addr ==0x0007)//Kp
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			else if(reg_addr ==0x0008)//Ki
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			else if(reg_addr ==0x0009)//Kd
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			else
			{
				TX485_Buffer[0] = RX485_Buffer[0];	//要返回的应答

				TX485_Buffer[1]=0x86;
				TX485_Buffer[2]=0x03;
				
				crc = MODBUS_CRC16(TX485_Buffer, 3);
				TX485_Buffer[3] = (u8)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[4] = (u8)(crc>>8);
				TX_Length = 5;		//要发送的字节数
			}			

}
void MODS_SendAckErr()
{
	/*
	          response[responseLength++] = MODBUS_SLAVE_ADDRESS;
            response[responseLength++] = function | 0x80;
            response[responseLength++] = 0x01; // 非法功能
	*/
}
	


