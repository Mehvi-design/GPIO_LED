
#include "main.h"
#define SL_ADDR  0x01
#define MODBUS_UART huart3
#define MODBUS_REG_COUNT      64
#define u8 uint8_t
#define u16 uint16_t

// ���ջ�����
uint8_t TX485_Buffer[256];
uint8_t RX_Count = 0;

// ��ʱ������֡������
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart3;



//// ��������ʼ������
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
//        // ��ѭ��
//    }
//}



// UART�����жϻص�
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == MODBUS_UART.Instance) {
        TX485_Buffer[RX_Count++] = MODBUS_UART.Instance->DR;
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_Base_Start_IT(&htim2);
        HAL_UART_Receive_IT(&MODBUS_UART, &TX485_Buffer[RX_Count], 1);
    }
}

// ��ʱ���жϻص���3.5�ַ�ʱ�䣩
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim2.Instance) {
        HAL_TIM_Base_Stop_IT(&htim2);
        RS485_MODBUS_RTU_req_pdu(TX485_Buffer, RX_Count);
        RX_Count = 0;
    }
}



u16 MODBUS_CRC16(u8 *pBytes, u8 Length)//MODBUS_RTU CRC16У��
{
	u8 i;
	u16 crc16;

	crc16 = 0xffff; // Ԥ��16λCRC�Ĵ���Ϊ0xffff����ȫΪ1��
	do
	{
		crc16 ^= (u16)*pBytes;		// ��8λ������16λCRC�Ĵ����ĵ�λ����򣬰ѽ������CRC�Ĵ���
		for (i = 0; i < 8; i++) // 8λ����
		{
			if (crc16 & 1)
				crc16 = (crc16 >> 1) ^ 0xA001; // ������λΪ0����CRC�Ĵ�������������һλ(����λ)����0����λ��
											   // ��������ʽ0xA001
			else
				crc16 >>= 1; // ������λΪ0����CRC�Ĵ�������������һλ(����λ)����0����λ
		}
		pBytes++;
	} while (--Length != 0);
	return (crc16);
}



void RS485_MODBUS_RTU_req_pdu()//����MODBUS_RTU������
{
	if (length < 4) return;  // ��С��Ч���󳤶�
	if (MODBUS_CRC16(RX485_Buffer, RX_Count) != 0)	 return;//�����ж�CRC16�Ƿ���ȷ, ����ȷ�����, ������Ҳ��������Ϣ
	if (RX485_Buffer[0] != SL_ADDR) return;	//Ȼ���ж�վ�ŵ�ַ�Ƿ���ȷ, �����Ƿ�㲥��ַ(��������Ϣ)

	switch (RX485_Buffer[1]) /* ��2���ֽ� ������ */
	{
			/*case 0x01: /* ��ȡ��Ȧ״̬/
					MODS_01H();
					break;

			case 0x02: /* ��ȡ����״̬ 
					MODS_02H();
					break;*/

			case 0x03: /* ��ȡ1�������������ּĴ��� ��һ���������ּĴ�����ȡ�õ�ǰ�Ķ�����ֵ*/
					MODS_03H();
					break;

			/* case 0x04: /* ��ȡ1������ģ��������Ĵ��� /
					MODS_04H();
					break;

			case 0x05: /* ǿ�Ƶ���Ȧ���� 
					MODS_05H();
					break;*/

			case 0x06: /* д�����������ּĴ��� (�洢��CPU��FLASH�У���EEPROM�еĲ���)*/
					MODS_06H();
					break;

			/* case 0x10: д����������ּĴ��� (�洢��CPU��FLASH�У���EEPROM�еĲ���)/
					MODS_10H();
					break;

			case 0x0F:
					MODS_0FH(); /* ǿ�ƶ����Ȧ����ӦD01/D02/D03�� /
					break;

			case 0x64: /* �ļ����� /
					MODS_64H();
					break;

			case 0x65:  /* ��ʱִ��С����-���� /
					MODS_65H();
					break;

			case 0x66:   /* SWD����ָ��(���ڴ棬д�ڴ��) /
					MODS_66H();
					break;

			case 0x70:   /* PC����֡������Ӧ�𡣷������ⰴ����Ϣ�� 
					MODS_70H();
					break;*/
							
			default: 
				// ��֧�ֵĹ�����
					MODS_SendAckErr(); /* ��������������� */
					break;


		}
	HAL_UART_Transmit(&MODBUS_UART, TX485_Buffer, TX_Length, 100);
}

void	MODS_03H()//��MODBUS_RTU�Ĵ���
{
	u8 i;
	u16	crc;
	reg_addr = ((u16)RX485_Buffer[2] << 8) + RX485_Buffer[3]; // �Ĵ�����ַ
	 if (reg_addr ==0x0001)//����ģʽ
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=0;
				TX485_Buffer[4]=����ģʽ;
				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//Ҫ���͵��ֽ���
				
			}
			else if(reg_addr ==0x0002)//�����ѹ
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=�����ѹ;
				TX485_Buffer[4]=�����ѹ;

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//Ҫ���͵��ֽ���
			}	
			else if(reg_addr ==0x0003)//�������
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)�������;
				TX485_Buffer[4]=(u8)(�������>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//Ҫ���͵��ֽ���
			}
				
			else if(reg_addr ==0x0004)//�����ѹ
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)�����ѹ;
				TX485_Buffer[4]=(u8)(�����ѹ>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//Ҫ���͵��ֽ���
			}
			else if(reg_addr ==0x0005)//�������
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)�������;
				TX485_Buffer[4]=(u8)(�������>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//Ҫ���͵��ֽ���
			}
			else if(reg_addr ==0x0006)//����ֹͣ�ź�
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)����ֹͣ�ź�;
				TX485_Buffer[4]=(u8)(����ֹͣ�ź�>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//Ҫ���͵��ֽ���
			}
			else if(reg_addr ==0x0007)//Kp
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)Kp;
				TX485_Buffer[4]=(u8)(Kp>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//Ҫ���͵��ֽ���
			}
			else if(reg_addr ==0x0008)//Ki
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)Ki;
				TX485_Buffer[4]=(u8)(Ki>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//Ҫ���͵��ֽ���
			}
			else if(reg_addr ==0x0009)//Kd
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(u8)Kd;
				TX485_Buffer[4]=(u8)(Kd>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[6] = (u8)(crc>>8);
				TX_Length = 7;		//Ҫ���͵��ֽ���
			}
			else
			{
				TX485_Buffer[0] = RX485_Buffer[0];	//Ҫ���ص�Ӧ��

				TX485_Buffer[1]=0x83;
				TX485_Buffer[2]=0x03;
				
				crc = MODBUS_CRC16(TX485_Buffer, 3);
				TX485_Buffer[3] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[4] = (u8)(crc>>8);
				TX_Length = 5;		//Ҫ���͵��ֽ���
			}
			
}

void	MODS_06H()// дMODBUS_RTU�Ĵ���
{
	u8 i;
	u16	crc;
	reg_addr = ((u16)RX485_Buffer[2] << 8) + RX485_Buffer[3]; // �Ĵ�����ַ
	 if (reg_addr ==0x0001)//����ģʽ
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��

//����

				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//Ҫ���͵��ֽ���
			}
		else if(reg_addr ==0x0002)//�����ѹ
			{
//����
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��


				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//Ҫ���͵��ֽ���
			}	
			
		else if(reg_addr ==0x0003)//�������
			{
//����
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//Ҫ���͵��ֽ���
			}
			else if(reg_addr ==0x0004)//�����ѹ
			{
//����
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//Ҫ���͵��ֽ���
			}
			else if(reg_addr ==0x0005)//�������
			{
//����
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//Ҫ���͵��ֽ���
			}	
			else if(reg_addr ==0x0006)//����ֹͣ�ź�
			{
//����
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//Ҫ���͵��ֽ���
			}	
			else if(reg_addr ==0x0007)//Kp
			{
//����
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//Ҫ���͵��ֽ���
			}	
			else if(reg_addr ==0x0008)//Ki
			{
//����
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//Ҫ���͵��ֽ���
			}	
			else if(reg_addr ==0x0009)//Kd
			{
//����
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//Ҫ���ص�Ӧ��

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[7] = (u8)(crc>>8);
				TX_Length = 8;		//Ҫ���͵��ֽ���
			}	
			else
			{
				TX485_Buffer[0] = RX485_Buffer[0];	//Ҫ���ص�Ӧ��

				TX485_Buffer[1]=0x86;
				TX485_Buffer[2]=0x03;
				
				crc = MODBUS_CRC16(TX485_Buffer, 3);
				TX485_Buffer[3] = (u8)crc;	//CRC��С��ģʽ, �ȷ����ֽڣ��󷢸��ֽڡ�
				TX485_Buffer[4] = (u8)(crc>>8);
				TX_Length = 5;		//Ҫ���͵��ֽ���
			}			

}
void MODS_SendAckErr()
{
	/*
	          response[responseLength++] = MODBUS_SLAVE_ADDRESS;
            response[responseLength++] = function | 0x80;
            response[responseLength++] = 0x01; // �Ƿ�����
	*/
}
	


