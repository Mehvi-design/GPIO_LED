/*
 * Function.c
 */
#include "Function.h"
#include "PID.h"
#include "u8g2.h"
#include "u8x8.h"
#include "string.h"
#include <math.h>
#include "bh1750_i2c_drv.h"
#define CONST_PI_32     3.14159265358979323846f
#define DCL_PERIOD 1.0f/25.0e3f
#define VIN_FILTER_FC 5000.0f * 2.0f * CONST_PI_32
#define VOUT_FILTER_FC 5000.0f * 2.0f * CONST_PI_32
#define IL_FILTER_FC 10000.0f * 2.0f * CONST_PI_32


#define SL_ADDR  0x01
#define MODBUS_UART huart3
#define MODBUS_REG_COUNT      64


// 接收缓冲区
uint8_t TX485_Buffer[256];
uint8_t RX485_Buffer[256];
uint8_t RX_Count = 0;
uint8_t TX_Length;
// 定时器用于帧间隔检测
extern TIM_HandleTypeDef htim2;
/*
 * ADC_Result_INJ[0] = IL PA3 CH4
*/
uint16_t ADC_Result_INJ;
/*
 * ADC_Result_REG[0] = IL PA3 IN4
 * ADC_Result_REG[1] = IO PC3 IN8
 * ADC_Result_REG[2] = IN PC1 IN7
 
*/

uint16_t ADC_Result_REG[4];
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, U8X8_UNUSED void *arg_ptr);
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
u8g2_t u8g2;
extern I2C_HandleTypeDef hi2c1;

value_t Vout = {//Battery
		.Lower_Limit = 0.0f,
		.Upper_Limit = 15.0f,
		.filter = {	.Output = 0.0f,
					.Input = 0.0f,
					.Input_L1 = 0.0f,
					.Output_L1 = 0.0f,
					.A1 = (VOUT_FILTER_FC * DCL_PERIOD - 2.0f)/(VOUT_FILTER_FC * DCL_PERIOD + 2.0f),
					.B0 =  (VOUT_FILTER_FC * DCL_PERIOD)/(VOUT_FILTER_FC * DCL_PERIOD + 2.0f),
					.B1 = (VOUT_FILTER_FC * DCL_PERIOD)/(VOUT_FILTER_FC * DCL_PERIOD + 2.0f)
		},
		.Raw = 0.0f,
		.Filted = 0.0f,
		.Gain = 0.0463000014,
};
value_t Vin = {//LED
		.Lower_Limit = 0.0f,
		.Upper_Limit = 15.0f,
		.filter = {	.Output = 0.0f,
					.Input = 0.0f,
					.Input_L1 = 0.0f,
					.Output_L1 = 0.0f,
					.A1 = (VIN_FILTER_FC * DCL_PERIOD - 2.0f)/(VIN_FILTER_FC * DCL_PERIOD + 2.0f),
					.B0 =  (VIN_FILTER_FC * DCL_PERIOD)/(VIN_FILTER_FC * DCL_PERIOD + 2.0f),
					.B1 = (VIN_FILTER_FC * DCL_PERIOD)/(VIN_FILTER_FC * DCL_PERIOD + 2.0f)
		},
		.Raw = 0.0f,
		.Filted = 0.0f,
		.Gain = 0.0468098521f,
};
value_t IL = {//To Battery Is Neg
		.Lower_Limit = 0.0f,
		.Upper_Limit = 15.0f,
		.filter = {	.Output = 0.0f,
					.Input = 0.0f,
					.Input_L1 = 0.0f,
					.Output_L1 = 0.0f,
					.A1 = (IL_FILTER_FC * DCL_PERIOD - 2.0f)/(IL_FILTER_FC * DCL_PERIOD + 2.0f),
					.B0 =  (IL_FILTER_FC * DCL_PERIOD)/(IL_FILTER_FC * DCL_PERIOD + 2.0f),
					.B1 = (IL_FILTER_FC * DCL_PERIOD)/(IL_FILTER_FC * DCL_PERIOD + 2.0f)
		},
		.Raw = 0.0f,
		.Filted = 0.0f,
		.Gain =0.0213500001,
};

PIDController pid_I = {
.Kd = 0.0f,.Ki=1.0f,.Kp=0.1f,.limMax=30.0f,.limMin=0.0f
,.limMaxInt = 30.0f,.limMinInt = 0.0f,.T = 1.0f/25.0e3f	
};

PIDController pid_V = {
.Kd = 0.0f,.Ki=50.0f,.Kp=2.0f,.limMax=30.0f,.limMin=-30.0f
,.limMaxInt = 30.0f,.limMinInt = -30.0f,.T = 1.0f/25.0e3f
};

float Charge = 0.0f;
float IL_Reference = 0.0f;
float Output_Counter = 0.0f;
float Output_Voltage = 5.0f;
uint8_t Error_flag = 0;
uint8_t State = 0;

void set_duty_buck(float duty){
	LL_HRTIM_TIM_SetCompare2(HRTIM1,LL_HRTIM_TIMER_C,(uint16_t)(duty * 5760.0F)+9);
	LL_HRTIM_TIM_SetCompare1(HRTIM1,LL_HRTIM_TIMER_C,11520 - (uint16_t)(duty * 5760.0F)-9);
}

void set_duty_boost(float duty){
	LL_HRTIM_TIM_SetCompare1(HRTIM1,LL_HRTIM_TIMER_C,(uint16_t)(duty * 5760.0F)+9);
	LL_HRTIM_TIM_SetCompare2(HRTIM1,LL_HRTIM_TIMER_C,11520 - (uint16_t)(duty * 5760.0F)-9);
}

void setup(void){
	
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay);
	u8g2_InitDisplay(&u8g2);  
	u8g2_SetPowerSave(&u8g2, 0);
	u8g2_SetFont(&u8g2,u8g2_font_lubR12_te);
	u8g2_ClearBuffer(&u8g2);
	u8g2_SendBuffer(&u8g2);
	
	IL.Offset = 2052.0f;
	LL_HRTIM_EnableOutput(HRTIM1,LL_HRTIM_OUTPUT_TC1|LL_HRTIM_OUTPUT_TC2);
	LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_C);
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0){}
	LL_ADC_Enable(ADC1);
	LL_ADC_REG_StartConversion(ADC1);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 4);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC_Result_REG);
	LL_DMA_EnableIT_TC(DMA1,LL_DMA_CHANNEL_1);
	LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
	
	HAL_Delay(2000);
	Output_Counter = Vout.Real;
	HAL_Delay(2000);
	if(Vin.Real >= Vout.Real){ //Boost Mode
		State = 1;
		Output_Voltage = 36.0f;
		//UART_SendByte(State);
	}	else{
		State = 2;
		Output_Voltage = 10.0f;
		//UART_SendUint16AsHex((int16_t)roundf(Output_Voltage * 100));
		//UART_SendFloat(Output_Voltage);
	}
}

void loop(void){

//	char temp[32] ="";
//	char temp2[32]="";
//	char temp3[32]="";
//	char temp4[32]="";
//	float Soc = (1.0 - 4.2f + Vout.Real) * 100.0f;
//	if(Soc > 100.0f){
//		Soc =  100.0f;
//	}else if(Soc < 0.0f){
//		Soc =  0.0f;
//	}
//	sprintf(temp,"%s,SOC:%.1f%%",IL.Real<0.2?"F":"O",Soc);
//	sprintf(temp2,"I:%.2f,%.2f",IL.Real,pid_V.limMax);
//	sprintf(temp3,"Cap:%dmAh",(int16_t)(Charge * 1000.0f));
//	sprintf(temp4,"V:%.2f",Vout.Real);
//	u8g2_ClearBuffer(&u8g2);
//	u8g2_DrawStr(&u8g2,0,14, temp);
//	u8g2_DrawStr(&u8g2,0,28, temp4);
//	u8g2_DrawStr(&u8g2,0,42, temp2);
//	u8g2_DrawStr(&u8g2,0,58, temp3);
//	
//	u8g2_SendBuffer(&u8g2);
//	
//	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == RESET){
//		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == RESET);
//		pid_V.limMax = (float)((float)pid_V.limMax + (float)0.1f);
//		HAL_Delay(200);
//	}
//	
//	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5) == RESET){
//		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5) == RESET);
//		pid_V.limMax= (float)((float)pid_V.limMax - (float)0.1f);

//		HAL_Delay(200);	
//	}		
}

void ISR_DMA1_TC(void){
/*
	* Vin.Real is the output voltage of buck
	* Vout.Real is the output voltage of the boost
*/
	static uint32_t i = 0;
	First_Order_Transfer_Func_Compute_with_Parameter(&IL.filter,(float)ADC_Result_REG[0]);
	First_Order_Transfer_Func_Compute_with_Parameter(&Vout.filter, (float)ADC_Result_REG[2]);
	First_Order_Transfer_Func_Compute_with_Parameter(&Vin.filter, (float)ADC_Result_REG[1]);
	Vout.Real = (float)Vout.filter.Output * Vout.Gain;
	Vin.Real = (float)Vin.filter.Output * Vin.Gain;
	IL.Real = (float)(IL.filter.Output - (float)IL.Offset) * IL.Gain;
	
	if(State == 1){
		if(Vin.Real < 12.0){
			LL_HRTIM_DisableOutput(HRTIM1,LL_HRTIM_OUTPUT_TC1|LL_HRTIM_OUTPUT_TC2);
		}
		if(Output_Voltage - Output_Counter > 0.01f){
			Output_Counter += 0.001;
		}else if(Output_Voltage - Output_Counter < -0.01f){
			Output_Counter -= 0.001;
		}
		PIDController_Update(&pid_V, Output_Counter, Vout.Real);
		PIDController_Update(&pid_I, pid_V.out, IL.Real);
		set_duty_boost(pid_I.out/Vin.Real);
	}else if(State == 2){
		PIDController_Update(&pid_V, Output_Voltage, Vin.Real);
		PIDController_Update(&pid_I, pid_V.out, -IL.Real);
		set_duty_buck(pid_I.out/Vout.Real);
	}
}

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	static uint8_t buffer[32];
	static uint8_t buf_idx;
	static uint8_t *data;
	static int addr;
	switch(msg)
	{
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t *)arg_ptr;
		while( arg_int >0)
		{
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;
	case U8X8_MSG_BYTE_INIT:
		
		break;
	case U8X8_MSG_BYTE_SET_DC:
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx=0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		addr = u8x8_GetI2CAddress(u8x8);
		HAL_I2C_Master_Transmit(&hi2c1, addr, buffer, buf_idx, 100);
		//HAL_I2C_Master_Transmit_DMA(&hi2c1, addr, buffer, buf_idx);
		break;
	default:
		return 0;
	}
	return 1;
}

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_DELAY_MILLI:
      HAL_Delay(arg_int);
      break;
    case U8X8_MSG_DELAY_I2C:
      break;
    case U8X8_MSG_GPIO_I2C_CLOCK:
			if ( arg_int == 0 ){
			}else{
			}
		break;
    case U8X8_MSG_GPIO_I2C_DATA:
      if ( arg_int == 0 ){
			}else{
			}
     break; 
  }
  return 1;
}

void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if(LL_USART_IsActiveFlag_RXNE(USART3)){
		RX485_Buffer[RX_Count++] = LL_USART_ReceiveData8(USART3);
		// 重置定时器
		__HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start_IT(&htim2);
	}
  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}



// 发送一个字节
void UART_SendByte(uint8_t byte) {
    while (!LL_USART_IsActiveFlag_TXE(USART3));
    LL_USART_TransmitData8(USART3, byte);
}
// 发送16位整数的十六进制字节
void UART_SendUint16AsHex(uint16_t value) {
    uint8_t lowByte = (uint8_t)(value & 0xFF);
    uint8_t highByte = (uint8_t)(value >> 8);

    UART_SendByte(lowByte);
    UART_SendByte(highByte);
}
// 发送一个32位浮点数
void UART_SendFloat(float value) {
    uint8_t *bytePtr = (uint8_t *)&value;
    for (int i = 0; i < 4; i++) {
        UART_SendByte(*bytePtr++);
    }
}
// 将两个字节组合成16位整数
/*
uint16_t CombineBytesToUint16(uint8_t lowByte, uint8_t highByte) {
    return (uint16_t)(highByte << 8) | lowByte;
}
*/
/* TIM3中断处理（HAL库处理） */
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

// 定时器中断回调（3.5字符时间）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim2.Instance) {
        HAL_TIM_Base_Stop_IT(&htim2);
        RS485_MODBUS_RTU_req_pdu();
        RX_Count = 0;
    }
}


/* Modbus CRC16计算 */
uint16_t MODBUS_CRC16(uint8_t *data, uint16_t length) {
    uint16_t crc16 = 0xFFFF;// 预置16位CRC寄存器为0xffff（即全为1）
    for (uint16_t i = 0; i < length; i++) {
        crc16 ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc16 & 0x0001) {
                crc16 >>= 1;
                crc16 ^= 0xA001;
            } else {
                crc16 >>= 1;
            }
        }
    }
    return crc16;
}


void RS485_MODBUS_RTU_req_pdu()//解析MODBUS_RTU功能码
{
	//if (length < 4) return;  // 最小有效请求长度
	if (MODBUS_CRC16(RX485_Buffer, RX_Count) != 0)	 return;//首先判断CRC16是否正确, 不正确则忽略, 不处理也不返回信息
	if (RX485_Buffer[0] != SL_ADDR) return;	//然后判断站号地址是否正确, 或者是否广播地址(不返回信息)

	switch (RX485_Buffer[1]) /* 第2个字节 功能码 */
	{
			/*case 0x01: // 读取线圈状态/
					MODS_01H();
					break;

			case 0x02: // 读取输入状态 
					MODS_02H();
					break;*/

			case 0x03: /* 读取1个或多个参数保持寄存器 在一个或多个保持寄存器中取得当前的二进制值*/
					MODS_03H();
					break;

			/* case 0x04: // 读取1个或多个模拟量输入寄存器 /
					MODS_04H();
					break;

			case 0x05: // 强制单线圈（） 
					MODS_05H();
					break;*/

			case 0x06: /* 写单个参数保持寄存器 (存储在CPU的FLASH中，或EEPROM中的参数)*/
					MODS_06H();
					break;

			/* case 0x10: 写多个参数保持寄存器 (存储在CPU的FLASH中，或EEPROM中的参数)/
					MODS_10H();
					break;

			case 0x0F:
					MODS_0FH(); // 强制多个线圈（对应D01/D02/D03） /
					break;

			case 0x64:// 文件下载 /
					MODS_64H();
					break;

			case 0x65:  // 临时执行小程序-废弃 /
					MODS_65H();
					break;

			case 0x66:   // SWD操作指令(读内存，写内存等) /
					MODS_66H();
					break;

			case 0x70:   // PC控制帧，无需应答。发送虚拟按键消息用 
					MODS_70H();
					break;*/
							
			default: 
				// 不支持的功能码
					MODS_SendAckErr(); /* 告诉主机命令错误 */
					break;

		}
	//HAL_UART_Transmit(&MODBUS_UART, TX485_Buffer, TX_Length, 100);
		LL_USART_DisableIT_RXNE(USART3);  // 关闭接收中断
		for(int i=0;i<TX_Length;i++){
			while(!LL_USART_IsActiveFlag_TXE(USART3)); // 等待发送寄存器空
			LL_USART_TransmitData8(USART3, TX485_Buffer[i]);
		}
		LL_USART_ClearFlag_TC(USART3);
    // 重新启用接收
    LL_USART_EnableIT_RXNE(USART3);
		//LL_USART_EnableIT_TC(USART3);  // 使能发送完成中断
}

void	MODS_03H()//读MODBUS_RTU寄存器
{
	int16_t POW,IV,IC,OV,OC;
	uint8_t i,reg_DATA_length;
	uint16_t	crc;
	uint16_t reg_addr = ((uint16_t)RX485_Buffer[2] << 8) + RX485_Buffer[3]; // 寄存器起始地址
	uint16_t reg_count = ((uint16_t)RX485_Buffer[4] << 8) + RX485_Buffer[5]; // 寄存器个数
	
	OV=(int16_t)roundf(Output_Voltage * 100);
	OC=(int16_t)roundf(Output_Counter * 100);
	
	IV=(int16_t)roundf(	Vin.Real * 100);
	POW=OV*OC*1.02;
	IC=POW/IV;
	
	if (reg_addr ==0)//测试
			{
				reg_DATA_length=(uint8_t)(reg_count*2);//字节长度
				TX485_Buffer[0] = RX485_Buffer[0];	//要返回的应答
				TX485_Buffer[1] = RX485_Buffer[1];	//要返回的应答
				TX485_Buffer[2]=reg_DATA_length;//字节长度
				
				//工作模式1是boost 2是buck
				TX485_Buffer[3]=0;
				TX485_Buffer[4]=State;
				
				TX485_Buffer[5]=(uint8_t)(OV >>8);//输出电压;
				TX485_Buffer[6]=(uint8_t)(OV & 0xFF);//(输出电压);
				
				TX485_Buffer[7]=(uint8_t)(OC >>8);//输出电流;
				TX485_Buffer[8]=(uint8_t)(OC & 0xFF);//(输出电流);			
				
				TX485_Buffer[9] =(uint8_t)(IV >>8);//输入电压;
				TX485_Buffer[10]=(uint8_t)(IV & 0xFF);//输入电压;
								
				TX485_Buffer[11]=(uint8_t)(IC >>8);//输入电流;
				TX485_Buffer[12]=(uint8_t)(IC & 0xFF);//(输入电流);

				TX485_Buffer[13]=0;//启动停止信号;
				TX485_Buffer[14]=1;//(启动停止信号);
				
				crc = MODBUS_CRC16(TX485_Buffer, 3 + reg_DATA_length);
				TX485_Buffer[reg_DATA_length+3] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[reg_DATA_length+4] = (uint8_t)(crc>>8);
				TX_Length = reg_DATA_length+5;		//要发送的字节数
			
			}
			/*
			else if (reg_addr ==0x0001)//工作模式
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				//工作模式1是boost 2是buck
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=0;
				TX485_Buffer[4]=State;
				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (uint8_t)(crc>>8);
				TX_Length = 7;		//要发送的字节数
				
			}
			else if(reg_addr ==0x0002)//输入电压
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(uint8_t)IV;//输入电压;
				TX485_Buffer[4]=(uint8_t)IV>>8;//输入电压;

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (uint8_t)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}	
			else if(reg_addr ==0x0003)//输入电流
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(uint8_t)IC;//输入电流;
				TX485_Buffer[4]=(uint8_t)IC>>8;//(输入电流>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (uint8_t)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
				
			else if(reg_addr ==0x0004)//输出电压
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;

				TX485_Buffer[3]=(uint8_t)OV;//输出电压;
				TX485_Buffer[4]=(uint8_t)OV>>8;//(输出电压>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (uint8_t)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else if(reg_addr ==0x0005)//输出电流
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;

				TX485_Buffer[3]=(uint8_t)OC;//输出电流;
				TX485_Buffer[4]=(uint8_t)OC>>8;//(输出电流>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (uint8_t)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else if(reg_addr ==0x0006)//启动停止信号
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=0;//启动停止信号;
				TX485_Buffer[4]=1;//(启动停止信号>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (uint8_t)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			*/
			/*
			else if(reg_addr ==0x0007)//Kp
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(uint8_t)Kp;
				TX485_Buffer[4]=(uint8_t)(Kp>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (uint8_t)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else if(reg_addr ==0x0008)//Ki
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(uint8_t)Ki;
				TX485_Buffer[4]=(uint8_t)(Ki>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (uint8_t)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}
			else if(reg_addr ==0x0009)//Kd
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答
				TX485_Buffer[2]=0x02;
				TX485_Buffer[3]=(uint8_t)Kd;
				TX485_Buffer[4]=(uint8_t)(Kd>>8);

				crc = MODBUS_CRC16(TX485_Buffer, 5);
				TX485_Buffer[5] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[6] = (uint8_t)(crc>>8);
				TX_Length = 7;		//要发送的字节数
			}*/
			else//错误应答
			{
				TX485_Buffer[0] = RX485_Buffer[0];	//要返回的应答

				TX485_Buffer[1]=0x83;
				TX485_Buffer[2]=0x03;
				
				crc = MODBUS_CRC16(TX485_Buffer, 3);
				TX485_Buffer[3] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[4] = (uint8_t)(crc>>8);
				TX_Length = 5;		//要发送的字节数
			}
			
}

void	MODS_06H()// 写MODBUS_RTU寄存器
{
	uint8_t i;
	uint16_t	crc;
	uint16_t reg_addr = ((uint16_t)RX485_Buffer[2] << 8) + RX485_Buffer[3]; // 寄存器地址
	 	 if (reg_addr ==0x0000)//测试
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

//操作

				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}
			else if (reg_addr ==0x0001)//工作模式
			{
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

//操作

				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}
			else if(reg_addr ==0x0002)//电压设定
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				// 合并两个字节为一个16位无符号整数
    	  //Output_Voltage= (float)((RX485_Buffer[5]) | RX485_Buffer[4] << 8);
				//Output_Voltage= (float)(CombineBytesToUint16(RX485_Buffer[4], RX485_Buffer[5]));
				
				Output_Voltage= 1.0f*(uint16_t)((RX485_Buffer[4] << 8) | RX485_Buffer[5]);
				//UART_SendFloat(Output_Voltage);
				
				//UART_SendUint16AsHex((int16_t)roundf(Output_Voltage * 100));
				//UART_SendUint16AsHex(CombineBytesToUint16(RX485_Buffer[4], RX485_Buffer[5]));
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			/*
			else if(reg_addr ==0x0003)//电流设定
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}
			else if(reg_addr ==0x0004)//输出电压
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				// 合并两个字节为一个16位无符号整数
    	  Output_Voltage= (float)((RX485_Buffer[5] << 8) | RX485_Buffer[4]);
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}
			else if(reg_addr ==0x0005)//输出电流
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			else if(reg_addr ==0x0006)//启动停止信号
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			else if(reg_addr ==0x0007)//Kp
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			else if(reg_addr ==0x0008)//Ki
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	
			else if(reg_addr ==0x0009)//Kd
			{
//操作
				
				for(i=0; i<6; i++)	TX485_Buffer[i] = RX485_Buffer[i];	//要返回的应答

				TX485_Buffer[4]=0;
				TX485_Buffer[5]=0;
				
				crc = MODBUS_CRC16(TX485_Buffer, 6);
				TX485_Buffer[6] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[7] = (uint8_t)(crc>>8);
				TX_Length = 8;		//要发送的字节数
			}	*/
			else//错误应答
			{
				TX485_Buffer[0] = RX485_Buffer[0];	//要返回的应答

				TX485_Buffer[1]=0x86;
				TX485_Buffer[2]=0x03;
				
				crc = MODBUS_CRC16(TX485_Buffer, 3);
				TX485_Buffer[3] = (uint8_t)crc;	//CRC是小端模式, 先发低字节，后发高字节。
				TX485_Buffer[4] = (uint8_t)(crc>>8);
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
	


