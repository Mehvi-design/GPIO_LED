/*
 * Function.h
 */

#ifndef INC_FUNCTION_H_
#define INC_FUNCTION_H_

#include <stdint.h>
#include "main.h"
#include "CtrlLib.h"
typedef struct{
	uint16_t Raw;
	float Filted;
	first_order_transfer_func_t filter;
	float Upper_Limit;
	float Lower_Limit;
	float Real;
	float Gain;
	float Offset;
}value_t;
void UART_SendByte(uint8_t byte);
void UART_SendUint16AsHex(uint16_t value);
void UART_SendFloat(float value);
void setup(void);
void loop(void);
void MODS_03H(void);
void MODS_06H(void);
void MODS_SendAckErr(void);
void RS485_MODBUS_RTU_req_pdu(void);

#endif /* INC_FUNCTION_H_ */
