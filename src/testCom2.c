/*
 * testCom2.c
 *
 *  Created on: 15.11.2010
 *      Author: ricky
 */
#define TEST_OFF
#ifdef TEST_ON

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/communication.h"

DT_leg leg_r, leg_l;
DT_byte Own_CpuID;

void master();
void slave();

DT_byte txArray[DT_RESULT_BUFFER_SIZE];
DT_byte rxArray[DT_RESULT_BUFFER_SIZE];

int main() {
	XM_init_cpu();
	XM_init_dnx();
	XM_init_com();
	//XM_init_remote();


	DT_byte i;
	for (i = 0; i < DT_RESULT_BUFFER_SIZE-3; i++)
		txArray[i] = i;

	XM_LED_OFF

	DNX_getConnectedIDs(&leg_r, &leg_l);
	Own_CpuID = COM_getCpuID(&leg_l);

	if (Own_CpuID == COM_MASTER) {
		master();
	} else {
		slave();
	}

	return 0;
}

void master() {
	DT_byte i = 0, k;
	while (1) {
		while (i < DT_RESULT_BUFFER_SIZE-3) {
			bool byteToBuffer;
			byteToBuffer = USART_TXBuffer_PutByte(&XM_com_data, txArray[i]);
			if (byteToBuffer) {
				i++;
			}
		}
		i = 0;
		UTL_wait(10);
	}
}

void slave() {
	/* Fetch received data as it is received. */
	DT_byte i = 0, l;
	DT_bool first_time = true;
	while (1) {
		while (i < DT_RESULT_BUFFER_SIZE-3) {
			if (USART_RXBufferData_Available(&XM_com_data)) {
				if (first_time) {
					USART_RXBuffer_GetByte(&XM_com_data);
					first_time = false;
				} else {
					rxArray[i] = USART_RXBuffer_GetByte(&XM_com_data);
					i++;
				}
			}
		}
		if (i >= DT_RESULT_BUFFER_SIZE-3) {
			DEBUG_BYTE((rxArray,127))
			for (l = 0; l < DT_RESULT_BUFFER_SIZE-3; l++) {
				if (rxArray[l] != txArray[l]) {
					XM_LED_ON
				}
			}
			i = 0;
		}
	}
}

#endif
