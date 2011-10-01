/**
 * \file	testDnx.c
 *
 * \brief	Testprogramm für Ansteuerung der Servos.
 */

#include "include/xmega.h"
#include "include/utils.h"
#include "include/dynamixel.h"

#define TEST_OFF
#ifdef TEST_ON

#define TIMEOUT_MAX 1

int main() {
	XM_init_cpu();
	XM_init_dnx();

	byte received_Data_L[XM_RX_BUFFER_SIZE];
	byte received_Data_R[XM_RX_BUFFER_SIZE];

	byte id;
	uint16_t angle1 = 50;
	uint16_t angle2 = 250;

	byte len = 0;
	int i;

	DEBUG(("DNX test start", sizeof("DNX test start")))
	id = 0x0C;
	for (i = 0; i < 2; i++) {
		UTL_wait(20);
		DNX_setAngle(id, angle1);
		len = 0;
		while (len == 0)
			len = XM_USART_receive(&XM_RX_buffer_L, received_Data_L);
		DEBUG_BYTE((received_Data_L, len))

		UTL_wait(20);
		DNX_setAngle(id, angle2);
		len = 0;
		while (len == 0)
			len = XM_USART_receive(&XM_RX_buffer_L, received_Data_L);
		DEBUG_BYTE((received_Data_L, len))
	}

	id = 0x09;
	for (i = 0; i < 2; i++) {
		UTL_wait(20);
		DNX_setAngle(id, angle1);
		len = 0;
		while (len == 0)
			len = XM_USART_receive(&XM_RX_buffer_R, received_Data_R);
		DEBUG_BYTE((received_Data_R, len))

		UTL_wait(20);
		DNX_setAngle(id, angle2);
		len = 0;
		while (len == 0)
			len = XM_USART_receive(&XM_RX_buffer_R, received_Data_R);
		DEBUG_BYTE((received_Data_R, len))
	}

	DEBUG(("finish", sizeof("finish")))
	while (1)
		return 0;
}

#endif /* TEST_ON */
