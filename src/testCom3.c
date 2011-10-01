/*
 * testCom3.c
 *
 *  Created on: 15.11.2010
 *      Author: ricky
 */
#define TEST_OFF TEST
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

DT_size len;
DT_byte result[DT_RESULT_BUFFER_SIZE];
DT_point p1;
DT_bool ans;

DT_byte txArray[DT_RESULT_BUFFER_SIZE];
DT_byte rxArray[DT_RESULT_BUFFER_SIZE];

int main() {
	XM_init_cpu();
	XM_init_dnx();

	DNX_getConnectedIDs(&leg_r, &leg_l);
	Own_CpuID = COM_getCpuID(&leg_l);

	XM_init_com(Own_CpuID);
	//XM_init_remote();

	switch (Own_CpuID) {
	case COM_MASTER:
		DEBUG(("Master",sizeof("Master")))
		break;
	case COM_SLAVE1B:
		DEBUG(("Slave1",sizeof("Slave1")))
		break;
	case COM_SLAVE3F:
		DEBUG(("Slave3",sizeof("Slave3")))
		break;
	default: //case NOCPUID:
		DEBUG(("NoCpuID",sizeof("NoCpuID")))
		break;
	}

	DT_byte i;
	for (i = 0; i < DT_RESULT_BUFFER_SIZE - 3; i++)
		txArray[i] = i;

	XM_LED_OFF

	if (Own_CpuID == COM_MASTER) {
		DEBUG(("master",sizeof("master")))
		master();
	} else {
		DEBUG(("slave",sizeof("slave")))
		slave();
	}

	return 0;
}

void master() {
	XM_LED_OFF
	while (1) {
		DEBUG(("pre_alive",sizeof("pre_alive")))
		if (COM_isAlive(COM_SLAVE1B)) {
			XM_LED_ON
		}
		DEBUG(("aft_alive",sizeof("aft_alive")))
	}
}

void slave() {
	XM_LED_OFF
	while (1) {
		len = COM_receive(&XM_com_data3, result);
		if (len == 0)
			continue;
		DEBUG(("sl_pck_rec",sizeof("sl_pck_rec")))

		if (result[2] != Own_CpuID && result[2] != COM_BRDCAST_ID)
			continue;
		DEBUG(("sl_for_me",sizeof("sl_for_me")))

		switch (result[4]) {
		case COM_STATUS:
			switch (result[5]) {
			case COM_IS_ALIVE:
				XM_LED_ON
				DEBUG(("rec_alive",sizeof("rec_alive")))
				COM_sendACK(COM_MASTER);
				DEBUG(("aft_ack",sizeof("aft_ack")))
				break;
			default:
				break;
			}
			break;
		case COM_ACTION:
			ans = KIN_makeMovement(&leg_l, &leg_r);
			break;
		case COM_POINT:
			// point aus Paket lesen
			p1 = COM_getPointFromPacket(result);

			KIN_calcServos(&p1, &leg_l);
			KIN_calcServos(&p1, &leg_r);

			COM_sendACK(COM_MASTER);
			break;

		default:
			// ERROR
			break;
		}
	}

}

#endif
