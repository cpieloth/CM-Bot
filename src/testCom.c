/**
 * \file	testCom.c
 *
 * \brief	Testprogramm für Kommunikation der CPUs.
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

int main() {
	XM_init_cpu();
	XM_init_dnx();

	DNX_getConnectedIDs(&leg_r, &leg_l);
	Own_CpuID = COM_getCpuID(&leg_l);

	XM_init_com(Own_CpuID);

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

	if (Own_CpuID == COM_MASTER) {
		master();
	} else {
		slave();
	}

	return 0;
}

void master() {
	DT_bool isAlive;
	do {
			if (COM_isAlive(COM_SLAVE1B)/* && COM_isAlive(COM_SLAVE3)*/) {
				isAlive = true;
			} else
				UTL_wait(5);
		} while (isAlive == false);

	XM_LED_OFF

	DT_point p1, p2;

	p1.x = 77.8553;
	p1.y = 77.8553;
	p1.z = -129.1041;

	p2.x = 95.9985;
	p2.y = -95.9985;
	p2.z = -116.2699;

	DT_bool flag;
	XM_LED_OFF
	while (1) {

		flag = COM_sendPoint(COM_SLAVE1B, &p1);
		if (flag) {
			XM_LED_ON
		} else {
			XM_LED_OFF
		}
		//COM_sendPoint(COM_SLAVE3, &p1);
		COM_sendAction(COM_BRDCAST_ID);
		UTL_wait(40);

		flag = COM_sendPoint(COM_SLAVE1B, &p2);
		if (flag) {
			XM_LED_ON
		} else {
			XM_LED_OFF
		}

		//COM_sendPoint(COM_SLAVE3, &p2);
		COM_sendAction(COM_BRDCAST_ID);

		UTL_wait(40);
	}
}

void slave() {
	DT_size len;
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_point p1;
	DT_bool ans;
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
				DEBUG(("sl_alive",sizeof("sl_alive")))
				XM_LED_ON
				COM_sendACK(COM_MASTER);
				break;
			default:
				break;
			}
			break;
		case COM_ACTION:
			DEBUG(("sl_action", sizeof("sl_action")))
			ans = KIN_makeMovement(&leg_l, &leg_r);
			break;
		case COM_POINT:
			DEBUG(("sl_point", sizeof("sl_point")))
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

#endif /* TEST_ON */
