/**
 * \file	testSpeed.c
 *
 * \brief	Testprogramm für Speed-Änderung der Servos.
 */

#define TEST_OFF
#ifdef TEST_ON

#include <stdio.h>
#include <math.h>
#include "include/kinematics.h"
#include "include/utils.h"
#include "include/evolutionaryHelper.h"
#include "include/evolutionaryAlgorithm.h"
#include "include/dynamixel.h"
#include "include/xmega.h"
#include "include/communication.h"
#include "include/movement.h"
#include "include/remote.h"

DT_leg leg_r, leg_l;
DT_byte MasterActive, SlavesActive, MasterInactive, SlavesInactive;
DT_byte cpuID;

void master();

int main(void) {
	XM_init_cpu();
	XM_init_dnx();
	// INIT: Hole benoetigte Daten
	DNX_getConnectedIDs(&leg_r, &leg_l);
	cpuID = COM_getCpuID(&leg_l);
	XM_init_com(cpuID);

	XM_init_remote();

	XM_LED_OFF

/*
	while(1){
		UTL_wait(40);
		DNX_setAngleAndSpeed(12, 60, 100, false);
		UTL_wait(40);
		DNX_setAngleAndSpeed(12, 100, 100, false);
	}
*/
	switch (cpuID) {
	case COM_MASTER:
		XM_init_remote();
		DEBUG(("Master",sizeof("Master")))
		master();
		break;
	case COM_SLAVE1B:
		DEBUG(("Slave1",sizeof("Slave1")))
		MV_slave(cpuID, &leg_r, &leg_l);
		break;
	case COM_SLAVE3F:
		DEBUG(("Slave3",sizeof("Slave3")))
		MV_slave(cpuID, &leg_r, &leg_l);
		break;
	default: //case NOCPUID:
		DEBUG (("NoCpuID",sizeof("NoCpuID")))
		XM_LED_OFF
		break;
	}
	return 0;
}

DT_point p1, p2;

void init_pMpS() {
	p1.x = 110.1041;
	p1.y = 0;
	p1.z = Z;
	p2.x = 77.8553;
	p2.y = 77.8553;
	p2.z = Z;
}

void move(DT_point * p) {
	DT_double speed = 100;
	MV_pointAndSpeed(&leg_l, p, speed, false);
	MV_pointAndSpeed(&leg_r, p, speed, false);
	DT_byte config = COM_CONF_LEFT | COM_CONF_RIGHT;
	COM_sendPointAndSpeed(COM_SLAVE1B, p, speed, config);
	COM_sendPointAndSpeed(COM_SLAVE3F, p, speed, config);
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

void master() {
	init_pMpS();
	DT_bool flag = true;
	while (1) {
		if (flag) {
			flag = false;
			move(&p1);
		} else {
			flag = true;
			move(&p2);
		}
		UTL_wait(40);
	}
}

#endif /* TEST_ON */
