/**
 * \file	testKin.c
 *
 * \brief	Testprogramm für die Kinematik.
 */

#define TEST_OFF
#ifdef TEST_ON

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/remote.h"
#include "include/communication.h"

DT_leg leg_r, leg_l;
DT_byte cpuID;

int main() {
	XM_init_cpu();
	XM_init_dnx();
	// INIT: Hole benoetigte Daten
	DNX_getConnectedIDs(&leg_r, &leg_l);
	cpuID = COM_getCpuID(&leg_l);
	XM_init_com(cpuID);
	XM_init_remote();
	XM_LED_OFF

	DT_cmd cmd = 0x0000;

	DT_point p1, p2;

	DNX_getConnectedIDs(&leg_r, &leg_l);

	p1.x = 77.8553;
	p1.y = 77.8553;
	p1.z = -129.1041;

	p2.x = 95.9985;
	p2.y = -95.9985;
	p2.z = -116.2699;

	DT_char flag = 0;
	while (1) {
		cmd = RMT_getCommand();
		if (RMT_isButton1Pressed(cmd)) {
			p1.x = p1.x + 1;
		}

		if (RMT_isButton2Pressed(cmd)) {
			p1.x = p1.x - 1;
		}

		if (RMT_isButton3Pressed(cmd)) {
			p1.y = p1.y + 1;
		}

		if (RMT_isButton4Pressed(cmd)) {
			p1.y = p1.y - 1;
		}
		if (RMT_isLeftPressed(cmd)) {
			p1.y = p1.z + 1;
		}
		if (RMT_isRightPressed(cmd)) {
			p1.y = p1.z - 1;
		}
		KIN_calcServos(&p1, &leg_l);
		KIN_calcServos(&p1, &leg_r);

		leg_r.hip.set_value = UTL_getDegree(leg_r.hip.set_value);
		leg_r.knee.set_value = UTL_getDegree(leg_r.knee.set_value);
		leg_r.foot.set_value = UTL_getDegree(leg_r.foot.set_value);

		leg_l.hip.set_value = UTL_getDegree(leg_l.hip.set_value);
		leg_l.knee.set_value = UTL_getDegree(leg_l.knee.set_value);
		leg_l.foot.set_value = UTL_getDegree(leg_l.foot.set_value);

		DNX_setAngle(leg_l.hip.id, leg_l.hip.set_value, false);
		DNX_setAngle(leg_l.knee.id, leg_l.knee.set_value, false);
		DNX_setAngle(leg_l.foot.id, leg_l.foot.set_value, false);

		DNX_setAngle(leg_r.hip.id, leg_r.hip.set_value, false);
		DNX_setAngle(leg_r.knee.id, leg_r.knee.set_value, false);
		DNX_setAngle(leg_r.foot.id, leg_r.foot.set_value, false);
	}
	return 0;
}

#endif /* TEST_ON */
