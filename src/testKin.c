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
#include "include/communication.h"
#include <math.h>

int main() {
	XM_init_cpu();
	XM_init_dnx();

	XM_LED_ON

	DT_leg leg_r, leg_l;
	DT_point p1, p2;

	DNX_getConnectedIDs(&leg_r, &leg_l);
	/*leg_r.hip.id = 13;
	 leg_r.knee.id = 14;
	 leg_r.foot.id = 15;
	 leg_l.hip.id = 16;
	 leg_l.knee.id = 17;
	 leg_l.foot.id = 18;
	 */
	//COM_getCpuID(&leg_r);

	p1.x = 77.8553;
	p1.y = 77.8553;
	p1.z = -129.1041;

	p2.x = 95.9985;
	p2.y = -95.9985;
	p2.z = -116.2699;

	DT_char flag = 0;
	while (1) {
		if (flag == 0) {
			KIN_calcServos(&p1, &leg_l);
			KIN_calcServos(&p1, &leg_r);
			/*leg_r.hip.set_value = 00;
			leg_r.knee.set_value = 0;
			leg_r.foot.set_value = 0;*/
			flag = 1;
			XM_LED_ON
		} else {
			KIN_calcServos(&p2, &leg_l);
			KIN_calcServos(&p2, &leg_r);
			/*leg_r.hip.set_value = M_PI / 8;
			leg_r.knee.set_value = M_PI / 8;
			leg_r.foot.set_value = M_PI / 8;*/
			flag = 0;
			XM_LED_OFF
		}

		leg_r.hip.set_value = UTL_getDegree(leg_r.hip.set_value);
		leg_r.knee.set_value = UTL_getDegree(leg_r.knee.set_value);
		leg_r.foot.set_value = UTL_getDegree(leg_r.foot.set_value);
/*
		leg_l.hip.set_value = UTL_getDegree(leg_l.hip.set_value);
		leg_l.knee.set_value = UTL_getDegree(leg_l.knee.set_value);
		leg_l.foot.set_value = UTL_getDegree(leg_l.foot.set_value);
*/
		DNX_setAngle(leg_r.hip.id, leg_r.hip.set_value, true);
		DNX_setAngle(leg_r.knee.id, leg_r.knee.set_value, true);
		DNX_setAngle(leg_r.foot.id, leg_r.foot.set_value, true);

		/*
		 DNX_setAngle(leg_l.hip.id, leg_l.hip.set_value, true);
		 DNX_setAngle(leg_l.knee.id, leg_l.knee.set_value, true);
		 DNX_setAngle(leg_l.foot.id, leg_l.foot.set_value, true);
		 */
		DNX_sendAction(DNX_BRDCAST_ID);

		UTL_wait(20);
	}
	XM_LED_OFF

	return 0;
}

#endif /* TEST_ON */
