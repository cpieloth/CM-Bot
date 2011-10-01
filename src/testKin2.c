/*
 * testKin2.c
 *
 *  Created on: 20.11.2010
 *      Author: christof
 */

#define TEST_OFF TEST
#ifdef TEST_ON

#include "include/datatypes.h"
#include "include/kinematics.h"
#include "include/utils.h"
#include <stdio.h>

#define SL_DST_Y	208.5
#define DST_X	168.5

DT_leg leg_r, leg_l;
DT_byte cpuID;
DT_transformation trans_r, trans_l;

int main() {
	// Bein vorne
	leg_r.hip.id = 1;
	leg_r.knee.id = 2;
	leg_r.foot.id = 3;

	leg_l.hip.id = 4;
	leg_l.knee.id = 5;
	leg_l.foot.id = 6;

	trans_r = KIN_getTransMat(&leg_r);
	trans_l = KIN_getTransMat(&leg_l);

	// vorne rechts
	DT_point pGlobal_r, pLocal_r;
	pGlobal_r.x = 103.4640 + DST_X;
	pGlobal_r.y = 37.6578 + SL_DST_Y;
	pGlobal_r.z = -129.1041;
	//pGlobal_r.z = 101.1041;

	printf("pGlobal_r ");
	UTL_printPoint(&pGlobal_r);

	pLocal_r = KIN_calcLocalPoint(&pGlobal_r, &trans_r);
	printf("pLocal_r ");
	UTL_printPoint(&pLocal_r);

	KIN_calcServos(&pLocal_r, &leg_r);
	printf("leg_r ");
	UTL_printLeg(&leg_r, UTL_DEG);

	printf("\n");

	// vorne links
	DT_point pGlobal_l, pLocal_l;
	pGlobal_l.x = -(103.4640 + DST_X);
	pGlobal_l.y = 37.6578 + SL_DST_Y;
	pGlobal_l.z = -129.1041;
	//pGlobal_l.z = 101.1041;

	printf("pGlobal_l ");
	UTL_printPoint(&pGlobal_l);

	pLocal_l = KIN_calcLocalPoint(&pGlobal_l, &trans_l);
	printf("pLocal_l ");
	UTL_printPoint(&pLocal_l);

	KIN_calcServos(&pLocal_l, &leg_l);
	printf("leg_l ");
	UTL_printLeg(&leg_l, UTL_DEG);

	return 0;
}

#endif /* TEST_ON */
