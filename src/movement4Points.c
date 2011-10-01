/**
 * \file movement4Points.c
 *
 *  \brief	Algorithmus fuer das Vorwaertslaufen ueber 4 Punkte.
 */

#define TEST_OFF TEST
#ifdef TEST_ON

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/communication.h"
#include "include/movement.h"

DT_leg leg_r, leg_l;
DT_byte cpuID;

void master();

DT_leg * ma_getLegForSide(DT_byte side) {
	if (side == COM_CONF_LEFT) {
		return &leg_l;
	} else {
		return &leg_r;
	}
}

void ma_setPoints(DT_point* const pFntDwn, DT_point* const pFntUp,
		DT_point* const pBckUp, DT_point* const pBckDwn) {
	XM_LED_OFF
	// Fix-Koordinaten fuer Master, Berechnung fuer Slaves ueber Offset
	pFntUp->x = 150.59391 + MV_DST_X;
	pFntUp->y = 86.94544;
	pFntUp->z = -52.89087;

	pFntDwn->x = 95.35293 + MV_DST_X;
	pFntDwn->y = 55.05204;
	pFntDwn->z = -129.10408;

	pBckUp->x = 150.59391 + MV_DST_X;
	pBckUp->y = -86.94544;
	pBckUp->z = -52.89087;

	pBckDwn->x = 95.35293 + MV_DST_X;
	pBckDwn->y = -55.05204;
	pBckDwn->z = -129.10408;

	DEBUG (("ma_set_pnt",sizeof("ma_set_pnt")))
	XM_LED_ON
}

int main() {
	XM_init_cpu();
	XM_init_dnx();
	// INIT: Hole benoetigte Daten
	DNX_getConnectedIDs(&leg_r, &leg_l);
	cpuID = COM_getCpuID(&leg_l);
	XM_init_com(cpuID);

	XM_LED_OFF

	KIN_setTransMat(&leg_r);
	KIN_setTransMat(&leg_l);

	XM_LED_ON

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

/* ___ Methoden fuer Master ___ */
void master() {
	DEBUG(("ma_chk_al",sizeof("ma_chk_al")))
	MV_masterCheckAlive();

	DEBUG(("ma_int_pnt",sizeof("ma_int_pnt")))
	DT_point pFntDwn, pFntUp, pBckUp, pBckDwn, pTmp;
	ma_setPoints(&pFntDwn, &pFntUp, &pBckUp, &pBckDwn);

	DT_byte side = COM_CONF_LEFT;
	DT_byte config, masterDwn, masterUp, slaveDwn, slaveUp;
	MV_switchLegs(&side, &masterDwn, &masterUp, &slaveDwn, &slaveUp);

	DT_byte state = 0;

	// Automat: 0(->1->2)+
	while (1) {
		XM_LED_ON
		switch (state) {
		case 0:
			DEBUG(("ma_int_pos",sizeof("ma_int_pos")))
			MV_doInitPosition(&leg_r, &leg_l);
			UTL_wait(30);
			state = 1;
			break;
		case 1:
			UTL_wait(5);

			pTmp = MV_getPntForCpuSide(&pFntDwn, COM_MASTER, masterDwn);
			MV_point(ma_getLegForSide(masterDwn), &pTmp, true);
			config = slaveDwn | COM_CONF_GLOB;
			pTmp = MV_getPntForCpuSide(&pFntDwn, COM_SLAVE1B, slaveDwn);
			COM_sendPoint(COM_SLAVE1B, &pTmp, config);
			pTmp = MV_getPntForCpuSide(&pFntDwn, COM_SLAVE3F, slaveDwn);
			COM_sendPoint(COM_SLAVE3F, &pTmp, config);

			MV_action(&leg_r, &leg_l);
			COM_sendAction(COM_BRDCAST_ID);

			UTL_wait(5);

			pTmp = MV_getPntForCpuSide(&pBckUp, COM_MASTER, masterUp);
			MV_point(ma_getLegForSide(masterUp), &pTmp, true);
			config = slaveUp | COM_CONF_GLOB;
			pTmp = MV_getPntForCpuSide(&pBckUp, COM_SLAVE1B, slaveUp);
			COM_sendPoint(COM_SLAVE1B, &pTmp, config);
			pTmp = MV_getPntForCpuSide(&pBckUp, COM_SLAVE3F, slaveUp);
			COM_sendPoint(COM_SLAVE3F, &pTmp, config);

			MV_action(&leg_r, &leg_l);
			COM_sendAction(COM_BRDCAST_ID);

			state = 2;
			break;
		case 2:
			pTmp = MV_getPntForCpuSide(&pBckDwn, COM_MASTER, masterDwn);
			MV_point(ma_getLegForSide(masterDwn), &pTmp, true);
			config = slaveDwn | COM_CONF_GLOB;
			pTmp = MV_getPntForCpuSide(&pBckDwn, COM_SLAVE1B, slaveDwn);
			COM_sendPoint(COM_SLAVE1B, &pTmp, config);
			pTmp = MV_getPntForCpuSide(&pBckDwn, COM_SLAVE3F, slaveDwn);
			COM_sendPoint(COM_SLAVE3F, &pTmp, config);

			pTmp = MV_getPntForCpuSide(&pFntUp, COM_MASTER, masterUp);
			MV_point(ma_getLegForSide(masterUp), &pTmp, true);
			config = slaveUp | COM_CONF_GLOB;
			pTmp = MV_getPntForCpuSide(&pFntUp, COM_SLAVE1B, slaveUp);
			COM_sendPoint(COM_SLAVE1B, &pTmp, config);
			pTmp = MV_getPntForCpuSide(&pFntUp, COM_SLAVE3F, slaveUp);
			COM_sendPoint(COM_SLAVE3F, &pTmp, config);

			MV_action(&leg_r, &leg_l);
			COM_sendAction(COM_BRDCAST_ID);

			MV_switchLegs(&side, &masterDwn, &masterUp, &slaveDwn, &slaveUp);

			state = 1;
			break;
		default:
			DEBUG(("ma_err",sizeof("ma_err")))
			break;
		}
	}
}

#endif /* TEST_ON */
