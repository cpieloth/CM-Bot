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

// TODO Versatz eintragen
#define SL_DST_Y	208.5
#define DST_X	168.5

DT_leg leg_r, leg_l;
DT_byte cpuID;

void master();
void ma_doInitPosition();
void ma_setPoints(DT_point* const , DT_point* const , DT_point* const ,
		DT_point* const );
void ma_prepareStep(DT_point* const , DT_point* const , const DT_bool);
void ma_doStep(DT_point* const , DT_point* const , const DT_bool);

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
	DT_point pFntDwn, pFntUp, pBckUp, pBckDwn;
	ma_setPoints(&pFntDwn, &pFntUp, &pBckUp, &pBckDwn);

	DT_byte state = 0;
	DT_point p1, p2;
	DT_byte config;

	p1.x = 77.8553;
	p1.y = 77.8553;
	p1.z = -129.1041;

	p2.x = 95.9985;
	p2.y = -95.9985;
	p2.z = -116.2699;

	state = 2;
	// Automat: 0(->1->2->3->4)+
	while (1) {
		XM_LED_ON
		switch (state) {
		case 0:
			DEBUG(("ma_int_pos",sizeof("ma_int_pos")))
			ma_doInitPosition();
			UTL_wait(40);
			state = 1;
			break;
		case 1:
			config = COM_CONF_LEFT;
			COM_sendPoint(COM_SLAVE1B, &p1, config);
			config = COM_CONF_RIGHT;
			COM_sendPoint(COM_SLAVE1B, &p1, config);
			COM_sendAction(COM_SLAVE1B);
			MV_point(&leg_l, &p1, false);
			MV_point(&leg_r, &p1, false);
			MV_action(&leg_r, &leg_l);
			UTL_wait(40);
			state = 2;
			break;
		case 2:
			config = COM_CONF_LEFT;
			COM_sendPoint(COM_SLAVE3F, &p1, config);
			config = COM_CONF_RIGHT;
			COM_sendPoint(COM_SLAVE3F, &p1, config);
			COM_sendAction(COM_SLAVE3F);
			UTL_wait(40);
			state = 4;
			break;
		case 3:
			MV_point(&leg_l, &p2, false);
			MV_point(&leg_r, &p2, false);
			MV_action(&leg_r, &leg_l);
			config = COM_CONF_LEFT;
			COM_sendPoint(COM_SLAVE1B, &p2, config);
			config = COM_CONF_RIGHT;
			COM_sendPoint(COM_SLAVE1B, &p2, config);
			COM_sendAction(COM_SLAVE1B);
			UTL_wait(40);
			state = 4;
			break;
		case 4:
			config = COM_CONF_LEFT;
			COM_sendPoint(COM_SLAVE3F, &p2, config);
			config = COM_CONF_RIGHT;
			COM_sendPoint(COM_SLAVE3F, &p2, config);
			COM_sendAction(COM_SLAVE3F);
			UTL_wait(40);
			state = 2;
			break;
		default:
			DEBUG(("ma_err",sizeof("ma_err")))
			break;
		}
	}
}

void ma_prepareStep(DT_point* const pDwn, DT_point* const pUp, DT_bool right) {
	DT_byte config;
	DT_point pTmp;
	// F_R, B_R, M_L: vorne unten
	pTmp = *pDwn;
	if (right) {
		pTmp.x = -pDwn->x;
		config = COM_CONF_LEFT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	}
	pTmp.y += SL_DST_Y;
	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	pTmp = *pDwn;
	if (right) {
		pTmp.x = -pDwn->x;
		config = COM_CONF_LEFT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	}
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	pTmp = *pDwn;
	if (right) {
		MV_point(&leg_r, &pTmp, true);
	} else {
		pTmp.x = -pDwn->x;
		MV_point(&leg_l, &pTmp, true);
	}

	UTL_wait(20);
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);

	// F_L, B_L, M_R: hinten oben
	pTmp = *pUp;
	if (right) {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_LEFT | COM_CONF_GLOB;
		pTmp.x = -pUp->x;
	}
	pTmp.y += SL_DST_Y;

	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	pTmp = *pUp;
	if (right) {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_LEFT | COM_CONF_GLOB;
		pTmp.x = -pUp->x;
	}
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	pTmp = *pUp;
	if (right) {
		pTmp.x = -pDwn->x;
		MV_point(&leg_l, &pTmp, true);
	} else {
		MV_point(&leg_r, &pTmp, true);
	}

	UTL_wait(20);
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

void ma_doStep(DT_point* const pDwn, DT_point* const pUp, DT_bool right) {
	DT_byte config;
	DT_point pTmp;
	// F_R, B_R, M_L: vorne unten
	pTmp = *pDwn;
	if (right) {
		pTmp.x = -pDwn->x;
		config = COM_CONF_LEFT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	}
	pTmp.y += SL_DST_Y;
	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	pTmp = *pDwn;
	if (right) {
		pTmp.x = -pDwn->x;
		config = COM_CONF_LEFT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	}
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	pTmp = *pDwn;
	if (right) {
		MV_point(&leg_r, &pTmp, true);
	} else {
		pTmp.x = -pDwn->x;
		MV_point(&leg_l, &pTmp, true);
	}

	UTL_wait(20);
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);

	// F_L, B_L, M_R: hinten oben
	pTmp = *pUp;
	if (right) {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_LEFT | COM_CONF_GLOB;
		pTmp.x = -pUp->x;
	}
	pTmp.y += SL_DST_Y;

	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	pTmp = *pUp;
	if (right) {
		config = COM_CONF_RIGHT | COM_CONF_GLOB;
	} else {
		config = COM_CONF_LEFT | COM_CONF_GLOB;
		pTmp.x = -pUp->x;
	}
	pTmp.y -= SL_DST_Y;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	pTmp = *pUp;
	if (right) {
		pTmp.x = -pDwn->x;
		MV_point(&leg_l, &pTmp, true);
	} else {
		MV_point(&leg_r, &pTmp, true);
	}

	UTL_wait(20);
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

void ma_setPoints(DT_point* const pFntDwn, DT_point* const pFntUp,
		DT_point* const pBckUp, DT_point* const pBckDwn) {
	XM_LED_OFF
	// Fix-Koordinaten fuer Master, Berechnung fuer Slaves ueber Offset
	pFntUp->x = 103.4640 + DST_X;
	pFntUp->y = 37.6578;
	pFntUp->z = 101.1041;

	pFntDwn->x = 103.4640 + DST_X;
	pFntDwn->y = 37.6578;
	pFntDwn->z = -129.1041;

	pBckUp->x = 103.4640 + DST_X;
	pBckUp->y = -37.6578;
	pBckUp->z = 101.1041;

	pBckDwn->x = 103.4640 + DST_X;
	pBckDwn->y = -37.6578;
	pBckDwn->z = -129.1041;

	DEBUG (("ma_set_pnt",sizeof("ma_set_pnt")))
	XM_LED_ON
}

void ma_doInitPosition() {
	XM_LED_OFF
	DT_byte config;
	// Punkt fuer Null-Stellung mittleres rechtes Bein als Bezug
	DT_point pTmp;
	pTmp.x = 190 + DST_X;
	pTmp.y = 0;
	pTmp.z = -14;
	const DT_point pNull = pTmp;

	// Master rechts
	// pTmp = pNull;
	// pTmp.x = pNull.x;
	// pTmp.y = pNull.y + 0;
	MV_point(&leg_r, &pTmp, true);

	// Master links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	// pTmp.y = pNull.y + 0;
	MV_point(&leg_l, &pTmp, true);

	// Slave3 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y + SL_DST_Y; // +/- pruefen
	config = COM_CONF_RIGHT | COM_CONF_GLOB;
	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	// Slave3 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y + SL_DST_Y; // +/- pruefen
	config = COM_CONF_LEFT | COM_CONF_GLOB;
	//COM_sendPoint(COM_SLAVE3, &pTmp, config);

	// Slave1 rechts
	pTmp = pNull;
	// pTmp.x = pNull.x;
	pTmp.y = pNull.y - SL_DST_Y; // +/- pruefen
	config = COM_CONF_RIGHT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	// Slave1 links
	pTmp = pNull;
	pTmp.x = -pNull.x;
	pTmp.y = pNull.y - SL_DST_Y; // +/- pruefen
	config = COM_CONF_LEFT | COM_CONF_GLOB;
	COM_sendPoint(COM_SLAVE1B, &pTmp, config);

	MV_action(&leg_r, &leg_l);
	COM_sendAction(COM_BRDCAST_ID);

	DEBUG(("ma_int_pos_ok",sizeof("ma_int_pos_ok")))
	XM_LED_ON
}

#endif /* TEST_ON */
