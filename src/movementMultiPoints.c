/**
 * \file movementMultiPoints.c
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
#include "include/remote.h"
#include <math.h>

DT_leg leg_r, leg_l;
DT_byte cpuID;

#define STEP_SIZE	3

DT_point ma_calcStartPoint(const DT_point* const v, DT_bool isDown,
		const DT_byte side) {
	/* TODO: neu
	 * - alpha = tan(vec.y/vec.x)
	 * - alpha prüfen ob zwischen 30, 15 oder 0 (und -30, -15)
	 * - dementsprechend mit Hilfe von DH-Berechnung die Punkte
	 *   berechnen für 30, 22.5, 15, 7.5 und 0
	 */
	DT_leg leg;
	DT_double alpha = v->y != 0 ? atan(v->y / v->x) : M_PI / 2;
	alpha = UTL_getDegree(alpha);
	if (40 < alpha && alpha < 50)
		alpha = 15;
	else if (-5 < alpha && alpha < 5)
		alpha = 0;
	else if (80 < alpha && alpha < 100)
		alpha = 30;
	alpha = 0;
	leg.hip.set_value = alpha;
	leg.knee.set_value = isDown == true ? 45 : 0;
	leg.hip.set_value = 90;

	DT_double dh03[4][4];
	dh03[0][1] = 0;
	KIN_calcDH(&leg, &dh03[0]);
	DT_point p = UTL_getPointOfDH(&dh03[0]);
	p.x += MV_DST_X;

	if (side == COM_CONF_LEFT) {
		p.x = -p.x;
	}

	// TODO Startpunkt für vektor im Arbeitsraum suchen
	/*
	 * - Arbeitsraum als Kugel erstellen
	 * - Kreis/Fläche aus Kugel und geg. z errechnen -> Arbeitsfläche eingrenzen -> Kreisegment
	 * - In definierter Abeitsfläche (Kreissegment) längste Sekante mit dem Vektor v suchen/berechnen und deren Schnittpunkte -> Startpunkt
	 * - wie neues z fuer Punkt berechnen, da z Abhänhig von x und y
	 */

	/*
	 DT_point p;
	 if (side == COM_CONF_LEFT) {
	 p.x = -(103.4640 + MV_DST_X);
	 p.y = 37.6578;
	 p.z = z;

	 } else {
	 p.x = 103.4640 + MV_DST_X;
	 p.y = 37.6578;
	 p.z = z;

	 }
	 */
	return p;
}

DT_point ma_movePnt(DT_point* const p, const DT_point* const v) {
	// TODO	wie neues z fuer Punkt berechnen, da z Abhänhig von x und y
	DT_point pMv;
	pMv.x = p->x + v->x;
	pMv.y = p->y + v->y;
	pMv.z = p->z + v->z;
	return pMv;
}

DT_point ma_calcUnitVec(const DT_point* const v) {
	// TODO Vektor auf gewuensche Schrittweite umrechnennach der
	DT_point vUnt = *v;
	return vUnt;
}

DT_point ma_negateVec(const DT_point* const v) {
	DT_point vNeg;
	vNeg.x = -v->x;
	vNeg.y = -v->y;
	vNeg.z = -v->z;
	return vNeg;
}

void ma_switchLegs(DT_byte* side, DT_byte* master_dwn, DT_byte* master_up,
		DT_byte* slave_dwn, DT_byte* slave_up) {
	if (*side == COM_CONF_LEFT) {
		*side = COM_CONF_RIGHT;

		*master_dwn = COM_CONF_RIGHT;
		*slave_dwn = COM_CONF_LEFT;

		*master_up = COM_CONF_LEFT;
		*slave_up = COM_CONF_RIGHT;
	} else {
		*side = COM_CONF_LEFT;

		*master_dwn = COM_CONF_LEFT;
		*slave_dwn = COM_CONF_RIGHT;

		*master_up = COM_CONF_RIGHT;
		*slave_up = COM_CONF_LEFT;
	}
}

DT_point ma_getPntForCpuSide(const DT_point* const p, const DT_byte cpuId) {
	DT_point pTmp = *p;
	if (cpuId == COM_SLAVE1B) {
		pTmp.x = -p->x;
		pTmp.y = p->y - MV_DST_Y;
	}
	if (cpuId == COM_SLAVE3F) {
		pTmp.x = -p->x;
		pTmp.y = p->y + MV_DST_Y;
	}
	return pTmp;
}

DT_leg * ma_getLegForSide(DT_byte side) {
	if (side == COM_CONF_LEFT) {
		return &leg_l;
	} else {
		return &leg_r;
	}

}

void master();
void ma_setInitialPoint(DT_point* const );
void ma_changeActiveLeg();
void ma_doStep(const DT_point* const );

int main() {
	XM_init_cpu();
	XM_init_dnx();
	// INIT: Hole benoetigte Daten
	DNX_getConnectedIDs(&leg_r, &leg_l);
	cpuID = COM_getCpuID(&leg_l);
	XM_init_com(cpuID);

	XM_init_remote();

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

	//DT_cmd cmd = 0x0000;

	MV_doInitPosition(&leg_r, &leg_l);

	//DT_double zUp = 101.1041;
	//DT_double zDwn = -129.1041;
	DT_point vDwn, vUp, pDwn, pUp, pTmp;

	vUp = ma_negateVec(&vDwn);
	DT_bool resDwn = true, resUp = true;
	DT_byte side = COM_CONF_LEFT;
	DT_byte config, masterDwn, masterUp, slaveDwn, slaveUp;
	DT_leg leg_up, leg_dwn;
	DT_bool firstStepOfMovement = true;

	ma_switchLegs(&side, &masterDwn, &masterUp, &slaveDwn, &slaveUp);

	leg_dwn = leg_r;
	leg_up = leg_l;

	while (1) {
		XM_LED_ON

		vDwn.x = 0;
		vDwn.y = -5;
		vDwn.z = 0;

		vUp = ma_negateVec(&vDwn);

		vUp = ma_calcUnitVec(&vUp);
		vDwn = ma_calcUnitVec(&vDwn);

		pDwn = ma_calcStartPoint(&vDwn, true, masterDwn);
		pUp = ma_calcStartPoint(&vUp, false, masterUp);

		do {
			// TODO MV_point und COM_send auswerten ob Punkt angefahren werden kann
			config = slaveDwn | COM_CONF_GLOB;
			pTmp = ma_getPntForCpuSide(&pDwn, COM_SLAVE1B);
			resDwn = resDwn && COM_sendPoint(COM_SLAVE1B, &pTmp, config);
			pTmp = ma_getPntForCpuSide(&pDwn, COM_SLAVE3F);
			resDwn = resDwn && COM_sendPoint(COM_SLAVE3F, &pTmp, config);
			pTmp = ma_getPntForCpuSide(&pDwn, COM_MASTER);
			//resDwn = resDwn && MV_point(&leg_dwn, &pTmp, false);
			MV_point(ma_getLegForSide(masterDwn), &pDwn, true);

			if (firstStepOfMovement == true) {
				firstStepOfMovement = false;
				if (resUp && resDwn) {
					MV_action(&leg_r, &leg_l);
					COM_sendAction(COM_BRDCAST_ID);
				}
				UTL_wait(5);
			}

			config = slaveUp | COM_CONF_GLOB;
			pTmp = ma_getPntForCpuSide(&pUp, COM_SLAVE1B);
			resUp = resUp && COM_sendPoint(COM_SLAVE1B, &pTmp, config);
			pTmp = ma_getPntForCpuSide(&pUp, COM_SLAVE3F);
			resUp = resUp && COM_sendPoint(COM_SLAVE3F, &pTmp, config);
			pTmp = ma_getPntForCpuSide(&pUp, COM_MASTER);
			//resUp = resUp && MV_point(&leg_up, &pTmp, false);
			MV_point(ma_getLegForSide(masterUp), &pUp, true);

			if (resUp && resDwn) {
				MV_action(&leg_r, &leg_l);
				COM_sendAction(COM_BRDCAST_ID);
			}

			//UTL_wait(20);

			pDwn = ma_movePnt(&pDwn, &vDwn);
			pUp = ma_movePnt(&pUp, &vUp);
		} while (resDwn && resUp);
		DEBUG(("switch_leg",sizeof("switch_leg")))
		ma_switchLegs(&side, &masterDwn, &masterUp, &slaveDwn, &slaveUp);
		resDwn = true;
		resUp = true;
		firstStepOfMovement = true;
		// TODO wechsle beine physisch

		/*
		 cmd = RMT_getCommand();

		 // Vorwärts & Rückwärts
		 if (RMT_isUpPressed(cmd)) {
		 DEBUG(("UP",sizeof("UP")))
		 p1.y = p1.y + STEP_SIZE;
		 }
		 if (RMT_isDownPressed(cmd)) {
		 DEBUG(("DWN",sizeof("DWN")))
		 p1.y = p1.y - STEP_SIZE;
		 }
		 // Links & Rechts
		 if (RMT_isLeftPressed(cmd)) {
		 DEBUG(("LFT",sizeof("LFT")))
		 p1.x = p1.x - STEP_SIZE;
		 }
		 if (RMT_isRightPressed(cmd)) {
		 DEBUG(("RGT",sizeof("RGT")))
		 p1.x = p1.x + STEP_SIZE;
		 }
		 // Hoch & Runter
		 if (RMT_isButton1Pressed(cmd)) {
		 DEBUG(("B1",sizeof("B1")))
		 p1.z = p1.z + STEP_SIZE;
		 }
		 if (RMT_isButton3Pressed(cmd)) {
		 DEBUG(("B3",sizeof("B3")))
		 p1.z = p1.z - STEP_SIZE;
		 }

		 ma_doStep(&p1);
		 */
	}
}

void ma_doStep(const DT_point* const point) {
	DT_byte config;

	config = COM_CONF_LEFT | COM_CONF_RIGHT;
	COM_sendPoint(COM_SLAVE1B, point, config);
	COM_sendPoint(COM_SLAVE3F, point, config);

	MV_point(&leg_l, point, false);
	MV_point(&leg_r, point, false);

	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

void ma_setInitialPoint(DT_point* const initPoint) {
	initPoint->x = 77.8553;
	initPoint->y = 77.8553;
	initPoint->z = -129.1041;
}

#endif /* TEST_ON */
