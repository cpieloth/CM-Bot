/**
 * \file	evolutionaryWalking.c
 *
 * \brief	Testprogramm für Evolutionären Algorithmus zur Startpunktfindung.
 */

#define TEST_ON
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

#define OFFSET 50
#define NO_OFFSET 0

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

	switch (cpuID) {
	case COM_MASTER:
		XM_init_remote();
		DEBUG(("Master",sizeof("Master")))
		master();
		break;
	case COM_SLAVE1B:
		DEBUG(("Slave1",sizeof("Slave1")))
		//MV_slave(cpuID, &leg_r, &leg_l);
		break;
	case COM_SLAVE3F:
		DEBUG(("Slave3",sizeof("Slave3")))
		//MV_slave(cpuID, &leg_r, &leg_l);
		break;
	default: //case NOCPUID:
		DEBUG (("NoCpuID",sizeof("NoCpuID")))
		XM_LED_OFF
		break;
	}
	return 0;
}

void invertVector(DT_vector * v) {
	v->x = -v->x;
	v->y = -v->y;
}

DT_point copyPoint(const DT_point * const p) {
	DT_point pNew;
	pNew.x = p->x;
	pNew.y = p->y;
	pNew.z = p->z;
	return pNew;
}

void switchLegs() {
	if (MasterActive == COM_CONF_LEFT) {
		MasterActive = COM_CONF_RIGHT;
		SlavesActive = COM_CONF_LEFT;
		MasterInactive = COM_CONF_LEFT;
		SlavesInactive = COM_CONF_RIGHT;
	} else {
		MasterActive = COM_CONF_LEFT;
		SlavesActive = COM_CONF_RIGHT;
		MasterInactive = COM_CONF_RIGHT;
		SlavesInactive = COM_CONF_LEFT;
	}
}

void initConf() {
	MasterActive = COM_CONF_LEFT;
	SlavesActive = COM_CONF_RIGHT;
	MasterInactive = COM_CONF_RIGHT;
	SlavesInactive = COM_CONF_LEFT;
}

void TripodGaitMove(DT_point* pM, DT_point* pS, const DT_double speed,
		const DT_double offset) {
	DT_double z;
	z = pM->z;
	if (MasterActive == COM_CONF_LEFT) {
		MV_pointAndSpeed(&leg_l, pM, speed, false);
		pM->z += offset;
		MV_pointAndSpeed(&leg_r, pM, speed, false);
	} else {
		MV_pointAndSpeed(&leg_r, pM, speed, false);
		pM->z += offset;
		MV_pointAndSpeed(&leg_l, pM, speed, false);
	}
	pM->z = z;
	z = pS->z;
	COM_sendPointAndSpeed(COM_SLAVE1B, pS, speed, SlavesActive);
	COM_sendPointAndSpeed(COM_SLAVE3F, pS, speed, SlavesActive);
	pS->z += offset;
	COM_sendPointAndSpeed(COM_SLAVE1B, pS, speed, SlavesInactive);
	COM_sendPointAndSpeed(COM_SLAVE3F, pS, speed, SlavesInactive);
	pS->z = z;
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

DT_point pM, pS, midM, midS, isectM, isectS, pMiddle;

void init_pMpSpMiddle() {
	pM.x = 110.1041;
	pM.y = 0;
	pM.z = Z;
	pS.x = 110.1041;
	pS.y = 0;
	pS.z = Z;
	pMiddle.x = 110.1041;
	pMiddle.y = 0;
	pMiddle.z = Z;
}

void calculateMovementPoints() {
	DT_double distM = getDistance(&pM, &isectM);
	DT_double distS = getDistance(&pS, &isectS);
	// Bewegung anhand von 3 Punkten auf einer geraden
	DT_double ratio;
	DT_point distV;
	if (distM <= distS) {
		// Mittelpunkt berechnen
		midM.x = (pM.x + isectM.x) / 2;
		midM.y = (pM.y + isectM.y) / 2;
		midM.z = Z;
		// Verhältnis der beiden Abstände wird zur Mittel- und Endpunktsberechnung verwendet
		ratio = distM / distS;
		distV.x = isectS.x - pS.x;
		distV.y = isectS.y - pS.y;
		isectS.x = pS.x + ratio * distV.x;
		isectS.y = pS.y + ratio * distV.y;
		isectS.z = Z;
		midS.x = (pS.x + isectS.x) / 2;
		midS.y = (pS.y + isectS.y) / 2;
		midS.z = Z;
	} else {
		midS.x = (pS.x + isectS.x) / 2;
		midS.y = (pS.y + isectS.y) / 2;
		midS.z = Z;
		// Verhältnis der beiden Abstände wird zur Mittel- und Endpunktsberechnung verwendet
		ratio = distS / distM;
		distV.x = isectM.x - pM.x;
		distV.y = isectM.y - pM.y;
		isectM.x = pM.x + ratio * distV.x;
		isectM.y = pM.y + ratio * distV.y;
		isectM.z = Z;
		midM.x = (pM.x + isectM.x) / 2;
		midM.y = (pM.y + isectM.y) / 2;
		midM.z = Z;
	}
}

void doStepMove(DT_point* pM, DT_point* pS, const DT_double speed) {
	if (MasterActive == COM_CONF_LEFT) {
		MV_pointAndSpeed(&leg_l, pM, speed, false);
	} else {
		MV_pointAndSpeed(&leg_r, pM, speed, false);
	}
	COM_sendPointAndSpeed(COM_SLAVE1B, pS, speed, SlavesActive);
	COM_sendPointAndSpeed(COM_SLAVE3F, pS, speed, SlavesActive);
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

void prepareStepMove(DT_point* pM, DT_point* pS, const DT_double speed,
		const DT_double offset) {
	DT_double z;
	z = pM->z;
	if (MasterInactive == COM_CONF_LEFT) {
		pM->z += offset;
		MV_pointAndSpeed(&leg_l, pM, speed, false);
	} else {
		pM->z += offset;
		MV_pointAndSpeed(&leg_r, pM, speed, false);
	}
	pM->z = z;
	z = pS->z;
	pS->z += offset;
	COM_sendPointAndSpeed(COM_SLAVE1B, pS, speed, SlavesInactive);
	COM_sendPointAndSpeed(COM_SLAVE3F, pS, speed, SlavesInactive);
	pS->z = z;
	COM_sendAction(COM_BRDCAST_ID);
	MV_action(&leg_r, &leg_l);
}

void doStep(const DT_double speed) {

}

void evolutionaryCalculation(DT_vector * v, const DT_double speed) {
	DT_individuum A, B;
	if (MasterActive == COM_CONF_LEFT)
		invertVector(v);
	A = evolutionaryAlgorithm(10, 5, v);
	pM = getPointFromIndividuum(&A);
	isectM = getIsectFromIndividuum(&A);
	invertVector(v);
	B = evolutionaryAlgorithm(10, 5, v);
	pS = getPointFromIndividuum(&B);
	isectS = getIsectFromIndividuum(&B);
}

void waitForButton3() {
	DT_cmd cmd;
	do {
		cmd = RMT_getCommand();
	} while (!RMT_isButton3Pressed(cmd));
}

void master() {
	DT_cmd cmd;
	DT_vector v;
	DT_double speed = 200;
	init_pMpSpMiddle();
	initConf();
	DT_point pM_old, pS_old;

	// Alle Beine auf dem Boden
	TripodGaitMove(&pM, &pS, speed, 0);
	pM_old = copyPoint(&pM);
	pS_old = copyPoint(&pS);
	while (1) {
		// Lokaler Vektor
		v.x = 0;
		v.y = 0;

		do {
			cmd = RMT_getCommand();
			if (RMT_isUpPressed(cmd))
				v.y += 10;
			if (RMT_isDownPressed(cmd))
				v.y -= 10;
			if (RMT_isRightPressed(cmd))
				v.x += 10;
			if (RMT_isLeftPressed(cmd))
				v.x -= 10;
			if (RMT_isButton1Pressed(cmd))
				TripodGaitMove(&pMiddle, &pMiddle, speed, NO_OFFSET);
			if (RMT_isButton2Pressed(cmd))
				switchLegs();
		} while (!RMT_isButton6Pressed(cmd));

		// Inaktive Beine in die Luft
		prepareStepMove(&pMiddle, &pMiddle, speed, OFFSET);
		// Inaktive Beine fahren in der Luft Startpunkt an
		evolutionaryCalculation(&v, speed);
		calculateMovementPoints();
		prepareStepMove(&isectM, &isectS, speed, OFFSET);
		UTL_wait(5);
		// Alle Beine auf dem Boden
		prepareStepMove(&isectM, &isectS, speed, NO_OFFSET);
		UTL_wait(10);
		// Beine wechseln
		switchLegs();
		// Inaktive Beine in die Luft
		prepareStepMove(&pM_old, &pS_old, speed, OFFSET);
		UTL_wait(5);
		prepareStepMove(&pMiddle, &pMiddle, speed, OFFSET);
		UTL_wait(5);
		// Aktive Beine führen Bewegung aus
		doStepMove(&midM, &midS, speed);
		doStepMove(&pM, &pS, speed);
		// Zwischenspeichern des alten Punktes
		pM_old = copyPoint(&pM);
		pS_old = copyPoint(&pS);
		UTL_wait(5);
		// Alle Beine wieder auf den Boden
		prepareStepMove(&pMiddle, &pMiddle, speed, NO_OFFSET);
	}
}

#endif
