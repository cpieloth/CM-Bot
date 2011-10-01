/**
 * \file	movement.c
 *
 * \brief	Stellt Grundfunktionen für einen Laufalgorithmus bereit.
 */

#include "include/movement.h"
#include "include/xmega.h"
#include "include/utils.h"
#include "include/communication.h"
#include "include/dynamixel.h"
#include "include/kinematics.h"

#define MV_DST_X	168.5

/**
 * \brief	Sendet das ACTION-Kommando an ein Bein.
 *
 * 			Sendet das ACTION-Kommando an alle Servos eines Beines.
 *
 * \param	leg_r	rechtes Bein
 * \param	leg_l	linkes Bein
 */
void MV_action(DT_leg* const leg_r, DT_leg* const leg_l) {
	DNX_sendAction(leg_r->hip.id);
	DNX_sendAction(leg_r->knee.id);
	DNX_sendAction(leg_r->foot.id);

	DNX_sendAction(leg_l->hip.id);
	DNX_sendAction(leg_l->knee.id);
	DNX_sendAction(leg_l->foot.id);
}

/**
 * \brief	Standard-Methode für einen Slave-Controller.
 *
 * 			Standard-Methode für einen Slave-Controller. Nimmt Befehle eines Masters entgegen und führt die entsprechenden Aktionen aus.
 *
 * \param	cpuID	ID des Controllers auf dem die Methode ausgeführt wird
 * \param	leg_r	rechtes Bein
 * \param	leg_l	linkes Bein
 */
void MV_slave(DT_byte cpuID, DT_leg* const leg_r, DT_leg* const leg_l) {
	DT_size len;
	DT_byte result[DT_RESULT_BUFFER_SIZE];

	while (1) {
		XM_LED_OFF
		len = COM_receive(&XM_com_data3, result);

		if (len == 0)
			continue;
		DEBUG (("sl_pck_rec",sizeof("sl_pck_rec")))
		if (result[2] != cpuID && result[2] != COM_BRDCAST_ID)
			continue;
		DEBUG (("sl_pck_acc",sizeof("sl_pck_acc")))

		XM_LED_ON
		switch (result[4]) {
		case COM_STATUS:
			DEBUG(("sl_rec_sts",sizeof("sl_rec_sts")))
			MV_slaveStatus(result, len);
			break;
		case COM_ACTION:
			DEBUG(("sl_rec_act",sizeof("sl_rec_act")))
			MV_action(leg_r, leg_l);
			break;
		case COM_POINT:
			DEBUG(("sl_rec_pnt",sizeof("sl_rec_pnt")))
			if(result[18] == COM_SPEED){
				DEBUG(("sl_rec_spd",sizeof("sl_rec_spd")))
				MV_slavePointAndSpeed(leg_r, leg_l, result, len);
			}else{
				MV_slavePoint(leg_r, leg_l, result, len);
			}
			break;
		case COM_ANGLE:
			DEBUG(("sl_rec_ang",sizeof("sl_rec_ang")))
			MV_slaveAngle(leg_r, leg_l, result, len);
			break;
		default:
			DEBUG (("sl_err",sizeof("sl_err")))
			break;
		}
	}
}

/**
 * \brief	Prüft ob die Slaves alive sind. (Master)
 *
 * 			Prüft blockierend ob die Slaves alive sind.
 */
void MV_masterCheckAlive() {
	// CHECK CPUs
	DT_bool isAlive = false;
	XM_LED_OFF
	do {
		if (COM_isAlive(COM_SLAVE1B) && COM_isAlive(COM_SLAVE3F)) {
			isAlive = true;
		} else
			UTL_wait(5);
	} while (isAlive == false);
	DEBUG (("ma_alv",sizeof("ma_alv")))
	XM_LED_ON
}

/**
 * \brief	Antwortet auf eine Status-Anfrage eines Masters. (Slave)
 *
 * 			Antwortet auf eine Status-Anfrage eines Masters. (Slave)
 *
 * \param	result	Anfrage-Paket zum Auswerten
 * \param	len	Länge des Pakets
 */
void MV_slaveStatus(const DT_byte* const result, const DT_size len) {
	switch (result[5]) {
	case COM_IS_ALIVE:
		COM_sendACK(COM_MASTER);
		DEBUG(("sl_snd_ack",sizeof("sl_snd_ack")))
		break;
	default:
		break;
	}
}

/**
 * \brief	Führt die benötigten Aktionen für einen empfangenen Punkt aus. (Slave)
 *
 * 			Führt die benötigten Aktionen für einen empfangenen Punkt aus. (Slave)
 *
 * \param	leg_r	rechtes Bein
 * \param	leg_l	linkes Bein
 * \param	result	Anfrage-Paket zum Auswerten
 * \param	len	Länge des Pakets
 */
void MV_slavePoint(DT_leg* const leg_r, DT_leg* const leg_l,
		const DT_byte* const result, DT_size len) {
	DT_point p = COM_getPointFromPacket(result);
	DT_bool isGlobal = COM_isGlobal(result);
	DT_bool ret;

	if (COM_isLeftLeg(result)) {
		ret = MV_point(leg_l, &p, isGlobal);
	}
	if (COM_isRightLeg(result)) {
		ret = MV_point(leg_r, &p, isGlobal);
	}
	if (ret == true) {
		COM_sendACK(COM_MASTER);
	} else {
		COM_sendNAK(COM_MASTER, COM_ERR_POINT_OUT_OF_BOUNDS);
	}
}

/**
 * \brief	Führt die benötigten Aktionen für einen empfangenen Punkt und Anfahrgeschwindigkeit aus. (Slave)
 *
 * 			Führt die benötigten Aktionen für einen empfangenen Punkt und Anfahrgeschwindigkeit aus. (Slave)
 *
 * \param	leg_r	rechtes Bein
 * \param	leg_l	linkes Bein
 * \param	result	Anfrage-Paket zum Auswerten
 * \param	len	Länge des Pakets
 */
void MV_slavePointAndSpeed(DT_leg* const leg_r, DT_leg* const leg_l,
		const DT_byte* const result, DT_size len) {
	DEBUG(("SPEED", sizeof("SPEED")))
	DT_point p = COM_getPointFromPacket(result);
	DT_bool isGlobal = COM_isGlobal(result);
	DT_bool ret;
	DT_double speed = COM_getSpeedFromPacket(result);
	if (COM_isLeftLeg(result)) {
		ret = MV_pointAndSpeed(leg_l, &p, speed, isGlobal);
	}
	if (COM_isRightLeg(result)) {
		ret = MV_pointAndSpeed(leg_r, &p, speed, isGlobal);
	}
	if (ret == true) {
		COM_sendACK(COM_MASTER);
	} else {
		COM_sendNAK(COM_MASTER, COM_ERR_POINT_OUT_OF_BOUNDS);
	}
}

/**
 * \brief	Führt die benötigten Aktionen für einen empfangenen Winkel aus. (Slave)
 *
 * 			Führt die benötigten Aktionen für einen empfangenen Winkel aus. (Slave)
 *
 * \param	leg_r	rechtes Bein
 * \param	leg_l	linkes Bein
 * \param	result	Anfrage-Paket zum Auswerten
 * \param	len	Länge des Pakets
 */
void MV_slaveAngle(DT_leg* const leg_r, DT_leg* const leg_l,
		const DT_byte* const result, DT_size len) {
	DT_double angle = COM_getAngleFromPacket(result);

	if (COM_isLeftLeg(result)) {
		if (COM_isHip(result))
			DNX_setAngle(leg_l->hip.id, angle, true);
		if (COM_isKnee(result))
			DNX_setAngle(leg_l->knee.id, angle, true);
		if (COM_isFoot(result))
			DNX_setAngle(leg_l->foot.id, angle, true);
	}
	if (COM_isRightLeg(result)) {
		if (COM_isHip(result))
			DNX_setAngle(leg_r->hip.id, angle, true);
		if (COM_isKnee(result))
			DNX_setAngle(leg_r->knee.id, angle, true);
		if (COM_isFoot(result))
			DNX_setAngle(leg_r->foot.id, angle, true);
	}
	COM_sendACK(COM_MASTER);
	// TODO COM_sendNAK(COM_MASTER, COM_ERR_POINT_OUT_OF_BOUNDS);
}

/**
 * \brief	Berechnet Winkel anhand des Punktes für die Servos und versendet diese.
 *
 * 			Berechnet Winkel anhand des Punktes für die Servos und versendet diese.
 *
 * \param	leg	Bein
 * \param	point	Punkt
 * \param	isGlobal	Weltkoordinate, wenn true
 */
DT_bool MV_point(DT_leg* const leg, const DT_point* const point,
		DT_bool isGlobal) {
	DT_bool ret;
	if (isGlobal == true) {
		DT_point pLocal = KIN_calcLocalPoint(point, &leg->trans);
		ret = KIN_calcServos(&pLocal, leg);
	} else
		ret = KIN_calcServos(point, leg);

	if (ret == true) {
		leg->hip.set_value = UTL_getDegree(leg->hip.set_value);
		leg->knee.set_value = UTL_getDegree(leg->knee.set_value);
		leg->foot.set_value = UTL_getDegree(leg->foot.set_value);

		DNX_setAngle(leg->hip.id, leg->hip.set_value, true);
		DNX_setAngle(leg->knee.id, leg->knee.set_value, true);
		DNX_setAngle(leg->foot.id, leg->foot.set_value, true);
		return true;
	} else {
		return false;
	}
}

/**
 * \brief	Berechnet Winkel anhand des Punktes für die Servos und versendet diese zusammen mit der Anfahrgeschwindigkeit.
 *
 * 			Berechnet Winkel anhand des Punktes für die Servos und versendet diese zusammen mit der Anfahrgeschwindigkeit.
 *
 * \param	leg	Bein
 * \param	point	Punkt
 * \param	speed	Anfahrgeschwindigkeit
 * \param	isGlobal	Weltkoordinate, wenn true
 */
DT_bool MV_pointAndSpeed(DT_leg* const leg, const DT_point* const point, const DT_double speed,
		DT_bool isGlobal) {
	DT_bool ret;
	if (isGlobal == true) {
		DT_point pLocal = KIN_calcLocalPoint(point, &leg->trans);
		ret = KIN_calcServos(&pLocal, leg);
	} else
		ret = KIN_calcServos(point, leg);

	if (ret == true) {
		leg->hip.set_value = UTL_getDegree(leg->hip.set_value);
		leg->knee.set_value = UTL_getDegree(leg->knee.set_value);
		leg->foot.set_value = UTL_getDegree(leg->foot.set_value);

		DNX_setAngleAndSpeed(leg->hip.id, leg->hip.set_value, speed, true);
		DNX_setAngleAndSpeed(leg->knee.id, leg->knee.set_value, speed, true);
		DNX_setAngleAndSpeed(leg->foot.id, leg->foot.set_value, speed, true);
		return true;
	} else {
		return false;
	}
}

/**
 * \brief	Fährt das rechte und linke Bein in eine Startposition.
 *
 * 			Fährt das rechte und linke Bein in eine Startposition.
 *
 * \param	leg_r	rechtes Bein
 * \param	leg_l	linkes Bein
 */
void MV_doInitPosition(DT_leg* const leg_r, DT_leg* const leg_l) {
	XM_LED_OFF
	DT_byte config;
	DT_double angleHip, angleKnee, angleFoot;
	// Punkt fuer Null-Stellung mittleres rechtes Bein als Bezug
	angleHip = 0;
	angleKnee = 0;
	angleFoot = 0;

	DNX_setAngle(leg_r->hip.id, angleHip, true);
	DNX_setAngle(leg_r->knee.id, angleKnee, true);
	DNX_setAngle(leg_r->foot.id, angleFoot, true);
	DNX_setAngle(leg_l->hip.id, angleHip, true);
	DNX_setAngle(leg_l->knee.id, angleKnee, true);
	DNX_setAngle(leg_l->foot.id, angleFoot, true);

	config = COM_CONF_HIP | COM_CONF_KNEE | COM_CONF_FOOT | COM_CONF_LEFT
			| COM_CONF_RIGHT;
	COM_sendAngle(COM_SLAVE1B, angleHip, config);
	COM_sendAngle(COM_SLAVE3F, angleHip, config);

	COM_sendAction(COM_BRDCAST_ID);
	MV_action(leg_r, leg_l);

	UTL_wait(40);

	angleHip = 0;
	angleKnee = 45;
	angleFoot = 45;

	DNX_setAngle(leg_r->hip.id, angleHip, true);
	DNX_setAngle(leg_r->knee.id, angleKnee, true);
	DNX_setAngle(leg_r->foot.id, angleFoot, true);
	DNX_setAngle(leg_l->hip.id, angleHip, true);
	DNX_setAngle(leg_l->knee.id, angleKnee, true);
	DNX_setAngle(leg_l->foot.id, angleFoot, true);

	config = COM_CONF_HIP | COM_CONF_LEFT | COM_CONF_RIGHT;
	COM_sendAngle(COM_SLAVE1B, angleHip, config);
	COM_sendAngle(COM_SLAVE3F, angleHip, config);

	config = COM_CONF_KNEE | COM_CONF_FOOT | COM_CONF_LEFT | COM_CONF_RIGHT;
	COM_sendAngle(COM_SLAVE1B, angleKnee, config);
	COM_sendAngle(COM_SLAVE3F, angleKnee, config);

	COM_sendAction(COM_BRDCAST_ID);
	MV_action(leg_r, leg_l);

	UTL_wait(40);

	DEBUG(("ma_int_pos_ok",sizeof("ma_int_pos_ok")))
	XM_LED_ON
}

/**
 * \brief	Wechselt die aktiven und passiven Beine.
 *
 * 			Wechselt die aktiven und passiven Beine (Oben/Unten). Übergebene Variablen können in den jeweiligen Laufalgorithmus ausgewertet werden.
 *
 * \param	side	Aktive Seite (Seite mit Bodenkontakt), mittleres Beinpaar als Referenz
 * \param	master_dwn	Seite (Links/Rechts) für den Master mit Bodenkontakt
 * \param	master_up	Seite (Links/Rechts) für den Master ohne Bodenkontakt
 * \param	slave_dwn	Seite (Links/Rechts) für den Slave mit Bodenkontakt
 * \param	slave_up	Seite (Links/Rechts) für den Master ohne Bodenkontakt
 */
void MV_switchLegs(DT_byte* side, DT_byte* master_dwn, DT_byte* master_up,
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

/**
 * \brief	Rechnet einen lokalen Punkt mit Bezug auf das mittlere Bein in einen globalen Punkt für einen Controller um.
 *
 * 			Rechnet einen lokalen Punkt mit Bezug auf das mittlere Bein in einen globalen Punkt für einen Controller um.
 *
 * \param	p	lokaler Punkt
 * \param	cpuId	ID des Controllers
 * \param	side	Seite (Links/Rechts)
 *
 * \return	Globaler Punkt
 */
DT_point MV_getPntForCpuSide(const DT_point* const p, const DT_byte cpuId,
		const DT_byte side) {
	DT_point pTmp = *p;
	if (cpuId == COM_SLAVE1B) {
		pTmp.y = p->y - MV_DST_Y;
	}
	if (cpuId == COM_SLAVE3F) {
		pTmp.y = p->y + MV_DST_Y;
	}
	if (side == COM_CONF_LEFT) {
		pTmp.x = -p->x;
	}
	return pTmp;
}
