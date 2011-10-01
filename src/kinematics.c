/**
 * \file	kinematics.c
 *
 * \brief	Lösungsmethoden der Kinematik.
 *
 * 			Lösungsmethoden der Kinematik speziell für den CM-Bot.
 */

#include <math.h>
#include <stdlib.h>

#include "include/kinematics.h"
#include "include/utils.h"
#include "include/dynamixel.h"

/**
 * \def	DIST_HK
 * \brief	Abstand von Hüfte zu Knie.
 *
 * \def	DIST_KF
 * \brief	Abstand von Knie zu Fuß.
 *
 * \def	DIST_FE
 * \brief	Abstand von Fuß zu Fußende.
 *
 * \def	DIST_DZ
 * \brief	Versatz in z-Richtung von Knie in Bezug auf Hüfte.
 */
#define DIST_HK 50
#define DIST_KF 85
#define DIST_FE 55
#define DIST_DZ -14

/**
 * \brief	Liefert fuer ein Bein die Struktur zur Koordinatentransformation.
 *
 * 			Liefert fuer ein Bein die Struktur zur Koordinatentransformation.
 *
 * \param	leg		Bein
 */
void KIN_setTransMat(DT_leg* const leg) {
	switch (leg->hip.id) {
	case 1:
		leg->trans.x = 168.5;
		leg->trans.y = 208.5;
		leg->trans.zRotation = false;
		break;
	case 4:
		leg->trans.x = -168.5;
		leg->trans.y = 208.5;
		leg->trans.zRotation = true;
		break;
	case 7:
		leg->trans.x = 168.5;
		leg->trans.y = 0;
		leg->trans.zRotation = false;
		break;
	case 10:
		leg->trans.x = -168.5;
		leg->trans.y = 0;
		leg->trans.zRotation = true;
		break;
	case 13:
		leg->trans.x = 168.5;
		leg->trans.y = -208.5;
		leg->trans.zRotation = false;
		break;
	case 16:
		leg->trans.x = -168.5;
		leg->trans.y = -208.5;
		leg->trans.zRotation = true;
		break;
	default:
		DEBUG(("KIN_noID", sizeof("KIN_noID")))
		leg->trans.x = 0;
		leg->trans.y = 0;
		leg->trans.zRotation = false;
		break;
	}
}

/**
 * \brief	Lösung des kinematischen Problems.
 *
 * 			Lösung der Denavit-Hartenberg-Transformation.
 *
 * \param	leg		Bein mit den Soll-Winkel der Gelenke
 * \param	dh03	Zielmatrix für die Lösung
 */
void KIN_calcDH(const DT_leg* const leg, DT_double** dh03) {
	dh03[0][0] = cos(leg->hip.set_value) * cos(leg->knee.set_value) * cos(
			leg->foot.set_value) - cos(leg->hip.set_value) * sin(
			leg->knee.set_value) * sin(leg->foot.set_value);
	dh03[0][1] = -cos(leg->hip.set_value) * cos(leg->knee.set_value) * sin(
			leg->foot.set_value) - cos(leg->hip.set_value) * cos(
			leg->foot.set_value) * sin(leg->knee.set_value);
	dh03[0][2] = -sin(leg->hip.set_value);
	dh03[0][3] = 50 * cos(leg->hip.set_value) + 85 * cos(leg->hip.set_value)
			* cos(leg->knee.set_value) - 55 * cos(leg->hip.set_value) * sin(
			leg->knee.set_value) * sin(leg->foot.set_value) + 55 * cos(
			leg->hip.set_value) * cos(leg->knee.set_value) * cos(
			leg->foot.set_value);

	dh03[1][0] = cos(leg->knee.set_value) * cos(leg->foot.set_value) * sin(
			leg->hip.set_value) - sin(leg->hip.set_value) * sin(
			leg->knee.set_value) * sin(leg->foot.set_value);
	dh03[1][1] = -cos(leg->knee.set_value) * sin(leg->hip.set_value) * sin(
			leg->foot.set_value) - cos(leg->foot.set_value) * sin(
			leg->hip.set_value) * sin(leg->knee.set_value);
	dh03[1][2] = cos(leg->hip.set_value);
	dh03[1][3] = 50 * sin(leg->hip.set_value) + 85 * cos(leg->knee.set_value)
			* sin(leg->hip.set_value) - 55 * sin(leg->hip.set_value) * sin(
			leg->knee.set_value) * sin(leg->foot.set_value) + 55 * cos(
			leg->knee.set_value) * cos(leg->foot.set_value) * sin(
			leg->hip.set_value);

	dh03[2][0] = -cos(leg->knee.set_value) * sin(leg->foot.set_value) - cos(
			leg->foot.set_value) * sin(leg->knee.set_value);
	dh03[2][1] = sin(leg->knee.set_value) * sin(leg->foot.set_value) - cos(
			leg->knee.set_value) * cos(leg->foot.set_value);
	dh03[2][2] = 0;
	dh03[2][3] = -85 * sin(leg->knee.set_value) - 55 * cos(leg->knee.set_value)
			* sin(leg->foot.set_value) - 55 * cos(leg->foot.set_value) * sin(
			leg->knee.set_value) - 14;

	dh03[3][0] = 0;
	dh03[3][1] = 0;
	dh03[3][2] = 0;
	dh03[3][3] = 1;
}

/**
 * \brief	Lösung des inversen kinematischen Problems.
 *
 * 			Lösung des inversen kinematischen Problems mit Hilfe eines geometrischen Verfahrens mit leichten Einschränkungen.
 *
 * \param	p	Punkt (Roboterkoorinate)
 * \param	leg	Bein für die zusetzenden Winkel
 *
 * \return	true, wenn Berechnung erfolgreich
 */
DT_bool KIN_calcServos(const DT_point* const p, DT_leg* const leg) {
	DT_double z = p->z - DIST_DZ;
	DT_double h, h2, h3;
	DT_double hip, knee, foot;
	DT_double alpha, beta, gamma;
	// STEP 1 (without dummy-axis)
	// angle for hip axis in x-y-plane
	h = sqrt(p->x * p->x + p->y * p->y);
	// v1 = asin(p.y / h);
	hip = atan(p->y / p->x); // should have better precision

	// STEP 2
	// angle for hip & foot axis in z-h' plane
	h2 = h - DIST_HK;
	h3 = sqrt(h2 * h2 + z * z);

	alpha = h2 != h3 ? acos(
			(-(h3 * h3) + DIST_FE * DIST_FE + DIST_KF * DIST_KF) / (2 * DIST_FE
					* DIST_KF)) : M_PI; // law of cosine
	beta = asin((DIST_FE / h3) * sin(alpha)); // law of sines
	gamma = asin(abs(z) / h3); // rules of right angle triangle, abs(z) 'cause length of trianglearm!

	// CASES
	if (z < 0) { // defined for z < 0: foot-axis is between h3 and h-axis
		knee = gamma - beta;
		foot = M_PI - alpha;
	} else { // defined for z >= 0: foot-axis is not between h3 and h-axis
		/* Knick nach oben */
		knee = -(gamma + beta);
		foot = M_PI - alpha;

		/* Knick nach unten
		 knee = -(gamma - beta);
		 foot = -(M_PI - alpha);
		 */
	}

	if(hip!=hip || knee != knee || foot!=foot)
		return false;

	leg->hip.set_value = hip;
	leg->knee.set_value = knee;
	leg->foot.set_value = foot;

	return true;
}

/**
 * \brief	Transformiert einen Punkt in das Roboterkoordinatensystem.
 *
 * 			Transformiert einen Punkt des Weltkoordinatensystems in das Roboterkoordinatensystem eines Beines.
 * 			Berechnet nur eine x-y-Verschiebung und eine Rotation von 0/180 Grad um die z-Achse.
 *
 * \param	p		Punkt im Weltkooridnatensystem
 * \param	trans	Struktur mit Informationen fuer die Koordinatentransformation
 *
 * \return	Punkt im Roboterkoordinatensystem eines Beines
 */
DT_point KIN_calcLocalPoint(const DT_point* const p,
		const DT_transformation* const trans) {
	DT_point pLocal;
	if (trans->zRotation == true) {
		pLocal.x = trans->x - p->x;
		pLocal.y = trans->y - p->y;
		pLocal.z = p->z;
	} else {
		pLocal.x = p->x - trans->x;
		pLocal.y = p->y - trans->y;
		pLocal.z = p->z;
	}
	return pLocal;
}

