/**
 * \file	movement.h
 *
 * \brief	Stellt Grundfunktionen für einen Laufalgorithmus bereit.
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "datatypes.h"

#define MV_DST_Y	208.5
#define MV_DST_X	168.5

void MV_action(DT_leg* const, DT_leg* const);
void MV_slave(DT_byte, DT_leg* const, DT_leg* const);
void MV_slaveStatus(const DT_byte* const, const DT_size);
void MV_slavePoint(DT_leg* const, DT_leg* const, const DT_byte* const, DT_size);
void MV_slavePointAndSpeed(DT_leg* const, DT_leg* const, const DT_byte* const, DT_size);
void MV_slaveAngle(DT_leg* const, DT_leg* const, const DT_byte* const, DT_size);
DT_bool MV_point(DT_leg* const, const DT_point* const, DT_bool);
DT_bool MV_pointAndSpeed(DT_leg* const, const DT_point* const, const DT_double, DT_bool);
void MV_masterCheckAlive();
void MV_doInitPosition (DT_leg* const, DT_leg* const);
void MV_switchLegs(DT_byte* side, DT_byte* master_dwn, DT_byte* master_up,
		DT_byte* slave_dwn, DT_byte* slave_up);
DT_point MV_getPntForCpuSide(const DT_point* const, const DT_byte, const DT_byte);

#endif /* MOVEMENT_H_ */
