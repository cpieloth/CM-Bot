/**
 * \file	utils.h
 *
 * \brief	Verschiedene Hilfsmethoden.
 *
 * 			Stellt verschiedene Hilfsmethoden für allgemeinen Gebrauch zur Verfügung.
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "datatypes.h"

#define UTL_DEG 1
#define UTL_RAD 0

#define DEBUG_ON debug
#ifdef DEBUG_ON
	#define DEBUG(output) UTL_printDebug output;
	#define DEBUG_BYTE(output) UTL_printDebugByte output;
#else
	#define DEBUG(output) /* no debug */
	#define DEBUG_BYTE(output) /* no debug */
#endif

void UTL_printMatrix(const DT_double** const, DT_size, DT_size);
void UTL_printLeg(const DT_leg* const, DT_type);
void UTL_printPoint(const DT_point* const);

DT_double UTL_getRadiant(DT_double);
DT_double UTL_getDegree(DT_double);
DT_point UTL_getPointOfDH(const DT_double** const);

void UTL_printDebug(const DT_char* const, DT_size);
void UTL_printDebugByte(const DT_byte* const, DT_size);
DT_byte UTL_byteToHexChar(DT_char* const, const DT_byte* const, DT_size);

void UTL_wait(DT_size);

#endif /* UTILS_H_ */
