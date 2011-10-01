/**
 * \file	evolutionaryHelper.h
 *
 * \brief	Hilfsfunktion für Evolutionären Algorithmus.
 *
 * 			Mathematische Funktionen.
 */

#include "datatypes.h"

#define Z  -129.1041

DT_bool isInArea(const DT_point*);
DT_double scorePoint(DT_vector* const , const DT_point* const , DT_point *);
void bubblesort(DT_individuum*, const DT_int);
void initEvoAlg();
DT_double max(DT_double, DT_double);
DT_double min(DT_double, DT_double);
DT_double getDistance(const DT_point* const , const DT_point* const );
void getFunctionOfPoints(DT_lin_func *, const DT_point* const ,
		const DT_point * const );
