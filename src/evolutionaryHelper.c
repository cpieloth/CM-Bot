/**
 * \file	evolutionaryHelper.c
 *
 * \brief	Hilfsfunktion für Evolutionären Algorithmus.
 *
 * 			Mathematische Funktionen.
 */

#include "include/evolutionaryHelper.h"
#include <math.h>
#include <stdio.h>

/**
 * \def	NO_VALUE
 * \brief	Kein Rückgabewert der Funktion.
 *
 * \def	FIRST_VALUE
 * \brief	Der erste Rückgabewert enthält einen gültigen Schnittpunkt.
 *
 * \def	SECOND_VALUE
 * \brief	Der zweite Rückgabewert enthält einen gültigen Schnittpunkt.
 */
#define NO_VALUE	 0x00
#define FIRST_VALUE  0x01
#define SECOND_VALUE 0x02

// Lokale Methoden
DT_bool isectLinFuncs(const DT_lin_func*, const DT_lin_func*, DT_point*);
void initFunctions();
void initPoints();
DT_double f_lin(const DT_lin_func *, const DT_double);
DT_double f_circ(const DT_half_circle*, const DT_double);
DT_bool isectLinFuncs(const DT_lin_func*, const DT_lin_func*, DT_point*);
DT_byte isectLinCirc(const DT_lin_func*, const DT_half_circle*, DT_point*,
		DT_point*);
DT_bool isBetweenPoints(const DT_point*, const DT_point*, const DT_point*);
DT_double getDistance(const DT_point* const , const DT_point* const );
DT_point getNearerPoint(const DT_point*, const DT_point*, const DT_point*);
DT_bool isVectorialPoint(const DT_point* const , const DT_vector*,
		const DT_point* const );

// globale Variablen
DT_point A, B, C, D, E, F, G;
DT_lin_func AB, CD;
DT_half_circle CEA, DFB;

// AB:  f(x) = 1.1875x + 0.4375
// CD:  g(x) = -1.1875x + 0.4375
// CEA: h(x) = sqrt(2874.60947 - x x)
// DFB: i(x) = sqrt(16852.36694 - x x)

/**
 * \brief	Initialisiert die globalen Funktionen.
 *
 * 			Initialisiert die globalen Funktionen zur Beschreibung der Schnittfläche des Arbeitsraums.
 */
void initFunctions() {
	AB.m = 1.1875;
	AB.n = 0.4375;
	CD.m = -1.1875;
	CD.n = 0.4375;
	CEA.sqr_r = 2874.60947;
	DFB.sqr_r = 16852.36694;
}

void initPoints() {
	A.x = 35.0;
	A.y = 42.0;
	A.z = Z;
	B.x = 83.0;
	B.y = 99.0;
	B.z = Z;
	C.x = -35.0;
	C.y = 42.0;
	C.z = Z;
	D.x = -83.0;
	D.y = 99.0;
	D.z = Z;
	E.x = 0.0;
	E.y = 55.0;
	E.z = Z;
	F.x = 0.0;
	F.y = 129.0;
	F.z = Z;
	// Centerpoint G
	G.x = 0;
	G.y = 100;
	G.z = Z;
}

void initEvoAlg() {
	initFunctions();
	initPoints();
}
// y = mx + n
DT_double f_lin(const DT_lin_func * f, const DT_double x) {
	return f->m * x + f->n;
}

//y = sqrt(r*r - x*x)
DT_double f_circ(const DT_half_circle* f1, const DT_double x) {
	return sqrt(f1->sqr_r - x * x);
}

DT_bool isectLinFuncs(const DT_lin_func* f1, const DT_lin_func* f2,
		DT_point* isect) {
	DT_double x = (f2->n - f1->n) / (f1->m - f2->m);
	if (x != x) {
		return false;
	} else {
		isect->x = x;
		isect->y = f_lin(f1, x);
		isect->z = Z;
		return true;
	}
}

DT_byte isectLinCirc(const DT_lin_func* f1, const DT_half_circle* f2,
		DT_point* isect1, DT_point* isect2) {
	DT_int retValue = 0;
	DT_double p = (2 * f1->m * f1->n) / (f1->m * f1->m + 1);
	DT_double q = (f1->n * f1->n - f2->sqr_r) / (f1->m * f1->m + 1);
	DT_double x1 = -p / 2 + sqrt((p * p) / 4 - q);
	DT_double x2 = -p / 2 - sqrt((p * p) / 4 - q);
	if (x1 != x1) {
	} else {
		isect1->x = x1;
		isect1->z = Z;
		if (fabs(f1->m) < 100000000)
			isect1->y = f_lin(f1, x1);
		else
			isect1->y = f_circ(f2, x1);
		retValue |= FIRST_VALUE;
	}
	if (x2 != x2 || x1 == x2) {
	} else {
		isect2->x = x2;
		isect2->z = Z;
		if (fabs(f1->m) < 100000000)
			isect2->y = f_lin(f1, x2);
		else
			isect2->y = f_circ(f2, x2);

		retValue |= SECOND_VALUE;
	}
	return retValue;
}

DT_bool isBetweenPoints(const DT_point* p1, const DT_point* p2,
		const DT_point* p_between) {
	if (p1->x > p2->x) {
		if (p1->x > p_between->x && p_between->x > p2->x)
			return true;
		else
			return false;
	} else {
		if (p2->x > p_between->x && p_between->x > p1->x)
			return true;
		else
			return false;
	}
}

DT_double getDistance(const DT_point* const p1, const DT_point* const p2) {
	return sqrt((p1->x - p2->x) * (p1->x - p2->x) + (p1->y - p2->y) * (p1->y
			- p2->y));
}

DT_point getNearerPoint(const DT_point* p_ref, const DT_point* p1,
		const DT_point* p2) {
	DT_double dist1 = sqrt((p_ref->x - p1->x) * (p_ref->x - p1->x) + (p_ref->y
			- p1->y) * (p_ref->y - p1->y));
	DT_double dist2 = sqrt((p_ref->x - p2->x) * (p_ref->x - p2->x) + (p_ref->y
			- p2->y) * (p_ref->y - p2->y));
	if (dist1 < dist2)
		return *p1;
	else
		return *p2;
}

DT_bool isInArea(const DT_point* p) {
	// Gerade berechnen, von p zu centerPoint
	// y = mx + n
	DT_lin_func g1;
	if (p->x != 0)
		g1.m = (G.y - p->y) / (G.x - p->x);
	else
		g1.m = 0;
	g1.n = G.y;

	DT_point isect, isect1;
	DT_bool isInArea = true;

	// Check Right Side
	if (isectLinFuncs(&g1, &AB, &isect)) {
		if (isBetweenPoints(&G, p, &isect))
			isInArea = false;
	}
	// Check Left Side
	if (isectLinFuncs(&g1, &CD, &isect)) {
		if (isBetweenPoints(&G, p, &isect))
			isInArea = false;
	}

	// Check lower Circle
	DT_byte LinCircConf = isectLinCirc(&g1, &CEA, &isect, &isect1);
	if (LinCircConf != NO_VALUE) {
		if (LinCircConf == FIRST_VALUE) {
			isect = isect;
		} else if (LinCircConf == SECOND_VALUE) {
			isect = isect1;
		} else if (LinCircConf == (FIRST_VALUE | SECOND_VALUE)) {
			isect = getNearerPoint(&G, &isect, &isect1);
		}
		if (isBetweenPoints(&G, p, &isect)) {
			isInArea = false;
		}
	}

	// Check upper Circle
	LinCircConf = isectLinCirc(&g1, &DFB, &isect, &isect1);
	if (LinCircConf != NO_VALUE) {
		if (LinCircConf == FIRST_VALUE) {
			isect = isect;
		} else if (LinCircConf == SECOND_VALUE) {
			isect = isect1;
		} else if (LinCircConf == (FIRST_VALUE | SECOND_VALUE)) {
			isect = getNearerPoint(&G, &isect, &isect1);
		}
		if (isBetweenPoints(&G, p, &isect)) {
			isInArea = false;
		}
	}
	return isInArea;
}

DT_bool isVectorialPoint(const DT_point* const p_ref, const DT_vector* v,
		const DT_point* const p_chk) {
	DT_point p_refVec;
	if (v->x != 0)
		p_refVec.x = p_ref->x + v->x / fabs(v->x);
	else
		p_refVec.x = p_ref->x;
	if (v->y != 0)
		p_refVec.y = p_ref->y + v->y / fabs(v->y);
	else
		p_refVec.y = p_ref->y;
	if (getDistance(p_ref, p_chk) >= getDistance(&p_refVec, p_chk))
		return true;
	else
		return false;
}

DT_double scorePoint(DT_vector* const v, const DT_point* const p, DT_point * s) {
	if ((v->x == 0) && (v->y == 0))
		return 0.0;
	// y = mx + n
	DT_lin_func f;
	if (v->x == 0) {
		v->x += 0.01;
		f.m = 0;
	} else
		f.m = v->y / v->x;
	f.n = p->y - f.m * p->x;

	printf("\nFunktion f: y = %f x + %f", f.m, f.n);

	DT_point isect, isect_tmp, isect_tmp1;
	isect.x = -10000;
	isect.y = -10000;
	if (isectLinFuncs(&AB, &f, &isect_tmp)) {
		if (isVectorialPoint(p, v, &isect_tmp)) {
			if (getDistance(p, &isect) > getDistance(p, &isect_tmp))
				isect = isect_tmp;
		}
	}
	if (isectLinFuncs(&CD, &f, &isect_tmp))
		if (isVectorialPoint(p, v, &isect_tmp))
			if (getDistance(p, &isect) > getDistance(p, &isect_tmp))
				isect = isect_tmp;
	// Lower Circle
	DT_byte LinCircConf = isectLinCirc(&f, &CEA, &isect_tmp, &isect_tmp1);
	if (LinCircConf != NO_VALUE) {
		if (LinCircConf == FIRST_VALUE) {
			if (!isVectorialPoint(p, v, &isect_tmp))
				LinCircConf = NO_VALUE;
		} else if (LinCircConf == SECOND_VALUE) {
			if (!isVectorialPoint(p, v, &isect_tmp1))
				LinCircConf = NO_VALUE;
		}
		if (LinCircConf == (FIRST_VALUE | SECOND_VALUE)) {
			if (!isVectorialPoint(p, v, &isect_tmp))
				LinCircConf = LinCircConf & SECOND_VALUE;
			if (!isVectorialPoint(p, v, &isect_tmp1))
				LinCircConf = LinCircConf & FIRST_VALUE;
		}
		if (LinCircConf != NO_VALUE) {
			if (LinCircConf == SECOND_VALUE)
				isect_tmp = isect_tmp1;
			if (LinCircConf == (FIRST_VALUE | SECOND_VALUE)) {
				isect_tmp = getNearerPoint(p, &isect_tmp, &isect_tmp1);
			}
			if (getDistance(p, &isect) > getDistance(p, &isect_tmp))
				isect = isect_tmp;
		}
	}
	// Upper Circle
	LinCircConf = isectLinCirc(&f, &DFB, &isect_tmp, &isect_tmp1);
	if (LinCircConf != NO_VALUE) {
		if (LinCircConf == FIRST_VALUE) {
			if (!isVectorialPoint(p, v, &isect_tmp))
				LinCircConf = NO_VALUE;
		} else if (LinCircConf == SECOND_VALUE) {
			if (!isVectorialPoint(p, v, &isect_tmp1))
				LinCircConf = NO_VALUE;
		}
		if (LinCircConf == (FIRST_VALUE | SECOND_VALUE)) {
			if (!isVectorialPoint(p, v, &isect_tmp))
				LinCircConf = LinCircConf & SECOND_VALUE;
			if (!isVectorialPoint(p, v, &isect_tmp1))
				LinCircConf = LinCircConf & FIRST_VALUE;
		}
		if (LinCircConf != NO_VALUE) {
			if (LinCircConf == SECOND_VALUE)
				isect_tmp = isect_tmp1;
			if (LinCircConf == (FIRST_VALUE | SECOND_VALUE)) {
				isect_tmp = getNearerPoint(p, &isect_tmp, &isect_tmp1);
			}
			if (getDistance(p, &isect) > getDistance(p, &isect_tmp))
				isect = isect_tmp;
		}
	}

	s->x = (isect.x > 0) ? isect.x - 1 : isect.x + 1;
	s->y = (isect.y > 0) ? isect.y - 1 : isect.y + 1;
	s->z = Z;
	return getDistance(p, &isect);
}

void bubblesort(DT_individuum * P, const DT_int n) {
	int i, j;
	for (i = n - 1; i > 0; i--) {
		for (j = 0; j < i; j++) {
			if (P[j].F < P[j + 1].F) {
				DT_individuum swp = P[j];
				P[j] = P[j + 1];
				P[j + 1] = swp;
			}
		}
	}
}

DT_double max(DT_double x, DT_double y) {
	if (x <= y)
		return y;
	else
		return x;
}

DT_double min(DT_double x, DT_double y) {
	if (x > y)
		return y;
	else
		return x;
}

void getFunctionOfPoints(DT_lin_func * f, const DT_point* const p1,
		const DT_point * const p2) {
	if ((p1->x - p2->x) != 0)
		f->m = (p1->y - p2->y) / (p1->x - p2->x);
	else
		f->m = 0;
	f->n = p1->y - f->m * p1->x;

}
