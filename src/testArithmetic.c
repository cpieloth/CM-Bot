/*
 * testArithmetic.c
 *
 *  Created on: 24.11.2010
 *      Author: christof
 */

#define TEST_OFF TEST
#ifdef TEST_ON

#include "include/utils.h"
#include "include/xmega.h"
#include "include/datatypes.h"
#include <stdlib.h>
#include <math.h>

const DT_double EPS = 0.0001;

DT_bool equalsDouble(DT_double val1, DT_double val2) {
	if (fabs(val1 - val2) < EPS) {
		return true;
	} else
		return false;
}

DT_bool equalsInteger(DT_int val1, DT_int val2) {
	if ((val1 - val2) == 0) {
		return true;
	} else
		return false;
}

int main() {
	XM_init_cpu();

	DT_int iRes;
	DT_double dRes;

	iRes = 123 + 321;
	if (equalsInteger(iRes, 444) == false) {
		DEBUG (("add int false", sizeof("add int false")))
		XM_LED_OFF
	}

	iRes = 321 - 123;
	if (equalsInteger(iRes, 198) == false) {
		DEBUG (("sub int false", sizeof("sub int false")))
		XM_LED_OFF
	}

	dRes = 123.00 + 321.00;
	if (equalsDouble(dRes, 444.00) == false) {
		DEBUG(("add dbl false", sizeof("add dbl false")))
		XM_LED_OFF
	}

	dRes = 321.00 - 123.00;
	if (equalsDouble(dRes, 198.00) == false) {
		DEBUG(("sub dbl false", sizeof("sub dbl false")))
		XM_LED_OFF
	}

	dRes = 123.00 * 321.00;
	if (equalsDouble(dRes, 39483.00) == false) {
		DEBUG(("mul dbl false", sizeof("mul dbl false")))
		XM_LED_OFF
	}

	dRes = 321.00 / 123.00;
	if (equalsDouble(dRes, 2.609756098) == false) {
		DEBUG(("div dbl false", sizeof("div dbl false")))
		XM_LED_OFF
	}

	dRes = sqrt(118.00);
	if (equalsDouble(dRes, 10.862780491) == false) {
		DEBUG(("sqrt dbl false", sizeof("sqrt dbl false")))
		XM_LED_OFF
	}

	dRes = tan(M_PI / 8);
	if (equalsDouble(dRes, 0.414213562) == false) {
		DEBUG(("tan dbl false", sizeof("tan dbl false")))
		XM_LED_OFF
	}

	dRes = sin(M_PI / 4);
	if (equalsDouble(dRes, 0.707106781) == false) {
		DEBUG(("sin dbl false", sizeof("sin dbl false")))
		XM_LED_OFF
	}

	dRes = cos(M_PI / 4);
	if (equalsDouble(dRes, 0.707106781) == false) {
		DEBUG(("cos dbl false", sizeof("cos dbl false")))
		XM_LED_OFF
	}

	dRes = 10.00 - 2.00;
	if (equalsDouble(dRes, 8.00 + (10 * EPS)) == false) {
		DEBUG(("test ok", sizeof("test ok")))
	} else {
		DEBUG(("test false", sizeof("test false")))
		XM_LED_OFF
	}

	DEBUG(("arithmetic test finish", sizeof("arithmetic test finish")))

	while (true)
		;
}

#endif /* TEST_ON */
