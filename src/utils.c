/**
 * \file	utils.c
 *
 * \brief	Verschiedene Hilfsmethoden.
 *
 * 			Stellt verschiedene Hilfsmethoden für allgemeinen Gebrauch zur Verfügung.
 */

#include <stdio.h>
#include <math.h>
#include "include/utils.h"

/**
 * \brief Debug-Ausgabe auf stdo oder USART.
 */
#define USART_ON
#ifdef USART_ON
#include "include/xmega.h"
#endif

/**
 * \brief	Gibt eine Matrix auf stdo aus.
 *
 * \param	mat	Point auf Matrix
 * \param	rows	Zeilenanzahl
 * \param	columns	Spaltenanzahl
 */
void UTL_printMatrix(const DT_double** const mat, DT_size rows, DT_size columns) {
	DT_size r, c;
	DT_char sep;
	printf("> DH04:\n");
	for (r = 0; r < rows; r++) {
		sep = ' ';
		for (c = 0; c < columns; c++) {
			printf("%c%10.5lf", sep, mat[r][c]);
			sep = '\t';

		}
		printf("\n");
	}
}

/**
 * \brief	Gibt die Soll-Winkel eines Beines auf stdo aus.
 *
 * \param	leg	Bein für das die Daten ausgegeben werden sollen
 * \param	type	Typ der Ausgabe in Bogenmaß oder Grad
 */
void UTL_printLeg(const DT_leg* const leg, DT_type type) {
	printf("> Bein: ");
	if (type == UTL_RAD)
		printf("Hüfte = %10.5lf; Knie = %10.5lf; Fuß = %10.5lf; (Bogenmaß)\n", leg->hip.set_value,
				leg->knee.set_value, leg->foot.set_value);
	else if (type == UTL_DEG)
		printf("Hüfte = %10.5lf; Knie = %10.5lf; Fuß = %10.5lf; (Winkel in Grad)\n",
				UTL_getDegree(leg->hip.set_value), UTL_getDegree(leg->knee.set_value), UTL_getDegree(leg->foot.set_value));
	else
		printf("type not supported!\n");
}

/**
 * \brief	Gibt einen Punkt auf stdo aus.
 *
 * \param p	Punkt zur Ausgabe
 */
void UTL_printPoint(const DT_point* const p) {
	printf("> Punkt: x = %10.5lf; y = %10.5lf; z = %10.5lf;\n", p->x, p->y, p->z);
}

/**
 * \brief	Umrechnung in das Bogenmaß.
 *
 * 			Rechnet einen Winkel in das Bogenmaß um.
 *
 * \param	angle Winkel in Grad
 *
 * \return	In das Bogenmaß umgerechneter Winkel
 */
DT_double UTL_getRadiant(DT_double angle) {
	return angle * (M_PI / 180);
}

/**
 * \brief	Umrechnung in das Gradmaß.
 *
 * 			Rechnet einen Winkel in das Gradmaß um.
 *
 * \param	radiant Winkel im Bogenmaß
 *
 * \return	In das Gradmaß umgerechneter Winkel
 */
DT_double UTL_getDegree(DT_double radiant) {
	return (radiant * 180) / M_PI;
}

/**
 * \brief	Extrahiert den Punkt aus der DH-Matrix.
 *
 * 			Extrahiert den Punkt ohne Orientierung aus der DH-Matrix.
 *
 * \param	dh DH-Matrix
 *
 * \return	Extrahierter Punkt
 */
DT_point UTL_getPointOfDH(const DT_double** const dh) {
	DT_point p;
	p.x = dh[0][3];
	p.y = dh[1][3];
	p.z = dh[2][3];
	return p;
}

/**
 * \brief	Debug-Ausgabe.
 *
 * 			Gibt einen Text auf der stdo oder der definierten Debug-USART aus.
 *
 * \param	msg	Text für die Ausgabe
 * \param	size	Länge des Textes
 */
void UTL_printDebug(const DT_char* const msg, DT_size size) {
	DT_size i;
#ifdef USART_ON
	while (!USART_IsTXDataRegisterEmpty(XM_debug_data.usart))
		;
	for (i = 0; i < size; i++) {
		USART_PutChar(XM_debug_data.usart, msg[i]);
		while (!USART_IsTXDataRegisterEmpty(XM_debug_data.usart))
			;
	}
	USART_PutChar(XM_debug_data.usart, ';');
	while (!USART_IsTXDataRegisterEmpty(XM_debug_data.usart))
				;
	USART_PutChar(XM_debug_data.usart, 0x00);
	while (!USART_IsTXDataRegisterEmpty(XM_debug_data.usart))
		;
#else
	for (i = 0; i < size; i++)
	printf("%c;", msg[i]);
#endif
}

/**
 * \brief	Debug-Ausgabe von Bytes.
 *
 * 			Gibt Bytes in Hexadezimal auf der stdo oder der definierten Debug-USART aus.
 *
 * \param	packet	Paket für die Ausgabe
 * \param	size	Größe des Pakets
 */
void UTL_printDebugByte(const DT_byte* const packet, DT_size size) {
	DT_char hex[2 * size];
	size = UTL_byteToHexChar(hex, packet, size);
	UTL_printDebug(hex, size);
}

/**
 * \brief 	Konvertierung eines Bytes in das Hexadezimal-Format.
 *
 * \param	dest	Pointer auf das Zielfeld
 * \param	src	Pointer auf das Quellfeld
 * \param	size	Größe des Quellfelds
 */
DT_byte UTL_byteToHexChar(DT_char* const dest, const DT_byte* const src, DT_size size) {
	DT_size i;
	DT_byte temp;
	for (i = 0; i < size; i++) {
		temp = src[i] & 0x0F;
		if (temp < 10) {
			dest[2 * i + 1] = temp + 48;
		} else {
			dest[2 * i + 1] = temp + 55;
		}
		temp = src[i] >> 4;
		if (temp < 10) {
			dest[2 * i] = temp + 48;
		} else {
			dest[2 * i] = temp + 55;
		}
	}
	return 2 * size;
}

/**
 * \brief	Abstraktion von einer Pause/Delay.
 *
 * 			Implementierung durch Schleifen (rounds * 64k).
 *
 * \param	rounds	Länge der Pause
 */
void UTL_wait(DT_size rounds) {
	DT_size r, i;
	for (r = 0; r < rounds; r++) {
		for (i = 0; i < 64000; i++)
			;
	}
}


