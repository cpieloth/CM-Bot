/**
 * \file	dynamixel.c
 *
 * \brief	Methoden zur Steuerung der Dynamixal AX-12.
 */

#include "include/dynamixel.h"
#include "include/utils.h"
#include "include/xmega.h"
#include <math.h>

#define START_BYTE 0xFF

// Instruction Set (Manual page 19)
#define PING 0x01
#define RD_DATA 0x02
#define WR_DATA 0x03
#define REG_WR 0x04
#define ACT 0x05
#define RESET 0x06
#define SYC_WR 0x83

// Control Table (Manuel page 12)
#define ID 0x03
#define BD 0x04
#define MAX_TMP 0x0B
#define STS_RT_LVL 0x10
#define ALR_SHUTDWN 0x12
#define GL_POS 0x1E
#define LED 0x19
#define MV_SPEED 0x20
#define PRT_POS 0x24
#define PRT_SPEED 0x26
#define PRT_TMP 0x2B

/**
 * \brief	Berechnet die Checksum.
 *
 * \param	packet	Paket
 * \param	l	Größe des pakets
 *
 * \return	Checksum
 */
DT_byte DNX_getChecksum(const DT_byte* const packet, DT_size l) {
	DT_size i;
	DT_byte chksm = 0;
	for (i = 2; i < l - 1; i++)
		chksm += packet[i];
	return ~chksm;
}

/**
 * \brief 	USART-Empfangsmethode.
 *
 * 			Diese Methode liest den jeweiligen USART-Buffer aus und prüft,
 * 			ob ein vollständiges Paket gemäß des Dynamixel-Protokoll empfangen wurde.
 *
 * \param	usart_data	USART
 * \param	dest		Byte-Array für Antwort-Paket
 *
 * \return	Länge des Antwortpakets
 */
DT_byte DNX_receive(USART_data_t* const usart_data, DT_byte* const dest) {
	USART_Buffer_t* const buffer = &usart_data->buffer;

	// Phantom-Paket verwerfen
	while (USART_RXBufferData_Available(usart_data)
			&& usart_data->lastPacketLength > 0) {
		usart_data->lastPacketLength--;
		USART_RXBuffer_GetByte(usart_data);
	}

	const DT_byte tempHead = buffer->RX_Head;
	const DT_byte tempTail = buffer->RX_Tail;

	if (usart_data->lastPacketLength > 0) {
		DEBUG(("DNX_se",sizeof("DNX_se")))
		return 0;
	}

	// Pruefen ob min. 4 Bytes im Buffer sind, um Laenge zu lesen
	if (USART_RXBuffer_checkPointerDiff(tempTail, tempHead, 4)) {
		//DEBUG(("DNX_le",sizeof("DNX_le")))
		return 0;
	}
	// Byte #1 und Byte #2 muessen laut Protokoll 0xFF sein
	else if ((buffer->RX[tempTail] != 0xFF) && (buffer->RX[(tempTail + 1)
			& USART_RX_BUFFER_MASK] != 0xFF)) {
		DEBUG(("DNX_ff",sizeof("DNX_ff")))
		return 0;
	}
	// Some data received. All data received if checksum is correct!
	else {
		// Calculate predicted length
		DT_byte length;
		length = buffer->RX[(tempTail + 3) & USART_RX_BUFFER_MASK];
		// Complete length = (FF + FF + ID + LENGTH) + length
		length += 4;
		// Prüfen ob Paket bereits komplett im Buffer
		if (USART_RXBuffer_checkPointerDiff(tempTail, tempHead, length)) {
			DEBUG(("DNX_uc",sizeof("DNX_uc")))
			return 0;
		}
		// Copy packet from buffer in destination array
		DT_byte i;
		for (i = 0; i < length; i++) {
			dest[i] = USART_RXBuffer_GetByte(usart_data);
		}
		DEBUG_BYTE((dest,25))
		// Pruefen ob Checksumme korrekt ist
		if (dest[length - 1] != DNX_getChecksum(dest, length)) {
			DEBUG(("DNX_cks",sizeof("DNX_cks")))
			usart_data->buffer.RX_Tail = 0;
			usart_data->buffer.RX_Head = 0;
			//DEBUG_BYTE((dest, length))
			return 0;
		}
		DEBUG(("DNX_ok",sizeof("DNX_ok")))
		return length;
	}
}

/**
 * \brief	Versenden von Daten an Dynamixel.
 *
 * 			Blockierendes Senden mit gleichzeitigem Empfangen der Antwort.
 *
 * \param	packet	Zuversendendes Paket
 * \param	l	Größe des Pakets
 * \param	result	Zielfeld für Antowort
 * \param	hasResponse	Wartet auf eine Antwort, wenn true
 *
 * \return Größe der empfangenen Antwort
 */
DT_byte DNX_send(DT_byte* const packet, DT_size l, DT_byte* const result,
		DT_bool hasResponse) {
	packet[l - 1] = DNX_getChecksum(packet, l);
	USART_data_t* usart_data;
	DT_byte len = 0;
	// packet[2] -> ID
	if (packet[2] == DNX_BRDCAST_ID) {
		XM_USART_send(&XM_servo_data_R, packet, l);
		XM_USART_send(&XM_servo_data_L, packet, l);
	} else {
		if ((packet[2] - 1) % 6 < 3) // Right: 1 - 3, 7 - 9, ...
			usart_data = &XM_servo_data_R;
		else
			// Left:  4 - 6, ...
			usart_data = &XM_servo_data_L;

		XM_USART_send(usart_data, packet, l);

		DT_size timeout = hasResponse ? 100 : 0;
		while (len == 0 && timeout > 0) {
			len = DNX_receive(usart_data, result);
			timeout--;
		}
	}
	return len;
}

/**
 * \brief	Konvertiert Winkel in Bezug auf einen neuen Nullpunkt.
 *
 * \param	value	Winkel in Grad
 *
 * \return	Konvertierter Winkel in Grad
 */
DT_double DNX_convertAngle(DT_double value) {
	value += 150;
	if (value >= 360)
		value -= 360;
	return value;
}

/**
 * \brief	Korrigiert Winkel für Dynamixel.
 *
 * 			Korrigiert Winkel für Dynamixel um hardwareseitige Veränderungen auszuschließen.
 *
 * \param	id	ID des Servos
 * \param	value	Winkel in Grad
 *
 * \return	Korrigierter Winkel in Grad
 */
DT_double DNX_correctAngles(DT_byte id, DT_double value) {
	switch ((id - 1) % 6) {
	case 0:
		value = 360 - value;
		break;
	case 1:
		// value = value;
		break;
	case 2:
		value = 360 - value;
		break;
	case 3:
		value = 360 - value;
		break;
	case 4:
		value = 360 - value;
		break;
	case 5:
		// value = value;
		break;
	}
	return value;
}

/**
 * \brief	Sendet einen Winkel an Servo.
 *
 * \param	id	ID des Servos
 * \param	value	Winkel in Grad
 * \param	regWrite	Winkel wird in Puffer des Servos gespeichert und erst bei ACTION angefahren, wenn true
 */
DT_bool DNX_setAngle(DT_byte id, DT_double value, DT_bool regWrite) {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len = 9;
	DT_byte packet[len];

	value = DNX_correctAngles(id, value);
	value = DNX_convertAngle(value);

	// TODO Rechnung prüfen
	DT_int tmp = floor(3.41 * ((double) value));
	DT_byte angle_l = tmp & 0xFF;
	DT_byte angle_h = tmp >> 8;

	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = len - 4; // length
	packet[4] = regWrite ? REG_WR : WR_DATA;
	packet[5] = GL_POS;
	packet[6] = angle_l; // Low
	packet[7] = angle_h; // High
	// packet[8] = checksum will set in send
	len = DNX_send(packet, len, result, true);
	// TODO status pruefen
	if (len > 0)
		return true;
	else
		return false;
}

/**
 * \brief	Sendet Winkel und Speed an Servo.
 *
 * \param	id	ID des Servos
 * \param	angle	Winkel in Grad
 * \param	speed	Anfahrgeschwindigkeit
 * \param	regWrite	Werte werden in Puffer des Servos gespeichert und erst bei ACTION ausgeführt, wenn true
 */
DT_bool DNX_setAngleAndSpeed(DT_byte id, DT_double angle, DT_double speed,
		DT_bool regWrite) {

	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len = 11;
	DT_byte packet[len];
	angle = DNX_correctAngles(id, angle);
	angle = DNX_convertAngle(angle);
	//DEBUG(("SET_AuS",sizeof("SET_AuS")))
	DT_int tmp = floor(3.41 * ((double) angle));
	DT_byte angle_l = tmp & 0xFF;
	DT_byte angle_h = tmp >> 8;
	tmp = floor(speed);
	DT_byte speed_l = tmp & 0xFF;
	DT_byte speed_h = tmp >> 8;

	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = len - 4; // length
	packet[4] = regWrite ? REG_WR : WR_DATA;
	packet[5] = GL_POS;
	packet[6] = angle_l; // Low
	packet[7] = angle_h; // High
	packet[8] = speed_l;
	packet[9] = speed_h;
	// packet[10] = checksum will set in send
	len = DNX_send(packet, len, result, true);
	// TODO status pruefen
	if (len > 0)
		return true;
	else
		return false;
}

/**
 * \brief	Vergibt einem Servos eine neue ID (ungetestet).
 *
 * \param	idOld	ID des zu verändernden Servos
 * \param	idNew	Zusetzende ID
 */
void DNX_setId(DT_byte idOld, DT_byte idNew) {
	const DT_size len = 8;
	DT_byte packet[8];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = idOld;
	packet[3] = len - 4; // length
	packet[4] = WR_DATA;
	packet[5] = ID;
	packet[6] = idNew;
	// packet[7] = checksum will set in send
	DNX_send(packet, len, result, true);

}

/**
 * \brief	Setzt die Anfahrgeschwindigkeit eines Servos (unvollendet).
 *
 * \param	id	ID des Servos
 * \param	speed	Geschwindigkeit
 */
void DNX_setSpeed(DT_byte id, DT_byte speed) {
	// TODO byte 7 richtige setzen
	const DT_size len = 9;
	DT_byte packet[len];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = len - 4; // length
	packet[4] = WR_DATA;
	packet[5] = MV_SPEED;
	packet[6] = speed;
	packet[7] = 0x00;
	// packet[8] = checksum will set in send
	DNX_send(packet, len, result, true);
}

/**
 * \brief	Schaltet die LED eines Servos an/aus.
 *
 * \param	id	ID des Servos
 * \param	value	Wert für LED (0x00 / 0x01)
 */
DT_bool DNX_setLed(DT_byte id, DT_byte value) {
	DT_size len = 8;
	DT_byte packet[len];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = len - 4; // length
	packet[4] = WR_DATA;
	packet[5] = LED;
	packet[6] = value;
	// packet[7] = checksum will set in send
	len = DNX_send(packet, len, result, true);
	// TODO status pruefen
	if (len > 0) {
		return true;
	} else
		return false;
}

/**
 * \brief	Sendet das ACTION-Kommando an einen Servo.
 *
 * 			Sendet das ACTION-Kommando an einen Servo.
 *
 * \param	id	ID des Servo
 */
void DNX_sendAction(DT_byte id) {
	DT_size len = 6;
	DT_byte packet[len];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = len - 4; // length
	packet[4] = ACT;
	// packet[5] = checksum will set in send
	DNX_send(packet, len, result, false);
}

/**
 * \brief	Liest den aktuellen Winkel eines Servos aus (unfertig).
 *
 * \param	id	ID des Servos
 *
 * \return Winkel in Grad
 */
DT_double DNX_getAngle(DT_byte id) {
	DT_size len = 7;
	DT_byte packet[len];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = len - 4; // length
	packet[4] = RD_DATA;
	packet[5] = PRT_POS;
	// packet[6] = checksum will set in send
	// TODO
	len = DNX_send(packet, len, result, true);
	return -1;
}

/**
 * \brief	Liest die Anfahrgeschwindigkeit eines Servos aus (unfertig).
 *
 * \param	id	ID des Servos
 *
 * \return	Geschwindigkeit
 */
DT_byte DNX_getSpeed(DT_byte id) {
	DT_size len = 7;
	DT_byte packet[len];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = len - 4; // length
	packet[4] = RD_DATA;
	packet[5] = PRT_SPEED;
	// packet[6] = checksum will set in send
	// TODO
	len = DNX_send(packet, len, result, true);
	return 0x00;
}

/**
 * \brief	Liest den Status der LED aus (unfertig).
 *
 * \param	id	ID des Servos
 *
 * \return	Wert der LED
 */
DT_byte DNX_getLed(DT_byte id) {
	DT_size len = 7;
	DT_byte packet[len];
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	packet[0] = START_BYTE;
	packet[1] = START_BYTE;
	packet[2] = id;
	packet[3] = len - 4; // length
	packet[4] = RD_DATA;
	packet[5] = LED;
	// packet[6] = checksum will set in send
	// TODO
	len = DNX_send(packet, len, result, true);
	return 0x00;
}

/**
 * \brief	Ermittlung der angeschlossenen Dynamixel.
 *
 * \param	leg_r	Bein rechts
 * \param	leg_l	Bein links
 */
void DNX_getConnectedIDs(DT_leg* const leg_r, DT_leg* const leg_l) {
	DT_byte id;
	DT_bool ans;

	for (id = 1; id <= 18; id++) {
		ans = DNX_setLed(id, 0x01);
		if (ans) {
			if ((id - 1) % 6 < 3) { // Right: 1 - 3, 7 - 9, ...
				if ((id - 1) % 3 == 0) {
					leg_r->hip.id = id;
				}
				if ((id - 1) % 3 == 1) {
					leg_r->knee.id = id;
				}
				if ((id - 1) % 3 == 2) {
					leg_r->foot.id = id;
				}
			} else {
				// Left:  4 - 6, ...
				if ((id - 1) % 3 == 0) {
					leg_l->hip.id = id;
				}
				if ((id - 1) % 3 == 1) {
					leg_l->knee.id = id;
				}
				if ((id - 1) % 3 == 2) {
					leg_l->foot.id = id;
				}
			}
		}
	}
}
