/**
 * \file	communication.c
 *
 * \brief	Methoden zur Kommunikation der CPUs.
 */

#include "include/communication.h"
#include "include/utils.h"
#include "include/xmega.h"

#define COM_START_BYTE 	0xFF

/**
 * \brief	Berechnet die Checksum.
 *
 * \param	packet	Paket
 * \param	l	Größe des pakets
 *
 * \return	Checksum
 */
DT_byte COM_getChecksum(const DT_byte* const packet, DT_size l) {
	DT_size i;
	DT_byte chksm = 0;
	for (i = 2; i < l - 1; i++)
		chksm += packet[i];
	return ~chksm;
}

/*
 *  \brief 	Zuweisung der CpuID.
 *
 * 			Achtung: vorher muss die Methode DNX_getConnectedIDs(leg_r, leg_l) ausgeführt werden,
 * 			damit in dem Bein-Struct die IDs der angeschlossenen Geräte stehen.
 *
 * \param	leg_r		Bein-Struktur mit IDs
 *
 * \return	ID	1->Master; 2,3->Slave; 0 Error
 */
DT_byte COM_getCpuID(const DT_leg* const leg_l) {
	if (leg_l->hip.id == 10)
		return COM_MASTER; // Master
	else if (leg_l->hip.id == 4)
		return COM_SLAVE3F; // Slave
	else if (leg_l->hip.id == 16)
		return COM_SLAVE1B; // Slave
	else
		return COM_NOCPUID; // falsche ID
}

/**
 * \brief 	USART-Empfangsmethode für Prozessorkommunikation.
 *
 * 			Diese Methode liest den jeweiligen USART-Buffer aus und prüft,
 * 			ob ein vollständiges Paket gemäß des Communication-Protokoll empfangen wurde.
 *
 * \param	usart_data	USART-Datenstruktur
 * \param	dest		Byte-Array für Antwort-Paket
 *
 * \return	Länge des Antwortpakets
 */
DT_byte COM_receive(USART_data_t* const usart_data, DT_byte* const dest) {
	USART_Buffer_t* const buffer = &usart_data->buffer;
	const DT_byte tempHead = buffer->RX_Head;
	const DT_byte tempTail = buffer->RX_Tail;

	/*
	 DEBUG_BYTE((&tempTail,1))
	 DEBUG_BYTE((&tempHead,1))
	 DEBUG_BYTE((buffer->RX, 127))
	 */

	// Init-Fehlerbytes ausfiltern
	if (buffer->RX[tempTail] != 0xFF && (tempHead != tempTail)) {
		DEBUG(("COM_i",sizeof("COM_i")))
		USART_RXBuffer_GetByte(usart_data);
		return 0;
	}
	// Pruefen ob min. 4 Bytes im Buffer sind, um Laenge zu lesen
	else if (USART_RXBuffer_checkPointerDiff(tempTail, tempHead, 4)) {
		DEBUG(("COM_le",sizeof("COM_le")))
		return 0;
	}
	// Byte #1 und Byte #2 muessen laut Protokoll 0xFF sein
	else if ((buffer->RX[tempTail] != 0xFF) && (buffer->RX[(tempTail + 1)
			& USART_RX_BUFFER_MASK] != 0xFF)) {
		DEBUG(("COM_ff",sizeof("COM_ff")))
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
			DEBUG(("COM_uc",sizeof("COM_uc")))
			return 0;
		}
		// Copy packet from buffer in destination array
		DT_byte i;
		for (i = 0; i < length; i++) {
			dest[i] = USART_RXBuffer_GetByte(usart_data);
		}

		// Pruefen ob Checksumme korrekt ist
		if (dest[length - 1] != COM_getChecksum(dest, length)) {
			DEBUG(("COM_cks",sizeof("COM_cks")))
			usart_data->buffer.RX_Tail = 0;
			usart_data->buffer.RX_Head = 0;
			// DEBUG_BYTE((dest, length))
			return 0;
		}

		DEBUG(("COM_ok",sizeof("COM_ok")))
		return length;
	}
}

/**
 * \brief	Versenden von Daten an anderen Controller.
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
DT_byte COM_send(DT_byte* const packet, DT_size l, DT_byte* const result,
		DT_bool hasResponse) {
	packet[l - 1] = COM_getChecksum(packet, l);
	DT_byte cpuID = packet[2];
	USART_data_t* usart_data;
	DT_byte len = 0;

	if (cpuID == COM_BRDCAST_ID) {
		DEBUG(("SND_BDC", sizeof("SND_BDC")));
		usart_data = &XM_com_data1;
		XM_USART_send(usart_data, packet, l);
		usart_data = &XM_com_data3;
		XM_USART_send(usart_data, packet, l);
	} else {
		if (cpuID == COM_SLAVE3F || cpuID == COM_MASTER) {
			DEBUG(("SND_S3_M", sizeof("SND_S3_M")))
			usart_data = &XM_com_data3;
		} else if (cpuID == COM_SLAVE1B) {
			DEBUG(("SND_S1", sizeof("SND_S1")))
			usart_data = &XM_com_data1;
		}
		// packet[2] -> ID
		XM_USART_send(usart_data, packet, l);

		DT_size timeout = hasResponse ? 1000 : 0;

		while (len == 0 && hasResponse /* && timeout > 0 */) {
			len = COM_receive(usart_data, result);
			timeout--;
		}
	}
	return len;
}

/**
 * \brief	Ruft den Status eines Controllers ab.
 *
 * 			Ruft den Status eines Controllers ab. Broadcast nicht möglich!
 *
 * \param	cpuID	ID des Controllers
 * \param	param	Parameter
 * \param	result	Zielfeld für Antowort
 *
 * \return Größe der empfangenen Antwort
 */
DT_size COM_requestStatus(DT_byte cpuID, DT_byte param, DT_byte* const result) {
	// Broadcast bei requestStatus nicht möglich
	if (cpuID == COM_BRDCAST_ID)
		return 0;
	const DT_size len = 7;
	DT_byte packet[len];
	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = cpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_STATUS;
	packet[5] = param;
	// packet[6] = checksum will set in send
	return COM_send(packet, len, result, true);

}

/**
 * \brief	Konvertiert einen Double-Wert in ein Byte-Array.
 *
 * 			Konvertiert einen Double-Wert in ein Byte-Array.
 *
 * \param	value	Double-Wert
 * \param	array	Zielfeld
 */
void COM_doubleToByteArray(const DT_double value, DT_byte* const array) {
	DT_byte* ptr = (DT_byte*) &value;
	for (DT_size i = 0; i < sizeof(DT_double); i++)
		array[i] = ptr[i];
}

/**
 * \brief	Konvertiert ein Byte-Array zu einem Double-Wert.
 *
 * 			Konvertiert ein Byte-Array zu einem Double-Wert.
 *
 * \param	array	Byte-Array mit Double-Wert
 *
 * \return	Double-Wert
 */
DT_double COM_byteArrayToDouble(const DT_byte* const array) {
	DT_double value;
	DT_byte* ptr = (DT_byte*) &value;
	for (DT_size i = 0; i < sizeof(DT_double); i++)
		ptr[i] = array[i];
	return value;
}

/**
 * \brief	Prüft ob Flag für rechtes Bein gesetzt ist.
 *
 * 			Prüft ob Flag für rechtes Bein gesetzt ist.
 *
 * \param	result	Zu prüfendes Packet
 *
 * \return	true, wenn Flag gesetzt
 */
DT_bool COM_isRightLeg(const DT_byte* const result) {
	return COM_CONF_RIGHT == (result[5] & COM_CONF_RIGHT);
}

/**
 * \brief	Prüft ob Flag für linkes Bein gesetzt ist.
 *
 * 			Prüft ob Flag für linkes Bein gesetzt ist.
 *
 * \param	result	Zu prüfendes Packet
 *
 * \return	true, wenn Flag gesetzt
 */
DT_bool COM_isLeftLeg(const DT_byte* const result) {
	return (COM_CONF_LEFT == (result[5] & COM_CONF_LEFT));
}

/**
 * \brief	Prüft ob Flag für globalen Punkt gesetzt ist.
 *
 * 			Prüft ob Flag für globalen Punkt gesetzt ist.
 *
 * \param	result	Zu prüfendes Packet
 *
 * \return	true, wenn Flag gesetzt
 */
DT_bool COM_isGlobal(const DT_byte* const result) {
	return (COM_CONF_GLOB == (result[5] & COM_CONF_GLOB));
}

/**
 * \brief	Prüft ob Flag für Hüftgelenk gesetzt ist.
 *
 * 			Prüft ob Flag für Hüftgelenk gesetzt ist.
 *
 * \param	result	Zu prüfendes Packet
 *
 * \return	true, wenn Flag gesetzt
 */
DT_bool COM_isHip(const DT_byte* const result) {
	return (COM_CONF_HIP == (result[5] & COM_CONF_HIP));
}

/**
 * \brief	Prüft ob Flag für Kniegelenk gesetzt ist.
 *
 * 			Prüft ob Flag für Kniegelenk gesetzt ist.
 *
 * \param	result	Zu prüfendes Packet
 *
 * \return	true, wenn Flag gesetzt
 */
DT_bool COM_isKnee(const DT_byte* const result) {
	return (COM_CONF_KNEE == (result[5] & COM_CONF_KNEE));
}

/**
 * \brief	Prüft ob Flag für Fußgelenk gesetzt ist.
 *
 * 			Prüft ob Flag für Fußgelenk gesetzt ist.
 *
 * \param	result	Zu prüfendes Packet
 *
 * \return	true, wenn Flag gesetzt
 */
DT_bool COM_isFoot(const DT_byte* const result) {
	return (COM_CONF_FOOT == (result[5] & COM_CONF_FOOT));
}

/**
 * \brief	Sendet einen Punkt an einen Controller.
 *
 * 			Sendet einen Punkt an einen Controller.
 *
 * \param	cpuID	ID des Controllers
 * \param	point	Zuversendender Punkt
 * \param	config	Parameter, z.B. global, left, right ...
 *
 * \return false, wenn Fehler
 */
DT_bool COM_sendPoint(DT_byte cpuID, const DT_point* const point,
		const DT_byte config) {
	DEBUG(("pre_snd_pnt",sizeof("pre_snd_pnt")))
	// Broadcast bei requestStatus nicht möglich
	if (cpuID == COM_BRDCAST_ID)
		return 0;
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len = 7 + 3 * sizeof(DT_double);
	DT_byte packet[len];

	// Point auf ByteArray casten
	COM_doubleToByteArray(point->x, &packet[6 + 0 * sizeof(DT_double)]);
	COM_doubleToByteArray(point->y, &packet[6 + 1 * sizeof(DT_double)]);
	COM_doubleToByteArray(point->z, &packet[6 + 2 * sizeof(DT_double)]);

	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = cpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_POINT;
	packet[5] = config;

	// packet[17] = checksum will set in send
	DEBUG(("aft_snd_pnt",sizeof("aft_snd_pnt")))
	len = COM_send(packet, len, result, true);
	if ((len > 0) && (result[4] == COM_ACK))
		return true;
	else
		return false;
}

/**
 * \brief	Sendet einen Punkt und Anfahrgeschwindigkeit an einen Controller.
 *
 * 			Sendet einen Punkt und Anfahrgeschwindigkeit zusammen in einem Packet an einen Controller.
 *
 * \param	cpuID	ID des Controllers
 * \param	point	Zuversendender Punkt
 * \param	speed	Anfahrgeschwindigkeit
 * \param	config	Parameter, z.B. global, left, right ...
 *
 * \return false, wenn Fehler
 */
DT_bool COM_sendPointAndSpeed(DT_byte cpuID, const DT_point* const point,
		const DT_double speed, const DT_byte config) {
	DEBUG(("pre_snd_pnt",sizeof("pre_snd_pnt")))
	// Broadcast bei requestStatus nicht möglich
	if (cpuID == COM_BRDCAST_ID)
		return 0;
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len = 24;
	DT_byte packet[len];

	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = cpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_POINT;
	packet[5] = config;

	// Point auf ByteArray casten
	COM_doubleToByteArray(point->x, &packet[6 + 0 * sizeof(DT_double)]);
	COM_doubleToByteArray(point->y, &packet[6 + 1 * sizeof(DT_double)]);
	COM_doubleToByteArray(point->z, &packet[6 + 2 * sizeof(DT_double)]);

	packet[18] = COM_SPEED;
	COM_doubleToByteArray(speed, &packet[19]);

	// packet[17] = checksum will set in send
	DEBUG(("aft_snd_pnt",sizeof("aft_snd_pnt")))
	len = COM_send(packet, len, result, true);
	if ((len > 0) && (result[4] == COM_ACK))
		return true;
	else
		return false;
}

/**
 * \brief	Ermittelt aus einem Packet den Punkt.
 *
 * 			Ermittelt aus einem Packet den Punkt.
 *
 * \param	result	Zu prüfendes Packet
 *
 * \return Point
 */
DT_point COM_getPointFromPacket(const DT_byte* const result) {
	DT_point p;

	p.x = COM_byteArrayToDouble(&result[6 + 0 * sizeof(DT_double)]);
	p.y = COM_byteArrayToDouble(&result[6 + 1 * sizeof(DT_double)]);
	p.z = COM_byteArrayToDouble(&result[6 + 2 * sizeof(DT_double)]);

	return p;
}

/**
 * \brief	Ermittelt aus einem Packet die Anfahrgeschwindigkeit.
 *
 * 			Ermittelt aus einem Packet die Anfahrgeschwindigkeit.
 *
 * \param	result	Zu prüfendes Packet
 *
 * \return Anfahrgeschwindigkeit
 */
DT_double COM_getSpeedFromPacket(const DT_byte* const result) {
	DT_double speed;
	speed = COM_byteArrayToDouble(&result[19]);
	return speed;
}

/**
 * \brief	Sendet einen Winkel an einen Controller.
 *
 * 			Sendet einen Winkel an einen Controller.
 *
 * \param	cpuID	ID des Controllers
 * \param	angle	Zuversendender Winkel
 * \param	config	Parameter, z.B. global, left, right ...
 *
 * \return false, wenn Fehler
 */
DT_bool COM_sendAngle(DT_byte cpuID, const DT_double angle,
		const DT_byte config) {
	DEBUG(("pre_snd_pnt",sizeof("pre_snd_pnt")))
	// Broadcast bei requestStatus nicht möglich
	if (cpuID == COM_BRDCAST_ID)
		return 0;
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len = 7 + sizeof(DT_double);
	DT_byte packet[len];

	// Angle auf ByteArray casten
	COM_doubleToByteArray(angle, &packet[6]);

	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = cpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_ANGLE;
	packet[5] = config;

	DEBUG(("aft_snd_pnt",sizeof("aft_snd_pnt")))
	len = COM_send(packet, len, result, true);
	if ((len > 0) && (result[4] == COM_ACK))
		return true;
	else
		return false;
}

/**
 * \brief	Ermittelt aus einem Packet den Winkel.
 *
 * 			Ermittelt aus einem Packet den Winkel.
 *
 * \param	result	Zu prüfendes Packet
 *
 * \return Winkel
 */
DT_double COM_getAngleFromPacket(const DT_byte* const result) {
	DT_double angle;
	angle = COM_byteArrayToDouble(&result[6]);
	return angle;
}

/**
 * \brief	Sendet das ACTION-Kommando an einen Controller.
 *
 * 			Sendet das ACTION-Kommando an einen Controller.
 *
 * \param	cpuID	ID des Controllers
 */
void COM_sendAction(DT_byte cpuID) {
	DEBUG(("snd_act",sizeof("snd_act")))
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	const DT_size len = 6;
	DT_byte packet[len];
	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = cpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_ACTION;
	// packet[5] = checksum will set in send
	COM_send(packet, len, result, false);
}

/**
 * \brief	Sendet eine isAlive-Anfrage an einen Controller.
 *
 * 			Sendet eine isAlive-Anfrage an einen Controller.
 *
 * \param	cpuID	ID des Controllers
 *
 * \return	true, wenn Alive
 */
DT_bool COM_isAlive(DT_byte cpuID) {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	DT_size len;
	len = COM_requestStatus(cpuID, COM_IS_ALIVE, result);
	if ((len > 0) && (result[4] == COM_ACK))
		return true;
	else
		return false;
}

/**
 * \brief	Sendet ein ACK an einen Controller.
 *
 * 			Sendet ein ACK an einen Controller.
 *
 * \param	cpuID	ID des Controllers
 */
void COM_sendACK(DT_byte cpuID) {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	const DT_size len = 6;
	DT_byte packet[len];
	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = cpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_ACK;
	// packet[5] = checksum will set in send
	COM_send(packet, len, result, false);
}

/**
 * \brief	Sendet ein NAK an einen Controller.
 *
 * 			Sendet ein NAK an einen Controller.
 *
 * \param	cpuID	ID des Controllers
 * \param	errCode	Fehlercode
 */
void COM_sendNAK(DT_byte cpuID, DT_byte errCode) {
	DT_byte result[DT_RESULT_BUFFER_SIZE];
	const DT_size len = 7;
	DT_byte packet[len];
	packet[0] = COM_START_BYTE;
	packet[1] = COM_START_BYTE;
	packet[2] = cpuID;
	packet[3] = len - 4; // length
	packet[4] = COM_NAK;
	packet[5] = errCode;
	// packet[6] = checksum will set in send
	COM_send(packet, len, result, false);
}
