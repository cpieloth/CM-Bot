/**
 * \file	remote.c
 *
 * \brief	Methoden zur Steuerung durch den RC-100 Remote Controller.
 */

#include "include/remote.h"
#include "include/xmega.h"
#include "include/utils.h"

// Commands
#define B_NON_PRESSED 0x0000

#define B_U 0x0001
#define B_D 0x0002
#define B_L 0x0004
#define B_R 0x0008

#define B_1 0x0010
#define B_2 0x0020
#define B_3 0x0040
#define B_4 0x0080
#define B_5 0x0100
#define B_6 0x0200

/**
 * \brief	Liest empfangene Befehle vom Remote-Controller aus.
 *
 * Instruction aus dem High- und Low-Teil zusammensetzen
 * Bsp:
 * 	Paket für B_2:
 *      0  1  2  3  4  5  ...
 * 		FF;55;20;DF;00;FF;FF;55;00;FF;00;FF;
 *            LL    HH
 * 	Eigentliche Information:
 * 		0x0020
 *  d.h.
 *      H = Paket[4]
 *      L = Paket[2]
 */
DT_cmd RMT_getCommand() {
	DT_byte result[DT_RESULT_BUFFER_SIZE], i;
	DT_cmd cmd = 0x0000;
	DT_cmd tmp_cmd = 0xFFFF;
	DT_bool button_release = false;
	// DT_size timeout = 65000;
	for (i = 0; i < DT_RESULT_BUFFER_SIZE; i++)
		result[i] = 0x00;
	if (RMT_receive(&XM_remote_data, result) > 0) {
		DEBUG(("CMD_EX", sizeof("CMD_EX")))
		cmd = (result[4] << 8) | result[2];
	} else {
		cmd = B_NON_PRESSED;
	}
	while (!button_release /* && timeout > 0*/) {
		RMT_receive(&XM_remote_data, result);
		tmp_cmd = (result[4] << 8) | result[2];
		if (tmp_cmd == B_NON_PRESSED)
			button_release = true;
		//timeout--;
	}
	//if(cmd!=0)DEBUG_BYTE((&cmd,sizeof(&cmd)))
	return cmd;
}

/**
 * \brief 	USART-Empfangsmethode für Remote-Controller.
 *
 * 			Diese Methode liest den Remote-USART-Buffer aus und prüft,
 * 			ob ein vollständiges Paket empfangen wurde.
 *
 * \param	usart_data	USART
 * \param	dest		Byte-Array für Antwort-Paket
 *
 * \return	Länge des Antwortpakets
 */
DT_byte RMT_receive(USART_data_t* const usart_data, DT_byte* const dest) {
	USART_Buffer_t* buffer = &usart_data->buffer;
	const DT_byte tempHead = buffer->RX_Head;
	const DT_byte tempTail = buffer->RX_Tail;
	DT_byte length = 6;

	// Sind Daten vorhanden
	if (USART_RXBuffer_checkPointerDiff(tempTail, tempHead, length)) {
		//DEBUG(("RMT_nd",sizeof("RMT_nd")))
		return 0;
	}
	// Byte #1 und Byte #2 muessen laut Protokoll 0xFF und 0x55 sein
	else if ((buffer->RX[buffer->RX_Tail] != 0xFF)
			&& (buffer->RX[(buffer->RX_Tail + 1) & USART_RX_BUFFER_MASK]
					!= 0x55)) {
		DEBUG(("RMT_ff",sizeof("RMT_ff")))
		return 0;
	} else {
		DT_byte i;
		for (i = 0; i < length; i++) {
			dest[i] = USART_RXBuffer_GetByte(usart_data);
		}
		DEBUG(("RMT_ok",sizeof("RMT_ok")))
		return length;
	}
}

/**
 * \brief 	Kein Taster gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_NonPressed(DT_cmd cmd) {
	if (cmd == B_NON_PRESSED)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster Up gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isUpPressed(DT_cmd cmd) {
	if ((cmd & B_U) == B_U)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster Down gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isDownPressed(DT_cmd cmd) {
	if ((cmd & B_D) == B_D)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster Left gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isLeftPressed(DT_cmd cmd) {
	if ((cmd & B_L) == B_L)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster Right gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isRightPressed(DT_cmd cmd) {
	if ((cmd & B_R) == B_R)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster 1 gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isButton1Pressed(DT_cmd cmd) {
	if ((cmd & B_1) == B_1)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster 2 gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isButton2Pressed(DT_cmd cmd) {
	if ((cmd & B_2) == B_2)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster 3 gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isButton3Pressed(DT_cmd cmd) {
	if ((cmd & B_3) == B_3)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster 4 gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isButton4Pressed(DT_cmd cmd) {
	if ((cmd & B_4) == B_4)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster 5 gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isButton5Pressed(DT_cmd cmd) {
	if ((cmd & B_5) == B_5)
		return true;
	else
		return false;
}

/**
 * \brief 	Ist Taster 6 gedrückt.
 *
 * 			Achtung: cmd muss zunächst durch getCommand abgerufen werden.
 *
 * \param	cmd		16-Bit Command-Wert
 *
 * \return	Bool
 */
DT_bool RMT_isButton6Pressed(DT_cmd cmd) {
	if ((cmd & B_6) == B_6)
		return true;
	else
		return false;
}
