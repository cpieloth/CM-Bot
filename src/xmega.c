/**
 * \file 	xmega.c
 *
 * \brief 	Spezifische Funktionen für den Mikrocontroller ATXmega128A1.
 */

#include "include/xmega.h"
#include "include/dynamixel.h"
#include "include/utils.h"
#include "include/clksys_driver.h"
#include "include/avr_compiler.h"
#include "include/communication.h"
#include <avr/io.h>
#include <stdlib.h>

/**
 * \brief 	Initialisierung der CPU.
 */
void XM_init_cpu() {
	cli();
	/******************************************************************
	 * System Clock 32MHz (XOSC Quarz 16MHz, PLL Faktor 2)
	 ******************************************************************/

	/* Nach dem Reset ist die Quelle des Systemtaktes der interne
	 2MHz RC-Oszillator (System Clock Selection: RC2MHz)
	 */

	// Oszillator XOSC konfigurieren (12..16MHz, 256 clocks startup time)
	CLKSYS_XOSC_Config(OSC_FRQRANGE_12TO16_gc, false,
			OSC_XOSCSEL_XTAL_256CLK_gc);

	// Oszillator XOSC enable
	CLKSYS_Enable(OSC_XOSCEN_bm);

	// Warten bis der Oszillator bereit ist
	do {
	} while (CLKSYS_IsReady(OSC_XOSCRDY_bm) == 0);

	// PLL source ist XOSC, Multiplikator x2
	CLKSYS_PLL_Config(OSC_PLLSRC_XOSC_gc, 2);

	// Enable PLL
	CLKSYS_Enable(OSC_PLLEN_bm);

	// Prescalers konfigurieren
	CLKSYS_Prescalers_Config(CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);

	// Warten bis PLL locked
	do {
	} while (CLKSYS_IsReady(OSC_PLLRDY_bm) == 0);

	// Main Clock Source ist Ausgang von PLL
	CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_PLL_gc);

	// Nun ist der System Clock 32MHz !

	/* Hinweis:
	 32kHz TOSC kann nicht in Verbindung mit PLL genutzt werden, da
	 die minimale Eingangsfrequenz des PLLs 400kHz betraegt.
	 */

	/******************************************************************
	 * Debug-Usart initialisieren, 8N1 250kBit
	 ******************************************************************/
	XM_PORT_DEBUG.DIRSET = PIN3_bm; // Pin3 von PortF (TXD0) ist Ausgang
	XM_PORT_DEBUG.DIRCLR = PIN2_bm; // Pin2 von PortF (RXD0) ist Eingang

	// Use USART and initialize buffers
	USART_InterruptDriver_Initialize(&XM_debug_data, &XM_USART_DEBUG,
			USART_DREINTLVL_OFF_gc);
	// USARTF0, 8 Data bits, No Parity, 1 Stop bit.
	USART_Format_Set(XM_debug_data.usart, USART_CHSIZE_8BIT_gc,
			USART_PMODE_DISABLED_gc, false);

	/* Bitrate einstellen

	 Beispiele BSEL in Abhaengigkeit von der Bitrate, fck = 32MHz, Error < 0,8%

	 7 = 250.000bps
	 30 = 128.000bps
	 34 =  57.600bps
	 51 =  38.400bps
	 68 =  28.800bps
	 103 =  19.200bps
	 138 =  14.400bps
	 207 =   9.600bps
	 416 =   4.800bps
	 832 =   2.400bps
	 1666 =   1.200bps

	 Bemerkung: Geprueft wurde mit 250.000bps im USBxpress Modus
	 */

	USART_Baudrate_Set(XM_debug_data.usart, 7, 0); // 250.000bps (BSEL = 7)

	/* Enable RX and TX. */
	//USART_Rx_Enable(XM_debug_data.usart);
	USART_Tx_Enable(XM_debug_data.usart);

	USART_GetChar(XM_debug_data.usart); // Flush Receive Buffer
	DEBUG(("DEBUG-USART ... ON", sizeof("DEBUG-USART ... ON")));

	// Init LED
	XM_PORT_LED.DIRSET = XM_LED_MASK;
	XM_LED_ON

	// Init Taster
	SWITCHPORT.DIRCLR = SWITCHMASK;
	SWITCHPORT.PIN2CTRL |= (0b011 << 3); // Pullup PQ2 aktivieren

	sei();
}

/**
 * \brief 	Initialisiert den Zigbee-Fernsteuerung.
 */
void XM_init_remote() {
	cli();
	// Set pins for TX and RX
	XM_PORT_REMOTE.DIRSET = PIN7_bm; // Pin6 of PortC (TXD0) is output
	XM_PORT_REMOTE.DIRCLR = PIN6_bm; // Pin7 of PortC (RXD0) is input

	// Use USARTC0 / USARTD0 and initialize buffers
	USART_InterruptDriver_Initialize(&XM_remote_data, &XM_USART_REMOTE,
			USART_DREINTLVL_OFF_gc);

	// 8 Data bits, No Parity, 1 Stop bit
	USART_Format_Set(XM_remote_data.usart, USART_CHSIZE_8BIT_gc,
			USART_PMODE_DISABLED_gc, false);

	// Enable RXC interrupt
	USART_RxdInterruptLevel_Set(XM_remote_data.usart, USART_RXCINTLVL_LO_gc);

	// Set Baudrate
	USART_Baudrate_Set(XM_remote_data.usart, 34, 0); // 57.600 bps (BSEL = 34)

	// Enable RX and TX
	USART_Rx_Enable(XM_remote_data.usart);
	//USART_Tx_Enable(XM_remote_data.usart);

	// Flush Receive Buffer
	USART_GetChar(XM_remote_data.usart); // Flush Receive Buffer

	// aktiviere Medium Level Interrupts
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

	sei();
}

/**
 * \brief 	Initialisiert die Servo-USARTs.
 */
void XM_init_dnx() {
	//Disable Interrupts
	cli();

	XM_servo_data_R.port = &XM_PORT_SERVO_R;
	XM_servo_data_L.port = &XM_PORT_SERVO_L;

	// Set pins for TX and RX
	XM_servo_data_R.port->DIRSET = PIN3_bm; // Pin3 of PortC (TXD0) is output
	XM_servo_data_R.port->DIRCLR = PIN2_bm; // Pin2 of PortC (RXD0) is input

	XM_servo_data_L.port->DIRSET = PIN3_bm;
	XM_servo_data_L.port->DIRCLR = PIN2_bm;

	// Set pin, dir and out for OE
	XM_servo_data_R.port->DIRSET = XM_OE_MASK;
	XM_servo_data_R.port->OUTSET = XM_OE_MASK;

	XM_servo_data_L.port->DIRSET = XM_OE_MASK;
	XM_servo_data_L.port->OUTSET = XM_OE_MASK;

	// Use USARTC0 / USARTD0 and initialize buffers
	USART_InterruptDriver_Initialize(&XM_servo_data_R, &XM_USART_SERVO_R,
			USART_DREINTLVL_HI_gc);
	USART_InterruptDriver_Initialize(&XM_servo_data_L, &XM_USART_SERVO_L,
			USART_DREINTLVL_HI_gc);

	// 8 Data bits, No Parity, 1 Stop bit
	USART_Format_Set(XM_servo_data_R.usart, USART_CHSIZE_8BIT_gc,
			USART_PMODE_DISABLED_gc, false);
	USART_Format_Set(XM_servo_data_L.usart, USART_CHSIZE_8BIT_gc,
			USART_PMODE_DISABLED_gc, false);

	// Enable RXC interrupt
	USART_RxdInterruptLevel_Set(XM_servo_data_R.usart, USART_RXCINTLVL_LO_gc);
	USART_RxdInterruptLevel_Set(XM_servo_data_L.usart, USART_RXCINTLVL_LO_gc);

	// Set Baudrate
	USART_Baudrate_Set(XM_servo_data_R.usart, 1, 0); // 1 Mbps (BSEL = 1)
	USART_Baudrate_Set(XM_servo_data_L.usart, 1, 0); // 1 Mbps (BSEL = 1)

	// Enable RX and TX
	USART_Rx_Enable(XM_servo_data_R.usart);
	USART_Rx_Enable(XM_servo_data_L.usart);

	USART_Tx_Enable(XM_servo_data_R.usart);
	USART_Tx_Enable(XM_servo_data_L.usart);

	// Flush Receive Buffer
	USART_GetChar(XM_servo_data_R.usart); // Flush Receive Buffer
	USART_GetChar(XM_servo_data_L.usart); // Flush Receive Buffer

	// Enable PMIC interrupt level high (für TX-Complete-Interrupt)
	PMIC.CTRL |= PMIC_HILVLEX_bm;
	// Enable PMIC interrupt level medium
	PMIC.CTRL |= PMIC_MEDLVLEX_bm;
	// Enable PMIC interrupt level low
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	// Enable Round-Robin-Scheduling
	PMIC.CTRL |= PMIC_RREN_bm;
	// Enable global interrupts
	sei();

}

/**
 * \brief 	Initialisiert USARTs für die CPU-Kommunikation.
 *
 * 			ID des Controllers wird aufgrund des Hardwaredefekts für die Master-Slave-Kommunikation benötigt.
 *
 * \param	cpuID	ID des Controllers
 */
void XM_init_com(DT_byte cpuID) {
	cli();

	XM_com_data3.port = &XM_PORT_COM3;

	// Set pins for TX and RX
	XM_com_data3.port->DIRSET = PIN3_bm; // Pin7 of PortC (TXD0) is output
	XM_com_data3.port->DIRCLR = PIN2_bm; // Pin6 of PortC (RXD0) is input

	// Use USARTE0 and initialize buffers
	USART_InterruptDriver_Initialize(&XM_com_data3, &XM_USART_COM3,
			USART_DREINTLVL_MED_gc);

	// 8 Data bits, No Parity, 1 Stop bit
	USART_Format_Set(XM_com_data3.usart, USART_CHSIZE_8BIT_gc,
			USART_PMODE_DISABLED_gc, false);

	// Enable RXC interrupt
	USART_RxdInterruptLevel_Set(XM_com_data3.usart, USART_RXCINTLVL_MED_gc);

	// Set Baudrate
	USART_Baudrate_Set(XM_com_data3.usart, 1, 0); // 1 Mbps (BSEL = 1)

	// Enable RX and TX
	USART_Rx_Enable(XM_com_data3.usart);
	USART_Tx_Enable(XM_com_data3.usart);

	// Flush Receive Buffer
	USART_GetChar(XM_com_data3.usart); // Flush Receive Buffer

	// Enable Interrupt Level Medium
	//PMIC.CTRL |= PMIC_LOLVLEX_bm;

	if(cpuID == COM_MASTER){
		XM_com_data1.port = &XM_PORT_COM1;

		// Set pins for TX and RX
		XM_com_data1.port->DIRSET = PIN7_bm; // Pin3 of PortC (TXD0) is output
		XM_com_data1.port->DIRCLR = PIN6_bm; // Pin2 of PortC (RXD0) is input

		// Use USARTE0 and initialize buffers
		USART_InterruptDriver_Initialize(&XM_com_data1, &XM_USART_COM1,
				USART_DREINTLVL_MED_gc);

		// 8 Data bits, No Parity, 1 Stop bit
		USART_Format_Set(XM_com_data1.usart, USART_CHSIZE_8BIT_gc,
				USART_PMODE_DISABLED_gc, false);

		// Enable RXC interrupt
		USART_RxdInterruptLevel_Set(XM_com_data1.usart, USART_RXCINTLVL_MED_gc);

		// Set Baudrate
		USART_Baudrate_Set(XM_com_data1.usart, 1, 0); // 1 Mbps (BSEL = 1)

		// Enable RX and TX
		USART_Rx_Enable(XM_com_data1.usart);
		USART_Tx_Enable(XM_com_data1.usart);

		// Flush Receive Buffer
		USART_GetChar(XM_com_data1.usart); // Flush Receive Buffer
	}

	sei();
}

/**
 * \brief 	USART-Sendemethode.
 *
 * 			Diese Methode setzt zunächst das jeweilige Output-Enable (!OE) auf Senden
 * 			und schreibt das zu sendende Paket in den USART-Buffer.
 * 			Anschließend wird der TX-Interrupt aktiviert, der ausgelöst wird, wenn das
 * 			letzte Paket gesendet wurde.
 *
 * \param	usart_data	USART-Datenstruktur der zu benutzenden USART
 * \param	txData		Byte-Array mit zu sendendem Paket
 * \param	bytes 		Länge des zu sendenden Pakets
 */
void XM_USART_send(USART_data_t* const usart_data, const DT_byte* const txData,
		DT_size bytes) {
	DT_size i = 0;

	// DEBUG_BYTE((txData, bytes))

	if (usart_data->usart == &XM_USART_DEBUG)
		return;

	// Paketgröße, die später beim Empfangen abgezogen werden muss
	usart_data->lastPacketLength = bytes;
	// Set OE to 0 -> Enable Send
	usart_data->port->OUTCLR = XM_OE_MASK;

	// Send data
	while (i < bytes) {
		DT_bool byteToBuffer;
		byteToBuffer = USART_TXBuffer_PutByte(usart_data, txData[i]);
		if (byteToBuffer) {
			i++;
		}
	}

	// Enable TXC interrupt to set OE to 1
	USART_TxdInterruptLevel_Set(usart_data->usart, USART_TXCINTLVL_HI_gc);
}


/**
 * \brief 	ISR für abgeschlossenen Sendevorgang der USARTC0 (SERVO L).
 */
ISR( USARTC0_TXC_vect)
{
	USART_TxdInterruptLevel_Set(&USARTC0, USART_TXCINTLVL_OFF_gc);
	XM_servo_data_L.port->OUTSET = XM_OE_MASK;
}

/**
 * \brief 	ISR für Sendebereitschaft der USARTC0 (SERVO L).
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&XM_servo_data_L);
}

/**
 * \brief 	ISR für Empfangsvorgang der USARTC0 (SERVO L).
 */
ISR( USARTC0_RXC_vect)
{
	USART_RXComplete(&XM_servo_data_L);
}

/**
 * \brief 	ISR für abgeschlossenen Sendevorgang der USARTD0 (SERVO R).
 */
ISR( USARTD0_TXC_vect)
{
	USART_TxdInterruptLevel_Set(&USARTD0, USART_TXCINTLVL_OFF_gc);
	XM_servo_data_R.port->OUTSET = XM_OE_MASK;
}

/**
 * \brief 	ISR für Sendebereitschaft der USARTD0 (SERVO R).
 */
ISR(USARTD0_DRE_vect)
{
	USART_DataRegEmpty(&XM_servo_data_R);
}

/**
 * \brief 	ISR für Empfangsvorgang der USARTD0 (SERVO R).
 */
ISR( USARTD0_RXC_vect)
{
	USART_RXComplete(&XM_servo_data_R);
}

/**
 * \brief 	ISR für abgeschlossenen Sendevorgang der USARTD0 (SERVO R).
 */
ISR( USARTD1_TXC_vect)
{
	USART_TxdInterruptLevel_Set(&USARTD1, USART_TXCINTLVL_OFF_gc);
}

/**
 * \brief 	ISR für Sendebereitschaft der USARTD0 (SERVO R).
 */
ISR(USARTD1_DRE_vect)
{
	USART_DataRegEmpty(&XM_com_data1);
}

/**
 * \brief 	ISR für Empfangsvorgang der USARTD0 (SERVO R).
 */
ISR( USARTD1_RXC_vect)
{
	USART_RXComplete(&XM_com_data1);
}


/**
 * \brief 	ISR für abgeschlossenen Sendevorgang der USARTD0 (SERVO R).
 */
ISR( USARTE0_TXC_vect)
{
	USART_TxdInterruptLevel_Set(&USARTE0, USART_TXCINTLVL_OFF_gc);
	//XM_com_data3.port->OUTSET = XM_OE_MASK;
}

/**
 * \brief 	ISR für Sendebereitschaft der USARTD0 (SERVO R).
 */
ISR(USARTE0_DRE_vect)
{
	USART_DataRegEmpty(&XM_com_data3);
}

/**
 * \brief 	ISR für Empfangsvorgang der USARTD0 (SERVO R).
 */
ISR( USARTE0_RXC_vect)
{
	USART_RXComplete(&XM_com_data3);
}

/**
 * \brief 	ISR für Empfangsvorgang der USARTE1 (REMOTE).
 */
ISR( USARTE1_RXC_vect)
{
	USART_RXComplete(&XM_remote_data);
}
