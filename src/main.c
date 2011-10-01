#define F_CPU 32000000UL

#include "avr/io.h"
#include "include/avr_compiler.h"
#include "include/usart_driver.h"
// #include "include/clksys_driver.h"
#include "math.h"

#define TEST_OFF
#ifdef TEST_ON

// Port-Defines
#define LEDPORT PORTQ        // Port PORTQ
#define DEBUGPORT PORTF
#define SERVOPORT PORTC

#define LEDMASK (1<<PIN3)    // Status LED an PQ3
#define DEBUGUSART USARTF0
#define SERVOUSART USARTC0
#define OE_L_MASK (1<<PIN0)

uint8_t buffer[512];

int main(void) {
	// Variablendeklaration
	uint8_t i;
	double testo = 1.78;

	// INIT
	LEDPORT.DIRSET = LEDMASK;
	SERVOPORT.DIRSET = OE_L_MASK;

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
	CLKSYS_Enable( OSC_XOSCEN_bm );

	// Warten bis der Oszillator bereit ist
	do {
	} while (CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0);

	// PLL source ist XOSC, Multiplikator x2
	CLKSYS_PLL_Config(OSC_PLLSRC_XOSC_gc, 2);

	// Enable PLL
	CLKSYS_Enable( OSC_PLLEN_bm );

	// Prescalers konfigurieren
	CLKSYS_Prescalers_Config(CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);

	// Warten bis PLL locked
	do {
	} while (CLKSYS_IsReady( OSC_PLLRDY_bm ) == 0);

	// Main Clock Source ist Ausgang von PLL
	CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_PLL_gc);

	// Nun ist der System Clock 32MHz !

	/* Hinweis:
	 32kHz TOSC kann nicht in Verbindung mit PLL genutzt werden, da
	 die minimale Eingangsfrequenz des PLLs 400kHz betr�gt.
	 */

	/******************************************************************
	 * Debug-Usart initialisieren, 8N1 250kBit
	 ******************************************************************/

	DEBUGPORT.DIRSET = PIN3_bm; // Pin3 von PortF (TXD0) ist Ausgang

	DEBUGPORT.DIRCLR = PIN2_bm; // Pin2 von PortF (RXD0) ist Eingang

	// USARTF0, 8 Data bits, No Parity, 1 Stop bit.
	USART_Format_Set(&DEBUGUSART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Bitrate einstellen

	 Beispiele BSEL in Abh�ngigkeit von der Bitrate, fck = 32MHz, Error < 0,8%

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

	 Bemerkung: Gepr�ft wurde mit 250.000bps im USBxpress Modus
	 */

	USART_Baudrate_Set(&DEBUGUSART, 7 , 0); // 250.000bps (BSEL = 7)

	/* Enable RX and TX. */
	USART_Rx_Enable(&DEBUGUSART);
	USART_Tx_Enable(&DEBUGUSART);

	USART_GetChar(&DEBUGUSART); // Flush Receive Buffer

	/******************************************************************
	 * Servo-Usart initialisieren, 8N1 250kBit
	 ******************************************************************/

	SERVOPORT.DIRSET = PIN3_bm; // Pin3 von PortC (TXD0) ist Ausgang

	SERVOPORT.DIRCLR = PIN2_bm; // Pin2 von PortC (RXD0) ist Eingang

	// USARTF0, 8 Data bits, No Parity, 1 Stop bit.
	USART_Format_Set(&SERVOUSART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	USART_Baudrate_Set(&SERVOUSART, 207 , 0); // 9.600bps (BSEL = 207)

	/* Enable RX and TX. */
	USART_Rx_Enable(&SERVOUSART);
	USART_Tx_Enable(&SERVOUSART);

	USART_GetChar(&SERVOUSART); // Flush Receive Buffer


	int len = 4;
	int checksum = 0;

	buffer[0] = 0xFF;
	buffer[1] = 0xFF;
	buffer[2] = 0xFE;
	buffer[3] = len;
	buffer[4] = 0x03;
	buffer[5] = 0x19;
	buffer[6] = 0x01;

	for (i = 2; i <= 6; i++)
		checksum += buffer[i];

	buffer[7] = ~checksum;

	/******************************************************************
	 * Main Loop
	 ******************************************************************/

	LEDPORT.OUTCLR = LEDMASK;
	SERVOPORT.OUTCLR = OE_L_MASK;
	delay_us( 500000 ); // 500ms
	sqrt(9);
	testo = cos(testo);
	while (!USART_IsTXDataRegisterEmpty(&DEBUGUSART))
		;
	USART_PutChar(&DEBUGUSART, 'D');

	for (i = 0; i <= 7; i++) {
		while (!USART_IsTXDataRegisterEmpty(&SERVOUSART))
			;
		USART_PutChar(&SERVOUSART, buffer[i]);
	}

	SERVOPORT.OUTSET = OE_L_MASK;


	while (1) {
	}
	return 0;
}
#endif
