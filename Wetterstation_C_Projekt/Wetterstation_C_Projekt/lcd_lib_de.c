/*=============================================================================

LCD_LIB_de	Funktionen fuer die LCD-Anzeige des MiniMEXLE
=========================================================

Dateiname: lcd_lib_de.h

Bibliothek fuer das LCD-Display des MiniMexle Boards bzw. optimiert fuer LCD-Display HD44780

Autor: 		Steffen Freihofer (2006)
Adaption: 	Thomas Pospiech (2007, Timing-Probleme geloest)
			G. Gruhler, D. Chilachava (2009, Init Ports und Dokumentation)
			P. Blinzinger (2020, Libary umgebaut fuer SimulIDE, kompatibel mit dem Mexle2020 Board)
			T. Fischer (Clean Code)

Version: 	1.5	vom 14.03.2022

=============================================================================*/

// Include von Header-Dateien
#include "lcd_lib_de.h"

// Funktionsprototypen

void lcd_enable (void);
void lcd_write (unsigned char byte);
void lcd_init (void);
void lcd_putc (unsigned char zeichen);
void lcd_putstr (char *string);
void lcd_gotoxy (unsigned char line, unsigned char pos);
void lcd_clearDisplay(void);
void lcd_clearline (unsigned char line);
void lcd_displayMessage(char *string, unsigned char, unsigned char);

// LCD-Ansteuerfunktionen =====================================================

// ----------------------------------------------------------------------------
// LCD_ENABLE:		Erzeugt einen HIGH-Impuls des Enable-Signals
// ----------------------------------------------------------------------------

void lcd_enable (void)
{
	PORT_CTRL |=  (1 << PIN_EN);		// PIN_EN=1   1  +-+
	_delay_us (DLY_CMD_SHRT);			//               | |
	PORT_CTRL &= ~(1 << PIN_EN);		// PIN_EN=0   0--+ +---
}

// ----------------------------------------------------------------------------
// LCD_WRITE:	Schreibt ein Byte im 4-Bit-Modus in das LCD
// ----------------------------------------------------------------------------

void lcd_write (unsigned char byte)	// Ausgabewert 8 Bit in "byte"
{
	unsigned char hiNibble;		// Definition lokale Variable "lowNibble"

	hiNibble = byte>>4;			// Ausgabewert zwischenspeichern

	PORT_DATA &= 0xF0;			// Untere 4 Bit auf Datenport loeschen
	PORT_DATA |= (hiNibble);	// Ausgabewert 4 Bit nach rechts schieben
								//    oberes Nibble auf Daten-Port LCD schreiben

	lcd_enable();				// Erzeugung Enable-Impuls zur Datenspeicherung
								// keine Wartezeit zwischen Nibbles notwendig

	byte &= 0x0F;				// Unteres Nibble Ausgabewert extrahieren

	PORT_DATA &= 0xF0;			// Untere 4 Bit auf Datenport loeschen
	PORT_DATA |= byte;			// unteres Nibble auf Daten-Port LCD schreiben

	lcd_enable();				// Erzeugung Enable-Impuls zur Datenspeicherung
}

// ----------------------------------------------------------------------------
// LCD_INIT:	Initialisiert das LCD 
// ----------------------------------------------------------------------------

void lcd_init (void)
{
	cli();						//globale Interrupts deaktivieren

	DDR_DATA  |= 0x0F;							// Bit 0..3 (LCD-Daten) auf Output
	DDR_CTRL  |= ((1<<PIN_EN) | (1<<PIN_RS));	// Bit RS: LCD Register Select und Bit E: LCD Enable auf Output
	PORT_DATA |= 0x0F;							// Port, Bit0..3 (LCD-Daten) SET 

	PORT_CTRL &= ~((1<<PIN_EN) | (1<<PIN_RS));	// Steuersignale auf LOW; PIN_EN und PIN_RS auf 0 setzen
	_delay_ms (DLY_CMD_LONG);					// Verzoegerung 55 us (interne Verarbeitung)

	PORT_DATA &= 0xF0;			// Display in 4-bit Modus initialisieren
	PORT_DATA |= CMD_MOD_4BIT1;
	lcd_enable();
	
	_delay_ms (DLY_INIT_LONG);
	lcd_enable ();
	_delay_ms (DLY_INIT_LONG);
	lcd_enable ();
		
	PORT_DATA &= 0xF0;			// Untere 4 Bit auf Datenport loeschen
	PORT_DATA |= CMD_MOD_4BIT2;	// 2. Bit auf Daten-Port LCD schreiben
	_delay_ms (DLY_INIT_SHRT);
	
	lcd_enable();
	_delay_us (DLY_CMD_LONG);	// Verzoegerung 55 us (interne Verarbeitung)
	
	lcd_write (CMD_FCT_DIS );	// 2 Zeilen, 5 x 8 Pixel
	_delay_us (DLY_CMD_LONG);	// Verzoegerung 55 us (interne Verarbeitung)
	
	lcd_write (CMD_ENB_DIS );	// Display einschalten
	_delay_us (DLY_CMD_LONG);	// Verzoegerung 55 us (interne Verarbeitung)
	
	lcd_write (CMD_INC_CSR );	// Cursor auf "increment mode" schalten
	_delay_us (DLY_CMD_LONG);	// Verzoegerung 55 us (interne Verarbeitung)

	lcd_write (CMD_CLR_DIS  );	//	Display loeschen (funktioniert nur, wenn es 2x aufgerufen wird)
	_delay_ms (DLY_INIT_SHRT);
	lcd_write (CMD_CLR_DIS  );
	_delay_ms (DLY_INIT_SHRT);

	sei();						// globale Interrrupts aktivieren
}

// ----------------------------------------------------------------------------
// LCD_PUTC:	Schreibt ein Zeichen an die entsprechende Speicherstelle
// ----------------------------------------------------------------------------

void lcd_putc (unsigned char character)	// Ausgabewert 8-Bit in "character"
{
	PORT_CTRL |= (1<<PIN_RS);			// Register Select auf HIGH: "Daten ausgeben"
	
	lcd_write (character   );			// Zeichen ausgeben
	_delay_us (DLY_CMD_LONG);			// Verzoegerung 55 us (interne Verarbeitung)
}

// ----------------------------------------------------------------------------
// LCD_PUTSTR:	Schreibt einen String auf das LCD
// ----------------------------------------------------------------------------

void lcd_putstr (char *string)			// Pointer auf String in "string"
{
	while (*string)						// solange nicht Stringende-Zeichen 0x00
	{
		lcd_putc (*string);				// Zeichen aus String holen und ausgeben
		string ++;						// Pointer auf naechstes Zeichen setzen
	}
}

// ----------------------------------------------------------------------------
// LCD_GOTOXY:	Verschiebt den Curser an die angegebene Position
// ----------------------------------------------------------------------------

void lcd_gotoxy (unsigned char line, unsigned char pos)
{
	unsigned char  cmd_Nxt_Lin = CMD_NXT_LIN;	// Setze Bit 6, falls line = 1
	if (line == 0) cmd_Nxt_Lin = 0;				// Ansonsten Bit 6 ist 0
	
	PORT_CTRL &= ~(1<<PIN_RS);					// Register Select auf LOW: "Control ausgeben"
		
	lcd_write (CMD_GOTO_XY + cmd_Nxt_Lin + pos);// Adresse Cursor in DDRAM-Adresse 
	_delay_us (DLY_CMD_LONG);					// Verzoegerung 55 us fuer Interne Verarbeitung				
}

// ----------------------------------------------------------------------------
// LCD_CLEARDISPLAY:	Loescht den gesamten Display-Inhalt
// ----------------------------------------------------------------------------

void lcd_clearDisplay(void)
{
	PORT_CTRL &= ~(1 << PIN_RS);		// Register Select auf LOW: "Control ausgeben"
	
	lcd_write (CMD_CLR_DIS );			// Befehlscode an Display ausgeben
	_delay_us (DLY_CMD_SHRT);			// Verzoegerung 2 us fuer interne Verarbeitung
}

// ----------------------------------------------------------------------------
// LCD_CLEARLINE:	Loescht den Inhalt einer Display-Zeile
// ----------------------------------------------------------------------------

void lcd_clearline (unsigned char line)
{
	if (line>1) return;					// Zeilenwerte ueber 1 sind nicht darstellbar
	lcd_gotoxy(line,0);					// addressiere gewuenschte Zeile 
	lcd_putstr("                ");		// Leerzeichen ausgeben
}

// ----------------------------------------------------------------------------
// LCD_DISPLAYMESSAGE:	Anzeige einer Stringvariablen auf dem Display
// ----------------------------------------------------------------------------

void lcd_displayMessage(char* string, unsigned char line, unsigned char pos)
{
	lcd_gotoxy(line,pos);				// Cursor positionieren
	lcd_putstr(string  );				// String ausgeben
}
