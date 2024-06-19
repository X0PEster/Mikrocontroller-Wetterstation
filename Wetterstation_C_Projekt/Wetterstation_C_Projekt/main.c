/*=====================================================
 Gruppe 17:   Wetterstation
 ======================================================
 
 Dateiname:    Wetterstation_main.c
 
 Autoren:       Andrej Wolf
				Nico Baumann
 
 Version:        0.1 vom 08.04.24
 
 Hardware:      MEXLE2020 Ver. 1.0 oder höher
				AVR-USB-PROGI Ver. 2.0
 
 Software:      Entwicklungsumgebung: Microchip-Studio 7.0
				C-Compiler: AVR/GNU C Compiler 5.4.0
 
 Funktion:		Die Wetterstation ließt Werte aus 3 Sensoren aus und kann dann ihre Daten grafisch darstellen. 
 			Sie hat 4 Funktionsoberflächen, die mit den Tastern S1 bis S4 ausgewählt werden können:
				
				1:Temperatur
				2:Windeschwindigkeit
				3:Luftfeuchtigkeit
				4:Kalender(Histogramm)
 
 Displayanzeige: Start (fuer 2s):
				+----------------+
				|  -Willkommen-  |
				|                |
				+----------------+ 
        
				Betrieb:
				+----------------+    +----------------+  +----------------+    +----------------+
				|aktuelle     *()|    |aktuelle      ()|  |aktuelle	 ()|    |                |
				|Temp.:      22°C|    |Luftfeucht.: 68%|  |Windges.: 22km/h|    |                |
				+----------------+    +----------------+  +----------------+    +----------------+
				*In den Klammern wird Animation eingefügt
 
 
 Tastenfunktion: Im Hauptprogramm main.c rufen S1 .. S4 die 4 Unterprogramme auf.
		 Die Unterprogramme enthalten die einzelnen Funktionen und ihre Oberflächen (siehe dort)
 
 
 Fuses im uC:    
 
 Header-Files:  		lcd_lib_de.h    (Library zur Ansteuerung LCD-Display Ver. 1.3)
				avr/io.h		(Library zur I/O-Konfiguration)
				stdbool.h		(Library für Bit-Variablen)
				util.delay.h	(Library für Delays)
				avr/interrupt.h	(Library für Interrupts)	
 
 =============================================================================*/
   
   
// Deklarationen ==============================================================
   
// Festlegung der Quarzfrequenz
#ifndef F_CPU                  // optional definieren
#define F_CPU 18432000UL        // ATmega 88 mit 18,432 MHz Quarz
#endif                        
   
   
// Include von Header-Dateien
#include <avr/io.h>              // I/O-Konfiguration (intern weitere Dateien)
#include <stdbool.h>        // Bibliothek fuer Bit-Variable
#include <avr/interrupt.h>        // Definition von Interrupts
#include <util/delay.h>          // Definition von Delays (Wartezeiten)
#include "lcd_lib_de.h"      // Header-Datei fuer LCD-Anzeige
   
   
// Makros
#define SET_BIT(BYTE, BIT)  ((BYTE) |=  (1 << (BIT))) // Bit Zustand in Byte setzen
#define CLR_BIT(BYTE, BIT)  ((BYTE) &= ~(1 << (BIT))) // Bit Zustand in Byte loeschen
#define TGL_BIT(BYTE, BIT)  ((BYTE) ^=  (1 << (BIT))) // Bit Zustand in Byte wechseln (toggle)
#define GET_BIT(BYTE, BIT)  ((BYTE) &   (1 << (BIT))) // Bit Zustand in Byte einlesen
   
// Konstanten
#define VORTEILER_WERT      90  // Faktor Vorteiler = 60
#define HUNDERTSTEL_WERT    10  // Faktor Hundertstel = 10
#define ZEHNTEL_WERT        10  // Faktor Zehntel = 10
   
#define ON_TIME             100 // "Ein-Zeit" in Inkrementen zu 100 ms
#define OFF_TIME            100 // "Aus-Zeit" in Inkrementen zu 100 ms
   
#define MIN_PER             143 // minimale Periodendauer in "Timerticks"          
#define MAX_PER             239 // maximale Periodendauer in "Timerticks"          
#define WAIT_SND            2000// Wartezeit zwischen zum Tonwechsel in ms
#define WAIT_LED            1000// Wartezeit zwischen zum Blinkwechsel der LED in ms
  
#define ASC_ZERO            0x30// ASCII-Zeichen '0'
#define ASC_ONE             0x31// ASCII-Zeichen '1'
   
   
// Variable
unsigned char vorteiler   = VORTEILER_WERT;   // Zaehlvariable Vorteiler
unsigned char hundertstel = HUNDERTSTEL_WERT; // Zaehlvariable Hundertstel
unsigned char modus       = 0;              // Programmmodus
   
int counter = 0000;          // Variable fuer Zaehler
   
bool timertick;              // Bit-Botschaft alle 0,111ms (Timer-Interrupt)
bool takt10ms;                // Bit-Botschaft alle 10ms
bool takt100ms;              // Bit-Botschaft alle 100ms
   
bool sw1_neu = 1;              // Bitspeicher fuer Taste 1
bool sw2_neu = 1;              // Bitspeicher fuer Taste 2
bool sw3_neu = 1;              // Bitspeicher fuer Taste 3
bool sw4_neu = 1;              // Bitspeicher fuer Taste 4
   
bool sw1_alt = 1;              // alter Wert von Taste 1
bool sw2_alt = 1;              // alter Wert von Taste 2
bool sw3_alt = 1;              // alter Wert von Taste 3
bool sw4_alt = 1;              // alter Wert von Taste 4
   
bool sw1_slope = 0;          // Flankenspeicher fuer Taste 1
bool sw2_slope = 0;          // Flankenspeicher fuer Taste 2
bool sw3_slope = 0;          // Flankenspeicher fuer Taste 3
bool sw4_slope = 0;          // Flankenspeicher fuer Taste 4
   
   
//Funktionsprototypen
void initDisplay(void)			//Initialisierung des Displays
void StartupScreen(void);		//Startscreen
void setup(void);			//Setup Eingänge Sensoren

void getChoice(void);			//Modusanwahl

void doTemp(void);			// Teilprogramm 1: Temperaturmessung
void showTempDisplay(void);		// Anzeige mit Wert und Thermometer

void doWind(void);				// Teilprogramm 2: Windgeschwindigkeitsmessung
void showWindDisplay(void);     // Anzeige mit Wert und Animation

void doHumid(void);				// Teilprogramm 3: Luftfeuchtigkeitsmessung
void showHumidDisplay(void);    // Anzeige mit Prozentwert und Animation

void doCalendar(void);			// Teilprogramm 4: Zaehler
void showCalendarDisplay(void); // Anzeige des Kalenders

   
// Hauptprogramm ==============================================================
   
int main()
{                   
    initDisplay();                  // Initialisierung LCD-Anzeige
       
    TCCR2A = 0;                     // Timer 2 auf "Normal Mode": Basistakt
    TCCR2B |= (1<<CS01);          // mit Prescaler /8 betreiben
    TIMSK2 |= (1<<TOIE2);         // Overflow-Interrupt aktivieren
   
    sei();                          // generell Interrupts einschalten
   
    while(1)                        // unendliche Schleife
    {
        switch(modus)               // Programmverteiler: Variable "modus"
        {
        case 0:                     // Modus 0: Hauptmenu
            showMainDisplay();
            getChoice();
            break;
   
        case 1:                     // Modus 1: Blinkende LED
            doTemp();        // Programm laeuft bis zum Abbruch
            modus = 0;              // danach auf Hauptmenu zurueckschalten
            break;
   
        case 2:                     // Modus 2: Soundgenerierung
            doWind();              // Programm laeuft bis zum Abbruch
            modus = 0;              // danach auf Hauptmenu zurueckschalten
            break;
   
        case 3:                     // Modus 3: Logische Funktionen
            doHumid();     // Programm laeuft bis zum Abbruch
            modus = 0;              // danach auf Hauptmenu zurueckschalten
            break;
           
        case 4:                     // Modus 4: Up-Down-Counter
            doCalendar
		    ();        // Programm laeuft bis zum Abbruch
            modus = 0;              // danach auf Hauptmenu zurueckschalten
            break;
        }
    }
    return 0;
}
   
   
// Interrupt-Routine ==========================================================
ISR(TIMER2_OVF_vect)
   
// In der Interrupt-Routine sind die Softwareteiler realisiert, durch die Takt-
// botschaften (10ms, 100ms) erzeugt werden. Die Interrupts werden von Timer 2
// ausgeloest.
   
{
    timertick = 1;                  // Botschaft 0,166ms senden
    --vorteiler;                    // Vorteiler dekrementieren
    if (vorteiler==0)               // wenn 0 erreicht: 10ms abgelaufen
    {
        vorteiler = VORTEILER_WERT; //  Vorteiler auf Startwert
        takt10ms = 1;               //  Botschaft 10ms senden
        readButton();
        --hundertstel;              //  Hunderstelzaehler dekrementieren
           
    if (hundertstel==0)             // wenn 0 erreicht: 100ms abgelaufen
        {
            hundertstel = HUNDERTSTEL_WERT; // Teiler auf Startwert
            takt100ms = 1;                  //  Botschaft 100ms senden
           
        }   
    }
}
 
   
// Funktion Tasten einlesen ===================================================
void readButton(void)
{
    //   Bitposition im Register:
    //          __76543210
    DDRC = DDRC &   0b11110000;     // Zunaechst Port C auf Eingabe schalten
    PORTC =         0b00001111;     // Pullup-Rs eingeschaltet
    _delay_us(1);                   // Umschalten der Hardware-Signale abwarten
 
    // Einlesen der 4 Tastensignale
    sw1_neu = (PINC & (1 << PC0));
    sw2_neu = (PINC & (1 << PC1));
    sw3_neu = (PINC & (1 << PC2));
    sw4_neu = (PINC & (1 << PC3));
   
    DDRC = DDRC | 0b00001111;      // Am Ende Port C wieder auf Ausgabe schalten
 
    // Auswerten der Flanken beim Druecken
   
    if ((sw1_neu==0)&(sw1_alt==1))  // wenn Taste 1 soeben gedrueckt wurde: 
        sw1_slope = 1;              //  Flankenbit Taste 1 setzen
   
    if ((sw2_neu==0)&(sw2_alt==1))  // wenn Taste 2 eben gedrueckt wurde:
        sw2_slope = 1;              //  Flankenbit Taste 2 setzen
           
    if ((sw3_neu==0)&(sw3_alt==1))  // wenn Taste 3 eben gedrueckt wurde:
        sw3_slope = 1;              //  Flankenbit Taste 3 setzen
       
    if ((sw4_neu==0)&(sw4_alt==1))  // wenn Taste 4 eben gedrueckt wurde:
        sw4_slope = 1;              //  Flankenbit Taste 4 setzen
   
    // Zwischenspeichern aktuelle Tastenwerte
    sw1_alt = sw1_neu;              // aktuelle Tastenwerte speichern
    sw2_alt = sw2_neu;              //  in Variable fuer alte Werte
    sw3_alt = sw3_neu;
    sw4_alt = sw4_neu;  
}
 
   
// Initialisierung Display-Anzeige ============================================
void initDisplay()            // Start der Funktion
{
    lcd_init();              // Initialisierungsroutine aus der lcd_lib
                       
    lcd_gotoxy(0,0);                // Cursor auf 1. Zeile, 1. Zeichen
    lcd_putstr("---Willkommen---"); // Ausgabe Festtext: 16 Zeichen
   
    //lcd_gotoxy(1,0);                // Cursor auf 2. Zeile, 1. Zeichen
    //lcd_putstr(""); // Ausgabe Festtext: 16 Zeichen
   
    _delay_ms(2000);            // Wartezeit nach Initialisierung
   
}
   
   

  
   

// Auswahl im Hauptmenu ermitteln =======================================================
void getChoice()
{
    if (sw1_slope)            // Wenn Flanke auf Taste 1
    {
        sw1_slope=0;            //  Flankenbit loeschen
        modus=1;                //  neuer Modus 1
    }
                   
    if (sw2_slope)            // Wenn Flanke auf Taste 2
    {
        sw2_slope=0;            //  Flankenbit loeschen
        modus=2;                //  neuer Modus 2
    }
           
    if (sw3_slope)            // Wenn Flanke auf Taste 3
    {
        sw3_slope=0;            //  Flankenbit loeschen
        modus=3;                //  neuer Modus 3
    }
                       
    if (sw4_slope)            // Wenn Flanke auf Taste 4
    {
        sw4_slope=0;            //  Flankenbit loeschen
        modus=4;                //  neuer Modus 4
	}

//Temperaturfunktion===========================================================================================================
//Initialisierung der Animation		0		1		2		3		4		5		6		7		
			Custom_Animation[][]=	{0b00100,0b01010,0b01010,0b01010,0b01010,0b10001,0b10001,0b01110},	//Temperatur -10-0°C
						{0b00100,0b01010,0b01010,0b01010,0b01010,0b10001,0b11111,0b01110},	//Temperatur   0-10°C
						{0b00100,0b01010,0b01010,0b01010,0b01010,0b11111,0b11111,0b01110},	//Temperatur   10-20°C
						{0b00100,0b01010,0b01010,0b01010,0b01110,0b11111,0b11111,0b01110},	//Temperatur   20-30°C	
						{0b00100,0b01010,0b01010,0b01110,0b01110,0b11111,0b11111,0b01110},	//Temperatur   30-40°C
						{0b00100,0b01010,0b01110,0b01110,0b01110,0b11111,0b11111,0b01110};	//Temperatur   40-50°C
				
// Schreiben der Custom Chars ins CGRAM 
// Setze den CGRAM-Adresszeiger auf 0
	for (uint8_t i = 0; i < 8; i++) {
  	lcd_write(Custom_Animation[i]);
	}
}
