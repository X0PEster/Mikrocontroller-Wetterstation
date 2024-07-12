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
 
 Funktion:		Die Wetterstation ließt Werte aus 3 Sensoren aus und kann dann ihre Daten grafisch darstellen. Sie hat 4 Funktionsoberflächen,
				die mit den Tastern S1 bis S4 ausgewählt werden können:
				
				1:Temperatur
				2:Windeschwindigkeit
				3:Luftfeuchtigkeit
				4:Kalender(Histogramm)
 
 Displayanzeige: Start (fuer 2s):
				+----------------+
				| Wetterstation  |
				|  Baumann Wolf  |
				+----------------+ 
        
				Betrieb (Hauptebene):
				+----------------+    +----------------+  +----------------+    +----------------+
				|				 |    |                |  |	               |    |                |
				|                |    |                |  |	               |    |                |
				+----------------+    +----------------+  +----------------+    +----------------+
				
 
 
 Tastenfunktion: Im Hauptprogramm main.c rufen S1 .. S4 die 4 Unterprogramme auf.
				 Die Unterprogramme enthalten die einzelnen Funktionen und ihre Oberflächen (siehe dort)
 
 
 Fuses im uC:    
 
 Header-Files:  lcd_lib_de.h    (Library zur Ansteuerung LCD-Display Ver. 1.3)
				avr/io.h		(Library zur I/O-Konfiguration)
				stdbool.h		(Library für Bit-Variablen)
				util.delay.h	(Library für Delays)
				avr/interrupt.h	(Library für Interrupts)	
 
 =============================================================================*/

//Deklarationen

// Festlegung der Quarzfrequenz
#ifndef F_CPU                  // optional definieren
#define F_CPU 18432000UL		// MiniMEXLE mit 18.432 MHz Quarz, statt 12288000UL
#endif


// Header-Dateien
#include <avr/io.h>          // I/O-Konfiguration (intern weitere Dateien)
#include <stdbool.h>         // Bibliothek fuer Bit-Variable
#include <avr/interrupt.h>   // Definition von Interrupts
#include <util/delay.h>      // Definition von Delays (Wartezeiten)
#include <string.h>
#include <math.h>
#include "lcd_lib_de.h"      // Header-Datei fuer LCD-Anzeige

//Funktionsprototypen
void initDisplay(void);		 //Initialisierung des Displays
void initTaster(void);		 //Initialisierung der Taster
void ReadTaster(void);		 //Auslesen der Taster
void MenuWahl(void);		 //Menu Aubuttonahl
void refreshTime(void);		 //Uhrfunktion
void showTime(void);		 //Uhrzeit-Anzeige-Funktion
void setTime(void);			 //Uhrzeit/Datum bearbeiten
void setTimeScreen(void);	 //Anzeige fuer Zeiteinstellung
void menuScreen(void);		 // Anzeige im Standardmenu
void temperatureMenu(void);	 //Menu fur Temperatur
void humidityMenu(void);	 //Menu fur Luftfeuchte
void temperatureScreen(void);	 //Anzeigefunktion fur das Temperatur-Menu
void humidityScreen(void);		 //Anzeigefunktion fur das Luftfeuchte-Menu
void windMenu(void);			 //Menu fur Windgeschwindigkeit
void windScreen(void);			 //Anzeigefunktion fur Wind-V-Menu



void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_write_custom_char(uint8_t loc, uint8_t* charmap);

int8_t readDHT(uint8_t*, uint8_t*);
int8_t getData(uint8_t*, uint8_t*);

//Konstanten
#define PRESCALER_VAL	90	//Faktor Vorteiler
#define CYCLE10MS_MAX	10	//Faktor Hundertstel
#define CYCLE100MS_MAX	10	//Faktor Zehntel

#define ASC_NULL            0x30        // Das Zeichen '0' in ASCII
#define ASC_COLON           0x3A        // Das Zeichen ':' in ASCII
#define ASC_DOT				0x2E		//' . '
#define ASC_LINE			0x7C		//' | '

#define INPUT_PIN_MASK      0b00001111

#define DHT_DDR DDRB
#define DHT_PORT PORTB
#define DHT_PIN PINB
#define DHT_INPUTPIN 1
#define DHT_TIMEOUT 200

// Define the LCD control pins
#define RS PD4
#define RW PD1
#define EN PD7

// Define the LCD port
#define LCD_PORT PORTC
#define LCD_DDR  DDRC

//Variablen
unsigned char softwarePrescaler = PRESCALER_VAL;    // Zaehlvariable Vorteiler
unsigned char cycle10msCount    = CYCLE10MS_MAX;    // Zaehlvariable Hundertstel
unsigned char cycle100msCount   = CYCLE100MS_MAX;   // Zaehlvariable Zehntel
unsigned char seconds           = 00;               // Variable Sekunden
unsigned char minutes           = 00;               // Variable Minuten
unsigned char hours             = 00;               // Variable Stunden
unsigned char day				= 01;				// Var. Tag
unsigned char month				= 01;				// Var. Monat
unsigned char year				= 24;				// Var. Jahr
unsigned char dayCount[]		={32,29,32,31,32,31,32,32,31,32,31,32};		// Anzahl Tage im Monat
unsigned char menu				= 0;				// Variable für Menu-Auswahl
bool timeMenu					= 0;				//menu in Zeitbearbeitung (hh:mm / dd.mm)

unsigned char windV				= 0;				//Variable Windgeschwindigkeit

uint8_t *temperature;
uint8_t *humidity;
	
bool timertick;                     // Bit-Botschaft alle 0,166ms (bei Timer-Interrupt)
bool cycle10msActive;               // Bit-Botschaft alle 10ms
bool cycle100msActive;              // Bit-Botschaft alle 100ms
bool cycle1sActive;                 // Bit-Botschaft alle 1s

bool button1_new = 1;				// Bitspeicher fuer Taste 1
bool button1_old = 1;				// alter Wert von Taste 1
bool button2_new = 1;               // Bitspeicher fuer Taste 2
bool button3_new = 1;               // Bitspeicher fuer Taste 3
bool button2_old = 1;               // alter Wert von Taste 2
bool button3_old = 1;               // alter Wert von Taste 3
bool button4_new = 1;				// Bitspeicher fuer Taste 4
bool button4_old = 1;				// alter Wert von Taste 4
bool button1_slope=0;				// Flankenwert fuer Taste 1
bool button2_slope=0;				// Flankenwert fuer Taste 2
bool button3_slope=0;				// Flankenwert fuer Taste 3
bool button4_slope=0;				// Flankenwert fuer Taste 4

uint8_t buttonState    = 0b00001111;        // Bitspeicher fuer Tasten

// Makros
#define SET_BIT(BYTE, BIT)  ((BYTE) |=  (1 << (BIT))) // Bit Zustand in Byte setzen
#define CLR_BIT(BYTE, BIT)  ((BYTE) &= ~(1 << (BIT))) // Bit Zustand in Byte loeschen
#define TGL_BIT(BYTE, BIT)  ((BYTE) ^=  (1 << (BIT))) // Bit Zustand in Byte wechseln (toggle)

int main(void)
{
	// Initialisierung
	initDisplay();              // Initialisierung LCD-Anzeige
	    
	TCCR0A = 0;                 // Timer 0 auf "Normal menu" schalten
	SET_BIT(TCCR0B, CS01);      // mit Prescaler /8 betreiben
	SET_BIT(TIMSK0, TOIE0);     // Overflow-Interrupt aktivieren*/
	    
	sei();                      // generell Interrupts einschalten
		
    while(1)
	{
	   if (cycle1sActive)             // alle Sekunden:
	   {
		   cycle1sActive = 0;         //      Botschaft "1s" loeschen
		   refreshTime();			  //      Uhr weiterzaehlen
	   }		
	  if (cycle10msActive)           // alle 100ms:
		{
			cycle10msActive = 0;     //      Botschaft "100ms" loeschen
			MenuWahl();
		}
		switch (menu)
			{
				case 0: menuScreen();break;
				
				case 1: setTime();menu=0;break;						 //Zeit bearbeiten + Zur Menu Auswahl
				
				case 2: temperatureMenu();menu=0;break;
				
				case 3: humidityMenu();menu=0;break;
				
				case 4: windMenu();menu=0;break;
			}
		}
return 0;
}




ISR (TIMER0_OVF_vect)
/*  In der Interrupt-Routine sind die Softwareteiler realisiert, die die Takt-
    botschaften (10ms, 100ms, 1s) fuer die gesamte Uhr erzeugen. Die Interrupts
    werden von Timer 0 ausgeloest (Interrupt Nr. 1)
  
*/
{
    timertick = 1;                  // Botschaft 0,166ms senden
    --softwarePrescaler;                    // Vorteiler dekrementieren
    if (softwarePrescaler==0)               // wenn 0 erreicht: 10ms abgelaufen
    {
        softwarePrescaler = PRESCALER_VAL; //    Vorteiler auf Startwert
        cycle10msActive = 1;               //    Botschaft 10ms senden
        --cycle10msCount;              //    Hunderstelzaehler dekrementieren
  
        if (cycle10msCount==0)         // wenn 0 erreicht: 100ms abgelaufen
        {
            cycle10msCount = CYCLE10MS_MAX; // Teiler auf Startwert
            cycle100msActive = 1;          //    Botschaft 100ms senden
            --cycle100msCount;              //    Zehntelzaehler dekrementieren
  
            if (cycle100msCount==0)         // wenn 0 erreicht: 1s abgelaufen
            {
                cycle100msCount = CYCLE100MS_MAX; //    Teiler auf Startwert
                cycle1sActive = 1;             //    Botschaft 1s senden
            }
        }
    }
}

void MenuWahl(void)
{
	ReadTaster();
	    if (button1_slope)            // Wenn Flanke auf Taste 1
	    {
		    button1_slope=0;            //  Flankenbit loeschen
		    menu=1;                //  neuer menu 1
	    }
	    
	    if (button2_slope)            // Wenn Flanke auf Taste 2
	    {
		    button2_slope=0;            //  Flankenbit loeschen
		    menu=2;                //  neuer menu 2
	    }
	    
	    if (button3_slope)            // Wenn Flanke auf Taste 3
	    {
		    button3_slope=0;            //  Flankenbit loeschen
		    menu=3;                //  neuer menu 3
	    }
	    
	    if (button4_slope)            // Wenn Flanke auf Taste 4
	    {
		    button4_slope=0;            //  Flankenbit loeschen
		    menu=4;                //  neuer menu 4
	    }
			button1_old = button1_new;
			button2_old = button2_new;              // aktuelle Tastenwerte speichern
			button3_old = button3_new;              //    in Variable fuer alte Werte
			button4_old = button4_new;
}


//Initialisierung - Display
void initDisplay(void)	//Start der Funktion
{
	lcd_init();			//Display-Initialisierung
	
	lcd_gotoxy(0,0);				//Cursor auf erste Stelle, erste Zeile bringen (1,1)
	lcd_putstr(" Wetterstation  ");	//Ausgabe Festtext (16 Zeichen)
	
	lcd_gotoxy(1,0);				//Cursor auf erste Stelle, zweite Zeile bringen (2,1)
	lcd_putstr("  Baumann Wolf  ");	//Ausgabe Festtext (16 Zeichen)
	
	_delay_ms(2000);				//Wartezeit nach Initialisierung
	
	lcd_gotoxy(0,0);                // Cursor auf 1. Zeile, 1. Zeichen
	lcd_putstr("00:00 | 00.00.00"); // Ausgabe Festtext: 16 Zeichen
	 
	lcd_gotoxy(1,0);                // Cursor auf 2. Zeile, 1. Zeichen
	lcd_putstr("                "); // Ausgabe Festtext: 16 Zeichen	
}

//Tastereinlesefunktion =======================================================
void ReadTaster(void)
{
    DDRC = DDRC &~INPUT_PIN_MASK;   // Port C auf Eingabe schalten
    PORTC |=      INPUT_PIN_MASK;   // Pullup-Rs eingeschaltet
    _delay_us(1);                   // Wartezeit Umstellung Hardware-Signal
    buttonState    = (PINC & INPUT_PIN_MASK) ;          // Hole den Schalterstatus von C1..C4, 0c1 ist hier offener SChalter
    DDRC |= INPUT_PIN_MASK;         // Port C auf Ausgabe schalten
    
    // Einlesen der Tastensignale
    button1_new	= (buttonState & (1 << PC0));	
    button2_new = (buttonState & (1 << PC1));
    button3_new = (buttonState & (1 << PC2));
    button4_new = (buttonState & (1 << PC3));
	
	 // Aubuttonerten der Flanken beim Druecken
	 
	 if ((button1_new==0)&(button1_old==1))  // wenn Taste 1 soeben gedrueckt wurde:
	 button1_slope = 1;              //  Flankenbit Taste 1 setzen
	 
	 if ((button2_new==0)&(button2_old==1))  // wenn Taste 2 eben gedrueckt wurde:
	 button2_slope = 1;              //  Flankenbit Taste 2 setzen
	 
	 if ((button3_new==0)&(button3_old==1))  // wenn Taste 3 eben gedrueckt wurde:
	 button3_slope = 1;              //  Flankenbit Taste 3 setzen
	 
	 if ((button4_new==0)&(button4_old==1))  // wenn Taste 4 eben gedrueckt wurde:
	 button4_slope = 1;              //  Flankenbit Taste 4 setzen
}


// Anzeigefunktion Uhr ========================================================
void showTime(void)
/*  Die Umrechnung der binaeren Zaehlwerte auf BCD ist folgendermaßen geloest: 
    Zehner: einfache Integer-Teilung (/10)
    Einer:  Modulo-Ermittlung (%10), d.h. Rest bei der Teilung durch 10
*/
{
    lcd_gotoxy(0,0);                    // Cursor auf Start der Zeitausgabe setzen
      
    lcd_putc(ASC_NULL + hours/10);    // Stunden Zehner als ASCII ausgeben
    lcd_putc(ASC_NULL + hours%10);    // Stunden Einer als ASCII ausgeben
    lcd_putc(ASC_COLON);                // Doppelpunkt ausgeben
      
    lcd_putc(ASC_NULL + minutes/10);    // Minuten als ASCII ausgeben
    lcd_putc(ASC_NULL + minutes%10);    //

	lcd_gotoxy(0,6);
	
	lcd_putc(ASC_LINE);
	
	lcd_gotoxy(0,8);
	
	lcd_putc(ASC_NULL + day/10);		//Tag als ASCII ausgeben
	lcd_putc(ASC_NULL + day%10);
	lcd_putc(ASC_DOT);

	lcd_putc(ASC_NULL + month/10);		//Monat als ASCII ausgeben
	lcd_putc(ASC_NULL + month%10);
	lcd_putc(ASC_DOT);

	lcd_putc(ASC_NULL + year/10);		//Jahr als ASCII asugeben
	lcd_putc(ASC_NULL + year%10);
}

// Anzeigenfunktion im Standardmenu ===========================================

void menuScreen(void)
{	
	showTime();							//Uhrzeit-Datum Anzeige
		
	lcd_gotoxy(1,5);
	lcd_putstr("     ");
	
	lcd_gotoxy(1,0);					//setTime-Menu Zeichen
	lcd_putc(0xC0);
	
	lcd_gotoxy(1,4);					//Temperatur-Menu Zeichen
	lcd_putc(0xD0);
	
	lcd_gotoxy(1,10);					//Luftfeuchte-Menu Zeichen
	lcd_putc(0XB1);
	
	lcd_gotoxy(1,15);					//WindV-Zeichen
	lcd_putc(0xC1);
}

// Stellfunktion ==============================================================
void setTime(void)
/*  Die Stellfunktion der Uhr wird alle 10ms aufgerufen. Dadurch wir eine
    Entprellung der Tastensignale realisiert. Das Stellen wir bei einer 
    fallenden Flanke des jeweiligen Tastensignals durchgefuehrt. Darum 
    muss fuer einen weiteren Stellschritt die Taste erneut betaetigt werden.
  
    Eine Flanke wird durch (alter Wert == 1) UND (aktueller Wert == 0) erkannt.
  
    Mit der Taste S2 werden die Stunden/Tage aufwaerts gestellt.
    Mit der Taste S3 werden die Minuten/Monate aufwaerts gestellt (kein Uebertrag)
  
    Veraenderte Variable: stunden		tag
                          minuten		monat
  
    Speicher fuer Bits:   button2Alt
                          button3Alt
*/
{
	button1_slope = 0;
	button1_old=button1_new;
	
	lcd_clearDisplay();					//Anzeige leeren
	
	while(!button1_slope)
	{
		if (cycle1sActive)             // alle Sekunden:
		{
			cycle1sActive = 0;         //      Botschaft "1s" loeschen
			refreshTime();			   //      Uhr weiterzaehlen
		}
		if(cycle100msActive)
		{
		setTimeScreen();				//Uhrzeit/Datum anzeigen
		}
		button1_slope = 0;
		button1_old=button1_new;
		
		ReadTaster();
		if((button4_new==0)&(button4_old==1))		//Wenn Taster 4 gedruckt Modus wechseln: Stunde:Minute/Tag.Monat
		{
			TGL_BIT(timeMenu,0);
		}
		switch(timeMenu){
					case 0:
						if ((button2_new==0)&(button2_old==1))  // wenn Taste 2 eben gedrueckt wurde:
						{
							hours++;                  //    Stunden hochzaehlen, Ueberlauf bei 23
							if (hours==24)
							hours = 00;
						}
						if ((button3_new==0)&(button3_old==1))  // wenn Taste 3 eben gedrueckt wurde:
						{
							minutes++;                  //    Minuten hochzaehlen, Ueberlauf bei 59
							if (minutes==60)
							minutes = 00;
						}break;
					case 1:
						  if ((button3_new==0)&(button3_old==1))  // wenn Taste 3 eben gedrueckt wurde:
						  {	month++;                  //    Monate hochzaehlen, Ueberlauf bei 12
							  if (month==13)			//Uberlauf bei 12 => ein Jahr dazu
							  {month = 1;
							  year++;}
						  }
						  if ((button2_new==0)&(button2_old==1))  // wenn Taste 2 eben gedrueckt wurde:
						  {
							  day++;                  //    Tage hochzaehlen, Ueberlauf bei dayCount
							  if((month==2)&&(year%4==0)){dayCount[1]=30;}else{dayCount[1]=29;}	//bei Tag-Uberlauf, kein Monat hochzahlen
							  if (day==dayCount[month-1])
							  day = 1;
						  }break;
					}
			button2_slope = 0;
			button3_slope = 0;
			button4_slope = 0;
			
			button2_old = button2_new;              // aktuelle Tastenwerte speichern
			button3_old = button3_new;              //    in Variable fuer alte Werte
			button4_old=button4_new;						
	}
button1_slope = 0;
button1_old=button1_new;
}


// Anzeigefunktion im Zeitbearbeitungsmodus ===================================
void setTimeScreen(void)
{
	showTime();						//Uhrzeit/Datum anzeige
	
	lcd_gotoxy(1,0);
	lcd_putc(0x7F);					// ':'
	
	lcd_gotoxy(1,5);				// '|'
	if(timeMenu==0)
	{lcd_putstr("h+  m+");}else
	{lcd_putstr("d+  m+");}
		
	lcd_gotoxy(1,15);
	lcd_putc(0x7E);					//Pfeil nach links "zurück"
}

// Zaehlfunktion Uhr ==========================================================
void refreshTime (void)              // wird jede Sekunde gestartet
/*  Die Uhr wird im Sekundentakt gezaehlt. Bei jedem Aufruf wird auch ein 
    "Tick" auf dem Lautsprecher ausgegeben. Ueberlaeufe der Sekunden zaehlen
    die Minuten, die Ueberlaeufe der Minuten die Stunden hoch.
  
    Veraenderte Variable:   sekunden		tag
                            minuten			monat			
                            stunden			jahr
*/
{
    seconds++;                     // Sekunden hochzaehlen
    if (seconds==60)               // bei Überlauf:
    {
        seconds = 0;               //  Sekunden auf 00 setzen
        minutes++;                  //  Minuten hochzaehlen
        if (minutes==60)            //  bei Ueberlauf:
        {
            minutes = 0;            //  Minuten auf 00 setzen
            hours++;              //  Stunden hochzaehlen
            if (hours==24)        //  bei Ueberlauf:
            {   
				hours = 0;        //  Stunden auf 00 setzen
				day++;
				if((month==2)&&(year%4==0)){dayCount[1]=30;}else{dayCount[1]=29;}	//nach Schaltjahr überprüfen
					
		  		if(day==dayCount[month-1])		//Monat bei Uberlauf weiterzahlen
				{
					day=1;	
					month++;
					if(month==13)				//Jahr  bei Uberlauf weiterzahlen
					{
						month=1;
						year++;
					}
				 }
			 }
		}
	}
}	

void temperatureMenu()
{	button1_slope = 0;
	button1_old=button1_new;
	
	lcd_gotoxy(0,0);
	
	lcd_clearDisplay();
	
	temperatureScreen();			//Temperatur-Anzeige
	
	while(!button1_slope)
	{	
		if (cycle1sActive)             // alle Sekunde:
		{
			cycle1sActive = 0;         //      Botschaft "1s" loeschen
			refreshTime();			   //      Uhr weiterzaehlen
			temperatureScreen();	   //	   Temperatur-Anzeige jede Sekunde aktualisieren
		}

		ReadTaster();

		getData(temperature,humidity);		//Temp. Daten aus Funktion bekommen
		
		ReadTaster();	
		
		button2_slope = 0;
		button3_slope = 0;
		button4_slope = 0;
		
		button2_old = button2_new;              // aktuelle Tastenwerte speichern
		button3_old = button3_new;              //    in Variable fuer alte Werte
		button4_old=button4_new;			
	}
	button1_slope = 0;
	button1_old=button1_new;
	
	lcd_clearDisplay();
}

void humidityMenu()
{	button1_slope = 0;
	button1_old=button1_new;
	
	lcd_gotoxy(0,0);
	
	lcd_clearDisplay();
	
	humidityScreen();		//Luftfeuchte-Anzeige
	
	while(!button1_slope)
	{
		if (cycle1sActive)             // alle Sekunde:
		{
			cycle1sActive = 0;         //      Botschaft "1s" loeschen
			refreshTime();			   //      Uhr weiterzaehlen
			humidityScreen();			//Luftfeuchte-Anzeige jede Sekunde aktualisieren
		}
		button1_slope = 0;
		button1_old=button1_new;
		
		ReadTaster();

		getData(temperature,humidity);		//Luftfeuchte-Daten von Funktion bekommen
		
		ReadTaster();
		
		button2_slope = 0;
		button3_slope = 0;
		button4_slope = 0;
		
		button2_old = button2_new;              // aktuelle Tastenwerte speichern
		button3_old = button3_new;              //    in Variable fuer alte Werte
		button4_old=button4_new;	
	}
	button1_slope = 0;
	button1_old=button1_new;
	lcd_clearDisplay();
}

int8_t readDHT(uint8_t *temperature, uint8_t *humidity)		//Sensordaten-Auslese-Funktion
{
	uint8_t i,j=0;
	int8_t bits[5]={};
	
	memset(bits, 0, sizeof(bits));		//Speicherplatz für Bits reservieren

	//MC Ports/Pins vorbereiten
	DHT_DDR |= (1<<DHT_INPUTPIN);	//output
	DHT_PORT |= (1<<DHT_INPUTPIN);	//high
	_delay_ms(100);
		
	//MC kontakt mit DHT
	DHT_PORT &= ~(1<<DHT_INPUTPIN);	//low
	_delay_ms(18);
	
	DHT_PORT |= (1<<DHT_INPUTPIN);	//high
	DHT_DDR &= ~(1<<DHT_INPUTPIN);	//input
	_delay_us(40);
	
	//Kontrolle: erstes DHT Antwort_Signal		-		wenn high-Signal vom Sensoer = Fehler
	if((DHT_PIN & (1<<DHT_INPUTPIN)))
	{return -1;}
	
	_delay_us(80);
	
	//Kontrolle: zweites DHT Antwort_Signal		-		wenn low-Signal vom Sensor = Fehler
	if(!(DHT_PIN & (1<<DHT_INPUTPIN)))
	{return -1;}	
	_delay_us(80);
	
	//Daten/Bits einlesen
	
	uint16_t timeoutcounter = 0;
	
	for(j=0;j<5;j++)	//Schleife um 5 einzelne Bytes aufzunehmen
	{
		uint8_t result = 0;
		for(i=0;i<8;i++)	//Schleife für einzelne Bits pro Byte
		{
			timeoutcounter=0;
			while(!(DHT_PIN & (1<<DHT_INPUTPIN)))	//auf high_input vom Sensor warten
			{
				timeoutcounter++;
				if(timeoutcounter>DHT_TIMEOUT){return -1;}		//wenn es zu lange dauert = Fehler
			}
		_delay_us(30);						//Wegen Zeitintervallen des Sensors, nach 30µs: wenn es noch high ist
		if(DHT_PIN & (1<<DHT_INPUTPIN))		// gibt der Sensor eine "1" aus, wenn low dann "0"	
		{
			result |= (1<<(7-i));
		}
		timeoutcounter=0;

		while(DHT_PIN & (1<<DHT_INPUTPIN))	//den high-Output vom Sensor auslaufen lassen, danach Übertragungsstart fürs nächste Byte
		{
			timeoutcounter++;
			if(timeoutcounter > DHT_TIMEOUT) {return -1;}
			}
		}
		bits[j]=result;
	}
	//Port zurücksetzen setzen
	DHT_DDR |= (1<<DHT_INPUTPIN);	//output
	DHT_PORT |= (1<<DHT_INPUTPIN);	//low
	_delay_ms(100);
	//cheksum
	if ((bits[0] + bits[1] + bits[2] + bits[3]) == bits[4]) {
		//return temperature and humidity
		*temperature=bits[2];
		*humidity = bits[0];
		return 0;		
	}
	return -1;
}

int8_t getData(uint8_t *temperature, uint8_t *humidity)
{
	return readDHT(temperature,humidity);	//Daten des Sensors übermitteln
}


void temperatureScreen()
{
	lcd_gotoxy(0,0);
	lcd_putstr("Temperatur:");

	lcd_gotoxy(1,0);
	lcd_putc(0x7F);
		
	lcd_gotoxy(1,4);
	
	lcd_putc(*temperature);			//Temperatur-Char ausgeben
}

void humidityScreen()
{
	lcd_gotoxy(0,0);
	lcd_putstr("Luftfeuchte:");

	lcd_gotoxy(1,0);
	lcd_putc(0x7F);
	
	lcd_gotoxy(1,4);
	
	lcd_putc(*humidity);		//Luftfeuchte-Char ausgeben
}

void windMenu()
{	button1_slope = 0;
	button1_old=button1_new;
	
	bool isActive=0;
	
	lcd_gotoxy(0,0);
	
	lcd_clearDisplay();
	
	windScreen();		//Anzeige für die WindV
	
	while(!button1_slope)
	{
		if (cycle1sActive)             // alle Sekunde:
		{
			cycle1sActive = 0;         //      Botschaft "1s" loeschen
			refreshTime();			   //      Uhr weiterzaehlen
			windScreen();				//WindV-Anzeige aktualisieren, jede Sekunde
			windV=0;
		}
		button1_slope = 0;
		button1_old=button1_new;
		
		DDRB &= ~(1<<6);			//DDR B auf input setzen
		
		if(PINB & (1<<6))			//Wenn auf PINB6 was ankommt,
		{
			if(isActive==0)			//WindV hochzählen
			{
				windV++;
			}
			isActive=1;				//und Status setzen, damit "Dauer-Hochzählen" zu vermeiden
		}else if(!(PINB & (1<<6)))
		{
			isActive=0;
		}
				
		ReadTaster();
		
		button2_slope = 0;
		button3_slope = 0;
		button4_slope = 0;
		
		button2_old = button2_new;              // aktuelle Tastenwerte speichern
		button3_old = button3_new;              //    in Variable fuer alte Werte
		button4_old=button4_new;
	}
	DDRB |= (1<<6);								//PortB auf output setzen
	
	button1_slope = 0;
	button1_old=button1_new;
	lcd_clearDisplay();
}

    uint8_t smiley[8] = {
	    0b00000,
	    0b01010,
	    0b01010,
	    0b00000,
	    0b10001,
	    0b01110,
	    0b00000,
	    0b00000
    };
	

void windScreen()
{
	lcd_gotoxy(0,0);
	lcd_putstr("Wind-V:");
		
	lcd_write_custom_char(0,smiley);
	lcd_gotoxy(0,10);
	lcd_data(0);

	lcd_gotoxy(1,0);
	lcd_putc(0x7F);
	
	lcd_gotoxy(1,4);
	
	lcd_putc(ASC_NULL+windV/10);		//WindV ausgeben
	lcd_putc(ASC_NULL+windV%10);
	
	lcd_putstr(" in Hz");
}


void lcd_init_zwei() {
	LCD_DDR = 0xFF; // Set LCD port as output
	_delay_ms(20);  // Wait for LCD to power up

	// Initialize LCD in 4-bit mode
	lcd_command(0x02);
	lcd_command(0x28);
	lcd_command(0x0C);
	lcd_command(0x06);
	lcd_command(0x01);
	_delay_ms(2);
}


void lcd_command(uint8_t cmd) {
	LCD_PORT = (cmd & 0xF0); // Send higher nibble
	LCD_PORT &= ~(1 << RS);  // RS = 0 for command
	LCD_PORT &= ~(1 << RW);  // RW = 0 for write
	LCD_PORT |= (1 << EN);   // EN = 1 for high pulse
	_delay_us(1);
	LCD_PORT &= ~(1 << EN);  // EN = 0 for low pulse
	_delay_us(200);

	LCD_PORT = (cmd << 4);   // Send lower nibble
	LCD_PORT &= ~(1 << RS);  // RS = 0 for command
	LCD_PORT &= ~(1 << RW);  // RW = 0 for write
	LCD_PORT |= (1 << EN);   // EN = 1 for high pulse
	_delay_us(1);
	LCD_PORT &= ~(1 << EN);  // EN = 0 for low pulse
	_delay_ms(2);
}

void lcd_data(uint8_t data) {
	LCD_PORT = (data & 0xF0); // Send higher nibble
	LCD_PORT |= (1 << RS);    // RS = 1 for data
	LCD_PORT &= ~(1 << RW);   // RW = 0 for write
	LCD_PORT |= (1 << EN);    // EN = 1 for high pulse
	_delay_us(1);
	LCD_PORT &= ~(1 << EN);   // EN = 0 for low pulse
	_delay_us(200);

	LCD_PORT = (data << 4);   // Send lower nibble
	LCD_PORT |= (1 << RS);    // RS = 1 for data
	LCD_PORT &= ~(1 << RW);   // RW = 0 for write
	LCD_PORT |= (1 << EN);    // EN = 1 for high pulse
	_delay_us(1);
	LCD_PORT &= ~(1 << EN);   // EN = 0 for low pulse
	_delay_ms(2);
}

void lcd_write_custom_char(uint8_t loc, uint8_t* charmap) {
	loc &= 0x07; // We only have 8 locations 0-7 for custom characters
	lcd_command(0x40 + (loc * 8)); // Set CGRAM address
	for (int i = 0; i < 8; i++) {
		lcd_data(charmap[i]);
	}
}
