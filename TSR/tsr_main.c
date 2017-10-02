/*!***********************************************************************
 * @file	tsr_main.c
 * @brief	Firmware für den TSR-Controller 
 *
 * @author	René Schönrock
 *
 *
 * @date	20.05.2010
 * @version	1.0
 *
 * @bug		encoder.
 * @warning none.
 *
 * @see		Datenblatt des ATiny2313 von Atmel
 * @see		./doc/2D1C9161d01.pdf
 * @see		http://de.wikipedia.org/wiki/Transformatorschaltrelais
 *
 * @note	TSR-Verfahren ist patentrechtlich geschützt
 * @see		www.emeko.de
 *
 * @todo
 *	
 * Details.
 *
 *
 *	Halbwellenausfall-Erkennung:
 *	Netzspannungsdeformationen, z.B. Halbwellenausfälle, können
 *	zu großen Sättigungsströmen im Transformator führen, die wesentlich größer als der Einschaltstrom 
 *	sein können. Das TSR reagiert auf die Halbwelleneinbrüche, indem es sofort ausschaltet, bevor die 
 *	Sättigungsströme entstehen und anschließend wieder mit dem Sanft-Einschalt-Verfahren einschaltet.
 *	Auf diese Weise wird das Auslösen der Sicherung vermieden.
 * 
 *	Andimmen:
 *	Das TSR kann auch zum sanften Einschalten von Kondensatorsiebglie-
 *	dern dienen wie sie z.B. bei Frequenzumrichtern im Netzeingangskreis vorhanden sind. Auch große 
 *	Siebkondensatoren nach einem Transformator werden damit sanft eingeschaltet. Dabei werden die 
 *	Spannungsimpulse kontinuierlich bis zu dem am Potentiometer eingestellten Wert verbreitert und dann 
 *	voll eingeschaltet.
 */

 #include <stdio.h>
 #include <avr/io.h>
 #include <string.h>
 #include <avr/eeprom.h>
 #include <avr/interrupt.h>
 #include <avr/pgmspace.h>
 #include <util/delay.h>
 #include <avr/wdt.h>

 #include "config.h"

 #include "types.h"

 //#include "usart.h"
 //#include "sin.h"

 #include "adc.h"

 #include "tsr_main.h"



 BYTE	encoder_val;

 BYTE	pulse_counter;

 BYTE	pulse_counter_start_val;

 WORD	pulse_width;


/**
 * @var		tsr_ee_reg
 * @brief	Registersatz im EEProm
 * @see		TSR_REG
 * @see		reg
 *	
 */
 /*
 TSR_REG tsr_ee_reg EEMEM =
 {
	//ab rel. 0x10 steht die Gerätesignatur und die Softwareversion
	__DEV_SIGNATURE__,
	__SW_RELEASE__,
	__SW_RELEASE_DATE__,
 };
*/


/*!***********************************************************************
 * @fn		ISR(TIMER1_COMPA_vect)
 * @brief	ISR für 
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(TIMER1_COMPA_vect)
{
	if (THYRISTOR_IN){
		THYRISTOR_OFF;

	}else {
		THYRISTOR_ON;

		if (pulse_counter)
			OCR1A += THYRISTOR_IGN_PULSE_TIME;
		else
			OCR1A += RELAY_BOUNCE_TIME;
	}

	if (RED_LED_IN) {
		RED_LED_OFF;
		GREEN_LED_OFF;
	}
	else {
		RED_LED_ON;
		GREEN_LED_ON;
	}
}


/*!***********************************************************************
 * @fn		ISR(TIMER1_COMPB_vect)
 * @brief	ISR für 
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(TIMER1_COMPB_vect)
{
	RELAIS_ON;
}


/*!***********************************************************************
 * @fn		ISR(SIG_OVERFLOW1)
 * @brief	ISR für 
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(TIMER1_OVF_vect)
{

	if (pulse_counter) {

		//TIMER1_STOP;

		if (GREEN_LED_IN) {
			GREEN_LED_OFF;
			RED_LED_OFF;
		}
		else {
			GREEN_LED_ON;
			RED_LED_ON;
		}
	}else {
		RED_LED_OFF;
		GREEN_LED_ON;
		TIMER1_STOP;
	}

}


/*!***********************************************************************
 * @fn		ISR(INT0_vect)
 * @brief	ISR für die Zero-Crossing-Detection
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(INT0_vect)
{

	if (pulse_counter) {

		if (!(pulse_counter & 0x01))
			TCNT1 = 0;

		if ((encoder_val&0x0C) == 0x0C) {	//variable Pulsbreite (andimmen)
			//OCR1A = TCNT1 + (PERIOD_TIME-MAX_PULSE_TIME);
			OCR1A = TCNT1 + (PERIOD_TIME-((MAX_PULSE_TIME/pulse_counter_start_val)*((pulse_counter_start_val-pulse_counter)+1)));
		}else
			OCR1A = TCNT1 + (PERIOD_TIME-pulse_width);

		if (--pulse_counter==1) {
			//letzter Puls (Relais wird aktiviert)
			TIMSK1	|= _BV(OCIE1B);
			if ((encoder_val&0x0C) == 0x0C) {	//variable Pulsbreite (andimmen)
				OCR1B = ((TCNT1 + 3*PERIOD_TIME - MAX_PULSE_TIME) - RELAY_TURN_ON_DELAY_TIME);
			}else
				OCR1B = ((OCR1A + 2*PERIOD_TIME) - RELAY_TURN_ON_DELAY_TIME);
				//OCR1B = ((0xFFFF - PERIOD_TIME) + 5000);
		}
	}
}


/*!***********************************************************************
 * @fn		ISR(INT1_vect)
 * @brief	ISR für den Encoder
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(INT1_vect)
{
	encoder_val = ~(((PIND & ENCODER_MASK)>>PD3)|0xF0);
}


/*!***********************************************************************
 * @fn		void init_Sys(void)
 * @brief	Systeminitialisierung
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
void tsr_start(void)
{
	PORTB &= ~(RELAIS_OUT | THYRISTOR_OUT);

	_delay_ms (20);

	pulse_counter_start_val = (0x08<<(encoder_val&0x03));

	pulse_counter = pulse_counter_start_val;

	pulse_width = (((encoder_val&0x0C)>>2)+8)*180;
	
	TIMSK1	= (_BV(TOIE1) | _BV(OCIE1A));

	TCNT1 = 0;

	TIMER1_START;	
}


/*!***********************************************************************
 * @fn		void init_Sys(void)
 * @brief	Systeminitialisierung
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
void init_Sys(void)
{
	DDRD &= ~(ZERO_CROSSING_INT | ENCODER_MASK);//ZCSignal Encoder-Bit0,1,2,3 werden Eingang
	EICRA = (0<<ISC11) | (1<<ISC10) |			//alle Flanke des Encoder-Bit 0 erzeugt einen Interrupt
			(1<<ISC01) | (0<<ISC00);			//fallende Flanke des ZCSignals erzeugt einen Interrupt
	EIMSK = (1<<INT1) | (1<<INT0);				//ZCS-Interrupt und Encoder-Interrupt aktiviern

	PORTB &= ~(RELAIS_OUT | THYRISTOR_OUT);
	DDRB |= (RELAIS_OUT | THYRISTOR_OUT);		//Relais- und Thyristor-Steuerping als Ausgang

	DDRB |= (RED_LED_OUT | GREEN_LED_OUT);		//Status-LEDs als Ausgang

	TCCR1A	= (	
				//(1<<COM1A1) | (1<<COM1A0) |
				//(1<<COM1B1) | (1<<COM1B0) |
				(0<<WGM11) | (0<<WGM10));		//PWM-Mode 3, Clear OC1A, OC1B on Compare Match, set OC1A, OC1B at TOP

	OCR1A  = 0x0000;
	//OCR1AL  = 0x00;
	OCR1B  = 0x0000;
	//OCR1BL  = 0x00;
	//ICR1	= 0xFF;
	//TIMSK	= 0;
	TCCR1B	= ((0<<WGM13) | (0<<WGM12)); //Timer an /8

	encoder_val = ~(((PIND & ENCODER_MASK)>>PD3)|0xF0);

	init_ADC();

	//Interrupts anwerfen
	sei();

	//Watchdogtimer aktivieren
	wdt_enable(WDTO_60MS);
}


/*!***********************************************************************
 * @fn		int main (void)
 * @brief	Hauptprogramm-Schleife
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	Type int (never returns)
 */
int main (void) {


	//Systeminitialisierug
	init_Sys();

	//Usart_Write ((PBYTE)str, strlen(str));

	tsr_start();

	//Endlosschleife 
	while(1) {

		//Watchdog zurücksetzen
		wdt_reset();

		_delay_ms (1);
	};

	return 0;
}
