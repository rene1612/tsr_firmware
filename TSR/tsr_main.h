/*! \file       tsr_main.h
 *  \brief      Firmware für den TSR-Controller
 *  \author     René Schönrock
 *  \version    1.0
 *  \date       20.05.2010
 *  \bug        keine.
 *  \warning    keine.
 *  \note       
 *  \see        
 *  \todo       
 *
 *  Details.
 */


#ifndef  __TSR_H_
 #define  __TSR_H_

   
  
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Diverse Definitionen und Konstanten
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
 
 
 
/*!
 * Main Reg-Defines
 */





/*!
 * special Reg-Defines
 */

/**
 * @def		RED_LED_OUT
 * @brief	Bitmaske für das LEDsignal(ROT) (PB7 = Pin10 am Controller)
 * @note	
 */
 #define RED_LED_OUT			(1<<PB6)

/**
 * @def		RED_LED_ON
 * @brief	Makro zum EINschalten
 * @note	
 * @note	Highaktiv
 */
 #define RED_LED_ON		PORTB |= RED_LED_OUT


/**
 * @def		RED_LED_OFF
 * @brief	Makro zum "AUSschalten" 
 * @note
 * @note	Highaktiv
 */
 #define RED_LED_OFF		PORTB &= ~RED_LED_OUT

/**
 * @def		RED_LED_IN
 * @brief	Bitmaske für das LEDsignal(GRÜN) (PB6 = Pin9 am Controller)
 * @note	
 */
 #define RED_LED_IN			(PINB&RED_LED_OUT)

/**
 * @def		GREEN_LED_OUT
 * @brief	Bitmaske für das LEDsignal(GRÜN) (PB6 = Pin9 am Controller)
 * @note	
 */
 #define GREEN_LED_OUT			(1<<PB7)

/**
 * @def		GREEN_LED_IN
 * @brief	Bitmaske für das LEDsignal(GRÜN) (PB6 = Pin9 am Controller)
 * @note	
 */
 #define GREEN_LED_IN			(PINB&GREEN_LED_OUT)

/**
 * @def		GREEN_LED_ON
 * @brief	Makro zum 
 * @note	Highaktiv
 */
 #define GREEN_LED_ON		PORTB |= GREEN_LED_OUT


/**
 * @def		GREEN_LED_OFF
 * @brief	Makro zum 
 * @note	 
 * @note	Highaktiv
 */
 #define GREEN_LED_OFF		PORTB &= ~GREEN_LED_OUT


/**
 * @def		ZERO_CROSSING_INT
 * @brief	Bitmaske für das Interruptsignal (PD2 = Pin6 am Controller)
 * @note	
 */
 #define ZERO_CROSSING_INT			(1<<PD2)


/**
 * @def		ENCODER_INT
 * @brief	Bitmaske für das Interruptsignal (PD3 = Pin7 am Controller)
 * @note	
 */
 #define ENCODER_INT			(1<<PD3)


/**
 * @def		RELAIS_OUT
 * @brief	Bitmaske für das Relaissteuersignal (PB4 = Pin16 am Controller)
 * @note	
 */
 #define RELAIS_OUT			(1<<PB1)


/**
 * @def		THYRISTOR_OUT
 * @brief	Bitmaske für das Thyristorsteuersignal (PB3 = Pin15 am Controller)
 * @note	
 */
 #define THYRISTOR_OUT			(1<<PB2)

/**
 * @def		THYRISTOR_IN
 * @brief	Bitmaske für das Thyristorsteuersignal (PB3 = Pin15 am Controller)
 * @note	
 */
 #define THYRISTOR_IN			(PINB&THYRISTOR_OUT)


/**
 * @def		ENCODER_BIT_0
 * @brief	Bitmaske für das Encoder-Bit 0 (PD3 = Pin7 am Controller)
 * @note	
 */
 #define ENCODER_BIT_0			(1<<PD3)

/**
 * @def		ENCODER_BIT_1
 * @brief	Bitmaske für das Encoder-Bit 1 (PD4 = Pin8 am Controller)
 * @note	
 */
 #define ENCODER_BIT_1			(1<<PD4)

/**
 * @def		ENCODER_BIT_0
 * @brief	Bitmaske für das Encoder-Bit 2 (PD5 = Pin9 am Controller)
 * @note	
 */
 #define ENCODER_BIT_2			(1<<PD5)

/**
 * @def		ENCODER_BIT_1
 * @brief	Bitmaske für das Encoder-Bit 3 (PD6 = Pin10 am Controller)
 * @note	
 */
 #define ENCODER_BIT_3			(1<<PD6)


/**
 * @def		ENCODER_MASK
 * @brief	Bitmaske für alle Encoder-Bits
 * @note	
 */
 #define ENCODER_MASK			(ENCODER_BIT_0 | ENCODER_BIT_1 | ENCODER_BIT_2 | ENCODER_BIT_3)


/**
 * @def		RELAIS_ON
 * @brief	Makro zum EINschalten des Relais (PB4 = Pin16 am Controller)
 * @note	Einschalten bedeute Relais zieht an, Arbeitskontakt Schließer wird geschlossen,
 *			230V werden über den Relaiskontakt auf die Last(Trafo) geschaltet.
 * @note	Highaktiv
 */
 #define RELAIS_ON		PORTB |= RELAIS_OUT


/**
 * @def		RELAIS_OFF
 * @brief	Makro zum AUSschalten des Relais (PB4 = Pin16 am Controller)
 * @note	Ausschalten bedeute Relais fällt ab, Arbeitskontakt Öffner wird geschlossen,
 *			die Last(Trafo) wird von den 230V getrennt und über einen Ballastwiederstand (R6)
 *			gebrückt.
 */
 #define RELAIS_OFF		PORTB &= ~RELAIS_OUT



/**
 * @def		THYRISTOR_ON
 * @brief	Makro zum EINschalten(Zünden) des Thyristors (PB3 = Pin15 am Controller)
 * @note	Einschalten bedeute Thyristor zündet,
 *			230V werden über den Thyristro auf die Last(Trafo) geschaltet.
 * @note	Highaktiv
 */
 #define THYRISTOR_ON		PORTB |= THYRISTOR_OUT


/**
 * @def		THYRISTOR_OFF
 * @brief	Makro zum "AUSschalten" des Thyristors (PB3 = Pin15 am Controller)
 * @note	Ausschalten bedeute, Thyristor verlischt, wenn der Strom durch 
 *			Thyristor und Last 0 wird.
 * @note	Highaktiv
 */
 #define THYRISTOR_OFF		PORTB &= ~THYRISTOR_OUT




/****************************************************************************
 * Global definitions
 */
 #define PERIOD_TIME	10000

 #define TIMER1_START	TCCR1B |= _BV(CS11)
 #define TIMER1_STOP	TCCR1B &= ~_BV(CS11)

/**
 * Offset-Defines für den direkten Zugriff auf die einzelnen Register
 */



/**
 * Bit-Defines für 
 */










 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //
 // Datenstructuren
 //
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/**
 * @struct	MOP425_REG
 * @brief	Registersatz des MOP425-Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
 /**
  * @var	unsigned char dev_signature
  * @brief	Register mit der Gerätekennung
  * @see	__DEV_SIGNATURE__
  * @see	DEV_SIGNATURE_REG
  * @see	config.h
  */
  unsigned char		dev_signature;

 /**
  * @var	unsigned int sw_release
  * @brief	Register mit der Softwareversion
  * @see	__SW_RELEASE__
  * @see	SW_REL_REG
  * @see	config.h
  */
  unsigned int		sw_release;						

 /**
  * @var	unsigned int sw_release_date
  * @brief	Register mit dem Datum der Softwareversion
  * Formatierung:
  *	- Byte 0 -> Tag
  *	- BYTE 1 -> Monat
  *	- BYTE 2 -> Jahr
  *	- BYTE 3 -> Jahr
  * @see	__SW_RELEASE_DATE__
  * @see	SW_REL_DATE_REG
  * @see	config.h
  */
  unsigned long		sw_release_date;						
 }TSR_REG;



 
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //
 // Funktionen(Prototypes)
 //
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 
#endif  //__TSR_H_
