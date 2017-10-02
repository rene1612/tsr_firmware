/**
 * @file	sin.h
 * @author	Ren� Sch�nrock
 * @brief	Sinustabelle.
 *
 *
 * @date	26.05.2010
 */


#ifndef  __SIN_H_
 #define  __SIN_H_


/**
 * @def		MAX_SAMPLING_POINTS
 * @brief	Anzahl der St�tzstellen f�r Sinus-Lookup-Tabelle
 */
 #define MAX_SAMPLING_POINTS	20



/**
 * @var		unsigned int sin_flash PROGMEM
 * @brief	Lookup-Tabelle f�r einen Sinus
 * @note	Die Tabelle steht im Programmspeicher
 * @note	Die Sinuswerte sind auf 1023 normiert
 * @see		MAX_SAMPLING_POINTS		
 */
 unsigned int sin_flash[MAX_SAMPLING_POINTS] PROGMEM =
  {
	0,		//0�;	0�s
	160,	//9�;	500�s
	316,	//18�;	1000�s
	464,	//27�;	1500�s
	601,	//36�;	2000�s
	723,	//45�;	2500�s
	827,	//54�;	3000�s
	911,	//63�;	3500�s
	972,	//72�;	4000�s
	1010,	//81�;	4500�s
	1023,	//90�;	5000�s
	1010,	//99�;	5500�s
	972,	//108�;	6000�s
	911,	//117�;	6500�s
	827,	//126�;	7000�s
	723,	//135�;	7500�s
	601,	//144�;	8000�s
	464,	//153�;	8500�s
	316,	//162�;	9000�s
	160,	//171�;	9500�s
  };



/**
 * @var		unsigned int sin_lookup
 * @brief	Kennlinienfeld f�r den LED-Thermistor
 * @note	Die Tabelle zum Programmstart in den RAM kopiert
 * @see		MAX_SAMPLING_POINTS		
 * @see		sin_flash		
 */
 unsigned int sin_lookup[MAX_SAMPLING_POINTS];

#endif
