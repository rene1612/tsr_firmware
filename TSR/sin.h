/**
 * @file	sin.h
 * @author	René Schönrock
 * @brief	Sinustabelle.
 *
 *
 * @date	26.05.2010
 */


#ifndef  __SIN_H_
 #define  __SIN_H_


/**
 * @def		MAX_SAMPLING_POINTS
 * @brief	Anzahl der Stützstellen für Sinus-Lookup-Tabelle
 */
 #define MAX_SAMPLING_POINTS	20



/**
 * @var		unsigned int sin_flash PROGMEM
 * @brief	Lookup-Tabelle für einen Sinus
 * @note	Die Tabelle steht im Programmspeicher
 * @note	Die Sinuswerte sind auf 1023 normiert
 * @see		MAX_SAMPLING_POINTS		
 */
 unsigned int sin_flash[MAX_SAMPLING_POINTS] PROGMEM =
  {
	0,		//0°;	0µs
	160,	//9°;	500µs
	316,	//18°;	1000µs
	464,	//27°;	1500µs
	601,	//36°;	2000µs
	723,	//45°;	2500µs
	827,	//54°;	3000µs
	911,	//63°;	3500µs
	972,	//72°;	4000µs
	1010,	//81°;	4500µs
	1023,	//90°;	5000µs
	1010,	//99°;	5500µs
	972,	//108°;	6000µs
	911,	//117°;	6500µs
	827,	//126°;	7000µs
	723,	//135°;	7500µs
	601,	//144°;	8000µs
	464,	//153°;	8500µs
	316,	//162°;	9000µs
	160,	//171°;	9500µs
  };



/**
 * @var		unsigned int sin_lookup
 * @brief	Kennlinienfeld für den LED-Thermistor
 * @note	Die Tabelle zum Programmstart in den RAM kopiert
 * @see		MAX_SAMPLING_POINTS		
 * @see		sin_flash		
 */
 unsigned int sin_lookup[MAX_SAMPLING_POINTS];

#endif
