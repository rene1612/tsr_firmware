/*! \file       usart.h
 *  \brief      Treiber für die USART vom ATMEGA.
 *  \author     René Schönrock
 *  \version    1.0
 *  \date       07.08.2006
 *  \bug        keine.
 *  \warning    keine.
 *  \note       
 *  \see        
 *  \todo       
 *
 *  Details.
 */


#ifndef  __USART_H_
 #define  __USART_H_

   
  
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Diverse Definitionen und Konstanten
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
 #define USART_NR       0
 #define USART_RX_INT	USART_RX_vect
 #define USART_TX_INT	USART_TX_vect
 #define USART_UCSRA	UCSR0A
 #define USART_UCSRB	UCSR0B
 #define USART_UCSRC	UCSR0C
 #define USART_UDR		UDR0
 #define USART_UBRRH	UBRR0H
 #define USART_UBRRL	UBRR0L

 
 #define USART_MAX_BR		115200L

 
 #define USART_PROCESS_ERROR      0x01
 #define USART_PROCESS_TIMEOUT    0x02
 #define USART_PROCESS_RECEIVE    0x04
 #define USART_PROCESS_TRANSMIT   0x08
 #define USART_PROCESS_CHAR_TRIG  0x10
 #define USART_PROCESS_FIFO_FULL  0x20
 
 
 
 /*
  * Main Reg-Defines
  */
 
 #define USART_READ		0x01
 #define USART_WRITE	0x02

 #define USART_RXEN		0x10
 #define USART_TXEN		0x08
 
 #define USART_RXIE     0x80
 #define USART_TXIE     0x40

 /*
  * special Reg-Defines
  */

  #define RL_STATE_ERR_EVENT  	0x01
  #define REC_TIMEOUT_EVENT  	0x02
  #define RHR_EVENT  		0x04
  #define THR_EVENT  		0x08
  #define CT_EVENT  		0x10
  #define FF_EVENT  		0x20
 
 
  #define RX_BUFFER_SIZE  	8
  #define TX_BUFFER_SIZE  	16
 

 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //
 // Datenstructuren
 //
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 typedef struct   _COM_STATE_FLAGS
  {
   unsigned int  rx_buff_overflow	:1;								//Tastenmodus
   unsigned int  rx_fe				:1;
   unsigned int  rx_ovr				:1;
   unsigned int  rx_pe				:1;
   unsigned int  dummy				:4;
  }COM_STATE_FLAGS;

 
 typedef struct
  {
   BYTE com_channel;      // 1
   BYTE flags;            // 1
  
   BYTE rx_buff[RX_BUFFER_SIZE];         // 2 
   BYTE tx_buff[TX_BUFFER_SIZE];         // 2

   BYTE rx_wr_index;      // 1
   BYTE rx_rd_index;      // 1
   BYTE rx_counter;       // 1

   BYTE tx_wr_index;      // 1
   BYTE tx_rd_index;      // 1
   BYTE tx_counter;       // 1

   //BYTE rx_buffer_size;   // 1
   //BYTE tx_buffer_size;   // 1

   BYTE event_mask;       // 1
   BYTE event;            // 1

   // This flag is set on Receiver buffer overflow
   COM_STATE_FLAGS com_state;  //1
   
   char tigger_char;    // 1
   
   // Handler
   //flash int*  pEventHandler;  // 2
  	/**
  	* @var	unsigned char max_msg_len
  	* @brief	Pointer auf eine Handlerroutine
  	*/
  int (*pEventHandler) (void*);
  
  }CFile;                //19



 
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //
 // Funktionen
 //
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



 BYTE Usart_InterruptEnable (BYTE ie_mask);


 BYTE Usart_SetLineCtrl (BYTE data_length, char parity, BYTE stop_bit, BYTE rx_tx_enable);


 BOOL Usart_SetTiggerChar (char tigger_char);
 

 BYTE Usart_SetBaudrate (ULONG baudrate);
 

 BOOL Usart_SetEventHandler (void *pEventHandler, BYTE event_mask);


 BYTE Usart_GetRxCount (void);


 BYTE Usart_OpenComPort (BYTE flags);


 BYTE Usart_CloseComPort (void);

 
 BYTE Usart_Read (PBYTE bBuffer, BYTE count);


 BYTE Usart_Write (PBYTE bBuffer, BYTE count);

 
 BOOL Usart_SetEventHandler (void *pEventHandler, BYTE event_mask);


 BYTE ProcessUsart(void);
  
 
#endif  //__USART_H
