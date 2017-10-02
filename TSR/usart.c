/*! \file       uSart.c
 *  \brief      Treiber für die Atmel Uart (ibis-usage).
 *  \author     René Schönrock
 *  \version    1.0
 *  \date       27.11.2006
 *  \bug        keine.
 *  \warning    keine.
 *  \note       
 *  \see        
 *  \todo       
 *
 *  Details.
 */


 #include <stdio.h>
 #include <avr/io.h>

 //#include <avr/eeprom.h>
 #include <avr/interrupt.h>

 #include <util/delay.h>
 #include <stdlib.h>

 #include <string.h>
 //#include <my_string.h>
 //#include <stdarg.h>

 #include "types.h"

 #include "usart.h"


 volatile BYTE   usart_task_scheduler;
 volatile BYTE   usart_scheduler_mask;
 CFile ComPortFile;
 CFile* pChannel;



	//Uart initialisieren
/*	if ( Usart_OpenComPort ((USART_READ | USART_WRITE)))
	{
		Usart_SetBaudrate (57600);
		Usart_InterruptEnable ((USART_RXIE | USART_TXIE));
		Usart_SetEventHandler (NULL, (RL_STATE_ERR_EVENT | CT_EVENT | FF_EVENT | THR_EVENT));
		Usart_SetTiggerChar ('\r' );	//CR
		Usart_SetLineCtrl (8,'n',1, (USART_RXEN | USART_TXEN));
	}
*/



 /**
  * @fn         void ibis_usart_rx_isr(void)
  * @brief      RX Interrupt Service Routine
  *
  * Auslesen des Uart-FIFO.
  *
  * @todo       manuelle Register-Rettung?
*/
 //interrupt [USART_RX_INT] void usart_rx_isr(void)
 ISR(USART_RX_INT)
 {
   char status, data;

   status = USART_UCSRA;
   data   = USART_UDR;

	if(!pChannel)
		return;                             
   
   if ((status & (_BV(FE0) | _BV(UPE0) | _BV(DOR0)))==0)   //wurde das Zeichen korrekt empfangen?
    {
     pChannel->rx_buff[pChannel->rx_wr_index]=data;
     if ( ++(pChannel->rx_wr_index) == RX_BUFFER_SIZE )
      { 
       pChannel->rx_wr_index=0;
      } 

     if ( ++(pChannel->rx_counter) > RX_BUFFER_SIZE )
      {
       //Fehlerbehandlung ???
       pChannel->rx_counter = 0;
       pChannel->com_state.rx_buff_overflow = 1;
       
       usart_task_scheduler |= USART_PROCESS_ERROR;
       //main_task_scheduler |= PROCESS_USART; 
      }
     else
      {
       if ( (pChannel->event_mask) & RHR_EVENT )
        {
         usart_task_scheduler |= USART_PROCESS_RECEIVE;
         //main_task_scheduler |= PROCESS_USART; 
        } 
       
       if ( ((pChannel->event_mask) & FF_EVENT) && ((pChannel->rx_counter) == RX_BUFFER_SIZE) )
        {
         usart_task_scheduler |= USART_PROCESS_FIFO_FULL;
         //main_task_scheduler |= PROCESS_USART; 
        }
       
       if ( ((pChannel->event_mask) & CT_EVENT) && (data==(pChannel->tigger_char)) )
        {
         usart_task_scheduler |= USART_PROCESS_CHAR_TRIG;
         //main_task_scheduler |= PROCESS_USART; 
        } 
      } 
    }
   else
    {
     //Fehlerbehandlung ???
     pChannel->com_state.rx_fe  = (status & _BV(FE0));
     pChannel->com_state.rx_pe  = (status & _BV(UPE0));
     pChannel->com_state.rx_ovr = (status & _BV(DOR0));
     
     usart_task_scheduler |= USART_PROCESS_ERROR;
     //main_task_scheduler |= PROCESS_USART; 
    }  
  
  }




// #pragma used+
 /**
  * @fn         BOOL usart_getchar(char *pData)
  * @brief      Read a character from the USART Receiver buffer ()
  */
 BOOL usart_getchar(char *pData)
  {
   if (!pChannel || !pChannel->rx_counter) return FALSE;
   
   *pData = pChannel->rx_buff[pChannel->rx_rd_index];
   
   if (++pChannel->rx_rd_index == RX_BUFFER_SIZE)
    {
     pChannel->rx_rd_index=0;
    } 
    
   cli();
   --pChannel->rx_counter;
   sei();
   return TRUE;
  }
// #pragma used-




/**
 * @fn         void usart_tx_isr(void)
 * @brief      USART Transmitter interrupt service routine
 * @param      keine
 * @return     keine       
 * @todo       manuelle Registersicherung?
 */
//interrupt [USART_TX_INT] void usart_tx_isr(void)
ISR(USART_TX_INT)
 {
 
  //#asm("sei");                                      // Für andere Interrupts unterbrechbar machen
	if (!pChannel)
		return;

  if (pChannel->tx_counter)                                 
   {
    --(pChannel->tx_counter);
      
    USART_UDR = pChannel->tx_buff[pChannel->tx_rd_index];
    if (++(pChannel->tx_rd_index) == TX_BUFFER_SIZE)
     { 
      pChannel->tx_rd_index=0;
     }
     
    if ( !(pChannel->tx_counter) && ((pChannel->event_mask) & THR_EVENT) )
     {
     
      pChannel->event_mask |= RHR_EVENT;
      
      usart_task_scheduler |= USART_PROCESS_TRANSMIT;
      //main_task_scheduler |= PROCESS_USART; 
     } 
   }
 }
//#pragma savereg+



 /**
  * @fn         BOOL usart_putchar(char c)
  * @brief      Write a character to the IBIS-USART Transmitter buffer
  * @todo       Busy-Waiting ändern!
  */
 BOOL usart_putchar(char c)
  {
   if (!pChannel)
   	return FALSE;
   
   if ( pChannel->tx_counter && pChannel->tx_counter >= TX_BUFFER_SIZE ) return FALSE;
   //while ( !(pChannel->tx_counter < pChannel->tx_buffer_size) );    //Busy-Waiting ???

   cli();
   if ( pChannel->tx_counter || !(USART_UCSRA & _BV(UDRE0)) )
    {
     pChannel->tx_buff[pChannel->tx_wr_index] = c;
     
     if (++(pChannel->tx_wr_index) == TX_BUFFER_SIZE) 
      {
       pChannel->tx_wr_index = 0;
      }
      
     ++(pChannel->tx_counter);
    }
   else
    {
     USART_UDR=c;
    }
   sei();
   return TRUE;
  }



/**
 * @fn         BYTE Usart_InterruptEnable (CFile* pCom, BYTE ie_mask)
 * @brief      Interrupts (RX, TX) für die USART aktivieren
 *
 * @param      pCom     Filehandle für den entspr. COM-Port
 * @param      ie_mask  Interrupt-Enable-Mask
 */
 BYTE Usart_InterruptEnable (BYTE ie_mask)
  {
   if (pChannel)
    {
     USART_UCSRB |= (ie_mask & 0xE0);
  
     return TRUE; 
    }
   else return FALSE;  
  }



//#pragma used+
/**
 * @fn         BYTE Usart_InterruptDisable (CFile* pCom, BYTE id_mask)
 * @brief      Interrupts (RX, TX) für die USART deaktivieren
 *
 * @param      pCom    Filehandle für den entspr. COM-Port
 * @param      id_mask Interrupt-Disable-Mask
 * @todo       
 */
 BYTE Usart_InterruptDisable (BYTE id_mask)
  {
   USART_UCSRB &= ~(id_mask & 0xE0);

   return TRUE; 
  }
//#pragma used-



/**
 * @fn         BYTE Usart_SetLineCtrl (CFile* pCom, BYTE data_length, char parity, BYTE stop_bit)
 * @brief      Com-Port Einstellungen (LineCtrl)
 *
 * @param      pCom             Filehandle für den entspr. COM-Port
 * @param      data_length      Anzahl der Datenbits
 * @param      parity           Art des Paritätsbits
 * @param      stop_bit         Anzahl der Stopbits
 * @param      rx_tx_enable     Receiver, Transmitter einschalten
 *
 * @return     TRUE | FALSE 
 * @note        nur asynchroner Modus, 5<=Datenbits<=8      
 * @todo       
 */
 BYTE Usart_SetLineCtrl (BYTE data_length, char parity, BYTE stop_bit, BYTE rx_tx_enable)
  {
   BYTE lcr;
   
   lcr = 0;
   
   if (data_length<5 || data_length>8) 
      return FALSE;
    
   lcr |= ((data_length - 5) << 1);
     
   switch (parity)
   {
       case 'n':    //keine Parität LCR[3]=0
             break;
       case 'o':    //odd
             lcr |= 0x30;
             break;
       case 'e':    //even
             lcr |= 0x20;
             break;
       default: return FALSE;      
   } 
   
   if (stop_bit == 2) lcr |= 0x08;
   
   USART_UCSRC = lcr;

   USART_UCSRB |= (rx_tx_enable & (USART_RXEN | USART_TXEN));
   
   return TRUE;    
  }



/**
 * @fn         BYTE Usart_SetBaudrate (CFile* pCom, ULONG baudrate)
 * @brief      Einstellung der Baudrate für den entspr. Com-Port
 *
 * @param      pCom             Filehandle für den entspr. COM-Port
 * @param      baudrate         Baudrate
 *
 * @return     TRUE | FALSE       
 * @todo       
 */
 BYTE Usart_SetBaudrate (ULONG baudrate)
  {
   WORD divisor;
   //BYTE double_factor;

   if (baudrate <= USART_MAX_BR)
    {
     //double_factor = ((USART_UCSRA & 0x02) >> 1) + 1;
     //divisor = (WORD)((_MCU_CLOCK_FREQUENCY_/sys_clk_devisor)/(baudrate*(16/double_factor))-1);
     divisor = (WORD)((F_CPU)/(baudrate*16)-1);
     
     USART_UBRRH = (BYTE)(divisor>>8);
     USART_UBRRL = (BYTE)divisor;
     
     return TRUE;
    }
   return FALSE;   
  }



/**
 * @fn         BYTE Usart_SetBaudrate (CFile* pCom, ULONG baudrate)
 * @brief      Einstellung der Baudrate für den entspr. Com-Port
 *
 * @param      pCom             Filehandle für den entspr. COM-Port
 * @param      baudrate         Baudrate
 *
 * @return     TRUE | FALSE       
 * @todo       
 */
 BOOL Usart_SetEventHandler (void *pEventHandler, BYTE event_mask)
 {
	if (!pChannel)
		return FALSE;

    pChannel->pEventHandler = pEventHandler;
    pChannel->event_mask = event_mask;
    
    usart_scheduler_mask |= event_mask;

   return TRUE;   
 }



/**
 * @fn         BOOL Usart_SetTiggerChar (CFile* pCom, char tigger_char)
 * @brief      Einstellung des Triggerzeichens
 *
 * @param      pCom             Filehandle für den entspr. COM-Port
 * @param      tigger_char      Zeicher, welches ein Trigger-Event auslösen soll
 *
 * @return     TRUE | FALSE       
 * @todo       
 */
 BOOL Usart_SetTiggerChar (char tigger_char)
 {
	if (!pChannel)
		return FALSE;

    pChannel->tigger_char = tigger_char;

   return TRUE;   
 }



/**
 * @fn         BYTE Usart_GetRxCount (CFile* pCom)
 * @brief      Ermittelt die Anzahl der gültigen Bytes im RX-FIFO
 *
 * @param      pCom             Filehandle für den entspr. COM-Port
 *
 * @return     Anzahl der Bytes im FIFO       
 * @todo       
 */
 BYTE Usart_GetRxCount (void)
 {
  BYTE rx_counter;

	if (!pChannel)
		return 0;

    cli();
    rx_counter = pChannel->rx_counter;
    sei();
    
    return (rx_counter);
 }



/**
 * @fn         CFile* Usart_OpenComPort (BYTE rx_buffer_size, BYTE tx_buffer_size, BYTE flags)
 * @brief      Usart Com-Port öffnen
 * @param      flags
 * @return     Zeiger vom Type CFile       
 * @todo       
 */
 BYTE Usart_OpenComPort (BYTE flags)
 {
  if ( pChannel ) //Kanal kann nur einmal geöffnet werden
   return 0;
     
  if ( (pChannel = (CFile*)&ComPortFile) )
   {
    pChannel->com_channel = 0;
    pChannel->flags = flags;
    pChannel->pEventHandler = (void*)NULL;

    pChannel->rx_wr_index = 0;
    pChannel->rx_rd_index = 0;
    pChannel->rx_counter = 0;
    //pChannel->rx_buffer_size = rx_buffer_size;

    pChannel->tx_wr_index = 0;
    pChannel->tx_rd_index = 0;
    pChannel->tx_counter = 0;
    //pChannel->tx_buffer_size = tx_buffer_size;

    USART_UCSRA = 0;
    USART_UCSRB = 0;
    USART_UCSRC = 0;
    USART_UBRRH = 0;
    USART_UBRRL = 0;
   
    usart_scheduler_mask |= (USART_PROCESS_ERROR | USART_PROCESS_RECEIVE);
    usart_task_scheduler = 0;
   
    //main_scheduler_mask |= PROCESS_USART;

    return 1;
   }
  return 0;
 }


/**
 * @fn         BYTE Usart_CloseComPort (CFile* pCom)
 * @brief      Usart Com-Port öffnen
 * @param      pCom Zeiger vom Type CFile auf den zu schließenden Com-Port
 * @return     TRUE | FALSE       
 * @todo       
 */
 BYTE Usart_CloseComPort (void)
 {
  if (pChannel)
   {
    usart_scheduler_mask = 0;
    //main_scheduler_mask &= ~PROCESS_USART;
    
    USART_UCSRA = 0;
    USART_UCSRB = 0;
    USART_UCSRC = 0;
    USART_UBRRH = 0;
    USART_UBRRL = 0;

	pChannel=NULL;
   
    return TRUE;
   }
  return FALSE;   
 }




/**
 * @fn         BYTE Usart_Read (CFile* pCom, PBYTE bBuffer, BYTE count)
 * @brief      Usart Com-Port lesen
 *
 * @param      pCom ist der Zeiger auf das Com-Handel
 * @param      bBuffer ist der Bytepointer auf den Puffer, in den die Bytes geschrieben werden sollen
 * @param      count Anzahl der Bytes, die gelesen werden sollen
 *
 * @return     Anzahl der gelesenen Bytes       
 * @todo       
 */
 BYTE Usart_Read (PBYTE bBuffer, BYTE count)
 {
  BYTE i;
  
  i=0;
  if (pChannel && (pChannel->flags & USART_READ))
   {
      while ( pChannel->rx_counter && (!count || i<count) )
       {
        bBuffer[i++] = pChannel->rx_buff[pChannel->rx_rd_index];
        if (++pChannel->rx_rd_index == RX_BUFFER_SIZE)
         pChannel->rx_rd_index = 0;
    
        cli();
        --pChannel->rx_counter;
        sei();
       }
      return i;
   }
  else 
   return FALSE;   
 }



/**
 * @fn         BYTE Usart_Write (CFile* pCom, PBYTE bBuffer, BYTE count)
 * @brief      Usart Com-Port schreiben
 *
 * @param      pCom ist der Zeiger auf das Com-Handel
 * @param      pBuffer ist der Bytepointer auf den Puffer in dem die Bytes stehen, die geschrieben werden sollen
 * @param      count Anzahl der Bytes, die geschrieben werden sollen
 *
 * @return     Anzahl der geschriebenen Bytes       
 * @todo       
 */
 BYTE Usart_Write (PBYTE pBuffer, BYTE count)
 {
  BYTE tx_bytes;
  
  if (pChannel && (pChannel->flags & USART_WRITE))
   {
    tx_bytes=0;
    while ( tx_bytes<count && usart_putchar(pBuffer[tx_bytes++]) ); 
    return tx_bytes; 
   }
  else return FALSE;   
 }



/**
 * @fn         BYTE ProcessUsart(void)
 * @brief      Arbeitsroutine für die USART
 * @param      keine
 * @return     TRUE | FALSE       
 * @todo       
 */
BYTE ProcessUsart(void)
  {
   BYTE result;
   BYTE (*pHandler) (BYTE event, void* pParam);

	if(!pChannel || !usart_task_scheduler)
		return 0;
   
   result = TRUE;

   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Arbeitstask 
   if((usart_task_scheduler & usart_scheduler_mask) & USART_PROCESS_ERROR)
    {
     pChannel->event |= RL_STATE_ERR_EVENT;
     usart_task_scheduler &= ~USART_PROCESS_ERROR;
    }
  
   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Arbeitstask 
   if((usart_task_scheduler & usart_scheduler_mask) & USART_PROCESS_TIMEOUT)
    {
     usart_task_scheduler &= ~USART_PROCESS_TIMEOUT;
    }
    
   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Arbeitstask 
   if((usart_task_scheduler & usart_scheduler_mask) & USART_PROCESS_RECEIVE)
    {
     pChannel->event |= RHR_EVENT;
     usart_task_scheduler &= ~USART_PROCESS_RECEIVE;
    }

   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Arbeitstask 
   if((usart_task_scheduler & usart_scheduler_mask) & USART_PROCESS_CHAR_TRIG)
    {
     pChannel->event |= CT_EVENT;
     usart_task_scheduler &= ~USART_PROCESS_CHAR_TRIG;
    }

   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Arbeitstask 
   if((usart_task_scheduler & usart_scheduler_mask) & USART_PROCESS_FIFO_FULL)
    {
     pChannel->event |= FF_EVENT;
     usart_task_scheduler &= ~USART_PROCESS_FIFO_FULL;
    }

   /////////////////////////////////////////////////////////////////////////////////////////////////////
   // Arbeitstask 
   if((usart_task_scheduler & usart_scheduler_mask) & USART_PROCESS_TRANSMIT)
    {
     pChannel->event |= THR_EVENT;
     usart_task_scheduler &= ~USART_PROCESS_TRANSMIT;
    }

    
   result = 0;
   if ( pChannel && (pChannel->event & pChannel->event_mask) && pChannel->pEventHandler )
    {
     pHandler = pChannel->pEventHandler;
     result = (*pHandler)(pChannel->event, (void*)pChannel);
    }
    
   //cli();
    
   return result;  
  }
 
