/*
 * drvr_usart.c
 *
 * Created: 8/18/2017 9:29:18 AM
 *  Author: akudlacek
 */


#include "drvr_usart.h"

#include "sam.h"
#include "ring_buffer.h"
#include "drvr_port.h"
#include "drvr_clock.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define SERCOM4_SYNC_WAIT(syncbusy_flags) while((SERCOM4->USART.SYNCBUSY.reg & syncbusy_flags) != 0)


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
//declare memory for buffer
static volatile uint8_t uart_tx_array[UART_TX_RING_BUFFER_SIZE_BYTE] = {0};
static volatile uint8_t uart_rx_array[UART_RX_RING_BUFFER_SIZE_BYTE] = {0};

//declare buffer
static volatile ring_buffer_t uart_tx_buffer;
static volatile ring_buffer_t uart_rx_buffer;


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Initialize Sercom peripheral for use with USART
*
*  \note With hardware handshaking
******************************************************************************/
void drvr_usart_init(void)
{
	SERCOM_USART_CTRLA_Type CTRLA_tmp = {0};
	SERCOM_USART_CTRLB_Type CTRLB_tmp = {0};

	/*Init pins*/
	drvr_port_pin_cfg(UC_UART_TX,  INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, C, LOW);
	drvr_port_pin_cfg(UC_UART_RX,  INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, C, LOW);
	drvr_port_pin_cfg(UC_UART_RTS, INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, C, LOW);
	drvr_port_pin_cfg(UC_UART_CTS, INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, C, LOW);

	//Initialize buffer structures
	ring_buffer_init(&uart_tx_buffer, uart_tx_array, UART_TX_RING_BUFFER_SIZE_BYTE);
	ring_buffer_init(&uart_rx_buffer, uart_rx_array, UART_RX_RING_BUFFER_SIZE_BYTE);

	/*Configure SERCOM4 USART*/
	SERCOM4->USART.CTRLA.bit.ENABLE = 0; //disable USART
	SERCOM4_SYNC_WAIT(SERCOM_USART_SYNCBUSY_ENABLE);

	//CTRLA
	CTRLA_tmp.bit.MODE   = 0x1;   //1//   USART with internal clock
	CTRLA_tmp.bit.CMODE  = 0;     //2//   Asynchronous communication.
	CTRLA_tmp.bit.RXPO   = 0x1;   //3//   RxD=PAD1
	CTRLA_tmp.bit.TXPO   = 0x2;   //4//   TxD=PAD0, RTS=PAD2, and CTS=PAD3
	CTRLA_tmp.bit.DORD   = 1;     //6//   LSB is transmitted first.
	CTRLA_tmp.bit.FORM   = 0x0;   //7.1// USART frame, no parity
	CTRLA_tmp.bit.SAMPR  = 0x0;   //      16x over-sampling using arithmetic baud rate generation.

	//CTRLB
	CTRLB_tmp.bit.CHSIZE = 0x0;   //5//   Character Size, 8 bits
	CTRLB_tmp.bit.PMODE  = 0;     //7.2// Even parity, not used from line above
	CTRLB_tmp.bit.SBMODE = 0;     //8//   Stop Bit Mode, One stop bit.
	CTRLB_tmp.bit.RXEN   = 1;     //10//  The receiver is enabled
	CTRLB_tmp.bit.TXEN   = 1;     //10//  The transmitter is enabled

	//BAUD - BAUD.reg = 65536*(1-SAMPR_val*(f_BAUD/f_REF))
	SERCOM4->USART.BAUD.reg = (uint16_t)(65536.0*(1.0-16.0*((float)DRVR_USART_BAUD/(float)drvr_clock_get_gen1_freq_hz())));

	/*Enable interrupts*/
	//INTENSET
	SERCOM4->USART.INTENSET.bit.ERROR = 1; //Error interrupt is enabled.
	SERCOM4->USART.INTENSET.bit.RXC   = 1; //Receive Complete interrupt is enabled.
	SERCOM4->USART.INTENSET.bit.DRE   = 1; //Data Register Empty interrupt is enabled.
	SERCOM4->USART.INTENSET.bit.TXC   = 0; //Transmit Complete Interrupt is disabled.

	/*Set config*/
	SERCOM4->USART.CTRLA.reg        = CTRLA_tmp.reg; //set config
	SERCOM4->USART.CTRLB.reg        = CTRLB_tmp.reg; //set config
	SERCOM4_SYNC_WAIT(SERCOM_USART_SYNCBUSY_MASK);

	/*Enable USART*/
	SERCOM4->USART.CTRLA.bit.ENABLE = 1; //enable USART
	SERCOM4_SYNC_WAIT(SERCOM_USART_SYNCBUSY_ENABLE);
}

/******************************************************************************
*  \brief TX string
*
*  \note must be null terminated
******************************************************************************/
void drvr_usart_tx_str(const char * const str)
{
	uint32_t i = 0;

	/****CRITICAL START****/
	SERCOM4->USART.INTENCLR.reg = 0x1; //Data Register Empty interrupt is disabled.

	while(str[i] != 0)
	{
		ring_buffer_put_data(&uart_tx_buffer, str[i]);
		i++;
	}

	SERCOM4->USART.INTENSET.bit.DRE = 1; //Data Register Empty interrupt is enabled.
	/****CRITICAL END****/
}

/******************************************************************************
*  \brief TX data array
*
*  \note
******************************************************************************/
void drvr_usart_tx_data(const uint8_t * const data, uint32_t length)
{
	uint32_t i = 0;

	/****CRITICAL START****/
	SERCOM4->USART.INTENCLR.reg = 0x1; //Data Register Empty interrupt is disabled.

	for(i = 0; i < length; i++)
	{
		ring_buffer_put_data(&uart_tx_buffer, data[i]);
	}

	SERCOM4->USART.INTENSET.bit.DRE = 1; //Data Register Empty interrupt is enabled.
	/****CRITICAL END****/
}

/******************************************************************************
*  \brief RX byte
*
*  \note returns data or -1 if no data
******************************************************************************/
int16_t drvr_usart_rx_byte(void)
{
	int16_t rx_character;

	/****CRITICAL START****/
	SERCOM4->USART.INTENCLR.reg = 0x4; //Receive Complete interrupt is disabled.

	rx_character = ring_buffer_get_data(&uart_rx_buffer);

	SERCOM4->USART.INTENSET.bit.RXC = 1; //Receive Complete interrupt is enabled.
	/****CRITICAL END****/

	//returns received data or -1 for no data
	return rx_character;
}


/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
/******************************************************************************
*  \brief interrupt for SERCOM4
*
*  \note CRITICAL RESOURCES: the complimentary put and get functions need to be protected
*        NEEDS PROTECTION
*            ring_buffer_get_data(&uart_rx_buffer)
*            ring_buffer_put_data(&uart_tx_buffer, ???)
*        DOES NOT NEED PROTECTION:
*
******************************************************************************/
void SERCOM4_Handler(void)
{
	int16_t data = 0;

	/*Error*/
	if(SERCOM4->USART.INTFLAG.bit.ERROR)
	{
		SERCOM4->USART.INTFLAG.reg = 0x80; //clear flag

		//IGNORING HARDWARE ERRORS
		asm("NOP");
	}

	/*RX complete*/
	if(SERCOM4->USART.INTFLAG.bit.RXC)
	{
		SERCOM4->USART.INTFLAG.reg = 0x4; //clear flag

		//read new data and put to rx buffer, and check for buffer overflow flag
		if(ring_buffer_put_data(&uart_rx_buffer, SERCOM4->USART.DATA.bit.DATA) == -1)
		{
			//IGNORING RX BUFFER OVERFLOWS
			asm("NOP");
		}
	}

	/*Data reg empty*/
	if(SERCOM4->USART.INTFLAG.bit.DRE)
	{

		//CLEARING DRE FLAG DOES NOTHING, TO STOP INTERRUPT DISABLE DRE INTERRUPT

		data = ring_buffer_get_data(&uart_tx_buffer); //get data from buffer

		//if there is data to send
		if(data != -1)
		{
			SERCOM4->USART.DATA.bit.DATA = (uint8_t)data; //load tx buffer
		}
		//if there is no data disable DRE interrupts
		else
		{
			//GLITCH: if i use SERCOM4->USART.INTENCLR.bit.DRE = 1; it clears all the interrupts for USART
			SERCOM4->USART.INTENCLR.reg = 0x1; //Data Register Empty interrupt is disabled.
		}
	}
}
