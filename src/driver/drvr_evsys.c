/*
 * drvr_evsys.c
 *
 * Created: 11/14/2017 2:44:09 PM
 *  Author: akudlacek
 */


#include "drvr_evsys.h"

#include "sam.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief EVSYS init
*
*  \note
******************************************************************************/
void drvr_evsys_init(void)
{
	EVSYS_CHANNEL_Type CHANNEL_tmp = {0};

	/*Configure EVSYS*/
	/*NOTE: The Event Users Multiplexer must be configured first*/

	//USERm (Event Users Multiplexer)
	//Channel 0 - used for overcurrent event
	EVSYS->USER[EVSYS_ID_USER_TCC0_EV_0].bit.CHANNEL = 0x1; //Channel Number 0 selected
	EVSYS->USER[EVSYS_ID_USER_DMAC_CH_0].bit.CHANNEL = 0x1; //Channel Number 0 selected

	//Channel 1 - used for DMA to start the ADC when it's transfer of the result and channel are complete
	EVSYS->USER[EVSYS_ID_USER_ADC0_START].bit.CHANNEL = 0x2; //Channel Number 1 selected

	//Channel 0
	CHANNEL_tmp.bit.ONDEMAND               =   0; //Generic clock for a channel is always on
	CHANNEL_tmp.bit.RUNSTDBY               =   0; //The channel is disabled in standby sleep mode
	CHANNEL_tmp.bit.EDGSEL                 = 0x0; //NO_EVT_OUTPUT - in Async mode edge detection is not required and must be disabled by software
	CHANNEL_tmp.bit.PATH                   = 0x2; //ASYNCHRONOUS
	CHANNEL_tmp.bit.EVGEN                  = EVSYS_ID_GEN_ADC0_WINMON; //select event generator

	EVSYS->CHANNEL[0].reg                  = CHANNEL_tmp.reg;   //single 32bit write

	//Channel 1
	CHANNEL_tmp.bit.ONDEMAND               =   0; //Generic clock for a channel is always on
	CHANNEL_tmp.bit.RUNSTDBY               =   0; //The channel is disabled in standby sleep mode
	CHANNEL_tmp.bit.EDGSEL                 = 0x0; //NO_EVT_OUTPUT - in Async mode edge detection is not required and must be disabled by software
	CHANNEL_tmp.bit.PATH                   = 0x2; //ASYNCHRONOUS
	CHANNEL_tmp.bit.EVGEN                  = EVSYS_ID_GEN_DMAC_CH_2; //select event generator

	EVSYS->CHANNEL[1].reg                  = CHANNEL_tmp.reg;   //single 32bit write
}


/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
#if 0
/******************************************************************************
*  \brief DMAC Interrupt
*
*  \note
******************************************************************************/
void EVSYS_Handler(void)
{
	/*Event Detected Channel 1*/
	if(EVSYS->INTFLAG.bit.EVD1)
	{

		EVSYS->INTFLAG.reg = EVSYS_INTFLAG_EVD1; //clear flag
	}

	/*Overrun Channel 1*/
	if(EVSYS->INTFLAG.bit.OVR1)
	{

		EVSYS->INTFLAG.reg = EVSYS_INTFLAG_OVR1; //clear flag
	}
}
#endif
