/*
 * drvr_wdt.c
 *
 * Created: 10/8/2018 9:23:11 AM
 *  Author: akudlacek
 */ 


#include "drvr_wdt.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define WDT_SYNC_WAIT(syncbusy_flags) while((WDT->SYNCBUSY.reg & syncbusy_flags) != 0)


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief start wdt
*
*  \note 
******************************************************************************/
void drvr_wdt_start(void)
{
	WDT->CONFIG.bit.PER = WDT_CONFIG_PER_CYC1024; //approx 1s
	WDT->CTRLA.bit.ENABLE = 1;
	WDT_SYNC_WAIT(WDT_SYNCBUSY_ENABLE);
}

/******************************************************************************
*  \brief start wdt
*
*  \note
******************************************************************************/
inline void drvr_wdt_clear(void)
{
	if(WDT->SYNCBUSY.bit.CLEAR == 0)
	{
		WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
	}
	
	//Cannot clear the WDT without syncing but sync takes an excessive amount of time
	//I believe this is actually syncing at 1.024kHz so you would essentially have to
	//slow the main loop to this speed in order to use the sync wait
	//WDT_SYNC_WAIT(WDT_SYNCBUSY_CLEAR);
}
