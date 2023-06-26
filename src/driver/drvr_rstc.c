/*
 * drvr_rstc.c
 *
 * Created: 10/2/2018 3:52:09 PM
 *  Author: akudlacek
 */


#include "drvr_rstc.h"

#include "sam.h"


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
/*To print reset cause*/
static const char rst_cause[8][40] = {
	"RESET: POR - POWER ON RESET",      //BIT0
	"RESET: BODCORE - BROWN OUT CORE",  //BIT1
	"RESET: BODVDD - BROWN OUT VDD",    //BIT2
	"RESET: BIT3 - UNUSED",             //BIT3
	"RESET: EXT - EXTERNAL",            //BIT4
	"RESET: WDT - WATCHDOG",            //BIT5
	"RESET: SYST - SYSTEM",             //BIT6
	"RESET: BIT7 - UNUSED"              //BIT7
};


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Gets cause of last reset
*
*  \note returns pointer to const i.e. string in flash
******************************************************************************/
inline uint8_t drvr_rstc_reset_cause_get(void)
{
	uint8_t bit_index;
	uint8_t rcause_value;

	rcause_value = RSTC->RCAUSE.reg;

	/*Decode reset cause and print*/
	for(bit_index = 0; bit_index < 8; bit_index++)
	{
		if(rcause_value == (1U << bit_index))
		{
			return bit_index;
		}
	}

	return 7;
}

/******************************************************************************
*  \brief Gets cause of last reset
*
*  \note returns pointer to const i.e. string in flash
******************************************************************************/
inline const char * drvr_rstc_reset_cause_decode(uint8_t bit)
{
	bit &= 7U;

	return rst_cause[bit];
}
