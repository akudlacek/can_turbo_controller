/*
 * drvr_tsens.c
 *
 * Created: 2/28/2018 9:56:19 AM
 *  Author: akudlacek
 */ 


#include "drvr_tsens.h"

#include "sam.h"
#include "drvr_nvmctrl.h"
#include "drvr_clock.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define FILTER_VALUE 20

//TSENS waits
#define TSENS_SYNC_WAIT(syncbusy_flags) while((TSENS->SYNCBUSY.reg & syncbusy_flags) != 0)


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
static volatile int32_t filtered_result = 0;


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief TSENS init
*
*  \note
******************************************************************************/
void drvr_tsens_init(void)
{
	float tsen_clk;
	char rev = 'A';
	
	/*Get chip revision*/
	rev += DSU->DID.bit.REVISION;
	
	/*Rev B has weird problems, one of which is inverted temp ERRATA_14476. Also it throws overflow flags with corrected gain value so just don't use if rev B*/
	if(rev != 'B')
	{
		TSENS->CTRLA.bit.ENABLE = 0; //Disable
		TSENS_SYNC_WAIT(TSENS_SYNCBUSY_ENABLE);
		
		/*Get clock generator 1 frequency*/
		tsen_clk = (float)drvr_clock_get_gen1_freq_hz();
		
		/*Wait for NVM to be ready*/
		while(nvm_is_ready() == 0);
		
		/*Configure TSENS*/
		//GAIN
		TSENS->GAIN.bit.GAIN        = (uint32_t)((float)NVM_TEMP_CALIB->bit.TSENS_GAIN * (tsen_clk / 48000000.0)); //corrected gain for frequency
		
		//OFFSET
		TSENS->OFFSET.bit.OFFSETC   = NVM_TEMP_CALIB->bit.TSENS_OFFSET;
		
		//CAL
		TSENS->CAL.bit.FCAL         = NVM_TEMP_CALIB->bit.TSENS_FCAL;
		TSENS->CAL.bit.TCAL         = NVM_TEMP_CALIB->bit.TSENS_TCAL;
		
		//CTRLC
		TSENS->CTRLC.bit.FREERUN    = 0; //TSENS is NOT in free running mode
		
		/*Enable TSENS*/
		TSENS->CTRLA.bit.ENABLE     = 1;
		TSENS_SYNC_WAIT(TSENS_SYNCBUSY_ENABLE);
		
		/*Start measurement*/
		TSENS->CTRLB.bit.START      = 1;
		
		/*Wait for fist measurement to complete*/
		while(TSENS->INTFLAG.bit.RESRDY != 1);
		
		/*Initialize the filtered result*/
		filtered_result = (((int32_t)(TSENS->VALUE.bit.VALUE << 8)) >> 8);
		
		//INTENSET
		TSENS->INTENSET.bit.RESRDY  = 1; //The Result Ready interrupt is enabled.
	}
}

/******************************************************************************
*  \brief TSENS get results
*
*  \note
******************************************************************************/
inline int32_t drvr_tsens_get(void)
{
	int32_t temp_dec_c;
	
	/*Copy result*/
	temp_dec_c = filtered_result;
	
	/*Start measurement*/
	TSENS->CTRLB.bit.START      = 1;
	
	return temp_dec_c;
}


/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
/******************************************************************************
*  \brief interrupt for TSENS
*
*  \note CRITICAL RESOURCES:
*        NEEDS PROTECTION
*            
*        DOES NOT NEED PROTECTION:
*            filtered_result
******************************************************************************/
void TSENS_Handler(void)
{
	/*Result Ready interrupt flag*/
	if(TSENS->INTFLAG.bit.RESRDY)
	{
		/*Filter Result*/
		filtered_result -= filtered_result / FILTER_VALUE;
		filtered_result += (((int32_t)(TSENS->VALUE.bit.VALUE << 8)) >> 8) / FILTER_VALUE;
		
		TSENS->INTFLAG.reg = TSENS_INTFLAG_RESRDY; //clear flag
	}
}
