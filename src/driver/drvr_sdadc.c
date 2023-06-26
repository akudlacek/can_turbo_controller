/*
 * drvr_sdadc.c
 *
 * Created: 3/1/2018 12:26:40 PM
 *  Author: akudlacek
 */ 


#include "drvr_sdadc.h"

#include "sam.h"
#include "drvr_port.h"


/******************************************************************************
*  Defines
******************************************************************************/
typedef enum sdadc_state_t
{
	EMPTY,
	ARMED,
	BUSY,
	READY
} sdadc_state_t;


/******************************************************************************
* Variables
******************************************************************************/
static volatile struct
{
	sdadc_state_t state;
	int32_t count;
	int32_t sum;
	
	uint8_t winmon_exceed_thresh_flag;
} sdadc;


/******************************************************************************
* Local Prototypes
******************************************************************************/


/******************************************************************************
*  \brief SDADC init
*
*  \note
******************************************************************************/
void drvr_sdadc_init(const int32_t win_mon_lo_thres)
{
	SDADC_CTRLA_Type       CTRLA_tmp     = {0};
	SDADC_INPUTCTRL_Type   INPUTCTRL_tmp = {0};
	SDADC_CTRLC_Type       CTRLC_tmp     = {0};
	SDADC_WINCTRL_Type     WINCTRL_tmp   = {0};
	SDADC_WINLT_Type       WINLT_tmp     = {0};
	SDADC_GAINCORR_Type    GAINCORR_tmp  = {0};
	
	/*Init pins*/
	drvr_port_pin_cfg(CURRENT_SENSE_POS, INPUT, NORMAL, PULL_DIS, IN_BUF_EN, B, LOW);
	drvr_port_pin_cfg(CURRENT_SENSE_NEG, INPUT, NORMAL, PULL_DIS, IN_BUF_EN, B, LOW);
	
	/*Configure SDADC*/
	/*NOTE: Using Generic Clock Generator 0*/
	/*Data out rate = (((GCLK_SDADC / PRESCALER) / 4) / OSR) ??*/
	
	//CTRLA
	CTRLA_tmp.bit.ENABLE          =    1; //The SDADC is enabled.
	
	//REFCTRL
	SDADC->REFCTRL.bit.REFRANGE   =  0x3; //Vref > 3.6V
	SDADC->REFCTRL.bit.REFSEL     =  0x3; //VDDANA Supply 2.7-5.5V
	
	//CTRLB
	SDADC->CTRLB.bit.SKPCNT       =    2; //The first valid sample starts from the third sample onward.
	SDADC->CTRLB.bit.OSR          =  0x0; //OSR64
	SDADC->CTRLB.bit.PRESCALER    =    0; //CLK_GEN_SDADC/2
	
	//EVCTRL
	SDADC->EVCTRL.bit.WINMONEO    =    1; //Window Monitor event output is enabled and an event will be generated.
	
	//INTENSET
	SDADC->INTENSET.bit.RESRDY    =    0; //The Result Ready interrupt is disabled.
	SDADC->INTENSET.bit.WINMON    =    1; //The Window Monitor interrupt is enabled.
	
	//INPUTCTRL
	INPUTCTRL_tmp.bit.MUXSEL      = 0x01; //AIN1 Select ADC AINN1 and AINP1 pins
	
	//CTRLC
	CTRLC_tmp.bit.FREERUN         =    1; //The SDADC is in free running mode
	
	//WINCTRL
	WINCTRL_tmp.bit.WINMODE       =  0x1; //ABOVE RESULT > WINLT
	
	//WINLT
	WINLT_tmp.bit.WINLT           = (win_mon_lo_thres & 0xFFFFFF); //Window Monitor Lower Threshold converted to 24bit value
	
	//GAINCORR
	GAINCORR_tmp.bit.GAINCORR     =    1; //Set gain correction to 1 to fix silicon REV B errata (also present in REV D).
	
	/*Set write synchronized registers*/
	SDADC->INPUTCTRL.reg          = INPUTCTRL_tmp.reg;
	SDADC->CTRLC.reg              = CTRLC_tmp.reg;
	SDADC->WINCTRL.reg            = WINCTRL_tmp.reg;
	SDADC->WINLT.reg              = WINLT_tmp.reg;
	SDADC->GAINCORR.reg           = GAINCORR_tmp.reg;
	SDADC->CTRLA.reg              = CTRLA_tmp.reg;
	
}

/******************************************************************************
*  \brief SDADC get average result from one period
*
*  \note Only for current sense
******************************************************************************/
drvr_sdadc_return_t drvr_sdadc_get(int32_t * const result)
{
	drvr_sdadc_return_t return_flag = SDADC_BUSY;
	
	switch(sdadc.state)
	{
		case EMPTY:
		//Enter ARMED state
		sdadc.state = ARMED;
		break;
		
		case ARMED:
		//Do nothing
		break;
		
		case BUSY:
		//Do nothing
		break;
		
		case READY:
		//Get result and enter EMPTY state
		*result = sdadc.sum / sdadc.count;
		sdadc.sum = 0;
		sdadc.count = 0;
		return_flag = SDADC_SUCCESS;
		sdadc.state = EMPTY;
		break;
	}
	
	return return_flag;
}

/******************************************************************************
*  \brief Check Window monitor flag
*
*  \note
******************************************************************************/
uint8_t drvr_sdadc_winmon(void)
{
	return sdadc.winmon_exceed_thresh_flag;
}

/******************************************************************************
*  \brief SDADC Pulse Task
*
*  \note
******************************************************************************/
inline void drvr_sdadc_pulse_task(void)
{
	switch(sdadc.state)
	{
		case EMPTY:
		//Do nothing
		break;
		
		case ARMED:
		//Enable RESRDY interrupts and enter BUSY state
		SDADC->INTENSET.reg = SDADC_INTENSET_RESRDY;
		sdadc.state = BUSY;
		break;
		
		case BUSY:
		//Disable RESRDY interrupts and enter READY state
		SDADC->INTENCLR.reg = SDADC_INTENCLR_RESRDY;
		sdadc.state = READY;
		break;
		
		case READY:
		//Do nothing
		break;
	}
}


/**************************************************************************************************
*                                       LOCAL FUNCTIONS
**************************************************************************************************/


/**************************************************************************************************
*                                           HANDLERS
**************************************************************************************************/
/******************************************************************************
*  \brief Interrupt for SDADC
*
*  \note
******************************************************************************/
void SDADC_Handler(void)
{
	/*Window Monitor interrupt flag*/
	if(SDADC->INTFLAG.bit.WINMON)
	{
		sdadc.winmon_exceed_thresh_flag = 1;
		
		SDADC->INTFLAG.reg = SDADC_INTFLAG_WINMON; //clear flag
	}
	
	/*Result Ready interrupt flag*/
	if(SDADC->INTFLAG.bit.RESRDY)
	{
		/*Accumulate samples for average*/
		sdadc.sum += (((int32_t)(SDADC->RESULT.bit.RESULT << 8)) >> 8);
		sdadc.count++;
		
		SDADC->INTFLAG.reg = SDADC_INTFLAG_RESRDY; //clear flag
	}
}
