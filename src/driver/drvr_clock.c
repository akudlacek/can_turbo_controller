/*
 * drvr_clock.c
 *
 * Created: 11/13/2017 11:53:47 AM
 *  Author: akudlacek
 */


#include "drvr_clock.h"

#include "sam.h"
#include "drvr_port.h"
#include "drvr_nvmctrl.h"


/*\note By default OSC48M is divided by 12 for 4MHz*/


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define OSC48M_STARTUP_FREQ_HZ 4000000UL

//OSCCTRL waits
#define OSCCTRL_OSC48M_SYNC_WAIT(syncbusy_flags) while((OSCCTRL->OSC48MSYNCBUSY.reg & syncbusy_flags) != 0)
#define OSCCTRL_DPLL_SYNC_WAIT(syncbusy_flags) while((OSCCTRL->DPLLSYNCBUSY.reg & syncbusy_flags) != 0)

//GCLK waits
#define GCLK_SYNC_WAIT(syncbusy_flags) while((GCLK->SYNCBUSY.reg & syncbusy_flags) != 0)

/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
/*Generic Clock Generator*/
static volatile uint32_t gclk0_freq_hz = OSC48M_STARTUP_FREQ_HZ;
static volatile uint32_t gclk1_freq_hz = 0UL;

/*****GLOBAL******/
const volatile uint32_t * const g_cpu_freq_hz_ptr = &gclk0_freq_hz;

/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Init clocks
*
*  \note GCLK GEN 0 drives the CPU
*        GCLK GEN 1 drives all other peripherals
*        A check for REV level is done to prevent the processor from using the
*        48MHz if REV B is detected. There is a errata for this. The GCLK gen 0
*        is only used for the CPU. Everything else is using GCLK 1 so changing
*        main clock should not effect peripherals.
******************************************************************************/
void drvr_clock_init(void)
{
	GCLK_GENCTRL_Type GENCTRL_tmp = {0};
	char rev = 'A';
	uint8_t gclk1_div = 1;

	/*Get chip revision*/
	rev += DSU->DID.bit.REVISION;

	/*****************************OSCCTRL*****************************/
	//Set OSC48M to 48MHz
	if(rev != 'B')
	{
		//CAL48M is only available for Rev D silicon.
		if(rev == 'D')
		{
			/*Wait for NVM to be ready*/
			while(nvm_is_ready() == 0);
			OSCCTRL->CAL48M.reg = NVM_SW_CALIB->bit.CAL48M_5V;
		}

		gclk0_freq_hz = 48000000UL;

		drvr_nvmctrl_init(5000, gclk0_freq_hz);           //set number of read wait states

		OSCCTRL->OSC48MSTUP.bit.STARTUP     =    0x7;     //21.333us - max start up delay
		OSCCTRL->OSC48MDIV.bit.DIV          = 0b0000;     //48MHz
		OSCCTRL_OSC48M_SYNC_WAIT(OSCCTRL_OSC48MSYNCBUSY_OSC48MDIV);

		gclk1_div = 12; //48MHz / 12 = 4MHz
	}
	//Leave default 4MHz
	else
	{
		gclk0_freq_hz = 4000000UL;

		drvr_nvmctrl_init(5000, gclk0_freq_hz);           //set number of read wait states

		gclk1_div = 1;
	}

	/*****************************GCLK*****************************/
	/*configure and enable generator 0 (Main Clock) for 48 or 4MHz depending on OSC48M configuration*/
	GENCTRL_tmp.bit.DIV      =    1; //divide by 1
	GENCTRL_tmp.bit.RUNSTDBY =    0; //Generator is stopped in Standby
	GENCTRL_tmp.bit.DIVSEL   =    0; //The Generator clock frequency equals the clock source frequency divided by GENCTRLn.DIV.
	GENCTRL_tmp.bit.OE       =    0; //Generator clock signal on pin GCLK_IO.
	GENCTRL_tmp.bit.OOV      =    0; //The GCLK_IO will be LOW when generator is turned off or when the OE bit is zero.
	GENCTRL_tmp.bit.IDC      =    0; //Generator output clock duty cycle is not balanced to 50/50 for odd division factors. DIV=1 is no division according to datasheet
	GENCTRL_tmp.bit.GENEN    =    1; //Generator is enabled
	GENCTRL_tmp.bit.SRC      = 0x06; //OSC48M oscillator output

	GCLK->GENCTRL[0].reg = GENCTRL_tmp.reg;
	GCLK_SYNC_WAIT(GCLK_SYNCBUSY_GENCTRL0);

	/*configure and enable generator 1 for 4MHz*/
	GENCTRL_tmp.bit.DIV      =    gclk1_div; //divide by 12 for 48MHz OSC48M or 1 for 4MHz OSC48M
	GENCTRL_tmp.bit.RUNSTDBY =            0; //Generator is stopped in Standby
	GENCTRL_tmp.bit.DIVSEL   =            0; //The Generator clock frequency equals the clock source frequency divided by GENCTRLn.DIV.
	GENCTRL_tmp.bit.OE       =            0; //No Generator clock signal on pin GCLK_IO.
	GENCTRL_tmp.bit.OOV      =            0; //The GCLK_IO will be LOW when generator is turned off or when the OE bit is zero.
	GENCTRL_tmp.bit.IDC      =            0; //Generator output clock duty cycle is not balanced to 50/50 for odd division factors.
	GENCTRL_tmp.bit.GENEN    =            1; //Generator is enabled
	GENCTRL_tmp.bit.SRC      =         0x06; //OSC48M oscillator output

	GCLK->GENCTRL[1].reg = GENCTRL_tmp.reg;
	GCLK_SYNC_WAIT(GCLK_SYNCBUSY_GENCTRL1);
	gclk1_freq_hz = gclk0_freq_hz / gclk1_div;

	/*Configure SERCOM4_CORE clock*/
	GCLK->PCHCTRL[SERCOM4_GCLK_ID_CORE].bit.WRTLOCK = 0;   //The Peripheral Channel register and the associated Generator register are not locked
	GCLK->PCHCTRL[SERCOM4_GCLK_ID_CORE].bit.GEN     = 0x1; //Generic Clock Generator 1
	GCLK->PCHCTRL[SERCOM4_GCLK_ID_CORE].bit.CHEN    = 1;   //The Peripheral Channel is enabled

	/*Configure EIC clock*/ //not used now
	//GCLK->PCHCTRL[EIC_GCLK_ID].bit.WRTLOCK = 0;   //The Peripheral Channel register and the associated Generator register are not locked
	//GCLK->PCHCTRL[EIC_GCLK_ID].bit.GEN     = 0x1; //Generic Clock Generator 1
	//GCLK->PCHCTRL[EIC_GCLK_ID].bit.CHEN    = 1;   //The Peripheral Channel is enabled

	/*Configure TCC0 clock*/
	GCLK->PCHCTRL[TCC0_GCLK_ID].bit.WRTLOCK = 0;   //The Peripheral Channel register and the associated Generator register are not locked
	GCLK->PCHCTRL[TCC0_GCLK_ID].bit.GEN     = 0x1; //Generic Clock Generator 1
	GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN    = 1;   //The Peripheral Channel is enabled

	/*Configure ADC0 clock*/
	GCLK->PCHCTRL[ADC0_GCLK_ID].bit.WRTLOCK = 0;   //The Peripheral Channel register and the associated Generator register are not locked
	GCLK->PCHCTRL[ADC0_GCLK_ID].bit.GEN     = 0x1; //Generic Clock Generator 1
	GCLK->PCHCTRL[ADC0_GCLK_ID].bit.CHEN    = 1;   //The Peripheral Channel is enabled

	/*Configure ADC1 clock*/
	GCLK->PCHCTRL[ADC1_GCLK_ID].bit.WRTLOCK = 0;   //The Peripheral Channel register and the associated Generator register are not locked
	GCLK->PCHCTRL[ADC1_GCLK_ID].bit.GEN     = 0x1; //Generic Clock Generator 1
	GCLK->PCHCTRL[ADC1_GCLK_ID].bit.CHEN    = 1;   //The Peripheral Channel is enabled

	/*Configure EVSYS CHANNEL 0 clock - When async mode GCLK for this channel is not mandatory*/
	//GCLK->PCHCTRL[EVSYS_GCLK_ID_0].bit.WRTLOCK = 0;   //The Peripheral Channel register and the associated Generator register are not locked
	//GCLK->PCHCTRL[EVSYS_GCLK_ID_0].bit.GEN     = 0x1; //Generic Clock Generator 1
	//GCLK->PCHCTRL[EVSYS_GCLK_ID_0].bit.CHEN    = 1;   //The Peripheral Channel is enabled

	/*Configure TSENS clock*/
	GCLK->PCHCTRL[TSENS_GCLK_ID].bit.WRTLOCK = 0;   //The Peripheral Channel register and the associated Generator register are not locked
	GCLK->PCHCTRL[TSENS_GCLK_ID].bit.GEN     = 0x1; //Generic Clock Generator 1
	GCLK->PCHCTRL[TSENS_GCLK_ID].bit.CHEN    = 1;   //The Peripheral Channel is enabled

	/*Configure SDADC clock*/
	GCLK->PCHCTRL[SDADC_GCLK_ID].bit.WRTLOCK = 0;   //The Peripheral Channel register and the associated Generator register are not locked
	GCLK->PCHCTRL[SDADC_GCLK_ID].bit.GEN     = 0x0; //Generic Clock Generator 0
	GCLK->PCHCTRL[SDADC_GCLK_ID].bit.CHEN    = 1;   //The Peripheral Channel is enabled

	/*****************************MCLK*****************************/
	//Some peripheral clock busses are on by default so they are not listed here

	/*enable SERCOM4 clock buss*/
	MCLK->APBCMASK.bit.SERCOM4_ = 1;

	/*enable TCC0 clock buss*/
	MCLK->APBCMASK.bit.TCC0_   = 1;

	/*enable ADC0 & ADC1 clock buss*/
	MCLK->APBCMASK.bit.ADC0_   = 1;
	MCLK->APBCMASK.bit.ADC1_   = 1;

	/*enable EVSYS clock buss*/
	MCLK->APBCMASK.bit.EVSYS_  = 1;

	/*enable TSENS clock buss*/
	MCLK->APBAMASK.bit.TSENS_  = 1;

	/*enable SDADC clock buss*/
	MCLK->APBCMASK.bit.SDADC_  = 1;
}

/******************************************************************************
*  \brief Get GCLK GEN 0 frequency
*
*  \note
******************************************************************************/
inline uint32_t drvr_clock_get_gen0_freq_hz(void)
{
	return gclk0_freq_hz;
}

/******************************************************************************
*  \brief Get GCLK GEN 1 frequency
*
*  \note
******************************************************************************/
inline uint32_t drvr_clock_get_gen1_freq_hz(void)
{
	return gclk1_freq_hz;
}
