/*
* drvr_tcc.c
*
* Created: 8/18/2017 1:34:30 PM
*  Author: akudlacek
*/


#include "drvr_tcc.h"

#include "sam.h"
#include "drvr_port.h"
#include "drvr_clock.h"
#include "drvr_adc.h"

#include "misc.h"
#include "output.h"
#include "erp4.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
/*TCC0 (HSS_PWM) 24-BIT*/

/* DSPWM
 * With clk at 4MHz, prescalar at 1 the absolute max and min for this timer are
 * ABSOLUTE MIN (TOP = 0xFFFFFF): 0.1192Hz
 * ABSOLUTE MAX (TOP = 1): 2000000Hz
 *
 * USABLE MAX (TOP = 10): 200000Hz
 */
#define TCC0_MAX_FREQ_HZ             200000.0 //Usable Max
#define TCC0_MIN_FREQ_HZ                 0.12 //Rounded Absolute Min
#define TCC0_INIT_DUTY_PCT                0.0

#define TCC0_PRESCALAR 1
#define TCC0_SYNC_WAIT(syncbusy_flags) while((TCC0->SYNCBUSY.reg & syncbusy_flags) != 0)

extern void assert_failed(char const *file, int line);


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
/****LOCAL****/
static tcc0_data_t tcc0_data;

/****GLOBAL****/
const tcc0_data_t * const tcc0_data_ptr = &tcc0_data;


/**************************************************************************************************
*                                         LOCAL PROTOTYPES
*************************************************^************************************************/
static inline uint32_t gen_top_val(const float desired_freq_hz, const uint32_t tcc_clk_src_freq_hz, const uint32_t tcc_prescaler);
static inline uint32_t gen_cc_val(const float desired_duty_pct, const uint32_t current_tcc_top);


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Initialize TCC peripheral
*
*  \note GCLK_TCC = 4MHz
*        TCC0 is 24 bit (found in Table 7-7. TCC Configuration Summary)
*        CC0 -> HSS_PWM_3
*        CC1 -> HSS_PWM_4
*        CC2 -> HSS_PWM_1
*        CC3 -> HSS_PWM_2
******************************************************************************/
void drvr_tcc0_init(const float freq_hz)
{
	TCC_CTRLA_Type       CTRLA_tmp = {0};
	TCC_CTRLBSET_Type CTRLBSET_tmp = {0};
	TCC_WAVE_Type         WAVE_tmp = {0};

	/*Init pins*/
	drvr_port_pin_cfg(HSS_PWM_1, OUTPUT, NORMAL, PULL_DIS, IN_BUF_DIS, F, LOW);
	drvr_port_pin_cfg(HSS_PWM_2, OUTPUT, NORMAL, PULL_DIS, IN_BUF_DIS, F, LOW);
	drvr_port_pin_cfg(HSS_PWM_3, OUTPUT, NORMAL, PULL_DIS, IN_BUF_DIS, F, LOW);
	drvr_port_pin_cfg(HSS_PWM_4, OUTPUT, NORMAL, PULL_DIS, IN_BUF_DIS, F, LOW);

	/*Disable TCC0*/
	TCC0->CTRLA.bit.ENABLE = 0;       //The peripheral is disabled.

	/*wait for enable to be written*/
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_ENABLE);

	/*Configure TCC0*/
	//CTRLA
	CTRLA_tmp.bit.ENABLE    =   1;  //enable tcc
	CTRLA_tmp.bit.PRESCALER = 0x0;  //Prescaler: GCLK_TCC/1
	CTRLA_tmp.bit.PRESCSYNC = 0x0;  //Reload or reset Counter on next GCLK

	//CTRLBSET
	CTRLBSET_tmp.bit.DIR    =   0;  //The timer/counter is counting up (incrementing).

	//DRVCTRL
	TCC0->DRVCTRL.bit.NRV0  =   0; //Non-Recoverable State 0 Output Value low
	TCC0->DRVCTRL.bit.NRE0  =   1; //Non-recoverable faults set the output to NRV0 level.
	//Waveform Output x Inversion
	TCC0->DRVCTRL.bit.INVEN4 = 0;
	TCC0->DRVCTRL.bit.INVEN5 = 0;
	TCC0->DRVCTRL.bit.INVEN6 = 0;
	TCC0->DRVCTRL.bit.INVEN7 = 0;

	//DBGCTRL
	TCC0->DBGCTRL.bit.DBGRUN = 0; //The TCC is halted when the device is halted in debug mode

	//EVCTRL
	TCC0->EVCTRL.bit.CNTEO  =   0; //Counter cycle output event is disabled and will not be generated
	TCC0->EVCTRL.bit.CNTSEL = 0x2; //BETWEEN
	TCC0->EVCTRL.bit.TCEI0  =   1; //Timer/Counter Event Input 0 Enable
	TCC0->EVCTRL.bit.EVACT0 = 0x7; //FAULT Non-recoverable Fault

	//INTENSET
	TCC0->INTENSET.bit.ERR  =   0; //Error Interrupt Enable
	TCC0->INTENSET.bit.CNT  =   0; //Counter Interrupt Enable
	TCC0->INTENSET.bit.OVF  =   1; //Overflow Interrupt Enable

	//WAVE
	//Compare output is set to DIR when TCC counter matches CCxvalue
	WAVE_tmp.bit.POL0       =   1;
	WAVE_tmp.bit.POL1       =   1;
	WAVE_tmp.bit.POL2       =   1;
	WAVE_tmp.bit.POL3       =   1;
	WAVE_tmp.bit.WAVEGEN    = 0x7;  //DSTOP, Dual-slope PWM

	//calculate value for PER & CC
	tcc0_data.top_val            = gen_top_val(freq_hz, drvr_clock_get_gen1_freq_hz(), TCC0_PRESCALAR);
	tcc0_data.cc_val[0]          = gen_cc_val(TCC0_INIT_DUTY_PCT, tcc0_data.top_val);
	tcc0_data.cc_val[1]          = gen_cc_val(TCC0_INIT_DUTY_PCT, tcc0_data.top_val);
	tcc0_data.cc_val[2]          = gen_cc_val(TCC0_INIT_DUTY_PCT, tcc0_data.top_val);
	tcc0_data.cc_val[3]          = gen_cc_val(TCC0_INIT_DUTY_PCT, tcc0_data.top_val);

	/*Set Write-Synchronized registers*/
	TCC0->CTRLBSET.reg      = CTRLBSET_tmp.reg;
	TCC0->WAVE.reg          = WAVE_tmp.reg;
	TCC0->PER.bit.PER       = tcc0_data.top_val;
	TCC0->CC[0].bit.CC      = tcc0_data.cc_val[0];
	TCC0->CC[1].bit.CC      = tcc0_data.cc_val[1];
	TCC0->CC[2].bit.CC      = tcc0_data.cc_val[2];
	TCC0->CC[3].bit.CC      = tcc0_data.cc_val[3];
	TCC0->CTRLA.reg         = CTRLA_tmp.reg; //enables TCC0

	TCC0_SYNC_WAIT(TCC_SYNCBUSY_MASK);

	/*record init duty cycle set above*/
	tcc0_data.duty_pct[0]       = TCC0_INIT_DUTY_PCT;
	tcc0_data.duty_pct[1]       = TCC0_INIT_DUTY_PCT;
	tcc0_data.duty_pct[2]       = TCC0_INIT_DUTY_PCT;
	tcc0_data.duty_pct[3]       = TCC0_INIT_DUTY_PCT;
}

/******************************************************************************
*  \brief SET DUTY (double buffered)
*
*  \note Set duty cycle in percent for TCC0
*        uses double buffering
*  \warning This will overwrite any pending duty values
******************************************************************************/
void drvr_tcc0_set_duty(float duty_pct, const uint8_t cc_channel)
{
	//if channel is invalid.
	if(cc_channel >= 4)
	{
		assert_failed(__FILE__, __LINE__);
		return;
	}

	/*Constrain duty_pct 100 to 0*/
	if(duty_pct > 100.0) duty_pct = 100.0;
	if(duty_pct < 0.0) duty_pct = 0.0;

	/*get new cc value*/
	tcc0_data.cc_val[cc_channel] = gen_cc_val(duty_pct, tcc0_data.top_val);

	TCC0_SYNC_WAIT(TCC_SYNCBUSY_CC(1U << cc_channel));

	/*if new value is the same the no change is needed so return and change nothing*/
	if(tcc0_data.cc_val[cc_channel] == TCC0->CC[cc_channel].bit.CC)
	{
		return;
	}

	/*lock buffer registers*/
	TCC0->CTRLBSET.bit.LUPD = 1;
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_CTRLB);

	/*Clear buffer valid flag, in case there is still a pending buffer waiting for update*/
	/*The reason the flags are cleared twice is due to a problem stated in the errata, I
	  did not see a problem here in this function but this was done more a precaution*/
	/*Clear only channel being changed*/
	TCC0->STATUS.reg = TCC_STATUS_CCBUFV(1U << cc_channel);
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_STATUS);
	TCC0->STATUS.reg = TCC_STATUS_CCBUFV(1U << cc_channel);
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_STATUS);

	/*load new cc value to buffer*/
	TCC0->CCBUF[cc_channel].bit.CCBUF = tcc0_data.cc_val[cc_channel];

	//no syncbusy bit, CCBUFx is copied into CCx at TCC update time

	/*wait for data in cc buffer to become valid*/
	while( !(TCC0->STATUS.reg & TCC_STATUS_CCBUFV(1U << cc_channel)) );

	/*unlock buffer registers*/
	TCC0->CTRLBCLR.reg = TCC_CTRLBCLR_LUPD; //LUPD
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_CTRLB);

	/*record duty cycle set to*/
	tcc0_data.duty_pct[cc_channel] = duty_pct;
}

/******************************************************************************
*  \brief SET ALL DUTY (double buffered)
*
*  \note Set duty cycle in percent for TCC0
*        uses double buffering
*  \warning This will overwrite any pending duty values
******************************************************************************/
void drvr_tcc0_set_all_duty(float duty_pct_hss_1, float duty_pct_hss_2, float duty_pct_hss_3, float duty_pct_hss_4)
{
	/*Constrain duty_pct 100 to 0*/
	if(duty_pct_hss_1 > 100.0) duty_pct_hss_1 = 100.0;
	if(duty_pct_hss_1 < 0.0) duty_pct_hss_1 = 0.0;
	if(duty_pct_hss_2 > 100.0) duty_pct_hss_2 = 100.0;
	if(duty_pct_hss_2 < 0.0) duty_pct_hss_2 = 0.0;
	if(duty_pct_hss_3 > 100.0) duty_pct_hss_3 = 100.0;
	if(duty_pct_hss_3 < 0.0) duty_pct_hss_3 = 0.0;
	if(duty_pct_hss_4 > 100.0) duty_pct_hss_4 = 100.0;
	if(duty_pct_hss_4 < 0.0) duty_pct_hss_4 = 0.0;

	/*get new cc value for each HSS channel*/
	tcc0_data.cc_val[0]          = gen_cc_val(duty_pct_hss_3, tcc0_data.top_val);
	tcc0_data.cc_val[1]          = gen_cc_val(duty_pct_hss_4, tcc0_data.top_val);
	tcc0_data.cc_val[2]          = gen_cc_val(duty_pct_hss_1, tcc0_data.top_val);
	tcc0_data.cc_val[3]          = gen_cc_val(duty_pct_hss_2, tcc0_data.top_val);

	/*lock buffer registers*/
	TCC0->CTRLBSET.bit.LUPD = 1;
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_CTRLB);

	/*Clear buffer valid flag, in case there is still a pending buffer waiting for update*/
	/*The reason the flags are cleared twice is due to a problem stated in the errata, I
	  did not see a problem here in this function but this was done more a precaution*/
	/*Clearing all of them at the same time*/
	TCC0->STATUS.reg = TCC_STATUS_CCBUFV_Msk;
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_STATUS);
	TCC0->STATUS.reg = TCC_STATUS_CCBUFV_Msk;
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_STATUS);

	/*load new cc value to buffer*/
	TCC0->CCBUF[0].bit.CCBUF = tcc0_data.cc_val[0];
	TCC0->CCBUF[1].bit.CCBUF = tcc0_data.cc_val[1];
	TCC0->CCBUF[2].bit.CCBUF = tcc0_data.cc_val[2];
	TCC0->CCBUF[3].bit.CCBUF = tcc0_data.cc_val[3];

	//no syncbusy bit, CCBUFx is copied into CCx at TCC update time

	/*wait for data in cc buffer to become valid*/
	while( !(TCC0->STATUS.reg & TCC_STATUS_CCBUFV_Msk) );

	/*unlock buffer registers*/
	TCC0->CTRLBCLR.reg = TCC_CTRLBCLR_LUPD; //LUPD
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_CTRLB);

	/*record duty cycle set to*/
	tcc0_data.duty_pct[0] = duty_pct_hss_3;
	tcc0_data.duty_pct[1] = duty_pct_hss_4;
	tcc0_data.duty_pct[2] = duty_pct_hss_1;
	tcc0_data.duty_pct[3] = duty_pct_hss_2;
}

/******************************************************************************
*  \brief SET DUTY wrapper for HSS 1
*
*  \note
*  \warning This will overwrite any pending duty values
******************************************************************************/
inline void drvr_tcc0_set_duty_hss_1(float duty_pct)
{
	drvr_tcc0_set_duty(duty_pct, 2);
}

/******************************************************************************
*  \brief SET DUTY wrapper for HSS 2
*
*  \note
*  \warning This will overwrite any pending duty values
******************************************************************************/
inline void drvr_tcc0_set_duty_hss_2(float duty_pct)
{
	drvr_tcc0_set_duty(duty_pct, 3);
}

/******************************************************************************
*  \brief SET DUTY wrapper for HSS 3
*
*  \note
*  \warning This will overwrite any pending duty values
******************************************************************************/
inline void drvr_tcc0_set_duty_hss_3(float duty_pct)
{
	drvr_tcc0_set_duty(duty_pct, 0);
}

/******************************************************************************
*  \brief SET DUTY wrapper for HSS 4
*
*  \note
*  \warning This will overwrite any pending duty values
******************************************************************************/
inline void drvr_tcc0_set_duty_hss_4(float duty_pct)
{
	drvr_tcc0_set_duty(duty_pct, 1);
}

/******************************************************************************
*  \brief SET FREQUENCY
*
*  \note Set frequency in Hz for TCC0
*        uses double buffering
*  \warning This will overwrite any pending duty or freq values
******************************************************************************/
void drvr_tcc0_set_freq_hz(float freq_hz)
{
	/*Constrain freq_hz*/
	if(freq_hz > TCC0_MAX_FREQ_HZ) freq_hz = TCC0_MAX_FREQ_HZ;
	if(freq_hz < TCC0_MIN_FREQ_HZ) freq_hz = TCC0_MIN_FREQ_HZ;

	/*get per value*/
	tcc0_data.top_val = gen_top_val(freq_hz, drvr_clock_get_gen1_freq_hz(), TCC0_PRESCALAR);

	TCC0_SYNC_WAIT(TCC_SYNCBUSY_PER);

	/*if new value is the same the no change is needed so return and change nothing*/
	if(tcc0_data.top_val == TCC0->PER.bit.PER)
	{
		return;
	}

	/*lock buffer registers*/
	TCC0->CTRLBSET.bit.LUPD = 1;
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_CTRLB);

	/*Clear buffer valid flags, in case there is still a pending buffer waiting for update*/
	/*This was done because calling this function twice within a short period would not update freq*/
	/*The reason the flags are cleared twice is due to a problem stated in the errata, it would
	  a hang up in TCC0_SYNC_WAIT(TCC_SYNCBUSY_STATUS); after setting the cc and per buffers*/
	/*Clearing all of them at the same time*/
	TCC0->STATUS.reg = (TCC_STATUS_CCBUFV_Msk | TCC_STATUS_PERBUFV);
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_STATUS);
	TCC0->STATUS.reg = (TCC_STATUS_CCBUFV_Msk | TCC_STATUS_PERBUFV);
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_STATUS);

	/*get cc value from new top val for all channels*/
	tcc0_data.cc_val[0] = gen_cc_val(tcc0_data.duty_pct[0], tcc0_data.top_val);
	tcc0_data.cc_val[1] = gen_cc_val(tcc0_data.duty_pct[1], tcc0_data.top_val);
	tcc0_data.cc_val[2] = gen_cc_val(tcc0_data.duty_pct[2], tcc0_data.top_val);
	tcc0_data.cc_val[3] = gen_cc_val(tcc0_data.duty_pct[3], tcc0_data.top_val);

	/*update buffers*/
	TCC0->CCBUF[0].bit.CCBUF = tcc0_data.cc_val[0];
	TCC0->CCBUF[1].bit.CCBUF = tcc0_data.cc_val[1];
	TCC0->CCBUF[2].bit.CCBUF = tcc0_data.cc_val[2];
	TCC0->CCBUF[3].bit.CCBUF = tcc0_data.cc_val[3];

	//no syncbusy bit, CCBUFx is copied into CCx at TCC update time

	TCC0->PERBUF.bit.PERBUF = tcc0_data.top_val;

	//no syncbusy bit

	/*wait for data in each buffer to become valid*/
	while( !(TCC0->STATUS.reg & (TCC_STATUS_CCBUFV_Msk | TCC_STATUS_PERBUFV)) );

	/*unlock buffer registers*/
	TCC0->CTRLBCLR.reg = TCC_CTRLBCLR_LUPD; //LUPD
	TCC0_SYNC_WAIT(TCC_SYNCBUSY_CTRLB);
}

/******************************************************************************
*  \brief Clears Non-recoverable Fault
*
*  \note Restarts TCC0 after overcurrent event
******************************************************************************/
inline void drvr_tcc0_reset_fault(void)
{
	TCC0->STATUS.reg = TCC_STATUS_FAULT0; //Clear fault from event input 0
}


/**************************************************************************************************
*                                         LOCAL FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Generate top reg value (period)
*
*  \note for Dual-Slope PWM Generation
******************************************************************************/
static inline uint32_t gen_top_val(const float desired_freq_hz, const uint32_t tcc_clk_src_freq_hz, const uint32_t tcc_prescaler)
{
	/*TOP = (f_CLK/(f_DESIRED*PRESCALAR))-1 for Normal Pulse-Width Modulation*/
	//return (uint32_t)(((float)tcc_clk_src_freq_hz / (desired_freq_hz * (float)tcc_prescaler)) - 1.0);

	/*TOP = f_CLK/(f_DESIRED*PRESCALAR*2) for Dual-Slope PWM Generation*/
	return (uint32_t)((float)tcc_clk_src_freq_hz / (desired_freq_hz * (float)tcc_prescaler * 2.0));
}


/******************************************************************************
*  \brief Generate compare match value
*
*  \note for Dual-Slope PWM Generation
*        This is the same as with Normal Pulse-Width Modulation
******************************************************************************/
static inline uint32_t gen_cc_val(const float desired_duty_pct, const uint32_t current_tcc_top)
{
	/*CC = (DUTY_%*PERIOD)/100 for Normal Pulse-Width Modulation*/
	return (uint32_t)((desired_duty_pct * (float)current_tcc_top) / 100.0);
}


/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
/******************************************************************************
*  \brief interrupt for TCC0
*
*  \note CRITICAL RESOURCES:
*        NEEDS PROTECTION
*
*        DOES NOT NEED PROTECTION:
*            drvr_adc0_pulse_task
******************************************************************************/
void TCC0_Handler(void)
{
	uint8_t i;

	/*Must Force a read synchronization of COUNT to look at count, used for debugging*/
	//TCC0->CTRLBSET.bit.CMD = 0x4; //READSYNC

	/*Overflow/Underflow (OVF)*/
	if(TCC0->INTFLAG.bit.OVF)
	{
		/*For open load detection*/
		for(i = 0; i < ERP_TOTAL_CH; i++)
		{
			out_opn_ld_chk_isr(&g_ch_inst[i].out_inst);
		}

		/*For ADC pulse current measurement*/
		drvr_adc0_pulse_isr_task();

		TCC0->INTFLAG.reg = TCC_INTFLAG_OVF; //clear flag
	}

//not used
#if 0

	/*Count (CNT) - refer also to description of EVCTRL.CNTSEL.*/
	/*This flag is set on the next CLK_TCC_COUNT cycle after a counter event occurs.*/
	/*They type of event is set by the value in TCC0->EVCTRL.bit.CNTSEL*/
	if(TCC0->INTFLAG.bit.CNT)
	{

		TCC0->INTFLAG.reg = TCC_INTFLAG_CNT; //clear flag
	}

	/*Capture Overflow Error (ERR)*/
	if(TCC0->INTFLAG.bit.ERR)
	{

		TCC0->INTFLAG.reg = TCC_INTFLAG_ERR; //clear flag
	}
#endif
}
