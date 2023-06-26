/*
* drvr_adc.c
*
* Created: 10/18/2017 8:29:18 AM
*  Author: akudlacek
*/


#include "drvr_adc.h"

#include "drvr_port.h"
#include "misc.h"
#include "drvr_nvmctrl.h"
#include "drvr_dma.h"
#include "drvr_divas.h"
#include "drvr_nvic.h"
#include "drvr_tcc.h"

/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define USE_BIAS_AND_LINEARITY_CALIB_FROM_NVM 1

#define ADC0_SYNC_WAIT(syncbusy_flags) while((ADC0->SYNCBUSY.reg & syncbusy_flags) != 0)
#define ADC1_SYNC_WAIT(syncbusy_flags) while((ADC1->SYNCBUSY.reg & syncbusy_flags) != 0)

extern void assert_failed(char const *file, int line);


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
/*****LOCAL******/
static adc0_data_t adc0_data;
static adc1_data_t adc1_data;

/*****GLOBAL******/
//DMA memory for window monitor event
const volatile ADC_SEQSTATUS_Type    g_drvr_adc0_dma_seq_win;

//DMA memory for ADC channel the corresponding result is for
const volatile ADC_SEQSTATUS_Type    g_drvr_adc0_dma_seq_buf[ADC0_DMA_BUFFER_LEN];

//DMA memory for ADC result
const volatile ADC_RESULT_Type       g_drvr_adc0_dma_res_buf[ADC0_DMA_BUFFER_LEN];

//Pointers to ADC data for monitoring but changes not allowed.
const adc0_data_t * const adc0_data_ptr = &adc0_data;
const adc1_data_t * const adc1_data_ptr = &adc1_data;

/**************************************************************************************************
*                                         LOCAL PROTOTYPES
*************************************************^************************************************/
static inline void    adc_en       (Adc * const adc_peripheral);
static inline void    adc_start    (Adc * const adc_peripheral);
static inline void    adc_dis     (Adc * const adc_peripheral);
static inline void    adc0_calc_avg(void);
static drvr_adc0_ch_t adc0_mux2ch  (const adc0_mux_t adc0_mux);
static drvr_adc1_ch_t adc1_mux2ch  (const adc1_mux_t adc1_mux);


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Init for ADC0
*
*  \note Used only for current sense
*        winmon_thres_cnt sets threshold that will generate an event out
******************************************************************************/
void drvr_adc0_init(const uint16_t winmon_thres_cnt)
{
	ADC_CTRLA_Type                CTRLA_tmp = {0};
	ADC_CTRLB_Type                CTRLB_tmp = {0};
	ADC_REFCTRL_Type            REFCTRL_tmp = {0};
	ADC_INTENSET_Type          INTENSET_tmp = {0};
	ADC_INPUTCTRL_Type        INPUTCTRL_tmp = {0};
	ADC_CTRLC_Type                CTRLC_tmp = {0};
	ADC_AVGCTRL_Type            AVGCTRL_tmp = {0};
	ADC_SAMPCTRL_Type          SAMPCTRL_tmp = {0};

	/*Init states and flags - no critical section, before ISR enabled*/
	adc0_data.i_sample_cnt                                    = 0;
	adc0_data.i_sample_cnt_max                                = 0;
	adc0_data.i_state                                         = ADC_STATE_ARMED;
	adc0_data.ch_result[ADC_CH_CURRENT_SENSE_1].adc_data_flag = ADC_DATA_EMPTY;
	adc0_data.ch_result[ADC_CH_CURRENT_SENSE_2].adc_data_flag = ADC_DATA_EMPTY;
	adc0_data.ch_result[ADC_CH_CURRENT_SENSE_3].adc_data_flag = ADC_DATA_EMPTY;
	adc0_data.ch_result[ADC_CH_CURRENT_SENSE_4].adc_data_flag = ADC_DATA_EMPTY;
	adc0_data.ch_result[ADC_CH_CURRENT_SENSE_1].peak_result   = 0;
	adc0_data.ch_result[ADC_CH_CURRENT_SENSE_2].peak_result   = 0;
	adc0_data.ch_result[ADC_CH_CURRENT_SENSE_3].peak_result   = 0;
	adc0_data.ch_result[ADC_CH_CURRENT_SENSE_4].peak_result   = 0;
	adc0_data.i_wm_flg.reg                                    = 0;

	/*Init pins*/
	drvr_port_pin_cfg(CURRENT_SENSE_1, INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(CURRENT_SENSE_2, INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(CURRENT_SENSE_3, INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(CURRENT_SENSE_4, INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, A, LOW);

	/*************************ADC0*************************/
	/*Disable and reset ADC0*/
	ADC0->CTRLA.bit.SWRST     =   1;
	ADC0_SYNC_WAIT(ADC_SYNCBUSY_SWRST);

	//CTRLA
	CTRLA_tmp.bit.ONDEMAND     =   0; //The ADC is always on
	CTRLA_tmp.bit.RUNSTDBY     =   0; //The ADC is halted during standby sleep mode
	CTRLA_tmp.bit.SLAVEEN      =   0; //The master-slave operation is disabled
	CTRLA_tmp.bit.ENABLE       =   0; //The ADC is disabled.
	CTRLA_tmp.bit.SWRST        =   0; //There is no reset operation ongoing

	//CTRLB
	CTRLB_tmp.bit.PRESCALER    = 0x0; //Peripheral clock divided by 2

	//REFCTRL
	REFCTRL_tmp.bit.REFCOMP    =   1; //Reference Buffer Offset Compensation Enable
	REFCTRL_tmp.bit.REFSEL     = 0x5; //Reference Selection: INTVCC2 VDDANA (5V)

	//EVCTRL
	ADC0->EVCTRL.bit.WINMONEO  =   1; //Window Monitor Event Out
	ADC0->EVCTRL.bit.RESRDYEO  =   0; //Result Ready Event Out
	ADC0->EVCTRL.bit.STARTINV  =   0; //Start Conversion Event Invert Enable
	ADC0->EVCTRL.bit.FLUSHINV  =   0; //Flush Event Invert Enable
	ADC0->EVCTRL.bit.STARTEI   =   1; //Start Conversion Event Input Enable
	ADC0->EVCTRL.bit.FLUSHEI   =   0; //Flush Event Input Enable

	//INTENSET
	INTENSET_tmp.bit.WINMON    =   0; //Window Monitor Interrupt Enable
	INTENSET_tmp.bit.OVERRUN   =   0; //Overrun Interrupt Enable
	INTENSET_tmp.bit.RESRDY    =   0; //Result Ready Interrupt Enable, RESRDY interrupt will interfere with DMA capture of ADC result

	//INPUTCTRL - These should not matter because using single ended and sequence mode
	INPUTCTRL_tmp.bit.MUXNEG   =   0x18; //Internal ground
	INPUTCTRL_tmp.bit.MUXPOS   =   CURRENT_SENSE_1_ADC0_MUX;

	//CTRLC
	CTRLC_tmp.bit.DUALSEL      = 0x0; //Dual Mode Trigger Selection, no effect in this config
	CTRLC_tmp.bit.WINMODE      = 0x1; //Window Monitor Mode: MODE1 RESULT > WINLT
	CTRLC_tmp.bit.R2R          =   1; //Rail-to-Rail Operation, Offset compensation(SAMPCTRL.OFFCOMP) must be written to one
	CTRLC_tmp.bit.RESSEL       = 0x1; //Conversion Result Resolution: 16BIT For averaging mode output
	CTRLC_tmp.bit.CORREN       =   0; //Digital Correction Logic, can be enabled on the fly with functions below
	CTRLC_tmp.bit.FREERUN      =   1; //Free Running Mode
	CTRLC_tmp.bit.LEFTADJ      =   0; //Left-Adjusted Result
	CTRLC_tmp.bit.DIFFMODE     =   0; //Differential Mode

	//AVGCTRL - Oversampling and Decimation for 13 bit result, See Table 38-3
	/**** If changing result resolution be sure to also adjust ADC0_RESULT_RESOLUTION_BITS ****/
	AVGCTRL_tmp.bit.ADJRES     = 0x1;
	AVGCTRL_tmp.bit.SAMPLENUM  = 0x2;

	//SAMPCTRL - It’s not possible to use OFFCOMP=1 and SAMPLEN>0
	SAMPCTRL_tmp.bit.OFFCOMP   =   CTRLC_tmp.bit.R2R; //Comparator Offset Compensation Enable, needed to be on as well as R2R
	SAMPCTRL_tmp.bit.SAMPLEN   =   0; //Sampling Time Length

	//WINLT
	ADC0->WINLT.bit.WINLT      = winmon_thres_cnt; //Threshold for window monitor

	//GAINCORR - See Microchip doc TB3185
	ADC0->GAINCORR.bit.GAINCORR = 2048; //Set to 1.0 (default)

	//OFFSETCORR - See Microchip doc TB3185
	ADC0->OFFSETCORR.bit.OFFSETCORR = 0; //Set to 0 (default)

	//CALIB
#if USE_BIAS_AND_LINEARITY_CALIB_FROM_NVM

	/*Wait for NVM to be ready*/
	while(nvm_is_ready() == 0);
	ADC0->CALIB.bit.BIASREFBUF = NVM_SW_CALIB->bit.ADC0_LINEARITY; //LINEARITY = BIASREFBUF
	ADC0->CALIB.bit.BIASCOMP   = NVM_SW_CALIB->bit.ADC0_BIASCAL;   //BIASCAL = BIASCOMP
#endif

	//SEQCTRL
	ADC0->SEQCTRL.bit.SEQEN = (1 << CURRENT_SENSE_1_ADC0_MUX) | (1 << CURRENT_SENSE_2_ADC0_MUX) | (1 << CURRENT_SENSE_3_ADC0_MUX) | (1 << CURRENT_SENSE_4_ADC0_MUX);

	/*Set Write-Synchronized registers and same settings*/
	ADC0->CTRLB.reg            = CTRLB_tmp.reg;
	ADC0->REFCTRL.reg          = REFCTRL_tmp.reg;
	ADC0->INTENSET.reg         = INTENSET_tmp.reg;
	ADC0->INPUTCTRL.reg        = INPUTCTRL_tmp.reg;
	ADC0->CTRLC.reg            = CTRLC_tmp.reg;
	ADC0->AVGCTRL.reg          = AVGCTRL_tmp.reg;
	ADC0->SAMPCTRL.reg         = SAMPCTRL_tmp.reg;
	ADC0->CTRLA.reg            = CTRLA_tmp.reg;

	/*Wait for all sync bits to be cleared*/
	ADC0_SYNC_WAIT(ADC_SYNCBUSY_MASK);

	/* Run to catch winmon events must restart every time one occurs */
	drvr_dma_start(DMA_CH0_ADC0_WIN);

	adc_en(ADC0);
	adc_start(ADC0); //Must run all the time to catch any over current events
}

/******************************************************************************
*  \brief Gets ADC0 values
*
*  \note
******************************************************************************/
drvr_adc_return_t drvr_adc0_get_res(uint16_t * const result, const drvr_adc0_ch_t ch)
{
	if(adc0_data.ch_result[ch].adc_data_flag == ADC_DATA_READY)
	{
		*result = adc0_data.ch_result[ch].result;
		adc0_data.ch_result[ch].adc_data_flag = ADC_DATA_EMPTY;

		return ADC_SUCCESS;
	}

	return ADC_BUSY;
}

/******************************************************************************
*  \brief Gets ADC0 peak results
*
*  \note
******************************************************************************/
uint16_t drvr_adc0_get_pk(const drvr_adc0_ch_t ch)
{
	return adc0_data.ch_result[ch].peak_result;
}

/******************************************************************************
*  \brief ADC0 Window monitor flag
*
*  \note CRITICAL SECTION: Do not need, one way data
*        BIT mapped to ADC0 channels
******************************************************************************/
wm_flg_t drvr_adc0_get_wm(void)
{
	return adc0_data.i_wm_flg;
}

/******************************************************************************
*  \brief Start of pulse task for ADC0
*
*  \note This state machine keeps the ADC sampling current at all times, starts
*        DMA transfer at the beginning of the PWM period, stops DMA transfer at
*        the end of the period, ensures enough samples have been received, and
*        provides a done state for the drvr_adc0_main_task to know that a full
*        period of samples has been acquired and needs processing.
*
*        The sm needs to keep the ADC sampling at all times in order for the
*        WINMON overcurrent detection to work. DMA transfers to two buffers,
*        one for the result of the ADC conversion and another for the channel
*        of which that conversion was for. A minimum number of samples is
*        needed to provide good accuracy when operating at high PWM
*        frequencies. This function is called by TCC0 interrupt every PWM
*        period.
******************************************************************************/
inline void drvr_adc0_pulse_isr_task(void)
{
	adc0_count_t num_seq = 0;
	adc0_count_t num_res = 0;

	switch(adc0_data.i_state)
	{
		case ADC_STATE_ARMED:
			//Turn free run off so DMA can control ADC start
			ADC0->CTRLC.bit.FREERUN = 0;
			ADC0_SYNC_WAIT(ADC_SYNCBUSY_CTRLC);
			ADC0->SWTRIG.bit.FLUSH = 1;
			ADC0_SYNC_WAIT(ADC_SYNCBUSY_SWTRIG);
			adc_start(ADC0);

			drvr_dma_start(DMA_CH1_ADC0_SEQ);
			drvr_dma_start(DMA_CH2_ADC0_RES);
			adc0_data.i_state = ADC_STATE_BUSY;
		break;

		case ADC_STATE_BUSY:
			//This makes sure a sufficient number of samples have been acquired. If the minimum number has not been reached
			//another period will be sampled in order to get a more accurate average.
			if(drvr_dma_get_xfr_size(DMA_CH2_ADC0_RES) <= ADC0_NUM_SAMP_MIN)
			{
				break; //stay in this state and collect more samples
			}
			
			num_seq = drvr_dma_abort(DMA_CH1_ADC0_SEQ);
			num_res = drvr_dma_abort(DMA_CH2_ADC0_RES);

			//Turn free run on so ADC keeps sampling in case of overcurrent event
			ADC0->CTRLC.bit.FREERUN = 1;
			ADC0_SYNC_WAIT(ADC_SYNCBUSY_CTRLC);
			adc_start(ADC0);

			adc0_data.i_sample_cnt = (num_res < num_seq ? num_res : num_seq); //use least
			adc0_data.i_sample_cnt_max = (adc0_data.i_sample_cnt_max > adc0_data.i_sample_cnt ? adc0_data.i_sample_cnt_max : adc0_data.i_sample_cnt);
			adc0_data.i_state = ADC_STATE_DONE;
		break;

		case ADC_STATE_DONE:
		break;

		default:
		break;
	}
}

/******************************************************************************
*  \brief ADC0 task
*
*  \note This should run in main loop
******************************************************************************/
drvr_adc_return_t drvr_adc0_main_task(void)
{
	drvr_adc_return_t drvr_adc_return = ADC_BUSY;

	/*
	State is not protected even though it is modified in an interrupt because
	it is protected by the state machine. Once the SM is ARMED or BUSY this function
	can no longer look at ADC data nor modify the SM variable. Only when DONE this
	function is permitted to look at the ADC data and change the state. Similarly
	the interrupt task cannot change any ADC data or state when it is DONE.
	Changing the state variable must always be done after the tasks have been
	completed.
	*/
	switch(adc0_data.i_state)
	{
		case ADC_STATE_DONE:
			adc0_calc_avg();
			drvr_adc_return = ADC_SUCCESS;
			adc0_data.i_state = ADC_STATE_ARMED;
		break;

		case ADC_STATE_ARMED:
		break;

		case ADC_STATE_BUSY:
		break;

		default:
		break;
	}

	return drvr_adc_return;
}

/******************************************************************************
*  \brief Window monitor triggered ADC task.
*
*  \note This task checks which ADC channel had an overcurent event and
*        disables that pin. It then clears the TCC fault thus restarting normal
*        operation for the rest of the pins. DMA is used to capture which pin
*        caused the overcurrent condition which is why the DMA interrupt
*        is the caller of this function. Once DMA has transfered the offender
*        this function can check which one it was.
*
*        This runs when a winmon event occurs, but since reading the result
*        reg clears the winmon interrupt flag this has to be moved to the
*        DMAC to run after DMA grabbed the CH. DMA can read the result
*        register before the interrupt gets handled now.
******************************************************************************/
inline void drvr_adc0_wm_isr_task(void)
{
	drvr_adc0_ch_t ch;
	uint8_t pin = HSS_PWM_1; //initializing pin to get rid of "-Wmaybe-uninitialized warning" but I added an assert failed to the default case to guard this issue

	//check which sequence DMA transfered during winmon event to find offending channel
	//no critical section, DMA stopped when this gets read
	ch = adc0_mux2ch(g_drvr_adc0_dma_seq_win.bit.SEQSTATE);

	//set winmon flag
	adc0_data.i_wm_flg.reg |= (1UL << ch);

	//match ADC current sense ch to HSS pin
	switch(ch)
	{
		case ADC_CH_CURRENT_SENSE_1:
			pin = HSS_PWM_1;
		break;

		case ADC_CH_CURRENT_SENSE_2:
			pin = HSS_PWM_2;
		break;

		case ADC_CH_CURRENT_SENSE_3:
			pin = HSS_PWM_3;
		break;

		case ADC_CH_CURRENT_SENSE_4:
			pin = HSS_PWM_4;
		break;

		default:
			assert_failed(__FILE__, __LINE__);
		break;
	}

	//reconfigure pin of HSS to remove control from TCC and set LOW or OFF
	drvr_port_pin_cfg(pin, OUTPUT, NORMAL, PULL_DIS, IN_BUF_DIS, PORT_EN, LOW);

	//restart DMA capture
	drvr_dma_start(DMA_CH0_ADC0_WIN);

	//resets fault as well as starts TCC0 again.
	drvr_tcc0_reset_fault();
}

/**************************************************************************************************************************************************************/
//Note: the following gain, offset, and correction enable functions were previously used to calibrate the ADC on the single channel firmware. They were left
// here for reference in case this calibration is to be used again. Since the ADC0 is used for all four channels the hardware corrections may not be enough and
// there may need to be additional software calibrations per channel.
/**************************************************************************************************************************************************************/

/******************************************************************************
*  \brief ADC0 Set Gain Correction value (0.5 <= GAINCORR < 2.0)
*
*  \note This input value is a ratio of slopes
*        Gains greater than 1.0 reduce the measured value at full scale
*        Gains less than 1.0 increase the measured value at full scale
*        For no gain compensation use 1.0
******************************************************************************/
void drvr_adc0_set_gain_corr(const float gain_corr_value)
{
	float tmp_gain_corr_val = 0;

	tmp_gain_corr_val = 2048.0/gain_corr_value;

	//make sure within valid range
	//The GAINCORR is a 12-bit register and the values can range from 1024 to 4095.
	if(tmp_gain_corr_val > 4095.0)
		tmp_gain_corr_val = 4095.0;

	else if(tmp_gain_corr_val < 1024.0)
		tmp_gain_corr_val = 1024.0;

	//GAINCORR - See Microchip doc TB3185
	ADC0->GAINCORR.bit.GAINCORR = (uint16_t)tmp_gain_corr_val & 0xFFF;
	ADC0_SYNC_WAIT(ADC_SYNCBUSY_GAINCORR);
}

/******************************************************************************
*  \brief ADC0 Set Offset Correction value (-2048 - 2047) 12BIT
*
*  \note This input value is 12BIT ADC code value
*        Positive offsets decrease measured value at low scale
*        Negative offsets increase the measured value at low scale
*        For no offset compensation use 0
******************************************************************************/
void drvr_adc0_set_offset_corr(int offset_corr_val)
{
	uint16_t tmp_offset_corr_val = 0;

	//make sure within valid range
	//The OFFSETCORR is also a 12-bit register and the value to be programmed
	//is in two’s complement format.
	if(offset_corr_val > 2047)
		offset_corr_val = 2047;

	else if(offset_corr_val < -2048)
		offset_corr_val = -2048;

	//Needs to be in 2s compliment just truncate for positive number
	if(offset_corr_val >= 0)
	{
		tmp_offset_corr_val = (uint16_t)offset_corr_val & 0x7FF;
	}
	//For negative truncate and set sign bit
	else
	{
		tmp_offset_corr_val = ((uint16_t)offset_corr_val & 0x7FF) | 0x800;
	}

	//OFFSETCORR - See Microchip doc TB3185
	ADC0->OFFSETCORR.bit.OFFSETCORR = tmp_offset_corr_val & 0xFFF;
	ADC0_SYNC_WAIT(ADC_SYNCBUSY_OFFSETCORR);
}

/******************************************************************************
*  \brief ADC0 Enable Digital Correction
*
*  \note Enable the digital result correction. 1 - ON, 0 - OFF
******************************************************************************/
void drvr_adc0_set_corr_en(const uint8_t on_off)
{
	/*This func not called often so can block all interrupts*/
	cpu_irq_enter_critical();

	//CTRLC
	ADC0->CTRLC.bit.CORREN = on_off & 1;
	ADC0_SYNC_WAIT(ADC_SYNCBUSY_CTRLC);

	//Need to flush ADC results because ADC trips winmon from corrupted results
	//Not sure why but it seems to stay corrupted after it happens
	ADC0->SWTRIG.bit.FLUSH = 1; //flush results
	ADC0_SYNC_WAIT(ADC_SYNCBUSY_SWTRIG);

	//clear results
	if(ADC0->INTFLAG.bit.RESRDY == 1)
	{
		ADC0->RESULT.bit.RESULT;
	}

	cpu_irq_leave_critical();
}

/******************************************************************************
*  \brief ADC0 get gain correction value
*
*  \note
******************************************************************************/
float drvr_adc0_get_gain_corr_val(void)
{
	return 2048.0 / (float)ADC0->GAINCORR.bit.GAINCORR;
}

/******************************************************************************
*  \brief ADC0 get offset correction value
*
*  \note
******************************************************************************/
int drvr_adc0_get_offset_corr_val(void)
{
	int x;

	//copy offset 12BIT signed to int.
	x = (int)ADC0->OFFSETCORR.bit.OFFSETCORR;

	//convert to int
	x = ( ((x >> 11) == 0) ? (x) : ((-1 ^ 0xFFF) | x) );

	return x;
}

/******************************************************************************
*  \brief ADC0 get digital correction enable value
*
*  \note 1 - ON, 0 - OFF
******************************************************************************/
uint8_t drvr_adc0_get_corr_en(void)
{
	return (uint8_t)ADC0->CTRLC.bit.CORREN;
}

/******************************************************************************
*  \brief Init for ADC1
*
*  \note 16 BIT result
******************************************************************************/
void drvr_adc1_init(void)
{
	ADC_CTRLA_Type                CTRLA_tmp = {0};
	ADC_CTRLB_Type                CTRLB_tmp = {0};
	ADC_REFCTRL_Type            REFCTRL_tmp = {0};
	ADC_INTENSET_Type          INTENSET_tmp = {0};
	ADC_INPUTCTRL_Type        INPUTCTRL_tmp = {0};
	ADC_CTRLC_Type                CTRLC_tmp = {0};
	ADC_AVGCTRL_Type            AVGCTRL_tmp = {0};
	ADC_SAMPCTRL_Type          SAMPCTRL_tmp = {0};

	/*Init states and flags - no critical section, before ISR enabled*/
	adc1_data.i_ch_result[ADC_CH_VIN_SENSE].adc_data_flag       = ADC_DATA_EMPTY;
	adc1_data.i_ch_result[ADC_CH_U_IN_1].adc_data_flag          = ADC_DATA_EMPTY;
	adc1_data.i_ch_result[ADC_CH_U_IN_2].adc_data_flag          = ADC_DATA_EMPTY;
	adc1_data.i_ch_result[ADC_CH_U_IN_3].adc_data_flag          = ADC_DATA_EMPTY;
	adc1_data.i_ch_result[ADC_CH_U_IN_4].adc_data_flag          = ADC_DATA_EMPTY;
	adc1_data.i_ch_result[ADC_CH_POT_GND_SENSE_1].adc_data_flag = ADC_DATA_EMPTY;
	adc1_data.i_ch_result[ADC_CH_POT_GND_SENSE_2].adc_data_flag = ADC_DATA_EMPTY;
	adc1_data.i_ch_result[ADC_CH_POT_GND_SENSE_3].adc_data_flag = ADC_DATA_EMPTY;
	adc1_data.i_ch_result[ADC_CH_POT_GND_SENSE_4].adc_data_flag = ADC_DATA_EMPTY;

	/*Init pins*/
	drvr_port_pin_cfg(VIN_SENSE,       INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(U_IN_1,          INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(U_IN_2,          INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(U_IN_3,          INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(U_IN_4_ADC,      INPUT, NORMAL, PULL_DIS, IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(POT_GND_SENSE_1, INPUT, NORMAL, PULL_EN,  IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(POT_GND_SENSE_2, INPUT, NORMAL, PULL_EN,  IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(POT_GND_SENSE_3, INPUT, NORMAL, PULL_EN,  IN_BUF_DIS, A, LOW);
	drvr_port_pin_cfg(POT_GND_SENSE_4, INPUT, NORMAL, PULL_EN,  IN_BUF_DIS, A, LOW);

	/*************************ADC1*************************/
	/*Disable and reset ADC1*/
	ADC1->CTRLA.bit.SWRST     =   1;
	ADC1_SYNC_WAIT(ADC_SYNCBUSY_SWRST);

	//CTRLA
	CTRLA_tmp.bit.ONDEMAND     =   0; //The ADC is always on
	CTRLA_tmp.bit.RUNSTDBY     =   0; //The ADC is halted during standby sleep mode
	CTRLA_tmp.bit.SLAVEEN      =   0; //The master-slave operation is disabled
	CTRLA_tmp.bit.ENABLE       =   1; //The ADC is enabled.
	CTRLA_tmp.bit.SWRST        =   0; //There is no reset operation ongoing

	//CTRLB
	CTRLB_tmp.bit.PRESCALER    = 0x0; //Peripheral clock divided by 2

	//REFCTRL
	REFCTRL_tmp.bit.REFCOMP    =   1; //Reference Buffer Offset Compensation Enable
	REFCTRL_tmp.bit.REFSEL     = 0x5; //Reference Selection: INTVCC2 VDDANA (5V)

	//EVCTRL
	ADC1->EVCTRL.bit.WINMONEO  =   0; //Window Monitor Event Out
	ADC1->EVCTRL.bit.RESRDYEO  =   0; //Result Ready Event Out
	ADC1->EVCTRL.bit.STARTINV  =   0; //Start Conversion Event Invert Enable
	ADC1->EVCTRL.bit.FLUSHINV  =   0; //Flush Event Invert Enable
	ADC1->EVCTRL.bit.STARTEI   =   0; //Start Conversion Event Input Enable
	ADC1->EVCTRL.bit.FLUSHEI   =   0; //Flush Event Input Enable

	//INTENSET
	INTENSET_tmp.bit.WINMON    =   0; //Window Monitor Interrupt Enable
	INTENSET_tmp.bit.OVERRUN   =   0; //Overrun Interrupt Enable
	INTENSET_tmp.bit.RESRDY    =   1; //Result Ready Interrupt Enable

	//INPUTCTRL - These should not matter because using single ended and sequence mode
	INPUTCTRL_tmp.bit.MUXNEG   =   0x18; //Internal ground
	INPUTCTRL_tmp.bit.MUXPOS   =   U_IN_2_ADC1_MUX;

	//CTRLC
	CTRLC_tmp.bit.DUALSEL      = 0x0; //Dual Mode Trigger Selection, no effect in this config
	CTRLC_tmp.bit.WINMODE      = 0x0; //Window Monitor Mode: No window mode (default)
	CTRLC_tmp.bit.R2R          =   1; //Rail-to-Rail Operation, Offset compensation(SAMPCTRL.OFFCOMP) must be written to one
	CTRLC_tmp.bit.RESSEL       = 0x1; //Conversion Result Resolution: 16BIT For averaging mode output
	CTRLC_tmp.bit.CORREN       =   0; //Digital Correction Logic, can be enabled on the fly with functions below
	CTRLC_tmp.bit.FREERUN      =   1; //Free Running Mode
	CTRLC_tmp.bit.LEFTADJ      =   0; //Left-Adjusted Result
	CTRLC_tmp.bit.DIFFMODE     =   0; //Differential Mode

	//AVGCTRL - 12bit Averaging
	/**** If changing result resolution be sure to also adjust ADC1_RESULT_RESOLUTION_BITS ****/
	AVGCTRL_tmp.bit.ADJRES     = 0x4;
	AVGCTRL_tmp.bit.SAMPLENUM  = 0x6;

	//SAMPCTRL - It’s not possible to use OFFCOMP=1 and SAMPLEN>0
	SAMPCTRL_tmp.bit.OFFCOMP   =   CTRLC_tmp.bit.R2R; //Comparator Offset Compensation Enable, needed to be on as well as R2R
	SAMPCTRL_tmp.bit.SAMPLEN   =   0; //Sampling Time Length

	//GAINCORR - See Microchip doc TB3185
	ADC1->GAINCORR.bit.GAINCORR = 2048; //Set to 1.0 (default)

	//OFFSETCORR - See Microchip doc TB3185
	ADC1->OFFSETCORR.bit.OFFSETCORR = 0; //Set to 0 (default)

	//CALIB
	#if USE_BIAS_AND_LINEARITY_CALIB_FROM_NVM

	/*Wait for NVM to be ready*/
	while(nvm_is_ready() == 0);
	ADC1->CALIB.bit.BIASREFBUF = NVM_SW_CALIB->bit.ADC1_LINEARITY; //LINEARITY = BIASREFBUF
	ADC1->CALIB.bit.BIASCOMP   = NVM_SW_CALIB->bit.ADC1_BIASCAL;   //BIASCAL = BIASCOMP
	#endif

	//SEQCTRL
	ADC1->SEQCTRL.bit.SEQEN = (1 << VIN_SENSE_ADC1_MUX) | (1 << U_IN_1_ADC1_MUX) | (1 << U_IN_2_ADC1_MUX) | (1 << U_IN_3_ADC1_MUX) |
	(1 << U_IN_4_ADC1_MUX) | (1 << POT_GND_SENSE_1_ADC1_MUX) | (1 << POT_GND_SENSE_2_ADC1_MUX) |
	(1 << POT_GND_SENSE_3_ADC1_MUX) | (1 << POT_GND_SENSE_4_ADC1_MUX);

	/*Set Write-Synchronized registers and same settings*/
	ADC1->CTRLB.reg            = CTRLB_tmp.reg;
	ADC1->REFCTRL.reg          = REFCTRL_tmp.reg;
	ADC1->INTENSET.reg         = INTENSET_tmp.reg;
	ADC1->INPUTCTRL.reg        = INPUTCTRL_tmp.reg;
	ADC1->CTRLC.reg            = CTRLC_tmp.reg;
	ADC1->AVGCTRL.reg          = AVGCTRL_tmp.reg;
	ADC1->SAMPCTRL.reg         = SAMPCTRL_tmp.reg;
	ADC1->CTRLA.reg            = CTRLA_tmp.reg;

	/*Wait for all sync bits to be cleared*/
	ADC1_SYNC_WAIT(ADC_SYNCBUSY_MASK);

	adc_en(ADC1);
	adc_start(ADC1);
}

/******************************************************************************
*  \brief Gets ADC1 values
*
*  \note This function only updates the result if ADC data is new and clears
*        the flag indicating this
******************************************************************************/
drvr_adc_return_t drvr_adc1_get_res(uint16_t * const result, const drvr_adc1_ch_t ch)
{
	drvr_adc_return_t drvr_adc_return = ADC_BUSY;

	drvr_nvic_irq_enter_critical(ADC1_IRQn);

	if(adc1_data.i_ch_result[ch].adc_data_flag == ADC_DATA_READY)
	{
		*result = adc1_data.i_ch_result[ch].result;
		adc1_data.i_ch_result[ch].adc_data_flag = ADC_DATA_EMPTY;

		drvr_adc_return = ADC_SUCCESS;
	}

	drvr_nvic_irq_leave_critical(ADC1_IRQn);

	return drvr_adc_return;
}

/******************************************************************************
*  \brief ADC1 channel initial fill
*
*  \note This function waits until the ADC1 has sampled all of its channels
*        Do not block ADC1 interrupts around this function
******************************************************************************/
void drvr_adc1_fill_wait(void)
{
	drvr_adc1_ch_t ch;

	for(ch = 0; ch < ADC1_NUM_CH; ch++)
	{
		while(adc1_data.i_ch_result[ch].adc_data_flag == ADC_DATA_EMPTY);
	}
}

/******************************************************************************
*  \brief Gets ADC1 values
*
*  \note This function returns the result regardless if it is old
******************************************************************************/
inline uint16_t drvr_adc1_get_res_now(const drvr_adc1_ch_t ch)
{
	return adc1_data.i_ch_result[ch].result;
}


/**************************************************************************************************
*                                         LOCAL FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief ADC Enable
*
*  \note
******************************************************************************/
static inline void adc_en(Adc * const adc_peripheral)
{
	adc_peripheral->CTRLA.bit.ENABLE = 1;
	while((adc_peripheral->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE) != 0);
}

/******************************************************************************
*  \brief ADC Start
*
*  \note
******************************************************************************/
static inline void adc_start(Adc * const adc_peripheral)
{
	adc_peripheral->SWTRIG.bit.START = 1;
	while((adc_peripheral->SYNCBUSY.reg & ADC_SYNCBUSY_SWTRIG) != 0);
}

/******************************************************************************
*  \brief ADC disable
*
*  \note
******************************************************************************/
static inline void adc_dis(Adc * const adc_peripheral)
{
	adc_peripheral->CTRLA.bit.ENABLE = 0;
	//Not waiting for sync because the sync bit never clears when you disable the ADC
	//while((adc_peripheral->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE) != 0);
}

/******************************************************************************
*  \brief Calc avg ADC0
*
*  \note CRITICAL RESOURCES: this is ran within already protected section
*        therefore no critical section protection found here
******************************************************************************/
static inline void adc0_calc_avg(void)
{
	divas_unsigned_result_t divide_result;
	drvr_adc0_ch_t ch;
	adc0_sum_t sum[ADC0_NUM_CH] = {0};
	adc0_count_t count[ADC0_NUM_CH] = {0};
	adc0_count_t i;

	/*Iterate through samples summing and counting*/
	for(i = 0; i < adc0_data.i_sample_cnt; i++)
	{
		ch = adc0_mux2ch(g_drvr_adc0_dma_seq_buf[i].bit.SEQSTATE);

		//get peak result
		if(g_drvr_adc0_dma_res_buf[i].bit.RESULT > adc0_data.ch_result[ch].peak_result)
		{
			adc0_data.ch_result[ch].peak_result = g_drvr_adc0_dma_res_buf[i].bit.RESULT;
		}

		sum[ch] += g_drvr_adc0_dma_res_buf[i].bit.RESULT;
		count[ch]++;
	}

	/*Compute average, set ready flag*/
	for(ch = 0; ch < ADC0_NUM_CH; ch++)
	{
		divide_result  = drvr_divas_udiv(sum[ch], count[ch]);
		adc0_data.ch_result[ch].result = divide_result.result;
		adc0_data.ch_result[ch].adc_data_flag = ADC_DATA_READY;
	}
}

/******************************************************************************
*  \brief changes ADC0 MUX to CH
*
*  \note
******************************************************************************/
static drvr_adc0_ch_t adc0_mux2ch(const adc0_mux_t adc0_mux)
{
	switch(adc0_mux)
	{
		case CURRENT_SENSE_1_ADC0_MUX:
		return ADC_CH_CURRENT_SENSE_1;

		case CURRENT_SENSE_2_ADC0_MUX:
		return ADC_CH_CURRENT_SENSE_2;

		case CURRENT_SENSE_3_ADC0_MUX:
		return ADC_CH_CURRENT_SENSE_3;

		case CURRENT_SENSE_4_ADC0_MUX:
		return ADC_CH_CURRENT_SENSE_4;

		default:
		assert_failed(__FILE__, __LINE__);
		return 0;
	}
}


/******************************************************************************
*  \brief changes ADC1 MUX to CH
*
*  \note
******************************************************************************/
static drvr_adc1_ch_t adc1_mux2ch(const adc1_mux_t adc1_mux)
{
	switch(adc1_mux)
	{
		case VIN_SENSE_ADC1_MUX:
		return ADC_CH_VIN_SENSE;

		case U_IN_1_ADC1_MUX:
		return ADC_CH_U_IN_1;

		case U_IN_2_ADC1_MUX:
		return ADC_CH_U_IN_2;

		case U_IN_3_ADC1_MUX:
		return ADC_CH_U_IN_3;

		case U_IN_4_ADC1_MUX:
		return ADC_CH_U_IN_4;

		case POT_GND_SENSE_1_ADC1_MUX:
		return ADC_CH_POT_GND_SENSE_1;

		case POT_GND_SENSE_2_ADC1_MUX:
		return ADC_CH_POT_GND_SENSE_2;

		case POT_GND_SENSE_3_ADC1_MUX:
		return ADC_CH_POT_GND_SENSE_3;

		case POT_GND_SENSE_4_ADC1_MUX:
		return ADC_CH_POT_GND_SENSE_4;

		default:
		assert_failed(__FILE__, __LINE__);
		return 0;
	}
}

/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
/******************************************************************************
*  \brief interrupt for ADC0
*
*  \note
******************************************************************************/
#if 0 //no interrupts are used here
void ADC0_Handler(void)
{
	/****WARNING: If you toggle a pin in the ADC interrupt it puts noise into results****/

	/*Window Monitor interrupt flag*/
	/*NOTE: Check WINMON first, reading the RESULT reg clears it and WINMON won't see it any more*/
	if(ADC0->INTFLAG.bit.WINMON)
	{

		ADC0->INTFLAG.reg = ADC_INTFLAG_WINMON; //clear flag
	}

	/*Overrun interrupt flag*/
	if(ADC0->INTFLAG.bit.OVERRUN)
	{

		ADC0->INTFLAG.reg = ADC_INTFLAG_OVERRUN; //clear flag
	}

	/*Result Ready interrupt flag*/
	if(ADC0->INTFLAG.bit.RESRDY)
	{

		ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY; //clear flag
	}
}
#endif

/******************************************************************************
*  \brief interrupt for ADC1
*
*  \note
******************************************************************************/
void ADC1_Handler(void)
{
	drvr_adc1_ch_t ch;

	/****WARNING: If you toggle a pin in the ADC interrupt it puts noise into results****/

#if 0 //these interrupts are not used

	/*Window Monitor interrupt flag*/
	/*NOTE: Check WINMON first, reading the RESULT reg clears it and WINMON won't see it any more*/
	if(ADC1->INTFLAG.bit.WINMON)
	{

		ADC1->INTFLAG.reg = ADC_INTFLAG_WINMON; //clear flag
	}

	/*Overrun interrupt flag*/
	if(ADC1->INTFLAG.bit.OVERRUN)
	{

		ADC1->INTFLAG.reg = ADC_INTFLAG_OVERRUN; //clear flag
	}

#endif

	/*Result Ready interrupt flag*/
	if(ADC1->INTFLAG.bit.RESRDY)
	{
		ch = adc1_mux2ch(ADC1->SEQSTATUS.bit.SEQSTATE);

		adc1_data.i_ch_result[ch].result = ADC1->RESULT.bit.RESULT;
		adc1_data.i_ch_result[ch].adc_data_flag = ADC_DATA_READY;

		ADC1->INTFLAG.reg = ADC_INTFLAG_RESRDY; //clear flag
	}
}
