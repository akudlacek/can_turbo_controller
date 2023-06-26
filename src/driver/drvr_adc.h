/*
* drvr_adc.h
*
* Created: 10/18/2017 8:35:14 AM
*  Author: akudlacek
*/


#ifndef DRVR_ADC_H_
#define DRVR_ADC_H_


#include <stdint.h>
#include "sam.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
typedef enum drvr_adc_return_t
{
	ADC_BUSY     = 0,
	ADC_SUCCESS  = 1
} drvr_adc_return_t;

/* ADC0
 * this will limit the number of samples as well as allocate buffer
 * this is shared between the 4 channels
 *
 * Ensure the following: ( ADC0_SUM_T_MAX >= (ADC0_RESULT_MAX  * (ADC0_DMA_BUFFER_LEN / 4)) )
 * This should not be a problem but I am documenting that I looked at it.
 * If adc0_sum_t is uint32_t, and worst case that ADC0_RESULT_RESOLUTION_BITS = 16
 * then ADC0_DMA_BUFFER_LEN has to be limited to 262,148
 */
#define ADC0_DMA_BUFFER_LEN 3300U                   //no more then max of adc0_count_t
typedef uint32_t                     adc0_sum_t;    //data type for ADC sum
typedef uint16_t                     adc0_count_t;  //data type for ADC count
#define ADC0_SUM_T_MAX               (UINT32_MAX)   //make sure this is consistent with adc0_sum_t's type
#define ADC0_RESULT_RESOLUTION_BITS  (13UL)         //match this to what the ADC is configured to
#define ADC0_RESULT_MAX              ((1UL << ADC0_RESULT_RESOLUTION_BITS) - 1UL)
#define ADC0_NUM_SAMP_MIN            500            //Min number of samples to get, otherwise wait until next pulse period

#if ADC0_SUM_T_MAX < (ADC0_RESULT_MAX  * (ADC0_DMA_BUFFER_LEN / 4))
#error "ADC0 buffer to large to hold sum result"
#endif

#define ADC0_V_TO_COUNT(volt)        ((uint16_t)(((float)ADC0_RESULT_MAX / VDDANA_INTVCC2_V) * (float)(volt)))
#define ADC0_COUNT_TO_V(count)       ((VDDANA_INTVCC2_V / (float)ADC0_RESULT_MAX) * (float)(count))
#define ADC0_mV_TO_COUNT(mV)         ((uint16_t)(((float)ADC0_RESULT_MAX / 5000.0) * (float)(mV)))
#define ADC0_COUNT_TO_mV(count)      ((5000.0 / (float)ADC0_RESULT_MAX) * (float)(count))

typedef enum drvr_adc0_ch_t
{
	ADC_CH_CURRENT_SENSE_1 = 0,
	ADC_CH_CURRENT_SENSE_2 = 1,
	ADC_CH_CURRENT_SENSE_3 = 2,
	ADC_CH_CURRENT_SENSE_4 = 3,

	ADC0_NUM_CH            = 4
} drvr_adc0_ch_t;

typedef union wm_flg_t
{
	struct
	{
		uint8_t ch1:1; //bit: 0 ch1
		uint8_t ch2:1; //bit: 1 ch2
		uint8_t ch3:1; //bit: 2 ch3
		uint8_t ch4:1; //bit: 3 ch4
		uint8_t    :4; //bit: 4 - 7 UNUSED
	} bit;
	uint8_t reg;
} wm_flg_t;

/*ADC1*/
#define ADC1_RESULT_RESOLUTION_BITS  (12)       //match this to what the ADC is configured to
#define ADC1_RESULT_MAX              ((1UL << ADC1_RESULT_RESOLUTION_BITS) - 1UL)

#define ADC1_V_TO_COUNT(volt)        ((uint16_t)(((float)ADC1_RESULT_MAX / VDDANA_INTVCC2_V) * (float)(volt)))
#define ADC1_COUNT_TO_V(count)       ((VDDANA_INTVCC2_V / (float)ADC1_RESULT_MAX) * (float)(count))
#define ADC1_mV_TO_COUNT(mV)         ((uint16_t)(((float)ADC1_RESULT_MAX / 5000.0) * (float)(mV)))
#define ADC1_COUNT_TO_mV(count)      ((5000.0 / (float)ADC1_RESULT_MAX) * (float)(count))

typedef enum drvr_adc1_ch_t
{
	ADC_CH_VIN_SENSE       = 0,
	ADC_CH_U_IN_1          = 1,
	ADC_CH_U_IN_2          = 2,
	ADC_CH_U_IN_3          = 3,
	ADC_CH_U_IN_4          = 4,
	ADC_CH_POT_GND_SENSE_1 = 5,
	ADC_CH_POT_GND_SENSE_2 = 6,
	ADC_CH_POT_GND_SENSE_3 = 7,
	ADC_CH_POT_GND_SENSE_4 = 8,

	ADC1_NUM_CH            = 9
} drvr_adc1_ch_t;

typedef enum adc_state_t
{
	ADC_STATE_DONE, //ADC has sampled and has data ready
	ADC_STATE_BUSY, //ADC is in process of gathering samples
	ADC_STATE_ARMED  //ADC is empty and waiting for next pulse
} adc_state_t;

typedef enum adc_data_flag_t
{
	ADC_DATA_EMPTY,
	ADC_DATA_READY
} adc_data_flag_t;

typedef struct adc0_ch_result_t
{
	adc_data_flag_t adc_data_flag;
	uint16_t result;
	uint16_t peak_result;          //peak result not cleared
} adc0_ch_result_t;

typedef struct adc1_ch_result_t
{
	adc_data_flag_t adc_data_flag;
	uint16_t result;
} adc1_ch_result_t;

typedef struct adc0_data_t
{
	volatile adc0_count_t i_sample_cnt;

	/* i_sample_cnt_max
	can calculate rough sample rate by multiplying this by PWM freq
	if this exceeds the ADC0_DMA_BUFFER_LEN an assert_failed will be triggered
	*/
	volatile adc0_count_t i_sample_cnt_max;
	volatile adc_state_t  i_state;
	volatile wm_flg_t     i_wm_flg;                   //bit encoded channel e.g. BIT0 -> ADC_CH_CURRENT_SENSE_1
	adc0_ch_result_t      ch_result[ADC0_NUM_CH];
} adc0_data_t;

typedef struct adc1_data_t
{
	volatile adc1_ch_result_t i_ch_result[ADC1_NUM_CH];
} adc1_data_t;

/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
extern const volatile ADC_SEQSTATUS_Type    g_drvr_adc0_dma_seq_win;                         //8bit
extern const volatile ADC_SEQSTATUS_Type    g_drvr_adc0_dma_seq_buf[ADC0_DMA_BUFFER_LEN];    //8bit
extern const volatile ADC_RESULT_Type       g_drvr_adc0_dma_res_buf[ADC0_DMA_BUFFER_LEN];    //16bit

extern const adc0_data_t * const adc0_data_ptr;
extern const adc1_data_t * const adc1_data_ptr;

/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void              drvr_adc0_init               (const uint16_t winmon_thres);
drvr_adc_return_t drvr_adc0_get_res            (uint16_t * const result, const drvr_adc0_ch_t ch);
uint16_t          drvr_adc0_get_pk             (const drvr_adc0_ch_t ch);
wm_flg_t          drvr_adc0_get_wm             (void);
void              drvr_adc0_pulse_isr_task     (void);
uint8_t           drvr_adc0_main_task          (void);
void              drvr_adc0_wm_isr_task        (void);
void              drvr_adc0_set_gain_corr      (const float gain_corr_value);
void              drvr_adc0_set_offset_corr    (const int offset_corr_val);
void              drvr_adc0_set_corr_en        (const uint8_t on_off);
float             drvr_adc0_get_gain_corr_val  (void);
int               drvr_adc0_get_offset_corr_val(void);
uint8_t           drvr_adc0_get_corr_en        (void);

void              drvr_adc1_init               (void);
drvr_adc_return_t drvr_adc1_get_res            (uint16_t * const result, const drvr_adc1_ch_t ch);
void              drvr_adc1_fill_wait          (void);
uint16_t          drvr_adc1_get_res_now        (const drvr_adc1_ch_t ch);

#endif /* DRVR_ADC_H_ */
