/*
 * drvr_tcc.h
 *
 * Created: 8/18/2017 1:34:41 PM
 *  Author: akudlacek
 */


#ifndef DRVR_TCC_H_
#define DRVR_TCC_H_


#include <stdint.h>


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
typedef struct tcc0_data_t
{
	uint32_t top_val;
	uint32_t  cc_val[4];
	float   duty_pct[4];
} tcc0_data_t;


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
extern const tcc0_data_t * const tcc0_data_ptr;


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void drvr_tcc0_init           (const float freq_hz);
void drvr_tcc0_set_duty       (float duty_pct, const uint8_t cc_channel);
void drvr_tcc0_set_all_duty   (float duty_pct_hss_1, float duty_pct_hss_2, float duty_pct_hss_3, float duty_pct_hss_4);
void drvr_tcc0_set_duty_hss_1 (float duty_pct);
void drvr_tcc0_set_duty_hss_2 (float duty_pct);
void drvr_tcc0_set_duty_hss_3 (float duty_pct);
void drvr_tcc0_set_duty_hss_4 (float duty_pct);
void drvr_tcc0_set_freq_hz    (float freq_hz);
void drvr_tcc0_reset_fault    (void);


#endif /* DRVR_TCC_H_ */
