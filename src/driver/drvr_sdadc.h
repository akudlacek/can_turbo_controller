/*
 * drvr_sdadc.h
 *
 * Created: 3/1/2018 12:26:48 PM
 *  Author: akudlacek
 */ 


#ifndef DRVR_SDADC_H_
#define DRVR_SDADC_H_


#include <stdint.h>


/******************************************************************************
* Defines
******************************************************************************/
typedef enum drvr_sdadc_return_t
{
	SDADC_SUCCESS,
	SDADC_BUSY
} drvr_sdadc_return_t;


/******************************************************************************
* Prototypes
******************************************************************************/
void                drvr_sdadc_init      (const int32_t win_mon_lo_thres);
drvr_sdadc_return_t drvr_sdadc_get       (int32_t * const result);
uint8_t             drvr_sdadc_winmon    (void);
void                drvr_sdadc_pulse_task(void);


#endif /* DRVR_SDADC_H_ */
