/*
 * drvr_rstc.h
 *
 * Created: 10/2/2018 3:52:20 PM
 *  Author: akudlacek
 */ 


#ifndef DRVR_RSTC_H_
#define DRVR_RSTC_H_


#include <stdint.h>


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
uint8_t      drvr_rstc_reset_cause_get   (void);
const char * drvr_rstc_reset_cause_decode(uint8_t bit);


#endif /* DRVR_RSTC_H_ */