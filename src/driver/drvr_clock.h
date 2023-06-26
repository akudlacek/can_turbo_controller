/*
 * drvr_clock.h
 *
 * Created: 11/13/2017 11:54:10 AM
 *  Author: akudlacek
 */ 


#ifndef DRVR_CLOCK_H_
#define DRVR_CLOCK_H_


#include <stdint.h>


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
extern const volatile uint32_t * const g_cpu_freq_hz_ptr;


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void     drvr_clock_init            (void);
uint32_t drvr_clock_get_gen0_freq_hz(void);
uint32_t drvr_clock_get_gen1_freq_hz(void);


#endif /* DRVR_CLOCK_H_ */
