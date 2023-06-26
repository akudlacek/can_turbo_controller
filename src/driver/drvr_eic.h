/*
 * drvr_eic.h
 *
 * Created: 8/23/2017 4:57:53 PM
 *  Author: akudlacek
 */ 


#ifndef DRVR_EIC_H_
#define DRVR_EIC_H_


#include <stdint.h>


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void drvr_eic_init(void);
void drvr_eic_register_interrupt(void * const func_ptr, const uint8_t extint_num);

#endif /* DRVR_EIC_H_ */