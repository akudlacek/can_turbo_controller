/*
 * drvr_nvic.h
 *
 * Created: 11/28/2017 10:40:36 AM
 *  Author: akudlacek
 */ 


#ifndef DRVR_NVIC_H_
#define DRVR_NVIC_H_

#include "sam.h"
#include "interrupt_sam_nvic.h"


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void drvr_nvic_init(void);
void drvr_nvic_irq_enter_critical(const IRQn_Type IRQn_num);
void drvr_nvic_irq_leave_critical(const IRQn_Type IRQn_num);


#endif /* DRVR_NVIC_H_ */
