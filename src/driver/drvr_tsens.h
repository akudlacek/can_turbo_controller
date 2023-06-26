/*
 * drvr_tsens.h
 *
 * Created: 2/28/2018 9:56:28 AM
 *  Author: akudlacek
 */ 


#ifndef DRVR_TSENS_H_
#define DRVR_TSENS_H_


#include <stdint.h>


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void   drvr_tsens_init(void);
int32_t drvr_tsens_get(void);


#endif /* DRVR_TSENS_H_ */
