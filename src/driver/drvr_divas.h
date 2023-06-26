/*
 * drvr_divas.h
 *
 * Created: 11/29/2017 9:00:06 AM
 *  Author: akudlacek
 */ 


#ifndef DRVR_DIVAS_H_
#define DRVR_DIVAS_H_


#include <stdint.h>


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
typedef struct divas_unsigned_result_t
{
	uint32_t result;
	uint32_t remainder;
	uint8_t div_by_zero_flag;
} divas_unsigned_result_t;

typedef struct divas_signed_result_t
{
	uint32_t result;
	uint32_t remainder;
	uint8_t div_by_zero_flag;
} divas_signed_result_t;


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
divas_unsigned_result_t drvr_divas_udiv(const uint32_t dividend, const uint32_t divisor);
divas_signed_result_t drvr_divas_sdiv(const int32_t dividend, const int32_t divisor);
divas_unsigned_result_t drvr_divas_sqrt(const uint32_t value);


#endif /* DRVR_DIVAS_H_ */