/*
 * drvr_divas.c
 *
 * Created: 11/29/2017 8:59:57 AM
 *  Author: akudlacek
 */ 


#include "drvr_divas.h"

#include "sam.h"


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Unsigned Divide
*
*  \note
******************************************************************************/
divas_unsigned_result_t drvr_divas_udiv(const uint32_t dividend, const uint32_t divisor)
{
	divas_unsigned_result_t result;
	
	DIVAS->CTRLA.bit.DLZ         =           0; //Enable leading zero optimization; 32-bit division takes 2-16 cycles.
	DIVAS->CTRLA.bit.SIGNED      =           0; //Unsigned division.
	
	DIVAS->DIVIDEND.bit.DIVIDEND =    dividend;
	DIVAS->DIVISOR.bit.DIVISOR   =     divisor; //starts the divide function
	
	while(DIVAS->STATUS.bit.BUSY);
	
	result.result           = DIVAS->RESULT.bit.RESULT;
	result.remainder        =       DIVAS->REM.bit.REM;
	result.div_by_zero_flag =    DIVAS->STATUS.bit.DBZ;
	
	return result;
}

/******************************************************************************
*  \brief Signed Divide
*
*  \note
******************************************************************************/
divas_signed_result_t drvr_divas_sdiv(const int32_t dividend, const int32_t divisor)
{
	divas_signed_result_t result;
	
	DIVAS->CTRLA.bit.DLZ         =           0; //Enable leading zero optimization; 32-bit division takes 2-16 cycles.
	DIVAS->CTRLA.bit.SIGNED      =           1; //Signed division.
	
	DIVAS->DIVIDEND.bit.DIVIDEND =    dividend;
	DIVAS->DIVISOR.bit.DIVISOR   =     divisor; //starts the divide function
	
	while(DIVAS->STATUS.bit.BUSY);
	
	result.result           = DIVAS->RESULT.bit.RESULT;
	result.remainder        =       DIVAS->REM.bit.REM;
	result.div_by_zero_flag =    DIVAS->STATUS.bit.DBZ;
	
	return result;
}

/******************************************************************************
*  \brief Unsigned Square Root
*
*  \note
******************************************************************************/
divas_unsigned_result_t drvr_divas_sqrt(const uint32_t value)
{
	divas_unsigned_result_t result;
	
	DIVAS->CTRLA.bit.DLZ         =           0; //Enable leading zero optimization; 32-bit division takes 2-16 cycles.
	DIVAS->CTRLA.bit.SIGNED      =           0; //Unsigned division.
	
	DIVAS->SQRNUM.bit.SQRNUM     =       value; //starts the square root function
	
	while(DIVAS->STATUS.bit.BUSY);
	
	result.result           = DIVAS->RESULT.bit.RESULT;
	result.remainder        =       DIVAS->REM.bit.REM;
	result.div_by_zero_flag =    DIVAS->STATUS.bit.DBZ;
	
	return result;
}
