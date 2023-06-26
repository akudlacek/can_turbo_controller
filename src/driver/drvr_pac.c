/*
 * drvr_pac.c
 *
 * Created: 4/25/2019 9:47:25 AM
 *  Author: akudlacek
 */ 


#include "drvr_pac.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/


/**************************************************************************************************
*                                         LOCAL PROTOTYPES
*************************************************^************************************************/


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
* \brief Lock a given peripheral's control registers.
*
* Locks a given peripheral's control registers, to deny write access to the
* peripheral to prevent accidental changes to the module's configuration.
*
* \warning Locking an already locked peripheral will cause a CPU
*          exception, and terminate program execution.
*
* \param[in] peripheral_id  ID for the peripheral to be locked, sourced via the
*                           \ref SYSTEM_PERIPHERAL_ID macro
* \param[in] key  Bitwise inverse of peripheral ID, used as key to
*                 reduce the chance of accidental locking. See
*                 \ref asfdoc_sam0_pac_bitwise_code
*
* \return Status of the peripheral lock procedure.
* \retval STATUS_OK                If the peripheral was successfully locked
* \retval STATUS_ERR_INVALID_ARG	If invalid argument(s) were supplied
******************************************************************************/
__no_inline drvr_pac_status_codes_t drvr_pac_lock(const uint32_t peripheral_id, const uint32_t key)
{
	/* Check if key is correct. */
	if (~peripheral_id != key) {
		//Assert(false);
		return DRVR_PAC_ERR_INVALID_ARG;
	}

	PAC->WRCTRL.reg = peripheral_id | PAC_WRCTRL_KEY(PAC_WRCTRL_KEY_SET_Val);

	return DRVR_PAC_OK;
}

/******************************************************************************
* \brief Lock a given peripheral's control registers until hardware reset.
*
* Locks a given peripheral's control registers, to deny write access to the
* peripheral to prevent accidental changes to the module's configuration.
* After lock, the only way to unlock is hardware reset.
*
* \warning Locking an already locked peripheral will cause a CPU
*          exception, and terminate program execution.
*
* \param[in] peripheral_id  ID for the peripheral to be locked, sourced via the
*                           \ref SYSTEM_PERIPHERAL_ID macro
* \param[in] key  Bitwise inverse of peripheral ID, used as key to
*                 reduce the chance of accidental locking. See
*                 \ref asfdoc_sam0_pac_bitwise_code
*
* \return Status of the peripheral lock procedure.
* \retval STATUS_OK                If the peripheral was successfully locked
* \retval STATUS_ERR_INVALID_ARG	If invalid argument(s) were supplied
******************************************************************************/
__no_inline drvr_pac_status_codes_t drvr_pac_lock_always(const uint32_t peripheral_id, const uint32_t key)
{
	/* Check if key is correct. */
	if (~peripheral_id != key) {
		//Assert(false);
		return DRVR_PAC_ERR_INVALID_ARG;
	}

	PAC->WRCTRL.reg = peripheral_id | PAC_WRCTRL_KEY(PAC_WRCTRL_KEY_SETLCK_Val);

	return DRVR_PAC_OK;
}

/******************************************************************************
* \brief Unlock a given peripheral's control registers.
*
* Unlocks a given peripheral's control registers, allowing write access to the
* peripheral so that changes can be made to the module's configuration.
*
* \warning Unlocking an already locked peripheral will cause a CUP
*          exception, and terminate program execution.
*
* \param[in] peripheral_id  ID for the peripheral to be unlocked, sourced via the
*                          \ref SYSTEM_PERIPHERAL_ID macro
* \param[in] key  Bitwise inverse of peripheral ID, used as key to
*                 reduce the chance of accidental unlocking. See
*                 \ref asfdoc_sam0_pac_bitwise_code
*
* \return Status of the peripheral unlock procedure.
* \retval STATUS_OK                If the peripheral was successfully locked
* \retval STATUS_ERR_INVALID_ARG	If invalid argument(s) were supplied
******************************************************************************/
__no_inline drvr_pac_status_codes_t drvr_pac_unlock(const uint32_t peripheral_id, const uint32_t key)
{
	/* Check if key is correct. */
	if (~peripheral_id != key) {
		//Assert(false);
		return DRVR_PAC_ERR_INVALID_ARG;
	}

	PAC->WRCTRL.reg = peripheral_id | PAC_WRCTRL_KEY(PAC_WRCTRL_KEY_CLR_Val);

	return DRVR_PAC_OK;
}


/**************************************************************************************************
*                                         LOCAL FUNCTIONS
*************************************************^************************************************/


/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
