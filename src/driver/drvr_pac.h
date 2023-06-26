/*
 * drvr_pac.h
 *
 * Created: 4/25/2019 9:47:41 AM
 *  Author: akudlacek
 */ 


#ifndef DRVR_PAC_H_
#define DRVR_PAC_H_


#include <stdint.h>
#include "sam.h"

/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
/**
* Retrieves the ID of a specified peripheral name, giving its peripheral bus
* location.
*
* \param[in] peripheral  Name of the peripheral instance
*
* \returns Bus ID of the specified peripheral instance.
 */
#define SYSTEM_PERIPHERAL_ID(peripheral)    ID_##peripheral

#define __no_inline                 __attribute__((noinline))

/** Status code error categories. */
typedef enum drvr_pac_status_categories_t
{
	DRVR_PAC_CATEGORY_OK                = 0x00,
	DRVR_PAC_CATEGORY_COMMON            = 0x10,
} drvr_pac_status_categories_t;

typedef enum drvr_pac_status_codes_t
{
	DRVR_PAC_OK                         = DRVR_PAC_CATEGORY_OK     | 0x00, //
	DRVR_PAC_ERR_INVALID_ARG            = DRVR_PAC_CATEGORY_COMMON | 0x07, //
} drvr_pac_status_codes_t;


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
/*name Peripheral Lock and Unlock*/
__no_inline drvr_pac_status_codes_t drvr_pac_lock       (const uint32_t peripheral_id, const uint32_t key);
__no_inline drvr_pac_status_codes_t drvr_pac_unlock     (const uint32_t peripheral_id, const uint32_t key);
__no_inline drvr_pac_status_codes_t drvr_pac_lock_always(const uint32_t peripheral_id, const uint32_t key);

/**
 * \brief Enable PAC interrupt.
 *
 * Enable PAC interrupt so can trigger execution on peripheral access error,
 * see \ref SYSTEM_Handler().
 *
 */
static inline void drvr_pac_enable_interrupt(void)
{
	PAC->INTENSET.reg = PAC_INTENSET_ERR;
}

/**
 * \brief Disable PAC interrupt.
 *
 * Disable PAC interrupt on peripheral access error.
 *
 */
static inline void drvr_pac_disable_interrupt(void)
{
	PAC->INTENCLR.reg = PAC_INTENCLR_ERR;
}

/**
 * \brief Enable PAC event output.
 *
 * Enable PAC event output on peripheral access error.
 *
 */
static inline void drvr_pac_enable_event(void)
{
	PAC->EVCTRL.reg = PAC_EVCTRL_ERREO;
}

/**
 * \brief Disable PAC event output.
 *
 * Disable PAC event output on peripheral access error.
 *
 */
static inline void drvr_pac_disable_event(void)
{
	PAC->EVCTRL.reg &= (~PAC_EVCTRL_ERREO);
}

#endif /* DRVR_PAC_H_ */
