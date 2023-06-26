/*
 * drvr_eic.c
 *
 * Created: 8/23/2017 4:57:16 PM
 *  Author: akudlacek
 */


#include "drvr_eic.h"

#include "sam.h"
#include "erp4.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define EXTINT_MASK(extint_num) (1ul << extint_num)

//EIC waits
#define EIC_SYNC_WAIT(syncbusy_flags) while((EIC->SYNCBUSY.reg & syncbusy_flags) != 0)

//SENSE Input Sense Configuration
typedef enum eic_sense_t
{
	EIC_SEN_NONE = 0x0, //No detection
	EIC_SEN_RISE = 0x1, //Rising-edge detection
	EIC_SEN_FALL = 0x2, //Falling-edge detection
	EIC_SEN_BOTH = 0x3, //Both-edge detection
	EIC_SEN_HIGH = 0x4, //High-level detection
	EIC_SEN_LOW  = 0x5  //Low-level detection
} eic_sense_t;


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
static void (*extint_interrupt_ptr[32])(void);


/**************************************************************************************************
*                                         LOCAL PROTOTYPES
*************************************************^************************************************/
static inline void interrupt_flag(uint8_t extint_num);
static inline void eic_config(uint8_t extint_num, uint8_t filten, eic_sense_t sense);


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Init EIC peripheral
*
*  \note Setting up only EIC's used
******************************************************************************/
void drvr_eic_init(void)
{
	//todo: make more modular without need for hard coded init for pins.
	/*Configure EIC*/
	EIC->CTRLA.bit.ENABLE = 0; //The EIC is disabled.
	EIC_SYNC_WAIT(EIC_SYNCBUSY_ENABLE);

	//1. Enable CLK_EIC_APB - done in drvr_gclk_init()
	EIC->CTRLA.bit.CKSEL = 1; //The EIC is clocked by CLK_ULP32K (LOW POWER)

	//The EXTINT channel below is edge detection is asynchronously operated.
	EIC->ASYNCH.bit.ASYNCH   |= EXTINT_MASK(EXTINT_OPEN_LOAD_SENSE_1) |
	                            EXTINT_MASK(EXTINT_OPEN_LOAD_SENSE_2) |
								EXTINT_MASK(EXTINT_OPEN_LOAD_SENSE_3) |
								EXTINT_MASK(EXTINT_OPEN_LOAD_SENSE_4);

	eic_config(EXTINT_OPEN_LOAD_SENSE_1, 0, EIC_SEN_FALL);
	eic_config(EXTINT_OPEN_LOAD_SENSE_2, 0, EIC_SEN_FALL);
	eic_config(EXTINT_OPEN_LOAD_SENSE_3, 0, EIC_SEN_FALL);
	eic_config(EXTINT_OPEN_LOAD_SENSE_4, 0, EIC_SEN_FALL);

	EIC->CTRLA.bit.ENABLE |= 1;         //The EIC is enabled.
	EIC_SYNC_WAIT(EIC_SYNCBUSY_ENABLE); //Wait for Write synchronization for CTRLA.ENABLE bit to complete.
}

/******************************************************************************
*  \brief Register a function to be used for EXTINT interrupt
*
*  \note 32 are possible although i believe ATSAMC21J18A doesn't use them all
*        also make sure they are actually checked in the EIC_Handler
******************************************************************************/
void drvr_eic_register_interrupt(void * const func_ptr, const uint8_t extint_num)
{
	extint_interrupt_ptr[extint_num] = func_ptr;
	EIC->INTENSET.bit.EXTINT = EXTINT_MASK(extint_num); //The external interrupt 'extint_num' is enabled.
}


/**************************************************************************************************
*                                         LOCAL FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Runs interrupt code registered in func ptr array if flag present
*
*  \note Use for each interrupt used
******************************************************************************/
static inline void interrupt_flag(uint8_t extint_num)
{
	/*IF FLAG PRESENT*/
	if(EIC->INTFLAG.reg & EXTINT_MASK(extint_num))
	{
		extint_interrupt_ptr[extint_num](); //run extint function
		EIC->INTFLAG.reg = EXTINT_MASK(extint_num); //clear flag
	}
}

/******************************************************************************
*  \brief Sets up config
*
*  \note
******************************************************************************/
static inline void eic_config(uint8_t extint_num, uint8_t filten, eic_sense_t sense)
{
	uint8_t config_num;
	uint8_t offset;

	//input constraints.
	extint_num &= 15;
	filten     &= 1;
	sense      &= 7;

	//The breakdown for which reg controls which EXTINT
	//CONFIG[0] for EXTINT[0] - EXTINT[7]
	//          FILTEN0 & SENSE0 for EXTINT[0]
	//          FILTEN1 & SENSE1 for EXTINT[1]
	//          FILTEN2 & SENSE2 for EXTINT[2]
	//          FILTEN3 & SENSE3 for EXTINT[3]
	//          FILTEN4 & SENSE4 for EXTINT[4]
	//          FILTEN5 & SENSE5 for EXTINT[5]
	//          FILTEN6 & SENSE6 for EXTINT[6]
	//          FILTEN7 & SENSE7 for EXTINT[7]
	//CONFIG[1] for EXTINT[8] - EXTINT[15]
	//          FILTEN0 & SENSE0 for EXTINT[8]
	//          FILTEN1 & SENSE1 for EXTINT[9]
	//          FILTEN2 & SENSE2 for EXTINT[10]
	//          FILTEN3 & SENSE3 for EXTINT[11]
	//          FILTEN4 & SENSE4 for EXTINT[12]
	//          FILTEN5 & SENSE5 for EXTINT[13]
	//          FILTEN6 & SENSE6 for EXTINT[14]
	//          FILTEN7 & SENSE7 for EXTINT[15]

	config_num = (extint_num / 8);
	offset     = (extint_num % 8) * 4;

	/*CONFIG FOR EXTINT*/
	EIC->CONFIG[config_num].reg |= ((uint32_t)filten << (offset + 3)) | ((uint32_t)sense << offset);
}

/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
/******************************************************************************
*  \brief Interrupt for EIC
*
*  \note Only the used interrupt are checked so registering more may cause
*        problems
*
*        CRITICAL RESOURCES:
*        NEEDS PROTECTION
*            interrupt_flag - depending on registered interrupt
*        DOES NOT NEED PROTECTION:
*
******************************************************************************/
void EIC_Handler(void)
{
	//not using anymore but left for example
	//interrupt_flag(EXTINT_OPEN_LOAD_SENSE_1);
	//interrupt_flag(EXTINT_OPEN_LOAD_SENSE_2);
	//interrupt_flag(EXTINT_OPEN_LOAD_SENSE_3);
	//interrupt_flag(EXTINT_OPEN_LOAD_SENSE_4);
}
