/*
 * drvr_nvic.c
 *
 * Created: 11/28/2017 10:40:27 AM
 *  Author: akudlacek
 */


#include "drvr_nvic.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
typedef struct drvr_nvic_data_t
{
	volatile uint32_t drvr_nvic_irq_critical_section_counter;
	volatile uint8_t drvr_nvic_irq_prev_interrupt_state;
} drvr_nvic_data_t;

extern void assert_failed (char const *file, int line);

/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
static volatile drvr_nvic_data_t drvr_nvic_data[PERIPH_COUNT_IRQn];


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief
*
*  \note
******************************************************************************/
void drvr_nvic_init(void)
{
	/*Set interrupt priority*/
	/*There are 4 priority levels (0-3) with 0 being the highest*/

	//0
	NVIC_SetPriority(TCC0_IRQn, 0);

	//1
	NVIC_SetPriority(SERCOM4_IRQn, 1); //was 3, moved up to handle com faster

	//2
	//NVIC_SetPriority(EIC_IRQn, 2); not used anymore
	//NVIC_SetPriority(ADC0_IRQn, 2); not used anymore
	NVIC_SetPriority(SysTick_IRQn, 2);
	NVIC_SetPriority(ADC1_IRQn, 2); //was 1, ADC1 processes slow changing or none critical events.

	//3
	NVIC_SetPriority(TSENS_IRQn, 3);
	NVIC_SetPriority(SYSTEM_IRQn, 3);
	NVIC_SetPriority(DMAC_IRQn, 3);


	/*Enable IRQs used*/
	//NVIC_EnableIRQ(ADC0_IRQn); not used anymore
	NVIC_EnableIRQ(ADC1_IRQn);
	NVIC_EnableIRQ(TCC0_IRQn);
	NVIC_EnableIRQ(SERCOM4_IRQn);
	//NVIC_EnableIRQ(EIC_IRQn); not used anymore
	NVIC_EnableIRQ(TSENS_IRQn);
	NVIC_EnableIRQ(SYSTEM_IRQn);
	NVIC_EnableIRQ(DMAC_IRQn);
}

/******************************************************************************
*  \brief Selective interrupt critical section entrance
*
*  \note use to disable peripheral interrupts with the, use
*        drvr_nvic_irq_leave_critical to leave
******************************************************************************/
void drvr_nvic_irq_enter_critical(const IRQn_Type IRQn_num)
{
	//Ensure valid interrupt number
	if(IRQn_num > PERIPH_COUNT_IRQn || IRQn_num < 0) return;

	if(drvr_nvic_data[IRQn_num].drvr_nvic_irq_critical_section_counter == 0)
	{
		if(NVIC_GetEnableIRQ(IRQn_num))
		{
			NVIC_DisableIRQ(IRQn_num);
			drvr_nvic_data[IRQn_num].drvr_nvic_irq_prev_interrupt_state = 1;
		}
		else
		{
			/* Make sure the to save the prev state as 0 */
			drvr_nvic_data[IRQn_num].drvr_nvic_irq_prev_interrupt_state = 0;
		}
	}

	drvr_nvic_data[IRQn_num].drvr_nvic_irq_critical_section_counter++;
}

/******************************************************************************
*  \brief Selective interrupt critical section exit
*
*  \note
******************************************************************************/
void drvr_nvic_irq_leave_critical(const IRQn_Type IRQn_num)
{
	//Ensure valid interrupt number
	if(IRQn_num > PERIPH_COUNT_IRQn || IRQn_num < 0) return;

	/* Check if the user is trying to leave a critical section when not in a critical section */
	if(drvr_nvic_data[IRQn_num].drvr_nvic_irq_critical_section_counter == 0)
	{
		assert_failed(__FILE__, __LINE__);
	}

	drvr_nvic_data[IRQn_num].drvr_nvic_irq_critical_section_counter--;

	/* Only enable interrupts when the counter reaches 0 and the state of the interrupt flag
	   was enabled when entering critical state */
	if((drvr_nvic_data[IRQn_num].drvr_nvic_irq_critical_section_counter == 0) && (drvr_nvic_data[IRQn_num].drvr_nvic_irq_prev_interrupt_state))
	{
		NVIC_EnableIRQ(IRQn_num);
	}
}
