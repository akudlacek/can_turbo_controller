/*
 * drvr_sys.c
 *
 * Created: 5/8/2019 1:30:33 PM
 *  Author: akudlacek
 */ 


#include "drvr_sys.h"

#include "asf.h"

#include <stdio.h>

//#include "cli.h"
#include "drvr_clock.h"
#include "conf_clocks.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/


/**************************************************************************************************
*                                         LOCAL PROTOTYPES
*************************************************^************************************************/


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
static volatile uint32_t sys_tick = 0;

/*****GLOBAL******/
const volatile uint32_t * const g_sys_tick_ptr = &sys_tick;


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief 
*
*  \note
******************************************************************************/
inline void drvr_sys_init(void)
{
	//todo: this is hacked in if you change the clock freq this will break
	//SysTick_Config(*g_cpu_freq_hz_ptr / DRVR_SYS_SYSTICK_HZ);     //SysTick timer
	SysTick_Config(CONF_CLOCK_DPLL_OUTPUT_FREQUENCY / DRVR_SYS_SYSTICK_HZ);     //SysTick timer
	
	sys_tick = 0;
}

/**************************************************************************************************
*                                         LOCAL FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief 
*
*  \note
******************************************************************************/


/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
/******************************************************************************
*  \brief Interrupt for system tick
*
*  \note CRITICAL RESOURCES:
*        NEEDS PROTECTION
*            none
*        DOES NOT NEED PROTECTION:
*            sys_tick_ms
******************************************************************************/
void SysTick_Handler(void)
{
	sys_tick++;
}

/******************************************************************************
*  \brief interrupt for SYSTEM
*         PM         - Power Manager
*         MCLK       - Main Clock
*         OSCCTRL    - Oscillators Controller
*         OSC32KCTRL - 32kHz Oscillators Controller
*         SUPC       - Supply Controller
*         PAC        - Protection Access Controller
*
*  \note CRITICAL RESOURCES:
*        NEEDS PROTECTION
*            none
*        DOES NOT NEED PROTECTION:
*            none
******************************************************************************/
void SYSTEM_Handler(void)
{
	//char string[100];
	//uint32_t pac_interrupt_flag[4] = {0};
	//
	////PM not implemented
	////MCLK not implemented
	////OSCCTRL not implemented
	////OSC32KCTRL not implemented
	////SUPC not implemented
	//
	///*PAC interrupt flags*/
	//if(PAC->INTFLAGAHB.reg != 0)
	//{
		//pac_interrupt_flag[0] = PAC->INTFLAGAHB.reg;
		//PAC->INTFLAGAHB.reg = PAC_INTFLAGAHB_MASK; //clear all flags
	//}
	//if(PAC->INTFLAGA.reg != 0)
	//{
		//pac_interrupt_flag[1] = PAC->INTFLAGA.reg;
		//PAC->INTFLAGA.reg = PAC_INTFLAGA_MASK; //clear all flags
	//}
	//if(PAC->INTFLAGB.reg != 0)
	//{
		//pac_interrupt_flag[2] = PAC->INTFLAGB.reg;
		//PAC->INTFLAGB.reg = PAC_INTFLAGB_MASK; //clear all flags
	//}
	//if(PAC->INTFLAGC.reg != 0)
	//{
		//pac_interrupt_flag[3] = PAC->INTFLAGC.reg;
		//PAC->INTFLAGC.reg = PAC_INTFLAGC_MASK; //clear all flags
	//}
	//
	//snprintf(string, sizeof(string),
	//"****PAC****\r\n"
	//"AHB: %10lX\r\n"
	//"A:   %10lX\r\n"
	//"B:   %10lX\r\n"
	//"C:   %10lX\r\n",
	//pac_interrupt_flag[0],
	//pac_interrupt_flag[1],
	//pac_interrupt_flag[2],
	//pac_interrupt_flag[3]
	//);
	//cli_print_nl(string);
}
