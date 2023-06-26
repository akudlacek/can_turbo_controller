/*
* drvr_dma.c
*
* Created: 11/14/2017 8:45:38 AM
*  Author: akudlacek
*/
/**************************************************************************************************
*                                           CH ALLOCATION
* CH0 will be used for ADC0's WINMON event
*     This captures the ADC channel on which an overcurrent threshold was reached.
*
* CH1 will be used for ADC0's SEQSTATUS (Sequence Status) register
*     This stores the ADC channel for each ADC result for current sense
*
* CH2 will be used for ADC0's RESULT register
*     This stores the ADC result for current sense
*
* CH0 needs to be the highest priority in order to get the overcurrent source channel
* CH1 needs to be higher priority then CH2 in order to ensure the order of ADC results and the channel within the buffer
*
* NOTE: The event system is only available on the least significant DMA channels (CH0 - CH3)
*************************************************^************************************************/

#include "drvr_dma.h"

#include "sam.h"
#include "drvr_adc.h"
#include "interrupt_sam_nvic.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
//Use for incremented source or address, use value from register and not what that value represents
#define SRC_ADDR(ADDR, BTCNT, BEATSIZE, STEPSIZE, STEPSEL)  ((uint32_t)ADDR + BTCNT * (BEATSIZE + 1) * (STEPSEL ? 1 << STEPSIZE : 1))
#define DST_ADDR(ADDR, BTCNT, BEATSIZE, STEPSIZE, STEPSEL)  ((uint32_t)ADDR + BTCNT * (BEATSIZE + 1) * (STEPSEL ? 1 : 1 << STEPSIZE))

#define CONF_MAX_USED_CHANNEL_NUM     3

extern void assert_failed(char const *file, int line);


/**************************************************************************************************
*                                         LOCAL PROTOTYPES
*************************************************^************************************************/
//descriptors define the source and destination of the data as well as the size of that data among other things.
//EVSYS outputs are also defined here
static inline void conf_dma_ch_0_descriptors(void);
static inline void conf_dma_ch_1_descriptors(void);
static inline void conf_dma_ch_2_descriptors(void);


/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
COMPILER_ALIGNED(16)
static DmacDescriptor descriptor_section[CONF_MAX_USED_CHANNEL_NUM];

COMPILER_ALIGNED(16)
static DmacDescriptor write_back_section[CONF_MAX_USED_CHANNEL_NUM];


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief DMA CRC-16 (CRC-CCITT) EXTERNAL (IO interface) init
*
*  \note
******************************************************************************/
void drvr_dma_crc_16_init(void)
{
	//CTRL
	DMAC->CTRL.bit.CRCENABLE = 0; //disable crc

	//Clear CRCBUSY flag
	DMAC->CRCSTATUS.bit.CRCBUSY = 1;

	//CRCCTRL
	DMAC->CRCCTRL.bit.CRCSRC      = DMAC_CRCCTRL_CRCSRC_IO_Val;             //I/O interface
	DMAC->CRCCTRL.bit.CRCPOLY     = DMAC_CRCCTRL_CRCPOLY_CRC16_Val;         //CRC-16 (CRC-CCITT)
	DMAC->CRCCTRL.bit.CRCBEATSIZE = DMAC_CRCCTRL_CRCBEATSIZE_BYTE_Val;      //BYTE
}

/******************************************************************************
*  \brief DMA CRC-16 (CRC-CCITT) EXTERNAL (IO interface)
*
*  \note for use outside of DMA peripheral
*        CRC16_CCIT_ZERO
*        CRC-16 (CRC-CCITT)
*        Calculator: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
******************************************************************************/
uint16_t drvr_dma_crc_16(const uint8_t * const message, const uint32_t num_bytes)
{
	uint16_t crc_result = 0;
	uint32_t index = 0;

	//Clear old result
	DMAC->CRCCHKSUM.bit.CRCCHKSUM = 0;

	//CTRL - ENABLE CRC
	DMAC->CTRL.bit.CRCENABLE = 1;

	while(index < num_bytes)
	{
		DMAC->CRCDATAIN.bit.CRCDATAIN = (uint32_t)message[index];
		index++;
	}

	//Clear CRCBUSY flag
	DMAC->CRCSTATUS.bit.CRCBUSY = 1;

	//Result
	crc_result = (uint16_t)DMAC->CRCCHKSUM.bit.CRCCHKSUM;

	//CTRL - DISABLE CRC
	DMAC->CTRL.bit.CRCENABLE = 0;

	return crc_result;
}

/******************************************************************************
*  \brief DMAC init
*
*  \note
******************************************************************************/
void drvr_dma_init(void)
{
	cpu_irq_enter_critical();

	/*configure descriptors*/
	conf_dma_ch_0_descriptors();
	conf_dma_ch_1_descriptors();
	conf_dma_ch_2_descriptors();


	/*configure DMAC*/
	//CTRL
	DMAC->CTRL.bit.LVLEN0       =   0; //Priority Level 0 Enable (LOWEST)
	DMAC->CTRL.bit.LVLEN1       =   1; //Priority Level 1 Enable
	DMAC->CTRL.bit.LVLEN2       =   1; //Priority Level 2 Enable
	DMAC->CTRL.bit.LVLEN3       =   1; //Priority Level 3 Enable (HIGHEST)
	DMAC->CTRL.bit.CRCENABLE    =   0; //CRC Enable

	//DBGCTRL
	DMAC->DBGCTRL.bit.DBGRUN    =   0; //Debug Run

	//QOSCTRL
	DMAC->QOSCTRL.bit.DQOS      = 0x3; //Data Transfer Quality of Service
	DMAC->QOSCTRL.bit.FQOS      = 0x3; //Fetch Quality of Service
	DMAC->QOSCTRL.bit.WRBQOS    = 0x3; //Write-Back Quality of Service

	//PRICTRL0
	DMAC->PRICTRL0.bit.RRLVLEN0 =   0; //Level 0 Round-Robin Scheduling Enable
	DMAC->PRICTRL0.bit.RRLVLEN1 =   0; //Level 1 Round-Robin Scheduling Enable
	DMAC->PRICTRL0.bit.RRLVLEN2 =   0; //Level 2 Round-Robin Scheduling Enable
	DMAC->PRICTRL0.bit.RRLVLEN3 =   0; //Level 3 Round-Robin Scheduling Enable

	//BASEADDR
	DMAC->BASEADDR.bit.BASEADDR = (uint32_t)descriptor_section;

	//WRBADDR
	DMAC->WRBADDR.bit.WRBADDR   = (uint32_t)write_back_section;

	//Enable module
	DMAC->CTRL.bit.DMAENABLE    =   1; //enable the DMA module.

	/*************************CH0*************************/
	//CHID
	DMAC->CHID.bit.ID           =   0;

	//CHCTRLA
	DMAC->CHCTRLA.bit.RUNSTDBY  =   0;
	DMAC->CHCTRLA.bit.ENABLE    =   0;
	DMAC->CHCTRLA.bit.SWRST     =   0;

	//CHCTRLB
	DMAC->CHCTRLB.bit.CMD       =    0;
	DMAC->CHCTRLB.bit.TRIGACT   =  0x2; //BEAT
	DMAC->CHCTRLB.bit.TRIGSRC   = 0x00; //DISABLE Only software/event triggers
	DMAC->CHCTRLB.bit.LVL       =  0x3; //Channel Priority Level 3
	DMAC->CHCTRLB.bit.EVOE      =    0;
	DMAC->CHCTRLB.bit.EVIE      =    1; //Channel Event Input Enable
	DMAC->CHCTRLB.bit.EVACT     =  0x1; //TRIG Normal Transfer and Conditional Transfer on Strobe trigger

	//CHINTENSET
	DMAC->CHINTENSET.bit.SUSP   =    1; //Channel Suspend Interrupt Enable
	DMAC->CHINTENSET.bit.TCMPL  =    1; //Channel Transfer Complete Interrupt Enable
	DMAC->CHINTENSET.bit.TERR   =    1; //Channel Transfer Error Interrupt Enable

	/*************************CH1*************************/
	//CHID
	DMAC->CHID.bit.ID           =   1;

	//CHCTRLA
	DMAC->CHCTRLA.bit.RUNSTDBY  =   0;
	DMAC->CHCTRLA.bit.ENABLE    =   0;
	DMAC->CHCTRLA.bit.SWRST     =   0;

	//CHCTRLB
	DMAC->CHCTRLB.bit.CMD       =    0;
	DMAC->CHCTRLB.bit.TRIGACT   =  0x2; //BEAT
	DMAC->CHCTRLB.bit.TRIGSRC   = 0x2A; //ADC0 RESRDY
	DMAC->CHCTRLB.bit.LVL       =  0x2; //Channel Priority Level 2
	DMAC->CHCTRLB.bit.EVOE      =    0;
	DMAC->CHCTRLB.bit.EVIE      =    0;
	DMAC->CHCTRLB.bit.EVACT     =    0;

	//CHINTENSET
	DMAC->CHINTENSET.bit.SUSP   =    1; //Channel Suspend Interrupt Enable
	DMAC->CHINTENSET.bit.TCMPL  =    1; //Channel Transfer Complete Interrupt Enable
	DMAC->CHINTENSET.bit.TERR   =    1; //Channel Transfer Error Interrupt Enable

	/*************************CH2*************************/
	//CHID
	DMAC->CHID.bit.ID           =   2;

	//CHCTRLA
	DMAC->CHCTRLA.bit.RUNSTDBY  =   0;
	DMAC->CHCTRLA.bit.ENABLE    =   0;
	DMAC->CHCTRLA.bit.SWRST     =   0;

	//CHCTRLB
	DMAC->CHCTRLB.bit.CMD       =    0;
	DMAC->CHCTRLB.bit.TRIGACT   =  0x2; //BEAT
	DMAC->CHCTRLB.bit.TRIGSRC   = 0x2A; //ADC0 RESRDY
	DMAC->CHCTRLB.bit.LVL       =  0x1; //Channel Priority Level 1
	DMAC->CHCTRLB.bit.EVOE      =    1; //Channel Event Output Enable
	DMAC->CHCTRLB.bit.EVIE      =    0;
	DMAC->CHCTRLB.bit.EVACT     =    0;

	//CHINTENSET
	DMAC->CHINTENSET.bit.SUSP   =    1; //Channel Suspend Interrupt Enable
	DMAC->CHINTENSET.bit.TCMPL  =    1; //Channel Transfer Complete Interrupt Enable
	DMAC->CHINTENSET.bit.TERR   =    1; //Channel Transfer Error Interrupt Enable

	cpu_irq_leave_critical();
}

/******************************************************************************
*  \brief Start DMA channel
*
*  \note
******************************************************************************/
void drvr_dma_start(const drvr_dma_ch_t channel)
{
	cpu_irq_enter_critical();

	DMAC->CHID.bit.ID = (uint8_t)channel;
	DMAC->CHCTRLA.bit.ENABLE = 1;

	cpu_irq_leave_critical();
}

/******************************************************************************
*  \brief DMA abort
*
*  \note
******************************************************************************/
uint32_t drvr_dma_abort(const drvr_dma_ch_t channel)
{
	uint32_t write_size;
	uint32_t total_size;
	uint32_t transfered_size;

	cpu_irq_enter_critical();

	DMAC->CHID.reg = (uint8_t)channel;
	DMAC->CHCTRLA.reg = 0;

	cpu_irq_leave_critical();

	/* Get transferred size */
	total_size = descriptor_section[(uint8_t)channel].BTCNT.reg;
	write_size = write_back_section[(uint8_t)channel].BTCNT.reg;
	transfered_size = total_size - write_size;

	return transfered_size;
}

/******************************************************************************
*  \brief DMA get current transfer size
*
*  \note
******************************************************************************/
uint32_t drvr_dma_get_xfr_size(const drvr_dma_ch_t channel)
{
	uint32_t write_size;
	uint32_t total_size;
	uint32_t transfered_size;

	/* Get transferred size */
	total_size = descriptor_section[(uint8_t)channel].BTCNT.reg;
	write_size = write_back_section[(uint8_t)channel].BTCNT.reg;
	transfered_size = total_size - write_size;

	return transfered_size;
}


/**************************************************************************************************
*                                         LOCAL FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Configure descriptor for CH0
*
*  \note
******************************************************************************/
static inline void conf_dma_ch_0_descriptors(void)
{
	/*************************descriptor_section*************************/
	//BTCTRL
	descriptor_section[0].BTCTRL.bit.STEPSIZE   = 0x0; //X1 Next ADDR = ADDR + (BEATSIZE+1) * 1
	descriptor_section[0].BTCTRL.bit.STEPSEL    =   0; //Step size settings apply to the destination address
	descriptor_section[0].BTCTRL.bit.DSTINC     =   0; //The Destination Address Increment is disabled
	descriptor_section[0].BTCTRL.bit.SRCINC     =   0; //The Source Address Increment is disabled
	descriptor_section[0].BTCTRL.bit.BEATSIZE   = 0x0; //8-bit bus transfer
	descriptor_section[0].BTCTRL.bit.BLOCKACT   = 0x0; //Channel will be disabled if it is the last block transfer in the transaction
	descriptor_section[0].BTCTRL.bit.EVOSEL     = 0x0; //Event generation disabled
	descriptor_section[0].BTCTRL.bit.VALID      =   1; //The descriptor is valid

	//BTCNT
	descriptor_section[0].BTCNT.bit.BTCNT       =   1;

	//SRCADDR
	descriptor_section[0].SRCADDR.bit.SRCADDR   = (uint32_t)(&ADC0->SEQSTATUS.reg); //ADC0's Sequence Status register

	//DSTADDR - not using DST_ADDR because DSTINC = 0
	descriptor_section[0].DSTADDR.bit.DSTADDR   = (uint32_t)(&g_drvr_adc0_dma_seq_win); //buffer address

	//DESCADDR
	descriptor_section[0].DESCADDR.bit.DESCADDR = 0;
}

/******************************************************************************
*  \brief Configure descriptor for CH1
*
*  \note
******************************************************************************/
static inline void conf_dma_ch_1_descriptors(void)
{
	/*************************descriptor_section*************************/
	//BTCTRL
	descriptor_section[1].BTCTRL.bit.STEPSIZE   = 0x0; //X1 Next ADDR = ADDR + (BEATSIZE+1) * 1
	descriptor_section[1].BTCTRL.bit.STEPSEL    =   0; //Step size settings apply to the destination address
	descriptor_section[1].BTCTRL.bit.DSTINC     =   1; //The Destination Address Increment is enabled
	descriptor_section[1].BTCTRL.bit.SRCINC     =   0; //The Source Address Increment is disabled
	descriptor_section[1].BTCTRL.bit.BEATSIZE   = 0x0; //8-bit bus transfer
	descriptor_section[1].BTCTRL.bit.BLOCKACT   = 0x0; //Channel will be disabled if it is the last block transfer in the transaction
	descriptor_section[1].BTCTRL.bit.EVOSEL     = 0x0; //Event generation disabled
	descriptor_section[1].BTCTRL.bit.VALID      =   1; //The descriptor is valid

	//BTCNT
	descriptor_section[1].BTCNT.bit.BTCNT       = ADC0_DMA_BUFFER_LEN;

	//SRCADDR
	descriptor_section[1].SRCADDR.bit.SRCADDR   = (uint32_t)(&ADC0->SEQSTATUS.reg); //ADC0's Sequence Status register

	//DSTADDR
	descriptor_section[1].DSTADDR.bit.DSTADDR   = DST_ADDR(g_drvr_adc0_dma_seq_buf, ADC0_DMA_BUFFER_LEN, 0x0, 0x0, 0); //buffer address

	//DESCADDR
	descriptor_section[1].DESCADDR.bit.DESCADDR = 0;
}

/******************************************************************************
*  \brief Configure descriptor for CH2
*
*  \note
******************************************************************************/
static inline void conf_dma_ch_2_descriptors(void)
{
	/*************************descriptor_section*************************/
	//BTCTRL
	descriptor_section[2].BTCTRL.bit.STEPSIZE   = 0x0; //X1 Next ADDR = ADDR + (BEATSIZE+1) * 1
	descriptor_section[2].BTCTRL.bit.STEPSEL    =   0; //Step size settings apply to the destination address
	descriptor_section[2].BTCTRL.bit.DSTINC     =   1; //The Destination Address Increment is enabled
	descriptor_section[2].BTCTRL.bit.SRCINC     =   0; //The Source Address Increment is disabled
	descriptor_section[2].BTCTRL.bit.BEATSIZE   = 0x1; //16-bit bus transfer
	descriptor_section[2].BTCTRL.bit.BLOCKACT   = 0x0; //Channel will be disabled if it is the last block transfer in the transaction
	descriptor_section[2].BTCTRL.bit.EVOSEL     = 0x3; //Event Output Selection - BEAT
	descriptor_section[2].BTCTRL.bit.VALID      =   1; //The descriptor is valid

	//BTCNT
	descriptor_section[2].BTCNT.bit.BTCNT       = ADC0_DMA_BUFFER_LEN;

	//SRCADDR
	descriptor_section[2].SRCADDR.bit.SRCADDR   = (uint32_t)(&ADC0->RESULT.reg); //ADC0 result register

	//DSTADDR
	descriptor_section[2].DSTADDR.bit.DSTADDR   = DST_ADDR(g_drvr_adc0_dma_res_buf, ADC0_DMA_BUFFER_LEN, 0x1, 0x0, 0); //buffer address

	//DESCADDR
	descriptor_section[2].DESCADDR.bit.DESCADDR = 0;
}


/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
/******************************************************************************
*  \brief DMAC Interrupt
*
*  \note
******************************************************************************/
void DMAC_Handler(void)
{
	cpu_irq_enter_critical();

	/*CH0 Interrupts Pending*/
	if(DMAC->INTSTATUS.bit.CHINT0)
	{
		DMAC->CHID.bit.ID = 0; //select CH

		/*Channel Suspend*/
		if(DMAC->CHINTFLAG.bit.SUSP)
		{
			assert_failed(__FILE__, __LINE__); //Shouldn't happen

			DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_SUSP; //clear flag
		}

		/*Channel Transfer Complete*/
		if(DMAC->CHINTFLAG.bit.TCMPL)
		{
			//An overcurrent has occurred and the channel it happened on has been captured
			drvr_adc0_wm_isr_task();

			DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL; //clear flag
		}

		/*Channel Transfer Error*/
		if(DMAC->CHINTFLAG.bit.TERR)
		{
			assert_failed(__FILE__, __LINE__); //Shouldn't happen

			DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TERR; //clear flag
		}
	}

	/*CH1 Interrupts Pending*/
	if(DMAC->INTSTATUS.bit.CHINT1)
	{
		DMAC->CHID.bit.ID = 1; //select CH

		/*Channel Suspend*/
		if(DMAC->CHINTFLAG.bit.SUSP)
		{
			assert_failed(__FILE__, __LINE__); //Shouldn't happen

			DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_SUSP; //clear flag
		}

		/*Channel Transfer Complete*/
		if(DMAC->CHINTFLAG.bit.TCMPL)
		{
			//This indicates the buffer has been exceeded, check ADC0_DMA_BUFFER_LEN
			assert_failed(__FILE__, __LINE__);

			DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL; //clear flag
		}

		/*Channel Transfer Error*/
		if(DMAC->CHINTFLAG.bit.TERR)
		{
			assert_failed(__FILE__, __LINE__); //Shouldn't happen

			DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TERR; //clear flag
		}
	}

	/*CH2 Interrupts Pending*/
	if(DMAC->INTSTATUS.bit.CHINT2)
	{
		DMAC->CHID.bit.ID = 2; //select CH

		/*Channel Suspend*/
		if(DMAC->CHINTFLAG.bit.SUSP)
		{
			assert_failed(__FILE__, __LINE__); //Shouldn't happen

			DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_SUSP; //clear flag
		}

		/*Channel Transfer Complete*/
		if(DMAC->CHINTFLAG.bit.TCMPL)
		{
			//This indicates the buffer has been exceeded, check ADC0_DMA_BUFFER_LEN
			assert_failed(__FILE__, __LINE__);

			DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL; //clear flag
		}

		/*Channel Transfer Error*/
		if(DMAC->CHINTFLAG.bit.TERR)
		{
			assert_failed(__FILE__, __LINE__); //Shouldn't happen

			DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TERR; //clear flag
		}
	}

	cpu_irq_leave_critical();
}
