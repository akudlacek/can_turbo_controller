/**
 * \file
 *
 * \brief SAM CAN basic Quick Start
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <string.h>
#include <conf_can.h>

#include "drvr_sys.h"
#include "cli.h"
#include "timer.h"

/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define CAN_RX_EXTENDED_FILTER_INDEX_0    0
#define CAN_RX_EXTENDED_FILTER_INDEX_1    1
#define CAN_RX_EXTENDED_FILTER_ID_0     0x18FFC600
#define CAN_RX_EXTENDED_FILTER_ID_0_BUFFER_INDEX     1
#define CAN_RX_EXTENDED_FILTER_ID_1     0x18FFC600

#define CAN_TX_BUFFER_INDEX    0


/**************************************************************************************************
*                                         LOCAL PROTOTYPES
*************************************************^************************************************/
static void configure_usart_cdc(void);
static void configure_can(void);
static void can_set_extended_filter_0(void);
//static void can_set_extended_filter_1(void);
static void can_send_extended_message(uint32_t id_value, uint8_t *data, uint32_t data_length);

static void cmd_line_init(void);
static int16_t rx_byte(void);
static void tx_strn(const char * const str);
static void set_pos(unsigned long pos);
static void configure_adc(void);

/**************************************************************************************************
*                                            VARIABLES
*************************************************^************************************************/
static struct usart_module cdc_instance;
static struct can_module can_instance;
static struct adc_module adc_instance;

static volatile uint32_t extended_receive_index = 0;
static struct can_rx_element_fifo_1 rx_element_fifo_1;
static struct can_rx_element_buffer rx_element_buffer;

uint16_t pos0 = 500;

/****CLI Command List****/
static const cli_command_t cli_cmd_list[] =
{
	CLI_HELP_CMD_LIST_ENTRY,

	//All memory commands
	{"set", CLI_ULINT_FPTR(set_pos), HELP("sets HE351VE vane pos (0 to 1000)")},

	CLI_CMD_LIST_END // must be LAST
};


/**************************************************************************************************
*                                         LOCAL FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief main
*
*  \note
******************************************************************************/
int main(void)
{
	system_init();
	configure_usart_cdc();
	configure_can();
	can_set_extended_filter_0(); //todo: needed?
	
	uint8_t tx_message_0[] = {(uint8_t)pos0, (uint8_t)(pos0 >> 8), 1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	TICK_TYPE last_time_ms = 0;
	TICK_TYPE last_time_ms1 = 0;

	drvr_sys_init();
	
	cmd_line_init();
	
	
	//todo: just pasting ADC code here will need to move this and use for analog input form pot
	//configure_adc();
	//
	//adc_start_conversion(&adc_instance);
	//uint16_t result;
	//do {
		///* Wait for conversion to be done and read out result */
	//} while (adc_read(&adc_instance, &result) == STATUS_BUSY);

	while(1)
	{
		cli_task();
		
		if(tmrCheckReset(&last_time_ms, 10))
		{
			//do stuff every 10mS
			tx_message_0[0] = (uint8_t)pos0;
			tx_message_0[1] = (uint8_t)(pos0 >> 8);
			can_send_extended_message(CAN_RX_EXTENDED_FILTER_ID_0, tx_message_0, CONF_CAN_ELEMENT_DATA_SIZE);
		}
		
		if(tmrCheckReset(&last_time_ms1, 1000))
		{
			//do stuff every 1000mS
			
			printf("time tick %lu\r\n", (unsigned long)last_time_ms1);
		}
	}
}

/******************************************************************************
*  \brief 
*
*  \note
******************************************************************************/
static void configure_usart_cdc(void)
{
	struct usart_config config_cdc;
	usart_get_config_defaults(&config_cdc);
	config_cdc.baudrate	 = 115200;
	config_cdc.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_cdc.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_cdc.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_cdc.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_cdc.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	stdio_serial_init(&cdc_instance, EDBG_CDC_MODULE, &config_cdc);
	usart_enable(&cdc_instance);
}

/******************************************************************************
*  \brief init can
*
*  \note
******************************************************************************/
static void configure_can(void)
{
	struct system_pinmux_config pin_config;
	struct can_config config_can;
	
	system_pinmux_get_config_defaults(&pin_config);
	
	//init TX pin
	pin_config.mux_position = CAN_TX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_TX_PIN, &pin_config);
	
	//init RX pin
	pin_config.mux_position = CAN_RX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_RX_PIN, &pin_config);
	
	//init can module
	can_get_config_defaults(&config_can);
	can_init(&can_instance, CAN_MODULE, &config_can);
	
	//start can module
	can_start(&can_instance);

	//Enable interrupts
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);
	//todo: there is a bunch of potentially usefull can interrupt sources here
	can_enable_interrupt(&can_instance, CAN_PROTOCOL_ERROR_ARBITRATION | CAN_PROTOCOL_ERROR_DATA);
}

/******************************************************************************
*  \brief
*
*  \note
******************************************************************************/
static void can_set_extended_filter_0(void)
{
	struct can_extended_message_filter_element et_filter;

	can_get_extended_message_filter_element_default(&et_filter);
	et_filter.F0.bit.EFID1 = CAN_RX_EXTENDED_FILTER_ID_0;
	et_filter.F0.bit.EFEC =	CAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STRXBUF_Val;
	et_filter.F1.bit.EFID2 = CAN_RX_EXTENDED_FILTER_ID_0_BUFFER_INDEX;

	can_set_rx_extended_filter(&can_instance, &et_filter, CAN_RX_EXTENDED_FILTER_INDEX_0);
	can_enable_interrupt(&can_instance, CAN_RX_BUFFER_NEW_MESSAGE);
}

/******************************************************************************
*  \brief
*
*  \note
******************************************************************************/
//static void can_set_extended_filter_1(void)
//{
	//struct can_extended_message_filter_element et_filter;
//
	//can_get_extended_message_filter_element_default(&et_filter);
	//et_filter.F0.bit.EFID1 = CAN_RX_EXTENDED_FILTER_ID_1;
//
	//can_set_rx_extended_filter(&can_instance, &et_filter, CAN_RX_EXTENDED_FILTER_INDEX_1);
	//can_enable_interrupt(&can_instance, CAN_RX_FIFO_1_NEW_MESSAGE);
//}

/******************************************************************************
*  \brief
*
*  \note
******************************************************************************/
static void can_send_extended_message(uint32_t id_value, uint8_t *data, uint32_t data_length)
{
	uint32_t i;
	struct can_tx_element tx_element;

	can_get_tx_buffer_element_defaults(&tx_element);
	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) | CAN_TX_ELEMENT_T0_XTD;
	tx_element.T1.bit.DLC = data_length;
	for (i = 0; i < data_length; i++)
	{
		tx_element.data[i] = *data;
		data++;
	}

	can_set_tx_buffer_element(&can_instance, &tx_element, CAN_TX_BUFFER_INDEX);
	can_tx_transfer_request(&can_instance, 1 << CAN_TX_BUFFER_INDEX);
}

/******************************************************************************
*  \brief Init all commands to be used in CLI
*
*  \note
******************************************************************************/
static void cmd_line_init(void)
{
	cli_conf_t cli_conf;

	/*Initialize the CLI driver*/
	cli_get_config_defaults(&cli_conf);

	cli_conf.rx_byte_fptr   = rx_byte;
	cli_conf.tx_string_fprt = tx_strn;
	cli_conf.enable         = CLI_ENABLED;
	cli_conf.echo_enable    = CLI_ECHO_ENABLED;
	cli_conf.cmd_list       = cli_cmd_list;

	cli_init(cli_conf);
}

/******************************************************************************
*  \brief
*
*  \note
******************************************************************************/
static int16_t rx_byte(void)
{
	uint16_t temp;
	enum status_code rtrn;
	
	rtrn = usart_read_wait(&cdc_instance, &temp);
	
	if (rtrn == STATUS_OK)
	{
		return (int16_t)temp;
	}
	else
	{
		return -1;
	}
}

/******************************************************************************
*  \brief
*
*  \note
******************************************************************************/
static void tx_strn(const char * const str)
{
	printf("%s", str);
}

/******************************************************************************
*  \brief
*
*  \note
******************************************************************************/
static void set_pos(unsigned long pos)
{
	pos0 = (uint16_t)pos;
}

/******************************************************************************
*  \brief
*
*  \note
******************************************************************************/
static void configure_adc(void)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);
	
	//todo: properly configure ADC pin this is from polled ADC example

	adc_init(&adc_instance, ADC1, &config_adc);
	adc_enable(&adc_instance);
}


/**************************************************************************************************
*                                             HANDLERS
*************************************************^************************************************/
/******************************************************************************
*  \brief 
*
*  \note 
******************************************************************************/
void CAN0_Handler(void)
{
	volatile uint32_t status, i, rx_buffer_index; //todo: why volatile? this was part of example
	
	status = can_read_interrupt_status(&can_instance);
	
	switch(status)
	{
		/* Rx FIFO 0 New Message Interrupt Enable. */
		case CAN_RX_FIFO_0_NEW_MESSAGE:
		//was standard message stuff
		break;

		/* Rx FIFO 0 Watermark Reached Interrupt Enable. */
		case CAN_RX_FIFO_0_WATERMARK:
		break;

		/* Rx FIFO 0 Full Interrupt Enable. */
		case CAN_RX_FIFO_0_FULL:
		break;

		/* Rx FIFO 0 Message Lost Interrupt Enable. */
		case CAN_RX_FIFO_0_LOST_MESSAGE:
		break;

		/* Rx FIFO 1 New Message Interrupt Enable. */
		case CAN_RX_FIFO_1_NEW_MESSAGE:
		can_get_rx_fifo_1_element(&can_instance, &rx_element_fifo_1, extended_receive_index);
		can_rx_fifo_acknowledge(&can_instance, 0, extended_receive_index);
		extended_receive_index++;
		if (extended_receive_index == CONF_CAN0_RX_FIFO_1_NUM)
		{
			extended_receive_index = 0;
		}

		printf("\n\r Extended message received in FIFO 1. The received data is: \r\n");
		for (i = 0; i < rx_element_fifo_1.R1.bit.DLC; i++)
		{
			printf("  %d",rx_element_fifo_1.data[i]);
		}
		printf("\r\n\r\n");
		break;

		/* Rx FIFO 1 Watermark Reached Interrupt Enable. */
		case CAN_RX_FIFO_1_WATERMARK:
		break;

		/* Rx FIFO 1 Full Interrupt Enable. */
		case CAN_RX_FIFO_1_FULL:
		break;

		/* Rx FIFO 1 Message Lost Interrupt Enable. */
		case CAN_RX_FIFO_1_MESSAGE_LOST:
		break;

		/* High Priority Message Interrupt Enable. */
		case CAN_RX_HIGH_PRIORITY_MESSAGE:
		break;

		/* Timestamp Completed Interrupt Enable. */
		case CAN_TIMESTAMP_COMPLETE:
		break;

		/* Transmission Cancellation Finished Interrupt Enable. */
		case CAN_TX_CANCELLATION_FINISH:
		break;

		/* Tx FIFO Empty Interrupt Enable. */
		case CAN_TX_FIFO_EMPTY:
		break;

		/* Tx Event FIFO New Entry Interrupt Enable. */
		case CAN_TX_EVENT_FIFO_NEW_ENTRY:
		break;

		/* Tx Event FIFO Watermark Reached Interrupt Enable. */
		case CAN_TX_EVENT_FIFO_WATERMARK:
		break;

		/* Tx Event FIFO Full Interrupt Enable. */
		case CAN_TX_EVENT_FIFO_FULL:
		break;

		/* Tx Event FIFO Element Lost Interrupt Enable. */
		case CAN_TX_EVENT_FIFO_ELEMENT_LOST:
		break;

		/* Timestamp Wraparound Interrupt Enable. */
		case CAN_TIMESTAMP_WRAPAROUND:
		break;

		/* Message RAM Access Failure Interrupt Enable. */
		case CAN_MESSAGE_RAM_ACCESS_FAILURE:
		break;

		/* Timeout Occurred Interrupt Enable. */
		case CAN_TIMEOUT_OCCURRED:
		break;

		/* Message stored to Dedicated Rx Buffer Interrupt Enable. */
		case CAN_RX_BUFFER_NEW_MESSAGE:
		for (i = 0; i < CONF_CAN0_RX_BUFFER_NUM; i++)
		{
			if (can_rx_get_buffer_status(&can_instance, i))
			{
				rx_buffer_index = i;
				can_rx_clear_buffer_status(&can_instance, i);
				can_get_rx_buffer_element(&can_instance, &rx_element_buffer, rx_buffer_index);
				if (rx_element_buffer.R0.bit.XTD)
				{
					printf("\n\r Extended message received in Rx buffer. The received data is: \r\n");
				}
				else
				{
					printf("\n\r Standard message received in Rx buffer. The received data is: \r\n");
				}
				
				for (i = 0; i < rx_element_buffer.R1.bit.DLC; i++)
				{
					printf("  %d",rx_element_buffer.data[i]);
				}
				printf("\r\n\r\n");
			}
		}
		break;

		/* Bit Error Corrected Interrupt Enable. */
		case CAN_BIT_ERROR_CORRECTED:
		printf("CAN_BIT_ERROR_CORRECTED\r\n");
		break;

		/* Bit Error Uncorrected Interrupt Enable. */
		case CAN_BIT_ERROR_UNCORRECTED:
		printf("CAN_BIT_ERROR_UNCORRECTED\r\n");
		break;

		/* Error Logging Overflow Interrupt Enable. */
		case CAN_ERROR_LOGGING_OVERFLOW:
		printf("CAN_ERROR_LOGGING_OVERFLOW\r\n");
		break;

		/* Error Passive Interrupt Enable. */
		case CAN_ERROR_PASSIVE:
		printf("CAN_ERROR_PASSIVE\r\n");
		break;

		/* Warning Status Interrupt Enable. */
		case CAN_WARNING_STATUS:
		printf("CAN_WARNING_STATUS\r\n");
		break;

		/* Bus_Off Status Interrupt Enable. */
		case CAN_BUS_OFF:
		break;

		/* Watchdog Interrupt Interrupt Enable. */
		case CAN_WATCHDOG:
		break;

		/* Protocol Error in Arbitration Phase Enable. */
		case CAN_PROTOCOL_ERROR_ARBITRATION:
		printf("CAN_PROTOCOL_ERROR_ARBITRATION\r\n");
		break;

		/* Protocol Error in Data Phase Enable. */
		case CAN_PROTOCOL_ERROR_DATA:
		printf("CAN_PROTOCOL_ERROR_DATA\r\n");
		break;

		/* Access to Reserved Address Enable. */
		case CAN_ACCESS_RESERVED_ADDRESS:
		printf("CAN_ACCESS_RESERVED_ADDRESS\r\n");
		break;
		
		default:
		//unknown CAN interrupt
		asm("nop");
		printf("CAN ERROR UNKNOWN ISR\r\n");
		break;
	}
	
	can_clear_interrupt_status(&can_instance, status);
}
