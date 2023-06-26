/*
* drvr_port.h
*
* Created: 8/18/2017 11:19:15 AM
*  Author: akudlacek
*/


#ifndef DRVR_PORT_H_
#define DRVR_PORT_H_


#include <stdint.h>
#include "sam.h"


/******************************************************************************
* Includes for PIN/PORT/MUX Definitions
******************************************************************************/
#include "erp4.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
typedef enum dir_t
{
	INPUT, //The corresponding I/O pin in the PORT group is configured as an input.
	OUTPUT //The corresponding I/O pin in the PORT group is configured as an output.
} dir_t;

typedef enum drvstr_t
{
	NORMAL, //Pin drive strength is set to normal drive strength.
	STRONG  //Pin drive strength is set to stronger drive strength.
} drvstr_t;

typedef enum pullen_t
{
	PULL_DIS, //Internal pull resistor is disabled, and the input is in a high-impedance configuration.
	PULL_EN   //Internal pull resistor is enabled, and the input is driven to a defined logic level in the absence of external input.
} pullen_t;

typedef enum inen_t
{
	IN_BUF_DIS, //Input buffer for the I/O pin is disabled, and the input value will not be sampled.
	IN_BUF_EN   //Input buffer for the I/O pin is enabled, and the input value will be sampled when required.
} inen_t;

typedef enum prf_mux_t
{
	A,       //Peripheral function A selected
	B,       //Peripheral function B selected
	C,       //Peripheral function C selected
	D,       //Peripheral function D selected
	E,       //Peripheral function E selected
	F,       //Peripheral function F selected
	G,       //Peripheral function G selected
	H,       //Peripheral function H selected
	I,       //Peripheral function I selected
	PORT_EN  //The peripheral multiplexer selection is disabled, and the PORT registers control the direction and output drive value.
} prf_mux_t;

typedef enum out_t
{
	LOW = 0, //The I/O pin output is driven low, or the input is connected to an internal pull-down.
	HIGH = 1 //The I/O pin output is driven high, or the input is connected to an internal pull-up.
} out_t;


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void drvr_port_pin_init(void);
void drvr_port_pin_out (const uint8_t pin_num, const out_t out);
void drvr_port_pin_dir (const uint8_t pin_num, const dir_t dir);
void drvr_port_pin_tgl (const uint8_t pin_num);
out_t drvr_port_pin_in (const uint8_t pin_num);
void drvr_port_pin_cfg (const uint8_t pin_num, const dir_t dir, const drvstr_t drvstr, const pullen_t pullen, const inen_t inen, const prf_mux_t prf_mux, const out_t out);


#endif /* DRVR_PORT_H_ */
