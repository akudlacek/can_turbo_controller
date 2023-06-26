/*
 * drvr_port.c
 *
 * Created: 8/18/2017 11:19:03 AM
 *  Author: akudlacek
 */


#include "drvr_port.h"

#include "sam.h"


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define GROUP(pin_num)             (pin_num < 32 ? 0 : 1)
#define PIN(pin_num, group)        (pin_num - (group * 32))
#define PORT_MASK(pin_num, group)  (1ul << PIN(pin_num, group))


/**************************************************************************************************
*                                            FUNCTIONS
*************************************************^************************************************/
/******************************************************************************
*  \brief Initializes all pins for low power and the used pins for
*         respective GPIO
*
*  \note peripheral use is enabled here, but peripheral has control
******************************************************************************/
void drvr_port_pin_init(void)
{
	uint8_t i = 0;
	
	//Configure all pins with pull down to minimize power usage, except SWD otherwise hot-plugging get disabled
	for(i=0; i<64; i++)
	{
		if((i != UC_SWCLK) && (i != UC_SWDIO))
		{
			drvr_port_pin_cfg(i, INPUT, NORMAL, PULL_EN, IN_BUF_DIS, PORT_EN, LOW);
		}
	}
}


/******************************************************************************
*  \brief Sets the output value or pull type for specified pin
*
*  \note 
******************************************************************************/
void drvr_port_pin_out(const uint8_t pin_num, const out_t out)
{
	uint8_t group = GROUP(pin_num);
	
	if(out == HIGH)
	{
		PORT->Group[group].OUTSET.reg = PORT_MASK(pin_num, group);
	}
	else if(out == LOW)
	{
		PORT->Group[group].OUTCLR.reg = PORT_MASK(pin_num, group);
	}
}


/******************************************************************************
*  \brief Sets the direction for specified pin
*
*  \note 
******************************************************************************/
void drvr_port_pin_dir(const uint8_t pin_num, const dir_t dir)
{
	uint8_t group = GROUP(pin_num);
	
	if(dir == OUTPUT)
	{
		PORT->Group[group].DIRSET.reg = PORT_MASK(pin_num, group);
	}
	else if(dir == INPUT)
	{
		PORT->Group[group].DIRCLR.reg = PORT_MASK(pin_num, group);
	}
}


/******************************************************************************
*  \brief Toggles pin output
*
*  \note 
******************************************************************************/
void drvr_port_pin_tgl(const uint8_t pin_num)
{
	uint8_t group = GROUP(pin_num);
	
	PORT->Group[group].OUTTGL.reg = PORT_MASK(pin_num, group);
}


/******************************************************************************
*  \brief Return input value for single pin
*
*  \note
******************************************************************************/
out_t drvr_port_pin_in(const uint8_t pin_num)
{
	uint8_t group = GROUP(pin_num);
	uint8_t pin   = PIN(pin_num, group);
	uint8_t value = 0;
	
	value = (PORT->Group[group].IN.reg & PORT_MASK(pin_num, group)) >> pin;
	
	return value;
}


/******************************************************************************
*  \brief Configures pin parameters
*
*  \note sets configuration, direction, then output value
******************************************************************************/
void drvr_port_pin_cfg(const uint8_t pin_num, const dir_t dir, const drvstr_t drvstr, const pullen_t pullen, const inen_t inen, const prf_mux_t prf_mux, const out_t out)
{
	uint8_t group = GROUP(pin_num);
	uint8_t pin   = PIN(pin_num, group);
	
	if(prf_mux != PORT_EN)
	{
		if(pin % 2 == 0) //if even
			PORT->Group[group].PMUX[pin/2].bit.PMUXE = prf_mux;
		else             //else odd
			PORT->Group[group].PMUX[pin/2].bit.PMUXO = prf_mux;
	}
	
	PORT->Group[group].PINCFG[pin].bit.DRVSTR = drvstr;
	PORT->Group[group].PINCFG[pin].bit.PULLEN = pullen;
	PORT->Group[group].PINCFG[pin].bit.INEN   = inen;
	PORT->Group[group].PINCFG[pin].bit.PMUXEN = (prf_mux != PORT_EN ? 1 : 0);
	
	drvr_port_pin_dir(pin_num, dir);
	drvr_port_pin_out(pin_num, out);
}
