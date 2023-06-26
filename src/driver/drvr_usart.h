/*
 * drvr_usart.h
 *
 * Created: 8/18/2017 9:29:27 AM
 *  Author: akudlacek
 */


#ifndef DRVR_USART_H_
#define DRVR_USART_H_


#include <stdint.h>


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define DRVR_USART_BAUD                 115200
#define UART_TX_RING_BUFFER_SIZE_BYTE   2000
#define UART_RX_RING_BUFFER_SIZE_BYTE   300


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void    drvr_usart_init   (void);
void    drvr_usart_tx_str (const char * const str);
void    drvr_usart_tx_data(const uint8_t * const data, const uint32_t length);
int16_t drvr_usart_rx_byte(void);


#endif /* DRVR_USART_H_ */
