/*
 * drvr_dma.h
 *
 * Created: 11/14/2017 8:45:46 AM
 *  Author: akudlacek
 */


#ifndef DRVR_DMA_H_
#define DRVR_DMA_H_


#include <stdint.h>


/**************************************************************************************************
*                                             DEFINES
*************************************************^************************************************/
#define COMPILER_ALIGNED(a)        __attribute__((__aligned__(a)))

typedef enum drvr_dma_ch_t
{
	DMA_CH0_ADC0_WIN = 0,
	DMA_CH1_ADC0_SEQ = 1,
	DMA_CH2_ADC0_RES = 2
} drvr_dma_ch_t;


/**************************************************************************************************
*                                            PROTOTYPES
*************************************************^************************************************/
void     drvr_dma_crc_16_init (void);
uint16_t drvr_dma_crc_16      (const uint8_t * const message, const uint32_t num_bytes);
void     drvr_dma_init        (void);
void     drvr_dma_start       (const drvr_dma_ch_t channel);
uint32_t drvr_dma_abort       (const drvr_dma_ch_t channel);
uint32_t drvr_dma_get_xfr_size(const drvr_dma_ch_t channel);


#endif /* DRVR_DMA_H_ */
