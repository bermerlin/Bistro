/*
 * $Id: adc.h,v 1.1 2003/12/11 01:35:00 bsd Exp $
 */

#ifndef __adc_h__
#define __adc_h__

void     adc_init(void);

void     adc_chsel(uint8_t channel);

void     adc_wait(void);

void     adc_start(void);

uint16_t adc_read(void);

uint16_t adc_readn(uint8_t channel, uint8_t n);

#endif
