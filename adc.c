/*
 * $Id: adc.c,v 1.3 2005/03/22 19:12:10 bsd Exp $
 */

/*
 * ATmega128 A/D Converter utility routines
 */

#include <avr/io.h>
#include <stdio.h>


/*
 * adc_init() - initialize A/D converter
 *
 * Initialize A/D converter to free running, start conversion, use
 * internal 5.0V reference, pre-scale ADC clock to 125 kHz (assuming
 * 16 MHz MCU clock) 
 */
void adc_init(void)
{
  /* configure ADC port (PORTF) as input */
  DDRF  = 0x00;
  PORTF = 0x00;
	// Bit 7:6 – REFS1:0: Reference Selection Bits
  ADMUX = _BV(REFS0);  // AVCC with external capacitor at AREF pin
  /*
  Bit 7 – ADEN: ADC Enable
  Bit 6 – ADSC: ADC Start Conversion
  Bit 5 – ADFR: ADC Free Running Select
  Bits 2:0 – ADPS2:0: ADC Prescaler Select Bits    111 == 128
  */
  ADCSR = _BV(ADEN)|_BV(ADSC)|_BV(ADFR) | _BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);
}


/*
 * adc_chsel() - A/D Channel Select
 *
 * Select the specified A/D channel for the next conversion
 */
void adc_chsel(uint8_t channel)
{
  /* select channel */
  /*
  0xe0 == 11100000  : clear MUX0 à MUX4
  0x07 == 00000111  : choisit parmi ADC0 à ADC7
  */
  
  ADMUX = (ADMUX & 0xe0) | (channel & 0x07);
}


/*
 * adc_wait() - A/D Wait for conversion
 *
 * Wait for conversion complete.
 */
void adc_wait(void)
{
  /* wait for last conversion to complete */
  /*
	Bit 4 – ADIF: ADC Interrupt Flag
  */
  
  while ((ADCSR & _BV(ADIF)) == 0)
    ;
}


/*
 * adc_start() - A/D start conversion
 *
 * Start an A/D conversion on the selected channel
 */
void adc_start(void)
{
  /* clear conversion, start another conversion */
  ADCSR |= _BV(ADIF);
}


/*
 * adc_read() - A/D Converter - read channel
 *
 * Read the currently selected A/D Converter channel.
 */
uint16_t adc_read(void)
{
  return ADC;
}


/*
 * adc_readn() - A/D Converter, read multiple times and average
 *
 * Read the specified A/D channel 'n' times and return the average of
 * the samples 
 */
uint16_t adc_readn(uint8_t channel, uint8_t n)
{
  uint16_t t;
  uint8_t i;

  adc_chsel(channel);
  adc_start();
  adc_wait();

  adc_start();

  /* sample selected channel n times, take the average */
  t = 0;
  for (i=0; i<n; i++) {
    adc_wait();
    t += adc_read();
    adc_start();
  }

  /* return the average of n samples */
  return t / n;
}


