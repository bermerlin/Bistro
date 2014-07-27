#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include "uart.h"

/*
 * Send character c down the UART Tx, wait until tx holding register
 * is empty.
 */
int uart_putchar(char c, FILE *stream)
{
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;

  return 0;
}

/*
 * Receive a character from the UART Rx.
 *
 * The internal line buffer is RX_BUFSIZE (80) characters long, which
 * includes the terminating \n (but no terminating \0).
 *
 * Input errors while talking to the UART will cause an immediate
 * return of -1 (error indication).  Notably, this will be caused by a
 * framing error (e. g. serial line "break" condition), by an input
 * overrun, and by a parity error (if parity was enabled and automatic
 * parity recognition is supported by hardware).
 *
 * Successive calls to uart_getchar() will be satisfied from the
 * internal buffer until that buffer is emptied again.
 */
int uart_getchar(FILE *stream)
{
  uint8_t c;
  char *cp;
  static char b[RX_BUFSIZE];
  static char *rxp;

  if (rxp == 0)
    for (cp = b;;)
    {
		loop_until_bit_is_set(UCSR0A, RXC0);
		if (UCSR0A & _BV(FE))
			return _FDEV_EOF;
		if (UCSR0A & _BV(DOR))
			return _FDEV_ERR;
		c = UDR0;

		if (c == '\r')
			c = '\n';
		if (c == '\n')
		{
			*cp = c;
			rxp = b;
			break;
		}

		if ((c >= (uint8_t)' ' && c <= (uint8_t)'\x7e') || c >= (uint8_t)'\xa0')
		{
			*cp++ = c;
			continue;
		}

		switch (c)
		{
			case 'c' & 0x1f:
				return -1;
		}
      }

	c = *rxp++;
	if (c == '\n')
		rxp = 0;

	return c;
	
}

