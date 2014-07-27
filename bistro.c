// 18/06/ 2014


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <uart.h>
#include <inttypes.h>
#include <util/delay.h>
#include <string.h>
#include <math.h>
#include <avr/eeprom.h>
#include "adc.h"
#include <avr/pgmspace.h>

#define PI 3.14159265 
#define DEG_TO_RAD ((double)(PI/180.0)) 
#define RAD_TO_DEG ((double) (180.0/PI))
#define DATA_SIZE 52

unsigned char dump_mode = 1;
unsigned char debug_mode = 0;
unsigned char sreg;
uint16_t tempor_1;
uint32_t somme;

// Calibration factors
double EEMEM ee_cal_offset = 0.0;
double EEMEM ee_cal_wind = 0.0;

typedef unsigned int icp_timer_t;
volatile icp_timer_t icp_start_time;
volatile icp_timer_t icp_period;
volatile icp_timer_t icp_1;

volatile uint16_t ms_count;
unsigned char i;

uint16_t aa;
unsigned char j;

volatile unsigned char wind0_flag;
double speed_through_water, apparent_wind_speed, wind_direction, heading;
volatile double speed_over_ground = 1.1;
volatile double course_over_ground = 111.0;

volatile unsigned char compteur, flag_go, rtc_count, flag_rtc;

double leeway, velocity_made_good;
double cal_offset;

typedef union
{
	unsigned char dump[DATA_SIZE];
	struct
	{
		double aws;
		double awa_raw;
		double awa;
		double leeway;  // leeway = 0 for now
		double stw;		// = GPS SOG (current = 0)
		double twa;
		double tws;
		double sog;
		double cog;
		double heading;   // = GPS COG for now (leeway = 0)
		double vmg;
		double wdir;
		int green;
		int blue;
	};
} dump_union;


// double: 12 * 4 = 48 bytes
// int: 2 * 2 = 4 bytes
// total: 52 bytes

dump_union dump_info;

static char ubuff[DATA_SIZE + 6];   
static volatile uint8_t txindex;

uint16_t green, blue;

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

static char buf[500];
volatile unsigned char temprec;
volatile unsigned char idx = 0;

//volatile double vol_sog;
//volatile double vol_cog;

unsigned char kbhit(void)
{
	// return nonzero if char waiting
	unsigned char b;
	b = 0;
	if(UCSR0A & (1<<RXC0))
		b = 1;
	return b;
}

void USART_Flush( void )
{
	unsigned char dummy;
	while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}

ISR(TIMER0_OVF_vect)
{
	compteur++;
	if(compteur == 6)
	{
		rtc_count++;
		if(rtc_count == 10)
		{
			rtc_count = 0;
			flag_rtc = 1;
		}
		
		compteur = 0;
		flag_go = 1;
	}
}

ISR(TIMER1_COMPA_vect)
{
	// set zero wind speed flag
	wind0_flag = 1;
}

ISR(TIMER1_CAPT_vect)
{
	icp_timer_t icr;
	
	icr = ICR1;		//capture timestamp
	
	if(wind0_flag)  // following  an output compare interrupt (or first pulse of new series)
	{
		icp_start_time = icr;			// Start of new pulse/period
		OCR1A = icr - 100;				//  Move timeout window
		wind0_flag = 0;
		icp_1 = 0;
	}
	else
	{
		icp_period = icr - icp_start_time;	// Length of previous period
		
		if(icp_period > 200)   // if debounced
		{
			icp_start_time = icr;
			OCR1A = icr - 100;				//  Move timeout window
			icp_1 = icp_period;
		}
	}
	TIFR |= _BV(ICF1);  // debounce
}

ISR(USART0_UDRE_vect)
{
	if(txindex < (DATA_SIZE + 6 - 1))
		UDR0 = ubuff[txindex++];
	else
	{
		UDR0 = ubuff[DATA_SIZE + 6 - 1];
		txindex = 0;
		UCSR0B &= ~(_BV(UDRIE0));
	}
}

ISR(USART1_RX_vect)
{
	temprec = UDR1;
	if(temprec != '\n')
	{
		buf[idx++] = temprec;
	} 
	else 
	{
		buf[idx] = '\0';
		idx = 0;
		if(buf[0] == '$')
		{
			if(buf[3] == 'R')
			{
				//$GPRMC,180846.600,A,4642.8826,N,07111.8704,W,0.00,0.00,050210,,,D*77
				//sscanf(&(buf[45]), "%lf,%lf", &vol_sog, &vol_cog);
				sscanf(&(buf[45]), "%lf,%lf", &speed_over_ground, &course_over_ground);
			}
		}
	}
}

/*
 * initialize timer 0 to generate an interrupt every 0.016384
 */
void init_timer(void)
{
  TIFR |= _BV(TOV0);		// clear Overflow Flag
  TCCR0  = _BV(CS02)|_BV(CS01)|_BV(CS00); // Normal Mode, prescale = 1024
  TCNT0  = 0;
  TIMSK |= _BV(TOIE0);  // enable overflow interrupt
  
  TCCR1B = _BV(ICES1) | _BV(CS12)| _BV(CS10);   // rising edge  prescale 1024
  TCNT1 = 0;
  // clear pending interrupts
  TIFR = _BV(ICF1);
  TIMSK	|= _BV(TICIE1);    /* ICP1 interrupt enable			*/
  /*
   * Setting the OCR (timeout) to 0 allows the full TCNT range for
   * the initial period.
  */
  OCR1A = 0;
  TIMSK	|= _BV(OCIE1A);
}

int main(void)
{
  
  unsigned char buf[24];
  double cal_wind;
  double cal_xc, cal_yc, cal_wcx0, cal_wcx1, cal_wcy0, cal_wcy1;
  
  double awa_raw, awa;
  
  //Port A : enable pull-up on all pins
  PORTA = 0xff;
  
  // Port B : enable pull-up on all pins : B0(LED) will be output
  PORTB = 0xff;
  
  // Port C : enable pull-ups on all pins
  PORTC = 0xff;
  
  // Port D		D2:RXD1	    D4:ICP1(wind pulse)
  // enable pull-up on all pins except D2  (D4 needs pull-up)
  //PORTD = _BV(PD0) | _BV(PD1) | _BV(PD3) | _BV(PD4)| _BV(PD5) | _BV(PD6) | _BV(PD7);
  PORTD = 0xff;
  
  // Port E		E0:RXD0 	E1:TXD0
  // enable pull-up on all pins except E0 and E1  
  PORTE = _BV(PE2) | _BV(PE3) | _BV(PE4) | _BV(PE5) | _BV(PE6) | _BV(PE7);
  
  // Port F    F0:ADC0(green)  	F1:ADC1(blue)
  // enable pull-up on all pins except F0 and F1
  PORTF = _BV(PF2) | _BV(PF3) | _BV(PF4) | _BV(PF5) | _BV(PF6) | _BV(PF7);
  
  init_timer();
  stdout = stdin = &uart_str;
  stderr = &uart_str;

  compteur = 0;
  rtc_count = 0;
  
  double true_wind_angle, true_wind_speed;
  
  /* USART0 */    // Serial-to-USB
  /* Set baud rate : 9600 @ 16MHz */
  //UBRR0L = (unsigned char)(103);
  
  /* Set baud rate : 115200 @ 16MHz */
  UBRR0L = (unsigned char)(8);

    /* Enable transmitter and receiver */
  UCSR0B =_BV(TXEN0) | _BV(RXEN0);
  
  /* USART1 */    // GPS
  /* Set baud rate : 9600 @ 16MHz */
  //UBRR1L = (unsigned char)(103);
  /* Set baud rate : 57600 @ 16MHz */
  UBRR1L = (unsigned char)(16);
  /* Enable receiver */
  UCSR1B = _BV(RXEN1) | _BV(RXCIE);

  _delay_loop_2(4000); 
  
  DDRB = 0x01;  /* enable PORTB 0 as an output */   // enable LED output
  
  wind0_flag = 1;
  flag_go = 0;
  flag_rtc = 0;
  
  /* enable interrupts */
  sei();
  
  for(aa = 1; aa < 5000; aa++)  // wait 5 seconds
	_delay_loop_2(4000);    // 1 ms delay
  
  j = 1;
  speed_through_water = 0.0;
  apparent_wind_speed = 0.0;
  
  /* initialize A/D Converter */
  adc_init();
  
  for(aa = 1; aa < 250; aa++)  // wait 1/4 second
	_delay_loop_2(4000);    // 1 ms delay

  // read calibration factors in EEPROM
  eeprom_read_block(&cal_offset, &ee_cal_offset, 4);
  eeprom_read_block(&cal_wind, &ee_cal_wind, 4);
 
  // Last calibration : May 29, 2014
  cal_xc = 712.9065;
  cal_yc = 713.4619;
  cal_wcx0 = 1.017720;
  cal_wcx1 = -0.000774446;
  cal_wcy0 = -0.000774446;
  cal_wcy1 = 1.000034;
  
  /*
  cal_xc = 741.2638;
  cal_yc = 744.6921;
  cal_wcx0 = 1.002073;
  cal_wcx1 = -0.006924865;
  cal_wcy0 = -0.006924865;
  cal_wcy1 = 1.023132;
  */
  
  ubuff[0] = 'B';
  ubuff[1] = 'I';
  ubuff[2] = 'S';
  ubuff[3] = 'T';
  ubuff[4] = 'R';
  ubuff[5] = 'O';
  txindex = 0;
  
  dump_info.sog = 4.4;
  dump_info.cog = 44.0;
  
  for(aa = 1; aa < 5000; aa++)  // wait 5 seconds
	_delay_loop_2(4000);    // 1 ms delay
  
  for(;;)
  { 	

	if(kbhit())     // data are available from USB-to-serial
	{
		/* List of valid commands received
			
			STOP<ENTER> : stop all transmission from system
			DUMP<ENTER> : transmit all binary data (separated by 'BISTRO' magic word) at 10 Hz (default mode)
			DEBUG<ENTER> : print debug info at 1 Hz
			
			stw<ENTER>  :   print last measured value of speed_through_water
			
			offset<ENTER> : print current value of wind vane offset stored in EEPROM
			woff2.6<ENTER> : store new value (2.6) of wind vane offset in EEPROM
			
			calwind<ENTER> : print current value of wind calibration factor stored in EEPROM
			wcv2.17<ENTER> : store new value (2.17) of wind calibration factor in EEPROM
		*/
		
		if(fgets(buf, sizeof buf - 1, stdin) == NULL)
		{
			USART_Flush();
			continue;
		}
		else
		{
			USART_Flush();
			printf("%s\r\n", buf);
		}
		
		if(strstr(buf, "stw") != NULL)
		{
			printf("stw :%4.2f knots\r\n", speed_through_water);
		}
	
		else if(strstr(buf, "calwind") != NULL)
		{
			printf("wind calibration :%4.2f\r\n", cal_wind);
		}
		else if(strstr(buf, "wcv") != NULL)    //   example : wcv0.78
		{
			sscanf(buf, "wcv%lf", &cal_wind);
			
			printf("new wind calibration :  %4.2f\r\n", cal_wind); 
			
			sreg = SREG;
			cli();
			eeprom_write_block(&cal_wind, &ee_cal_wind, 4);
			SREG = sreg;
			eeprom_busy_wait();
		}
		else if(strstr(buf, "STOP") != NULL)
		{
			debug_mode = 0;
			dump_mode = 0;
		}
		else if(strstr(buf, "DEBUG") != NULL)
		{
			debug_mode = 1;
			dump_mode = 0;
		}
		else if(strstr(buf, "DUMP") != NULL)
		{
			debug_mode = 0;
			dump_mode = 1;
		}
		else if(strstr(buf, "offset") != NULL)
		{
			printf("offset calibration :%4.2f\r\n", cal_offset);
		}
		else if(strstr(buf, "woff") != NULL)
		{
			sscanf(buf, "woff%lf", &cal_offset);
			
			printf("new offset calibration:  %4.2f\r\n", cal_offset); 
			
			sreg = SREG;
			cli();
			eeprom_write_block(&cal_offset, &ee_cal_offset, 4);
			SREG = sreg;
			eeprom_busy_wait();
		}
	}
    green = adc_readn(0, 10);  /* sample channel #0 10 times, take average */
    blue = adc_readn(1, 10);   /* sample channel #1 10 times, take average */
	
	// DEBUG
	//green = 1023;
	//blue = 777;
    
	double dgreen = ((double)green) - cal_xc;
    double dblue = ((double)blue) - cal_yc;
    double xmap = cal_wcx0 * dgreen + cal_wcx1 * dblue;
    double ymap = cal_wcy0 * dgreen + cal_wcy1 * dblue;
    
	awa_raw = atan2(ymap, xmap) * RAD_TO_DEG;
	
	if(wind0_flag)
	{
		apparent_wind_speed = 0.0;
	}
	else
	{
		sreg = SREG;
		cli();
		tempor_1 = icp_1;
		SREG = sreg;
		
		somme = (uint32_t)(tempor_1);
		
		if(tempor_1 == 0)
		{
			apparent_wind_speed = 0.0;
		}
		else
		{
			apparent_wind_speed = (15625.0 / ((double)somme) + 0.55) * 0.729 * cal_wind;  
		}
	}
	
	// correct awa for offset
	awa = awa_raw + cal_offset;
	if(awa > 180.0)
		awa = awa - 360.0;
	else if(awa < -180.0)
		awa = awa + 360.0;

	// get GPS data
	sreg = SREG;
	cli();
	//speed_over_ground = vol_sog;
	//course_over_ground = vol_cog;
	dump_info.sog = speed_over_ground;
	dump_info.cog = course_over_ground;
	SREG = sreg;
	
	leeway = 0.0;
	//heading = course_over_ground;              // not corrected for leeway for now
	heading = dump_info.cog;
	//speed_through_water = speed_over_ground;   // no current
	speed_through_water = dump_info.sog;
	
	double stw_x = 0.0;			// leeway = 0.0, no lateral speed for now
	double stw_y = speed_through_water;  // small correction for leeway will be needed later
	
	double awa_rad = (270.0 - awa) * DEG_TO_RAD;
	double aws_x = apparent_wind_speed * cos(awa_rad);
	double aws_y = apparent_wind_speed * sin(awa_rad);
	double tws_x = aws_x + stw_x;
	double tws_y = aws_y + stw_y;
	true_wind_speed = sqrt(tws_x * tws_x + tws_y * tws_y);
		
	double betaprime = 270.0 - atan2(tws_y, tws_x) * RAD_TO_DEG;
	
	if(speed_through_water == 0.0)
		true_wind_angle = awa;
	else
	{
		if(awa >= 0.0)
			true_wind_angle = fmod(betaprime, 360.0);
		else
			true_wind_angle = betaprime - 360.0;
		
		if(true_wind_angle > 180.0)
			true_wind_angle = true_wind_angle - 360.0;
		else if(true_wind_angle < -180.0)
			true_wind_angle = true_wind_angle + 360.0;
	}

	velocity_made_good = speed_through_water * cos((-true_wind_angle + leeway) * DEG_TO_RAD);
	
	wind_direction = heading + true_wind_angle;
	if(wind_direction > 360.0)
		wind_direction -= 360.0;
	else if(wind_direction < 0.0)
		wind_direction += 360.0;
	
	dump_info.wdir = wind_direction;
	dump_info.twa = true_wind_angle;
	dump_info.vmg = velocity_made_good;
	dump_info.aws = apparent_wind_speed;
	dump_info.green = green;
	dump_info.blue = blue;
	dump_info.awa_raw = awa_raw;
	dump_info.awa = awa;
	dump_info.leeway = leeway;
	dump_info.stw = speed_through_water;
	//dump_info.sog = speed_over_ground;
	//dump_info.cog = course_over_ground;
	dump_info.heading = heading;
	dump_info.tws = true_wind_speed;
	
	if(dump_mode)
	{
		for(i = 6; i < (DATA_SIZE + 6); i++)
			ubuff[i] = dump_info.dump[i - 6];
		UCSR0B |= _BV(UDRIE0);
	}
	
	/*
	if(debug_mode)
			printf("%i %i\r\n", green, blue);
	*/
	
	
	if(flag_rtc)  // once per second
	{
		if(debug_mode)
		{
			//printf("awa_raw = %6.2f   aws = %6.2f\r\n", awa_raw, apparent_wind_speed);
			printf("sog = %6.2f   cog = %6.2f\r\n", dump_info.sog, dump_info.cog);
		}
		flag_rtc = 0;
	} 
	
			
	// wait for 10 Hz timer signal
	while(!flag_go);		//	10.17252604  Hz  (6 overflows of timer 0 : 98.304 ms)
	flag_go = 0;
	
	PORTB ^= 0x01;  // toggle LED
  }
}

