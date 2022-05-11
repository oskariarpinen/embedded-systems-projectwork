/*
 * main.c
 *
 * Created: 01.05.2022
 * Author: Oskari Arpinen
 */ 

#define F_CPU 16000000UL
#define FOSC 16000000UL // Clock Speed
#define BAUD 9600
#define MYUBRR (FOSC/16/BAUD-1)

#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 85 // 0b1010101

#define LISTENING 0
#define ALARM 1
#define UNARMED 2

volatile int8_t g_STATE = LISTENING;

// NOTE: USART = Universal Synchronous Asynchronous Receiver Transmitter
//       UART  = Universal Asynchronous Receiver Transmitter

static void
USART_init(uint16_t ubrr) // unsigned int
{
	/* Set baud rate in the USART Baud Rate Registers (UBRR) */
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;
	
	/* Enable receiver and transmitter on RX0 and TX0 */
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0); //NOTE: the ATmega328p has 1 UART: 0
	// UCSR0B |= (1 << 4) | (1 << 3);
	
	/* Set frame format: 8 bit data, 2 stop bit */
	UCSR0C |= (1 << USBS0) | (3 << UCSZ00);
	// UCSR0C |= (1 << 3) | (3 << 1);
	
}

static void
USART_Transmit(unsigned char data, FILE *stream)
{
	/* Wait until the transmit buffer is empty*/
	while(!(UCSR0A & (1 << UDRE0)))
	{
		;
	}
	
	/* Put the data into a buffer, then send/transmit the data */
	UDR0 = data;
}

static char
USART_Receive(FILE *stream)
{
	/* Wait until the transmit buffer is empty*/
	while(!(UCSR0A & (1 << UDRE0)))
	{
		;
	}
	
	/* Get the received data from the buffer */
	return UDR0;
}

// Setup the stream functions for UART
FILE uart_output = FDEV_SETUP_STREAM(USART_Transmit, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, USART_Receive, _FDEV_SETUP_READ);

ISR
(TIMER1_COMPA_vect)
{
	TCNT1 = 0; // reset timer counter
}


int 
main(void)
{
	USART_init(MYUBRR);
    stdout = &uart_output;
    stdin = &uart_input;	
    /* set up the ports and pins */
    DDRB |= (1 << PB1); // OC1A is located in digital pin 9
    
    // Enable interrupts
    sei();
    
    /* set up the 16-bit timer/counter1, mode 9 used */
    TCCR1B  = 0; // reset timer/counter 1
    TCNT1   = 0;
    TCCR1A |= (1 << 6); // set compare output mode to toggle
    // TCCR1A |= 0b01000000;
    // TCCR1A |= 0x40;
    
    // mode 9 phase correct
    TCCR1A |= (1 << 0); // set register A WGM[1:0] bits
    // TCCR1A |= 0b00000001;
    TCCR1B |= (1 << 4); // set register B WBM[3:2] bits
    // TCCR1B |= 0b00010000;
    
    TIMSK1 |= (1 << 1); // enable compare match A interrupt
    // TIMSK1 |= 0b00000100;
    
    OCR1A = 15297; // C5 523 Hz, no prescaler
    //OCR1A = 2462;   // A7 3250 Hz, no prescaler, calculated
    //OCR1A = 2440;   // A7 3250 Hz, no prescaler, empirical
    //OCR1A = 1016;   // B2  123 Hz, 64 prescaler	
	
	/* set MISO as output, pin 12 (PB4)*/
	DDRB  = (1 << PB4);
	/* set SPI enable */
	SPCR  = (1 << 6);
	
	unsigned int spi_receive_data = 0;
	
    while(1)
    {
		
		spi_receive_data = SPDR;
		g_STATE = spi_receive_data;
		
		printf("%d\n\r",spi_receive_data);
		
		switch (g_STATE)
		{
			case LISTENING:
				TCCR1B &= ~(1 << 0);
				printf("Listening...\n\r");
				
			break;
			
			case ALARM:
				TCCR1B |= (1 << 0);
				_delay_ms(500);
				TCCR1B &= ~(1 << 0);
				printf("ALARM!!\n\r");
			break;
			
			case UNARMED:
				TCCR1B &= ~(1 << 0);
				printf("Unarmed\n\r");
		}
		
		_delay_ms(1000);
    }
}