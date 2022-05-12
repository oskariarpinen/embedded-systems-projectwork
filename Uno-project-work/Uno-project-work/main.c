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
#define RESET 3
#define TIMEOUT 4
#define INCORRECTPSWRD 5

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
    
    // mode 9 phase correct
    TCCR1A |= (1 << 0); // set register A WGM[1:0] bits
    // TCCR1A |= 0b00000001;
    TCCR1B |= (1 << 4); // set register B WBM[3:2] bits
    // TCCR1B |= 0b00010000;
    TIMSK1 |= (1 << 1); // enable compare match A interrupt
    // TIMSK1 |= 0b00000100;
    
    OCR1A = 15297; // C5 523 Hz, no prescaler	
	
	/* set MISO as output, pin 12 (PB4)*/
	DDRB  = (1 << PB4);
	/* set SPI enable */
	SPCR  = (1 << 6);
	
	// Pin for the red LED, arduino pin 7
	DDRD |= (1 << PD7);
	
	// Pin for the green LED, arduino pin 6
	DDRD |= (1 << PD6);
	
	// Pin for the yellow LED, arduino pin 5
	DDRD |= (1 << PD5);
	
	//Pin for the active buzzer, arduino pin 4
	DDRD |= (1 << PD4);
	
	unsigned int spi_receive_data = 0;
	
    while(1)
    {
		
		spi_receive_data = SPDR;
		g_STATE = spi_receive_data;

		switch (g_STATE)
		{
			case LISTENING:
				PORTD &=  ~(1 << PD6);
				
				TCCR1B &= ~(1 << 0);
				printf("System is active..\n\r");
				
			break;
			
			case ALARM:
				//Turn off green led
				PORTD &=  ~(1 << PD6);
				
				//Turn on yellow led
				PORTD |=  (1 << PD5);

				printf("Motion detected. Input password...\n\r");
			break;
			
			case UNARMED:
				//Turn on green led
				PORTD |=  (1 << PD6);
				
				// Turn off yellow led
				PORTD &=  ~(1 << PD5);
				
				TCCR1B &= ~(1 << 0);
				printf("System is unarmed. Press reset button to rearm the system. \n\r");
				
			break;
			
			case RESET:
				printf("The system is resetting\n\r");
				printf("Motion detection active in:\n\r");
				g_STATE = 0;
				for (int i = 6; i >= 1; --i)
				{
					printf("%d\n\r",i);
					_delay_ms(1000);
				}
			break; 
			
			case TIMEOUT:
				PORTD |=  (1 << PD7);
				PORTD |=  (1 << PD4);
				TCCR1B |= (1 << 0);
				printf("TIMEOUT! ALARM! TIMEOUT! \n\r");
				_delay_ms(500);
				PORTD &=  ~(1 << PD7);
				TCCR1B &= ~(1 << 0);
				PORTD &=  ~(1 << PD4);
			break;
			
			case INCORRECTPSWRD:
				printf("Incorrect password. Try again. \n\r");
				_delay_ms(1000);
			break;		
		}
		
		_delay_ms(500);
    }
}