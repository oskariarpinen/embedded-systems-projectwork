/*
 * main.c
 *
 * Created: 23.4.2022
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
#include "keypad.h"
#include <stdbool.h>

volatile int8_t g_STATE = 0;

#define SLAVE_ADDRESS 85 // 0b1010101

#define ARMED 0
#define MOTIONDETECTED 1
#define DISARMED 2

int8_t g_user_given_password[4];

static void
USART_init(uint16_t ubrr)
{
	/* Set baud rate in the USART Baud Rate Registers (UBRR) */
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;
	
	/* Enable receiver and transmitter on RX0 and TX0 */
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0); //NOTE: the ATmega2560 has 4 UARTs: 0,1,2,3
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
	
	/* Puts the data into a buffer, then sends/transmits the data */
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



void
input_password(int password_length){
	
	/* This function is resposible for the keypad password input */
	
	int index = 0;
	int8_t temp_read = 122;
	
	KEYPAD_Init();
	USART_init(MYUBRR);
	stdout = &uart_output;
	stdin = &uart_input;
	
	// Key press indicator led
	DDRB |= (1 << PB5);
	
	
	while (1)
	{
		printf("%c%c%c%c \n\r",g_user_given_password[0],g_user_given_password[1],g_user_given_password[2],g_user_given_password[3]);
		// Get keypress using the provided keypad functions
		temp_read = KEYPAD_GetKey();
		
		// If the press is "#", exits the input loop
		if (temp_read == 35)
		{
			break;
		}
		
		// If the press is 0-9 & the index is not above 3
		else if (temp_read >= 48 && temp_read <= 57 && index <= 3)
		{
			PORTB &= ~(1 << PB7);
			PORTB |=  (1 << PB5);
			
			// Store the number to the current index position
			g_user_given_password[index] = temp_read;
			index++;
			printf("%c",temp_read);
			printf("\n\r");
			
			// Wait until the key is depressed 
			while(temp_read != 122)
			{
				temp_read = KEYPAD_GetKey();
				_delay_ms(100);
			}
			
			PORTB &=  ~(1 << PB5);
		}
		
		// Backspace, if "*" is received, removes the last number inputted
		else if (temp_read == 42 && index >= 1)
		{
			PORTB |=  (1 << PB7);
			g_user_given_password[index-1] = 0;
			index--;
			
			// Wait until the key is depressed
			while(temp_read != 122)
			{
				temp_read = KEYPAD_GetKey();
				_delay_ms(100);
			}
			
			PORTB &=  ~(1 << PB7);
		}
		
		_delay_ms(100);
	}
}

bool
compare_passwords(int8_t stored_password[], int8_t given_password[], int password_length)
{
	/* This function compares the "stored" password and the user inputted password */
	
	int index;
	
	// Goes through the array and compares each item, when it finds the first that does not match, returns 0
	for (index = 0; index <= password_length-1; index++ )
	{
		if (stored_password[index] != given_password[index])
		{
			return 0;
		}
	}
	// If no different elements are found, returns 1
	return 1;
}

void
transmit_to_uno(int8_t status)
{
	USART_init(MYUBRR);
	stdout = &uart_output;
	stdin = &uart_input;
	
    unsigned char twi_send_data[20] = "MOTION DETECTED\n";
    char test_char_array[16]; // 16-bit array, assumes that the int given is 16-bits
    uint8_t twi_status = 0;
		
    // Initialize TWI
    // set SCL frequency to 400 kHz, using equation in p. 242 of ATmega2560 datasheet
    TWSR = 0x00; // TWI status register prescaler value set to 1
    TWBR = 0x03; // TWI bit rate register.
    TWCR |= (1 << TWEN); // enable TWI
    // TWCR |= (1 << 2);
        
    // Start transmission by sending START condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    // TWCR = (1 << 7) | (1 << 5) | (1 << 2);
        
    // wait for the TWINT to set
    while (!(TWCR & (1 << TWINT)))
    {
	    ;
    }
        
    // read the status from TWI status register, 0xF8 is used to mask prescaler bits so that
    // only the status bits are read
    twi_status = (TWSR & 0xF8);
        
    // print the status bits to the UART for monitoring
    itoa(twi_status, test_char_array, 16);
        
    // Send slave address and write command to enter MT mode
    TWDR = 0b10101010; // load slave address and write command
    // Slave address = 85 + write bit '0' as a LSB ---> 170
        
    // clear TWINT to start transmitting the slave address + write command
    TWCR = (1 << TWINT) | (1 << TWEN);
    // TWCR = (1 << 7) | (1 << 2);
        
    // wait for the TWINT to set
    while (!(TWCR & ( 1<< TWINT)))
    {
	    ;
    }
        
    // read the status from TWI status register, 0xF8 is used to mask prescaler bits so that
    // only the status bits are read
    twi_status = (TWSR & 0xF8);
        
    itoa(twi_status, test_char_array, 16);
    printf(test_char_array);
    printf(" ");
        
    // transmit data to the slave
    for(int8_t twi_data_index = 0; twi_data_index < sizeof(twi_send_data); twi_data_index++)
    {
	        
	    TWDR = twi_send_data[twi_data_index]; // load data
	        
	    // "clear" TWINT to start transmitting the data
	    TWCR = (1 << TWINT) | (1 << TWEN);
	    // TWCR = (1 << 7) | (1 << 2);
	        
	    // wait for the TWINT to set
	    while (!(TWCR & ( 1<< TWINT)))
	    {
		    ;
	    }

	    // read the status from TWI status register, 0xF8 is used to mask prescaler bits so that
	    // only the status bits are read
	    twi_status = (TWSR & 0xF8);
	    itoa(twi_status, test_char_array, 16);
	    printf(test_char_array);
	    printf(" ");
	        
    }
        
    // stop transmission by sending STOP
    TWCR = (1 << TWINT) | (1 << TWSTO) |(1 << TWEN);
    //TWCR = (1 << 7) | (1 << 4) |(1 << 2);
    printf("\n\r");
}

int
main(void)
{

	USART_init(MYUBRR);
	stdout = &uart_output;
	stdin = &uart_input;
	
	// Defining the keypad pins as input
	DDRK = 0b00000000;
  
	// Motion sensor input pin, MEGA pin 2
	DDRE &= ~(1 << PE4);
  
	// Alarm output, MEGA pin 13
	DDRB |= (1 << PB7);
	
	// Disarmed led, MEGA pin 12
	DDRB |= (1 << PB6);
  
	// Motion sensor state variable
	bool s_sensor_state = 0;

	bool password_state = 0;
	
	int password_length = 4;
	
	int8_t status;
	
	// Defining the password for the alarm system
	int8_t stored_password[4];
	stored_password[0] = 49;
	stored_password[1] = 50;
	stored_password[2] = 51;
	stored_password[3] = 52;
	
	while (1)
	{
		/* 
			State machine - switch case 
			Three different states: ARMED, MOTIONDETECTED and DISARMED
		*/
		
		
		switch(g_STATE) 
		{
			case ARMED:
				printf("State: ARMED\n\r");
				PORTB &= ~(1 << PB7);
				s_sensor_state = (PINE & (1 << PE4));
				if (0 != s_sensor_state)
				{
					g_STATE = 1;
				}
				_delay_ms(1000);
			break;
		  
			case MOTIONDETECTED:
				status = 1;
				// transmit_to_uno(status);
				printf("State: MOTIONDETECTED\n\r");
				PORTB |=  (1 << PB7);
				_delay_ms(500);
				
				input_password(password_length);
				
				password_state = compare_passwords(stored_password, g_user_given_password, 4);
				if (password_state != 0)
				{
					PORTB |=   (1 << PB6);
					g_STATE = 2;
				}
				
			break;
		  
			case DISARMED:
				printf("State: UNARMED\n\r");
				_delay_ms(1000);
		}
	}
	
	return 0;
}