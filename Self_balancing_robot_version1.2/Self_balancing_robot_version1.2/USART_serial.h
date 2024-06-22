/*
 * UART_Terminal.c
 *
 * Created: 2/29/2016 7:47:39 PM
 * Author : Mohamed_Soley
 */


//Description of the ATmega32 UART data panel. Code samples: 155-157. Page

#ifndef USART_serial
#define USART_serial

#include <avr/io.h>

/*Formula 151 Page of the datasheet*/



void USART_init(void);
void USART_Transmit(unsigned char data);
void USART_Transmit_float(float USART_data);
unsigned char USART_Receive( void );
void USART_Transmit_string(char* StringPtr);

void USART_init(void)
{
	UCSRA=(1<<U2X); //double transmission speed
	
	//UCSRB=(1<<RXEN) | (1<<TXEN); //enable UART as transmitter and receiver.
	
	UCSRB |=(1<<RXEN) | (1<<TXEN)| (1<<RXCIE);// enavle receive interrupt
	
	UCSRC=(1<<URSEL) |(1<<UCSZ0) | (1<<UCSZ1); //8-bit data, NO parity, one stop bit and asynchronous
	
	// baud rate=9600 & Fosc=8MHz -->  UBBR=103 for equation
	//baud rate=57600 & Fosc=8MHz -->  UBBR=16 for equation
	UBRRH=0;
	UBRRL=103;
}

void USART_Transmit( unsigned char data )	//155. Datasheet
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) )
	;
	/* Put data into buffer, sends the data */
	UDR = data;
}

unsigned char USART_Receive( void )			//258. Datasheet
{
	/* Wait for data to be received */
	while ( !(UCSRA & (1<<RXC)) )
	;
	/* Get and return received data from buffer */
	return UDR;
}

void USART_Transmit_float(float USART_data)	//End line 0x0A
{
	uint8_t i=0;
	char CSV_data[10];
	
	dtostrf(USART_data,7,3,CSV_data);	
	while (CSV_data[i]!=0)
	{
		USART_Transmit(CSV_data[i]);
		i++;
	}
}

void USART_Transmit_int(int int_to_string)	//End line 0x0A
{
	uint8_t Ns=10;
	uint8_t i=0;
	char string_to_USART[10];
	
	itoa(int_to_string,string_to_USART,Ns);	
	while (string_to_USART[i]!=0)
	{
		USART_Transmit(string_to_USART[i]);
		i++;
	}
}

void USART_Transmit_string(char* StringPtr)
{
	while(*StringPtr != 0x00)
	{
		USART_Transmit(*StringPtr);
		StringPtr++;
	}
}
#endif