//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

/*
//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include <avr/interrupt.h>
#include <avr/io.h>

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

void serial_setup(long baud) {
  UBRR0H = ( baud & 0xFF00) >> 8; // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
  UBRR0L = ( baud & 0x00FF); // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
  UCSR0C |= (1 << UCSZ00) | (1 << UCSZ10); // Use 8-bit character sizes
  //UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);   // Turn on the transmission, reception, and Receive interrupt
  UCSR0B |= (1 << RXEN0);
  interrupts();
}


// serial receive interrupt
ISR(USART0_RX_vect) {
}
*/