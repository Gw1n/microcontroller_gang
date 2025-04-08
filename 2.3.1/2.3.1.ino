/* Includes */
#include <avr/io.h>
#include <string.h>

uint8_t LED13 = 1<<PB5;
void InitUSART(uint32_t baud_rate);
void ReceiveLine(char* string_input_ptr, uint16_t string_length);
uint8_t ReceiveByte(void);

int main()
{
  InitUSART(9600);
  // set up PB5, built-in LED, as output
  DDRB |= (1 << PB5);
  PORTB &= ~(1 << PB5); // turn off
  String input_string;
  String *string_input_ptr = &input_string;

  while(1)
  {
    ReceiveLine(*string_input_ptr);
    if (input_string == "HIGH")
    {
      PORTB |= (1<<PB5);
    }
    else if (input_string == "LOW")
    {
      PORTB &= ~(1<<PB5);
    }
  }
}

uint8_t ReceiveByte(void)
// USART receive function based on polling of the Receive Complete (RXC0) bit
{
// Wait for data to be received
// When the receive buffer is full the RXC0 bit in UCSR0A is set to ‘1’
// signaling that the buffer is ready to be read
do{} while (!(UCSR0A & (1<<RXC0)));
// If receive is complete, the RXCO bit is set to HIGH.
// Now data can be read out.
return UDR0;
}

void ReceiveLine(String& string_input_ref)
{
uint8_t c; // Arduino: char is int8_t but byte is uint8_t.
string_input_ref.remove(0, string_input_ref.length()); // empty string
c = ReceiveByte();
// while received character is neither '\n' nor '\r'
while((c != '\n') && (c != '\r'))
{
string_input_ref += static_cast<char>(c); // concatenate char c to String
c = ReceiveByte();
}
return;
}

void InitUSART(uint32_t baud_rate) {
// set baud rate
uint16_t ubrr_val = (F_CPU - 8 * baud_rate)/(16*baud_rate);
UBRR0 = ubrr_val;
// enable Rx, Tx, and set 8N1 protocoll
UCSR0B = (1<<RXEN0)|(1<<TXEN0);
UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}