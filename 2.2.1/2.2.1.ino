#include <avr/io.h>
void InitUSART(uint32_t baud_rate);
void TransmitByte(uint8_t data);
uint8_t ReceiveByte(void);

void InitUSART(uint32_t baud_rate) {
  uint16_t ubrr_val = (F_CPU - 8 * baud_rate)/(16 * baud_rate);
  UBRR0 = ubrr_val;
  UCSR0B = (1 << RXEN0)|(1 << TXEN0); // enable transmitter and receiver
  UCSR0C = (1 << UCSZ01)|(1 << UCSZ00); // set data bits to 8 (stop bit is 1 by default)
}

void TransmitByte(uint8_t data) {
  do{} while (!(UCSR0A & (1 << UDRE0))); // wait until the transmit buffer flag is set
  UDR0 = data;
} 

uint8_t ReceiveByte(void) {
  do{} while (!(UCSR0A & (1 << RXC0))); // wait until the receive buffer flag is set
  return UDR0;
}

int main(void) {
InitUSART(9600);
while(true) {
  TransmitByte(ReceiveByte());
}
return (0);
}
