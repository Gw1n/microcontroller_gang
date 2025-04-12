#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>   // for itoa()
#include <string.h>   // for strcmp()

void InitUSART(uint32_t baud_rate);
void InitADC(void);
uint16_t ReadADCSingleConversion(uint8_t channel);
void TransmitByte(uint8_t data);
void TransmitSensorData(uint16_t left_sensor, uint16_t right_sensor);

int main(void)
{
  InitUSART(9600);
  InitADC();
  EIMSK |= (1<<INT1); // Enable INT1
  EICRA |= (1<<ISC11); // INT1 on falling edge
  sei(); // enable interrupts
  uint8_t right_sensor_pin = 0; //ADC[0] i.e A0
  uint8_t left_sensor_pin = 1; //ADC[1] i.e A1
  uint16_t left_sensor_vals = 0;
  uint16_t right_sensor_vals = 0;
  do
  {
    left_sensor_vals = ReadADCSingleConversion(left_sensor_pin);
    right_sensor_vals = ReadADCSingleConversion(right_sensor_pin);
    TransmitSensorData(left_sensor_vals, right_sensor_vals);
    _delay_ms(100);
  }
  while(true);
}

uint16_t ReadADCSingleConversion(uint8_t channel)
{
  const uint8_t kMuxMask = ((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
  // clear MUX
  ADMUX &= ~kMuxMask;
  // set input channel
  ADMUX |= channel;
  // start single conversion
  ADCSRA |= (1<<ADSC);
  // wait until it is finished
  do {} while( ADCSRA & (1<<ADSC) );
  // return result
  return ADC;
}

void InitUSART(uint32_t baud_rate)
// sets BAUD rate depending on content in UBRR,
// enables receiver and transmitter
// sets frame format: 8 data bits, 1 (or 2) stop bit(s)
{
  // Set USART Baud rate using register UBRR0
  //official formula ubrr_val = F_CPU/(baud_rate*16) - 1;
  uint16_t ubrr_val = (F_CPU - 8 * baud_rate)/(16*baud_rate); // clever rounding
  UBRR0 = ubrr_val;
  UCSR0B = (1<<RXEN0) | (1<<TXEN0) ; // Enable receiver and transmitter
  // Set frame format: 8 data , 1 or 2 stop bits
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
  //UCSR0C |= (1 << USBS0); // 2 stop bits
}

void TransmitByte(uint8_t data)
// Transmit function based on polling of data register empty (UDRE0) bit.
{
  // Wait for empty transmit buffer using a polling strategy
  // Polling: keep on checking if something has changed
  // if UDRE0 in UCSROA register zero, then buffer is full, no writing.
  // if UDRE0 is HIGH, then the buffer is empty, and new data can be written.
  do{} while (!(UCSR0A & (1<<UDRE0)));
  // If data register empty (UDREO bit is one), put data into buffer to send
  UDR0 = data;
}

void TransmitSensorData(uint16_t left_sensor, uint16_t right_sensor)
{
  char Buffer[10];
  char text1[] = "Left Sensor Reading: ";
  uint8_t i = 0;
  
  while(text1[i] != '\0')
  {
    TransmitByte(text1[i]);
    i++;
  }
  
  itoa(left_sensor, Buffer, 10); //converting left_sensor value to decimal string
  i = 0;
  while(Buffer[i] != '\0')
  {
    TransmitByte(Buffer[i]);
    i++;
  }

  TransmitByte(' ');
  TransmitByte(' ');
  
  char text2[] = "Right Sensor Reading: ";
  i = 0;
  while(text2[i] != '\0')
  {
    TransmitByte(text2[i]);
    i++;
  }
  
  itoa(right_sensor, Buffer, 10); //converting right_sensor value to decimal string
  i = 0;
  while(Buffer[i] != '\0')
  {
    TransmitByte(Buffer[i]);
    i++;
  }
  
  TransmitByte('\n');
}

void InitADC(void)
{
  // internal Vcc (+5 V) as REF voltage
  ADMUX = (1<<REFS0);
  // prescale divison factor 128, ADC clock: 125 kHz, sampling rate: 9.6 KHz
  ADCSRA = (1<<ADPS2) | (1<<ADPS1)| (1<<ADPS0);
  // endable ADC
  ADCSRA |= (1<<ADEN);
  // do first ADC run (initialisation) to warm up
  ADCSRA |= (1<<ADSC); // start single conversion
  // Wait until first conversion is finished, ADSC bit resets to 0 on ADC complete
  do {} while ( ADCSRA & (1<<ADSC) );
  // Read ADC register to clean ADC
  uint16_t garbage = ADC;
}
