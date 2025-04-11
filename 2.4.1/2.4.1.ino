#include <avr/io.h>
#include <stdint.h>

void InitADC(void);
uint16_t ReadADCSingleConversion(uint8_t);

int main(void)
{
  DDRB |= (1 << PB5);
  uint16_t adc_input = 0;
  const uint8_t kInChannel = 1;
  InitADC();
  while(1){
    adc_input = ReadADCSingleConversion(kInChannel);'
    //Basically when your input analog voltage is greater than 4V 
    if (adc_input > 818) // analog input >= 4 V
    // ADC = 4/5 * 1023 = 818 //(Vin/Vref)*[max value for 10 bits]
      PORTB |= (1 << PB5); // onboard LED on
    else
      PORTB &= ~(1 << PB5); // onboard LED off
  }
return 0;
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

uint16_t ReadADCSingleConversion(uint8_t channel)
{
  // initializing bit mask for ADMUX bits (4:0]
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
