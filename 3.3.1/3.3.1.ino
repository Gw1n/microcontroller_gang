#include <avr/io.h>
#include <util/delay.h> //including this in order to be able to produce a delay

int main(void) {
  DDRB |= (1<<PB1); // set OC1A as output (Pin D9)
  TCCR1A = (1<<COM1A0); // Toggle OC1A on Compare Match Mode
  TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10); // CTC Mode & Prescaler = 1024 hence, the frequency we need is about 15.6 KHz
  while(1) {
    /*
     * f_counter = f_clock/(2*N*(1+OCR1A))
     * N-> Prescale Value
     * OCR1A-> TOP
     * 
     * since we now have our desired counter frequencies, and we know our clock frequencies, we can easily find our TOP(OCR1A) values by this rearranged formula
     * OCR1A = (f_clock/(2*N*f_counter)) - 1
     */
  OCR1A = 7813; //for a frequency of 1 Hertz, OCR1A's value is 7813
  _delay_ms(5000); //5 second delay
  OCR1A = 7813/2; //for a frequency of 2 Hertz, we can divide the OCR1A_value_for_1Hz by 2
  _delay_ms(5000);
  OCR1A = 7813/4; //for a frequency of 4 Hertz, we can divide the OCR1A_value_for_1Hz by 4
  _delay_ms(5000);
  OCR1A = 7813/6; //for a frequency of 6 Hertz, we can divide the OCR1A_value_for_1Hz by 6
  _delay_ms(5000);
  OCR1A = 7813/8; //for a frequency of 8 Hertz, we can divide the OCR1A_value_for_1Hz by 8
  _delay_ms(5000);
  }
}
