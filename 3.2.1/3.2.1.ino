#include <avr/io.h>
#include <avr/interrupt.h>

ISR(TIMER0_OVF_vect)
{
  PORTC |= (1<<PC0) | (1<<PC1);
}

ISR(TIMER0_COMPA_vect)
{
  PORTC &= ~(1<<PC0);
}

ISR(TIMER0_COMPB_vect)
{
  PORTC &= ~(1<<PC1);
}

int main(void)
{
  DDRC = (1<<PC0) | (1<<PC1);

  TCCR0B = (1<<CS01);
  TIMSK0 |= (1<<TOIE0) | (1<<OCIE0B) | (1<<OCIE0A);
  OCR0A = 50;
  OCR0B = 205;

  sei();

  while(1){;}
}