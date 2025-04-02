#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

const uint8_t LED1 = 1<<PD5;
const uint8_t LED2 = 1<<PD6;
const uint8_t LED12 = 1<<PB4;
const uint8_t LED13 = 1<<PB5;
const uint8_t SWITCH1 = 1<<PD3;

ISR(INT1_vect);
void DoSomethingImportant();

int main()
{
  EIMSK |= (1<<INT1);
  EICRA |= (1<<ISC11);

  sei();

  DDRB |= (LED12 | LED13);
  PORTD |= SWITCH1;
  DDRD |= (LED1 | LED2);

  PORTD |= LED1;

  while(1)
  {
    DoSomethingImportant();
  }
  return 0;
}

ISR(INT1_vect)
  {
    _delay_ms(5);
    if((PIND & SWITCH1) == 0)
    {
      PORTD &= ~LED1;
      PORTD &= ~LED2;
      for (int i = 0; i < 2; i++)
      {
        PORTD ^= LED1;
        _delay_ms(1000);
        PORTD ^= LED1;
        PORTD ^= LED2;
        _delay_ms(1000);
        PORTD ^= LED2;
        PORTB ^= LED12;
        _delay_ms(1000);
        PORTB ^= LED12;
        PORTB ^= LED13;
        _delay_ms(1000);
        PORTB ^= LED13;
      }
      PORTD |= LED1;
    }
  }

void DoSomethingImportant()
{
  PORTD ^= (LED1 | LED2);
  _delay_ms(1000);
  return;
}