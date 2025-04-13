#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
int main(void)
{
  uint8_t fade_done = 0; // track if fade has already run
  const uint8_t SWITCH1 = (1 << PD3);
  const uint8_t SWITCH2 = (1 << PD2); // bit masks for the switches
  PORTD |= (SWITCH1 | SWITCH2); // activate pull-up resistors
  TCCR0B = (1<<CS02) | (1<<CS00); // 1024 prescaling for Timer0 for a 61Hz output (it's as close to 50Hz as we can get)
  TCCR1B = (1<<CS12) | (1<<CS10) | (1<<WGM12); // 1024 prescaling for Timer1 and WGM12 set for fast PWM
  DDRD = (1<<PD5) | (1<<PD6) | (1<<PB1) | (1<<PB2); //Pin OC0A and OC0B as OUTPUT
  DDRB = (1<<PB1) | (1<<PB2); // Pin OC1A and OC1B as OUTPUT
  PORTD &= ~((1<<PD5) | (1<<PD6)); //Initialise pin outputs to 0
  OCR0B = 0;
  OCR0A = 0;
  OCR1B = 0;
  OCR1A = 0;//Initialise PWM values to 0
  uint8_t i_1;
  uint8_t i_2;
  while(1) {
    TCCR0A &= ~((1<<COM0A1) | (1<<COM0B1)); // Disconnect PWM from pins because LEDs won't fully turn off othwerwise
    TCCR1A &= ~((1<<COM1A1) | (1<<COM1B1));
    PORTD &= ~((1<<PD5) | (1<<PD6)); // Set pins to low
    PORTB &= ~((1<<PB1) | (1<<PB2));
    if((PIND & SWITCH1) == 0) {
      TCCR0A = (1<<COM0B1) | (1<<COM0A1) | (1<<WGM01) | (1<<WGM00);
      // Fast PWM, 8-bit, non-inverted mode
      // Clear OC0A on compare match when up-counting.
      // Clear OC0B on compare match when up-counting.
      // Set on compare match when down 
      i_1=0;
      while (i_1 < 255) {
        OCR0B = i_1;
        OCR0A = 255-i_1;
        _delay_ms(10);
        i_1++;
        }
     }
     if((PIND & SWITCH2) == 0) {  
       TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10); 
       // Fast PWM 8-bit, non-inverted mode
       // Clear OC1A on compare match when up-counting
       // Clear OC1B on compare match when up-counting
       // Set on compare match when down
       i_2 = 0;
       while (i_2 < 255) {
        OCR1B = 255-i_2;
        OCR1A = i_2;
        _delay_ms(10);
        i_2++;
       }
     }
  }
}
