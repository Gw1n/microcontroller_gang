/* Includes */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

/* Global Variables */
const uint8_t LED1 = 1 << PD5;
const uint8_t LED2 = 1 << PD6;
const uint8_t LED13 = 1<<PB5; //built in LED
const uint8_t LED4 = 1<<PD4;
const uint8_t LED5 = 1<<PD7;
const uint8_t LED6 = 1<<PB0;
const uint8_t LED7 = 1<<PB1;


const uint8_t SWITCH1 = 1<<PD3;
const uint8_t SWITCH2 = 1<<PD2;


/* Function Declarations */
ISR(INT1_vect);
void DoSomethingImportant();

int main(void)
{
    /* Setup */
    EIMSK |= (1 << INT1);          // Enable ext. INT1
    EICRA = (1 << ISC11);          // External interrupt on falling edge

    sei();                         // enable interrupts

    DDRB |= LED13;                 // set PB5 to output
    PORTD |= (SWITCH1 | SWITCH2);              // set pull-up at PD3 and PD2
    DDRD |= (LED1 | LED2);         // set pins for LED1, LED2 to output pins

    /* Event Loop */
    PORTD |= LED1;
    while (1)
    {
        DoSomethingImportant();
    }

    return 0;
}

/* Interrupt Service Routine (ISR) */
ISR(INT1_vect)
{
    _delay_ms(5); // wait
    if ((PIND & SWITCH1) == 0) // still pressed?
    {
        PORTB |= LED13;          // turn on LED13
        PORTD |= (LED1 | LED2);  // turn on LED1 and LED2
        _delay_ms(3000);         // wait 3 sec

        PORTB &= ~LED13;         // LED13 off
        PORTD &= ~(LED1);        // LED1 off
    }
    else if((PIND & SWITCH2) == 0)
    {
      PORTB |= LED13;          // turn on LED13
      PORTD &= ~LED1;
      PORTD &= ~LED2;
      for (int i = 0; i < 2; i++)
      {
        PORTD ^= LED4;
        _delay_ms(1000);
        PORTD ^= LED4;
        PORTD ^= LED5;
        _delay_ms(1000);
        PORTD ^= LED5;
        PORTB ^= LED6;
        _delay_ms(1000);
        PORTB ^= LED6;
        PORTB ^= LED7;
        _delay_ms(1000);
        PORTB ^= LED7;
      }
      PORTB &= ~LED13;         // LED13 off
    }
}


    

void DoSomethingImportant()
{
    PORTD ^= (LED1 | LED2);      // toggle LEDs
    _delay_ms(1000);
    return;
}