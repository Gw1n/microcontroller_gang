/* Includes */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

/* Global Variables */
const uint8_t LED1 = 1<<PD5; //pin D5
const uint8_t LED2 = 1<<PD6; //pin D6
const uint8_t LED3 = 1<<PD7; //pin D7
const uint8_t LED4 = 1<<PB0; //pin D8
const uint8_t LED5 = 1<<PB1; //pin D9
const uint8_t LED6 = 1<<PB2; //pin D10
const uint8_t LED7 = 1<<PB3; //pin D11
const uint8_t LED8 = 1<<PB4; //pin D12
const uint8_t SWITCH1 = 1<<PD4; //pin D4
const uint8_t SWITCH2 = 1<<PD3; //pin D3
const uint8_t SWITCH3 = 1<<PD2; //pin D2
//SWITCH 3 = PD2 since this corresponds to interrupt vector INT0 



/* Function Declarations */
ISR(INT0_vect);
void startLEDseq(const int delay_in_ms);
void delay_(int i);

int main(void)
{
    /* Setup */
    EIMSK |= (1 << INT0);          // Enable ext. INT1
    EICRA = (1 << ISC01);          // External interrupt on falling edge for INT0

    sei();                         // enable interrupts

    // set pull-up at PD3 and PD2 and PD4
    PORTD |= (SWITCH1 | SWITCH2 | SWITCH3);    
              
    // set pins for LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8 to output pins
    DDRD |= (LED1 | LED2 | LED3);         
    DDRB |= (LED4 | LED5 | LED6 | LED7 | LED8);

    //initially, turn OFF the LEDs
    PORTD &= ~(LED1 | LED2 | LED3);
    PORTB &= ~(LED4 | LED5 | LED6 | LED7 | LED8);
    /* Event Loop */
    while (1)
    {
      //if Switch 1 is pressed
        if(!(PIND & SWITCH1))
        { 
          //runs the LED sequence at a 500 ms interval speed
          startLEDseq(500);
        }
      //if Switch 2 is pressed
        else if(!(PIND & SWITCH2))
        {
         //runs the LED sequence at a 200 ms interval speed
          startLEDseq(200);
        }
    }
    return 0;

}

//must create own delay function due to C++ not allowing any variables as parameters for delay
void delay_(int i) 
{
  if(i == 200 || !(PIND & SWITCH2)) 
  //this allows us to press and hold mid sequence to change frequency to intervals of 200 ms for LED sequence 
  {
    _delay_ms(200);
  }
  else if(i == 500 || !(PIND & SWITCH1))
  //this allows us to press and hold mid sequence to change frequency to intervals of 500 ms for LED sequence 
  {
    _delay_ms(500);
  }
}

//We are defining an external interrupt here 
//for the case where we do not want our first and last LEDs to react with the sequence
ISR(INT0_vect){
    _delay_ms(5);
    if(!(PIND & SWITCH3)) //to check if switch still pressed
    {
      //strategy used: we are setting the respective LEDs as INPUTS instead of OUTPUTS
      //to make them unresponsive to LED sequence function
      DDRD &= ~(LED1); //first LED
      DDRB &= ~(LED8); //last LED
    }
}

//function which runs the LED loop as per the task
void startLEDseq(const int delay_in_ms) //input-> the delay at which the loop must run
{
  //since our interrupt turns these LEDs off sometimes, we are setting them back to OUTPUTS
    DDRD |= (LED1);
    DDRB |= (LED8);
    
    for(int i = 0; i < 3; i++)
    {//alternatively turning ON and OFF the desired LEDs as per sequence
      PORTD |= (LED1 | LED2 | LED3);
      delay_(delay_in_ms);
      PORTD ^= LED1;
      PORTB ^= (LED4);
      delay_(delay_in_ms);
      PORTD ^= (LED2);
      PORTB ^= (LED5);
      delay_(delay_in_ms);
      PORTD ^= (LED3);
      PORTB ^= (LED6);
      delay_(delay_in_ms);
      PORTB ^= (LED4 | LED7);
      delay_(delay_in_ms);
      PORTB ^= (LED5);
      PORTB ^= (LED8);
      delay_(delay_in_ms);
      PORTB ^= (LED8);
      PORTB ^= (LED5);
      delay_(delay_in_ms);
      PORTB ^= (LED4 | LED7);
      delay_(delay_in_ms);
      PORTD ^= (LED3);
      PORTB ^= (LED6);
      delay_(delay_in_ms);
      PORTD ^= (LED2);
      PORTB ^= (LED5);
      delay_(delay_in_ms);
      PORTB ^= (LED4);
      PORTD ^= LED1;
      if(i==2) //makes sure in the last run no extra delay is added!
      delay_(delay_in_ms); 
    }
    PORTD &= ~(LED1 | LED2 | LED3);
    PORTB &= ~(LED4);
 }
