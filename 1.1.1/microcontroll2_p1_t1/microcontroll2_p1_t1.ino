// not necessary since arduino sets it by default #define F_CPU16000000UL
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>


int main (void)
{
  const uint8_t led_1 = (1<<PD5); //00000001 << PD5 = 00100000
  const uint8_t led_2 = (1<<PD6); //00000001 << PD6 = 01000000
  //hence portD pin5 and pin6 are outputs
  //on the arduino borad, these are the D5 and D6 pins
  PORTD = 0; //initializing in the start everything to output LOW (0 Volts)

  // |= is the SET BIT function which sets the given Bit masks to DDRD
  
  DDRD |= (led_1 | led_2); //SETS the Pins on port D (so PortD now looks like: 01100000)
  PORTD |= led_1; //Setting led_1 to HIGH(5V)
  while(1) //while(true) i.e forever loop
  {
    _delay_ms(1000);
    // here we are toggling between the two bitmasks
    PORTD ^=(led_1|led_2); //TOGGLE OPERATION i.e toggles between Led_1 and Led_2
  }
}
