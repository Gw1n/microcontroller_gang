#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t pwm_counter = 0;

ISR(TIMER0_OVF_vect) {
    pwm_counter++;
    if (pwm_counter >= 100) {
      pwm_counter = 0;
    }
    PORTC = 0;
    if (pwm_counter < 1)   PORTC |= (1 << PC1);  // P2 = 1%
    if (pwm_counter < 20)  PORTC |= (1 << PC2);  // P3 = 20%
    if (pwm_counter < 40)  PORTC |= (1 << PC3);  // P4 = 40%
    if (pwm_counter < 60)  PORTC |= (1 << PC4);  // P5 = 60%
    if (pwm_counter < 100) PORTC |= (1 << PC5);  // P6 = 100%
    
}

int main(void) {
    
    DDRC = (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
    TCCR0B |= (1 << CS01);      //(1<< CS00); //prescaler8,64,1024
    TIMSK0 |= (1 << TOIE0);
    sei();
    while (1) {;}
}
