#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t pwm_counter = 0;
//percentage for each LEDs
volatile uint8_t PWM0 = 0;   
volatile uint8_t PWM1 = 1;   
volatile uint8_t PWM2 = 20;   
volatile uint8_t PWM3 = 40;   
volatile uint8_t PWM4 = 60;
volatile uint8_t PWM5 = 100; 

ISR(TIMER0_OVF_vect){//interrupt vector that gets activated when the timer gets overflowed 
    pwm_counter++;
    if (pwm_counter >= 100) {
      pwm_counter = 0;
    }
    PORTC = 0;
    if (pwm_counter < PWM1)   PORTC |= (1 << PC1);  // P2 = 1%
    if (pwm_counter < PWM2)  PORTC |= (1 << PC2);  // P3 = 20%
    if (pwm_counter < PWM3)  PORTC |= (1 << PC3);  // P4 = 40%
    if (pwm_counter < PWM4)  PORTC |= (1 << PC4);  // P5 = 60%
    if (pwm_counter < PWM5) PORTC |= (1 << PC5);  // P6 = 100%
    
}

int main(void) {
    
    DDRC = (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
    TCCR0B |= (1 << CS01);      //we use the 8-bit timer over here so the maximum value is 255
    TIMSK0 |= (1 << TOIE0);     // Enabler of the overflow interrupt
    sei();
    while (1) {;}
}
