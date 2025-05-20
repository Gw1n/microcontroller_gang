#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>

void InitUSART(uint32_t baud_rate);
uint8_t ReceiveByte(void);
void controlMotor(uint8_t motorNum, float speed);
void InitTimer1();
void InitMotors();

void InitUSART(uint32_t baud_rate) {
  uint16_t ubrr_val = (F_CPU - 8 * baud_rate)/(16 * baud_rate);
  UBRR0 = ubrr_val;
  UCSR0B = (1 << RXEN0)|(1 << TXEN0); // enable transmitter and receiver
  UCSR0C = (1 << UCSZ01)|(1 << UCSZ00); // set data bits to 8 (stop bit is 1 by default)
}

uint8_t ReceiveByte(void) {
  do{} while (!(UCSR0A & (1 << RXC0))); // wait until the receive buffer flag is set
  return UDR0;
}

void InitTimer1(){
// Clear OC1A and OC1B on Compare Match / Set OC1A and OC1B at Bottom;
// Wave Form Generator: Fast PWM 14, TOP = ICR1
TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); // prescaler = 8
ICR1 = 15999;
// PWM frequency = (16000000 Hz)/8/(TOP+1) = (1000 Hz)/8 = 125 Hz
// The value ICR1 together with OCR1A and OCR1B determines the duty cycles
// observed at pin OC1A (PB1, Pin15) and OC1B (PB2, Pin16), respectively.
}

void InitMotors() {
    // Direction control pins as output: PB0 and PB3
    // PWM output pins as output: PB1 (OC1A connected to ENA), PB2 (OC1B connected to ENB)
    DDRB |= (1 << PB0) | (1 << PB3) | (1 << PB1) | (1 << PB2);
}

void controlMotor(uint8_t motorNum, float speed) {
    // uint8_t absSpeed = abs(speed);
    if (motorNum == 1) { // right motor
        if (speed >= 0) {
            PORTB &= ~(1 << PB0);
            PORTB |= (1 << PB1); // forward movement
        } else {
            PORTB &= ~(1 << PB1); // backwards movement
            PORTB |= (1 << PB0);
        }
        OCR1A = abs(speed)*ICR1; // PWM for Motor 1
    } else if (motorNum == 2) { // left motor
        if (speed >= 0) {
            PORTB &= ~(1 << PB3); 
            PORTB |= (1 << PB2); // forwards movement
        } else {
           PORTB &= ~(1 << PB2); // backwards movement
           PORTB |= (1 << PB3);
        }
        OCR1B = abs(speed)*ICR1; // PWM for Motor 2
    }
}

int main(void) {  
  InitUSART(9600);
  InitTimer1();
  InitMotors();
  while (1) {
        char cmd = ReceiveByte();
        switch (cmd) {
            case 'w': // forward
                controlMotor(1, 1);
                controlMotor(2, 1);
                break;
            case 's': // backward
                controlMotor(1, -1);
                controlMotor(2, -1);
                break;
            case 'a': // turn left
                controlMotor(1, 0.25);
                controlMotor(2, 0);
                break;
            case 'd': // turn right
                controlMotor(1, 0);
                controlMotor(2, 0.25);
                break;
            default:
                controlMotor(1, 0);
                controlMotor(2, 0);
                break;
        }
    }
}
