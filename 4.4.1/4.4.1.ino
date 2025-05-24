#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#define SENSOR_LEFT_CHANNEL   PC0   // A0
#define SENSOR_RIGHT_CHANNEL  PC1   // A1
#define POT_CHANNEL           7   // A7

#define MODE_DETECT_PIN       PD4

#define M1_IN1 PB0
#define M1_IN2 PB1
#define M2_IN1 PB2
#define M2_IN2 PB3

float threshold;
uint16_t EEMEM eeprom_threshold;
ISR(INT0_vect);
const uint8_t SW2_PIN = (1 << PD2);


void ADC_init()
{
    ADMUX = (1 << REFS0); // AVcc reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable, prescaler 64
}

uint16_t read_ADC(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void PWM_init()
{

  TCCR1A |= ((1<<COM1A1)|(1<<COM1B1)|(1<<WGM11));
  //OCR1A and OCR1B as Fast PWM modes with ICR1 as TOP
  TCCR1B |= ((1<<WGM13)|(1<<WGM12)|(1<<CS11)); //prescaler of 8
  //hence we are using a PWM frequency of 125 Hz
  ICR1 = 15999;
}

void Motor1_set(uint16_t speed, bool forward)
{
    if (!forward) {
        PORTB &= ~(1 << M1_IN1);
        PORTB |= (1 << M1_IN2);
    } else {
        PORTB &= ~(1 << M1_IN2);
        PORTB |= (1 << M1_IN1);
    }
    OCR1A = speed; // EN1 connected to OC1A
}

void Motor1_set(int16_t speed)
{
    if (speed > 14399)
    {
      speed = 14399;
    }
    if (speed > 0) {
        PORTB &= ~(1 << M1_IN1);
        PORTB |= (1 << M1_IN2);
    } else {
        PORTB &= ~(1 << M1_IN2);
        PORTB |= (1 << M1_IN1);
    }
    OCR1A = abs(speed); // EN1 connected to OC1A
}

void Motor2_set(uint16_t speed, bool forward)
{
    if (!forward) {
        PORTB &= ~(1 << M2_IN2);
        PORTB |= (1 << M2_IN1);
    } else {
        PORTB &= ~(1 << M2_IN1);
        PORTB |= (1 << M2_IN2);
    }
    OCR1B = speed; // EN2 connected to OC1B
}

void Motor2_set(int16_t speed)
{
    if (speed > 14399)
    {
      speed = 14399;
    }
    if (speed > 0) {
        PORTB &= ~(1 << M2_IN2);
        PORTB |= (1 << M2_IN1);
    } else {
        PORTB &= ~(1 << M2_IN1);
        PORTB |= (1 << M2_IN2);
    }
                
    OCR1B = abs(speed); // EN2 connected to OC1B
}

void threshold_calibration()
{
        threshold = (read_ADC(SENSOR_LEFT_CHANNEL) + read_ADC(SENSOR_RIGHT_CHANNEL)) / 1.5;

        _delay_ms(50);

    eeprom_write_word(&eeprom_threshold, threshold);
}

ISR(INT0_vect)
{
  _delay_ms(5);
  if ((PIND & SW2_PIN) != 0) {
        threshold_calibration();
    }

  threshold = eeprom_read_word(&eeprom_threshold);

}

long AVRmap(uint16_t inVal, int inMin, int inMax, int outMin, int outMax)
{
  return ((long) (inVal - inMin) * (outMax - outMin) / (inMax - inMin)) - outMin;
}


int main(void)
{
    // I/O Setup
    EIMSK |= (1 << INT0);          
    EICRA = (1 << ISC01); 

    sei();

    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3); // Motor pins
    PORTD |= (SW2_PIN) | (1 << MODE_DETECT_PIN);          // Pull-ups

    ADC_init();
    PWM_init();

    threshold = 700;

    while (1) {
      uint8_t bangbang_mode = (PIND & (1 << MODE_DETECT_PIN)); // PD4 shorted to GND?
      uint16_t left_val = read_ADC(SENSOR_LEFT_CHANNEL);
      uint16_t right_val = read_ADC(SENSOR_RIGHT_CHANNEL);
      uint16_t pot_val = read_ADC(POT_CHANNEL) >> 2; // 0–255
      pot_val = AVRmap(pot_val, 0, 255, 159, 14399);

        if (bangbang_mode) {


            bool left_black = left_val < threshold;
            bool right_black = right_val < threshold;


            if (left_black && right_black) {
                Motor1_set(pot_val, 1); // forward
                Motor2_set(pot_val, 1);
            }
            else if (left_black && !right_black) {
                Motor1_set(0, 1);
                Motor2_set(pot_val, 1);
            }
            else if (!left_black && right_black) {
                Motor1_set(pot_val, 1);
                Motor2_set(0, 1);
            }
            else {
                Motor1_set(pot_val, 0); // backward
                Motor2_set(pot_val, 0);
            }
        } 
        else {
            int16_t error = right_val - left_val;
            float k = 0.005;

            if (abs(error) > 40)
            {
              int16_t speed1 = pot_val * (1.0 + ((float)error * k));
              int16_t speed2 = pot_val * (1.0 - ((float)error * k));


                Motor1_set(speed1);
                Motor2_set(speed2);
            }
            else if ((abs(error) <= 40) && (right_val > threshold))
            {
                Motor1_set(pot_val);
                Motor2_set(pot_val);
            }
            else if ((abs(error) <= 40) && (right_val < threshold))
            {
                Motor1_set(-pot_val);
                Motor2_set(-pot_val);
            }
        }
    }
}

