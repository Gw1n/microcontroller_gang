#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#define SENSOR_LEFT_CHANNEL   PC0   // A0
#define SENSOR_RIGHT_CHANNEL  PC1   // A1
#define POT_CHANNEL           PC2   // A2

#define SW2_PIN               PD2
#define MODE_DETECT_PIN       PD4

#define M1_IN1 PB0
#define M1_IN2 PB0
#define M2_IN1 PB3
#define M2_IN2 PB3

uint16_t threshold;
uint16_t EEMEM eeprom_threshold;

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
    // Fast PWM 8-bit: WGM13:0 = 0b0101 (Mode 5)
    TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
    TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11); // Prescaler 8, Fast PWM 8-bit
    ICR1 = 15999;
}

void Motor1_set(uint8_t speed, uint8_t forward)
{
    if (forward) {
        PORTB |= (1 << M1_IN1);
        PORTB &= ~(1 << M1_IN2);
    } else {
        PORTB &= ~(1 << M1_IN1);
        PORTB |= (1 << M1_IN2);
    }
    OCR1A = speed; // EN1 connected to OC1A (PB1 or PD6 if you switch timers)
}

void Motor2_set(uint8_t speed, uint8_t forward)
{
    if (forward) {
        PORTB |= (1 << M2_IN1);
        PORTB &= ~(1 << M2_IN2);
    } else {
        PORTB &= ~(1 << M2_IN1);
        PORTB |= (1 << M2_IN2);
    }
    OCR1B = speed; // EN2 connected to OC1B
}

void threshold_calibration()
{
    while (!(PIND & (1 << SW2_PIN))) {
        threshold = (read_ADC(SENSOR_LEFT_CHANNEL) + read_ADC(SENSOR_RIGHT_CHANNEL)) / 2;
        _delay_ms(50);
    }
    eeprom_write_word(&eeprom_threshold, threshold);
}

int main(void)
{
    Serial.begin(9600);
    // I/O Setup
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3); // Motor pins
    DDRD &= ~(1 << SW2_PIN) & ~(1 << MODE_DETECT_PIN);         // Inputs
    PORTD |= (1 << SW2_PIN) | (1 << MODE_DETECT_PIN);          // Pull-ups

    ADC_init();
    PWM_init();

    uint8_t bangbang_mode = !(PIND & (1 << MODE_DETECT_PIN)); // PD4 shorted to GND?

    if (!(PIND & (1 << SW2_PIN))) {
        threshold_calibration();
    }

    threshold = eeprom_read_word(&eeprom_threshold);
    if (threshold == 0xFFFF) threshold = 512;

    while (1) {
      //uint8_t bangbang_mode = !(PIND & (1 << MODE_DETECT_PIN)); // PD4 shorted to GND?
        if (bangbang_mode) {
          //Serial.println("Test");
            uint16_t left_val = read_ADC(SENSOR_LEFT_CHANNEL);
            uint16_t right_val = read_ADC(SENSOR_RIGHT_CHANNEL);
            //uint8_t pot_val = read_ADC(POT_CHANNEL) >> 2; // 0–255
            uint8_t pot_val = 128;

            uint8_t left_black = left_val < threshold;
            uint8_t right_black = right_val < threshold;

            //Serial.println(right_val);

            if (left_black && right_black) {
                Motor1_set(pot_val, 1); // forward
                Motor2_set(pot_val, 1);
            }
            else if (left_black && !right_black) {
                Motor1_set(pot_val / 2, 1);
                Motor2_set(pot_val, 1);
            }
            else if (!left_black && right_black) {
                Motor1_set(pot_val, 1);
                Motor2_set(pot_val / 2, 1);
            }
            else {
                Motor1_set(pot_val, 0); // backward
                Motor2_set(pot_val, 0);
            }
        } else {
            Motor1_set(0, 1);
            Motor2_set(0, 1);
        }

        _delay_ms(50);
    }
}
