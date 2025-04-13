#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define SW2_PIN PD2
#define PD4_PIN PD4
#define PD5_PIN PD5
#define LED_PIN PB5

volatile uint16_t blink_counter = 0;
volatile uint16_t blink_interval = 500;

volatile uint8_t timerBlinkEnabled = 1; // controls whether Timer2 ISR blinks

char buffer[64];

// USART
void USART_send(char c) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void USART_sendString(const char* str) {
  while (*str) USART_send(*str++);
}

// ADC read
uint16_t readADC(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

// TIMER2 ISR (100ms)
ISR(TIMER2_COMPA_vect) {
  if (!timerBlinkEnabled) return; // don't blink if in override

  blink_counter++;
  if (blink_counter >= blink_interval / 100) {
    blink_counter = 0;
    PORTB ^= (1 << LED_PIN);
  }
}

void setup() {
  DDRB |= (1 << LED_PIN);
  PORTB &= ~(1 << LED_PIN);

  DDRD &= ~((1 << SW2_PIN) | (1 << PD4_PIN) | (1 << PD5_PIN));
  PORTD |= (1 << SW2_PIN) | (1 << PD4_PIN) | (1 << PD5_PIN);

  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);

  uint16_t ubrr = F_CPU / 16 / 9600 - 1;
  UBRR0H = (ubrr >> 8);
  UBRR0L = ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

  TCCR2A |= (1 << WGM21);
  OCR2A = 156;
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << OCIE2A);

  sei();
}

void blinkFiveTimes() {
  timerBlinkEnabled = 0;
  PORTB &= ~(1 << LED_PIN); // ensure off

  for (uint8_t i = 1; i <= 5; i++) {
    PORTB |= (1 << LED_PIN); // ON
    snprintf(buffer, sizeof(buffer), "Blink %d ON\r\n", i);
    USART_sendString(buffer);
    _delay_ms(500);

    PORTB &= ~(1 << LED_PIN); // OFF
    snprintf(buffer, sizeof(buffer), "Blink %d OFF\r\n", i);
    USART_sendString(buffer);
    _delay_ms(500);
  }

  blink_counter = 0; // reset timer blink phase
  PORTB &= ~(1 << LED_PIN);
  timerBlinkEnabled = 1;
}

void loop() {
  static uint8_t sw2Prev = 1;
  uint8_t sw2Now = (PIND & (1 << SW2_PIN));

  if (!sw2Now && sw2Prev) {
    _delay_ms(20);
    if (!(PIND & (1 << SW2_PIN))) {
      blinkFiveTimes(); // clean override!
    }
  }
  sw2Prev = sw2Now;

  // Blink frequency control
  if (timerBlinkEnabled) {
    uint8_t pd4 = !(PIND & (1 << PD4_PIN));
    uint8_t pd5 = !(PIND & (1 << PD5_PIN));

    if (pd4 && pd5)      blink_interval = 1000;
    else if (pd4)        blink_interval = 5000;
    else if (pd5)        blink_interval = 2000;
    else                 blink_interval = 500;
  }

  // ADC reading
  uint16_t batteryRaw = readADC(0);
  uint16_t potRaw     = readADC(1);
  float batteryV = batteryRaw * (5.0 / 1023.0);
  float potV     = potRaw * (5.0 / 1023.0);

  char batteryStr[8], potStr[8];
  dtostrf(batteryV, 4, 2, batteryStr);
  dtostrf(potV,     4, 2, potStr);

  snprintf(buffer, sizeof(buffer),
           "Battery: %s V | Pot: %s V | LED: %s\r\n",
           batteryStr, potStr,
           (PORTB & (1 << LED_PIN)) ? "ON" : "OFF");
  USART_sendString(buffer);

  _delay_ms(500);
}
