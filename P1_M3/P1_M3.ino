int main(void) {
  const uint8_t LED13 = 1 << PB5;
  const uint8_t SWITCH = 1 << PD3;
  DDRB |= LED13;
  PORTD |= SWITCH;
  while(1) {
    if((PIND & SWITCH) == 1 << PD3) {   
      PORTB |= LED13;
    }
    else {
      PORTB &= ~LED13;
    }
  }
  return 0;
}

