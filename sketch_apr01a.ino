uint8_t setBit(uint8_t value, uint8_t bitPos) {
  return value | (1 << bitPos);
}
uint8_t clearBit(uint8_t value, uint8_t bitPos) {
  return value & ~(1 << bitPos);
}
uint8_t toggleBit(uint8_t value, uint8_t bitPos) {
  return value ^ (1 << bitPos);
}
uint8_t checkBit(uint8_t value, uint8_t bitPos) {
  return (value >> bitPos) & 1;
}
void blinkLED() {
  PORTB ^= (1 << PORTB5);
  delay(500);
}
void setup() {
 Serial.begin(9600);
 DDRB |= (1 << DDB5);
}

void loop() {
  uint8_t value = 0b00000100;
  Serial.println(setBit(value, 3),BIN);   
  Serial.println(clearBit(value, 3),BIN);  
  Serial.println(toggleBit(value, 2),BIN);
  Serial.println(checkBit(value, 3)); 
  blinkLED();
}
