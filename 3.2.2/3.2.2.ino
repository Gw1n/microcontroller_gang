#include <avr/io.h>

int main(void)
{
  DDRD = (1<<PD5) | (1<<PD6);

  TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00) | (1<<WGM01);
  TCCR0B = (1<<CS01);
  OCR0A = 50;
  OCR0B = 205;

  while(1){;}
}