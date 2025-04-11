#include <avr/io.h>

void InitUSART(uint32_t baud_rate);
void TransmitByte(uint8_t data);

void InitUSART(uint32_t baud_rate){
  uint16_t ubrr_val = (F_CPU - 8*baud_rate)/(16*baud_rate);
  UBRR0 = ubrr_val;
  UCSR0B = (1<<RXEN0)|(1<<TXEN0); //enabling USART receiver and transmitter
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

void TransmitByte(uint8_t data) {
do{} while (!(UCSR0A & (1<<UDRE0)));    //Waits until the data register is empty and ready to transmit
UDR0 = data;     //LOADS the byte into USART register
}

void TransmitString(const char* sent_string){
  uint16_t i = 0;
  while (sent_string[i]!='\0'){
    TransmitByte(*(sent_string+i));
    i++;
  }
}

int main(void){
  InitUSART(9600);
  //test code
  TransmitByte('H');
  TransmitByte(105);
  //task mentioned 
  char message1[] = "\nHello IRO!";
  TransmitString(message1);
  char message2[] = {'\n','H','e','l','l','o',' ','B','R','O','!','\0'};
  TransmitString(message2);

  return(0);
}
