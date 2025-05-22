#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
const uint8_t S1 = (1<<PD2);
const uint8_t S2 = (1<<PD3);
const uint8_t S3 = (1<<PD4);
const uint8_t S4 = (1<<PD5);
const uint8_t Capacitor_Power = (1<<PB1);
const uint8_t Capacitor_Data = (  PC0);

void InitUSART(uint32_t baud_rate);
void TransmitByte(uint8_t data);
void TransmitLine(const char* text);
void InitADC(uint8_t channel);
//volatile used in order to avoid any compiler optimizaton to the values read 
//into adc_result (since the values are changing very frequently)
void init_square_wave(double freq_hz);
void init_S3_square_wave();
void stop_square_wave(void);

volatile uint16_t adc_result = 0; 
bool mode_S3 = false;
volatile int t1_ovf_counter = 0;


int main(void) 
{
  InitUSART(9600);
  sei();

  DDRB |= Capacitor_Power;
  DDRD &= ~(S1 | S2 | S3 | S4 );        // Set PD2–PD5 as input
  PORTD |= (S1 | S2 | S3 | S4 );        // Enable internal pull-ups
  
  InitADC(Capacitor_Data);
  char buffer[10]; 
  //a Serial Buffer to simply store the serial data as a string form (converted from int in this case) before sending data to Serial Monitor
  uint32_t Voltage = 0;
  while(1)  
  {

    if(!(PIND & S1))
    {
      _delay_ms(20);  // debounce delay
      if (!(PIND & S1)) {
        mode_S3 = false;
        stop_square_wave();
        DDRB |= Capacitor_Power;
        PORTB |= (Capacitor_Power);
        //this will show the charging behaviour of the capacitor with a stable 5V charging voltage.
      }
    }
    else if(!(PIND & S2)) 
    {
      _delay_ms(20);  // debounce delay
      if (!(PIND & S2)) {
        mode_S3 = false;
        DDRB |= Capacitor_Power;
        init_square_wave(0.144); 
      //this is at the cut off frequency of 0.144 Hz using formula cut off freq = 1/(2*pi*R*C)
      //OCR1A i.e PB1 i.e Capacitor_Power will now be a square wave output (HIGH AND LOW AT 50% duty cycle) 
      //Hence will generate a time period of 6.9 seconds
      }
    }
    else if(!(PIND & S3))
    {
      _delay_ms(20);  // debounce delay
      if (!(PIND & S3)) {
        t1_ovf_counter = 0;
        mode_S3 = true;
        DDRB |= Capacitor_Power;
        init_S3_square_wave();    //1/10th of the cut off frequency
        //Hence will generate a time period of 69 seconds
      }
    }
    else if(!(PIND & S4))
    {
      _delay_ms(20);  // debounce delay
      if (!(PIND & S4)) {
        mode_S3 = false;
        DDRB |= Capacitor_Power;
        init_square_wave(1.44); //10x the cut off frequency
        //Hence will generate a time period of 0.69 seconds
      }
    }
    Voltage = (uint32_t)(((double)(5000.0f/1023.0f))*adc_result);
    ultoa(Voltage, buffer, 10); 
      //itoa -> unsigned long to string converter //((adc_result*5.0)/1023) -> conversion to voltage
    TransmitLine(buffer);
    TransmitLine(" V\n");
    _delay_ms(100);
  }
  return (0);
}

ISR(TIMER1_OVF_vect)
{
    if(t1_ovf_counter >= 9 && mode_S3 == true) // 7814/0.0144 = 542569 --> 542569/65535 = 8.5 --> approx 9 
    //hence we must toggle every 8 T1 overflows to generate square wave for S2 at 0.1*cutoff_freq
    {
      PORTB ^= (Capacitor_Power);
      t1_ovf_counter = 0;
    }
    else if(mode_S3 == true)
    {
      t1_ovf_counter++;
    }
}

ISR(ADC_vect) //will get called everytime a new result -> can be controlled by altering sampling frequency
{
    adc_result = ADC; // Read the latest ADC value (ADCL then ADCH)
}

void InitUSART(uint32_t baud_rate) {
  // set baud rate
  uint16_t ubrr_val =
  (F_CPU - 8 * baud_rate)/(16*baud_rate);
  UBRR0 = ubrr_val;
      // enable Rx, Tx, and set 8N1 protocoll
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void TransmitByte(uint8_t data) {
  // Wait for empty transmit buffer
  do{} while (!(UCSR0A & (1<<UDRE0)));
  // send byte
  UDR0 = data;
}

void TransmitLine(const char* text)
{
  int i = 0;
  while(text[i] != '\0')
  {
    TransmitByte(text[i]);
    i++;
  }
}

void InitADC(uint8_t channel)
{
  ADMUX=(1<<REFS0); // internal Vcc (+5 V) as REF voltage
  const uint8_t kMuxMask = ((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
  ADMUX &= ~kMuxMask; // clear MUX
  ADMUX |= channel;                       // set input channel
        // prescale divison factor 128, ADC clock speed: 125 kHz, sampling freq: 9,6 kHz
  ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADCSRA |= (1 << ADEN);                  // endable ADC
  ADCSRA |= (1 << ADATE);                 // enable automatic triggering (free running mode)
  ADCSRA |= (1 << ADIE);                  // (locally) enable ADC interrupts
  ADCSRA |= (1 << ADSC);                  // start converting
}

void init_square_wave(double freq_hz) {
    uint16_t prescaler = 1024;
    
    DDRB |= (1 << PB1);                   // Set PB1 (OC1A) as output
    TCCR1A = (1 << COM1A0);               // Toggle OC1A on compare match
    TCCR1B = (1 << WGM12);                // CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10);  // Prescaler 1024
    TIMSK1 = 0;
    OCR1A = (7813/freq_hz);
}

void init_S3_square_wave()
{
  DDRB |= (1 << PB1); 
  stop_square_wave();
    //by calling the stop_square_wave function essentially we do 
    //TCCR1A = 0; //Normal mode with TOP = MAX(65535)
    //TCCR1B = 0;
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
  TIMSK1 |= (1<<TOIE1);
}

void stop_square_wave(void)
{
    TCCR1A = 0;          // disable toggle on OC1A
    TCCR1B = 0;          // stop Timer-1 clock
}
