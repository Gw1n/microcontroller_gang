#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

const uint8_t LED_P1 = (1<<PD3);      //pin D3
const uint8_t LED_BuiltIn = (1<<PB5); //pin D13
const uint8_t SERVO = (1<<PB1);       //pin D9
const uint8_t POT = (1<<PC0);         //pin A0

void init_Timer1();
void init_Timer2();
void init_Timer0();
void initADC(void);
uint16_t ReadADCSingleConversion(uint8_t channel);
void ServoMovement(uint16_t PotADC_values);
long AVRmap(uint16_t inVal,int inMin,int inMax,int outMin,int outMax);
ISR(TIMER0_OVF_vect); //Timer 0 Overflow Interrupt
volatile int overflowCounter = 0;  
//counter for counting timer 2's overflow
//notice how this is a volatile int 
//done in order to not face loss of data during compiler optimization!

int main(void)
{
  initADC();      //initialising Analog->Digital Conversion
  init_Timer0();  //initialising Timer 0
  init_Timer1();  //initialising Timer 1
  init_Timer2();  //initialising Timer 2
  int i; 
  //defining LED_P1 as an OUTPUT
  DDRD |= (LED_P1);
  //defining SERVO and the Built in LED at D13 as OUTPUTS
  DDRB |= ((SERVO) | (LED_BuiltIn));
  
  sei(); //enables interrupts
  
  while(1)
  {
    //fade in LOOP for LED 
    for(i = 0; i < 255; i++) 
    {
      OCR2B = i; //OCR2B is the TOP value for counter 2
      _delay_ms(5);
      ServoMovement(ReadADCSingleConversion(POT)); 
      //reading the potentiometer values with the inner function
      //mapping the servo motor's pulse with the outer function 
      //(mapped as per potentiometer values)
    }
    //fade out LOOP for LED
    for(i; i >= 0;i--)
    {
      OCR2B = i;      
      _delay_ms(5);
      ServoMovement(ReadADCSingleConversion(POT));
    }
  }}
  
ISR(TIMER0_OVF_vect) //Interrupt for timer 2 OVERFLOW
{
  if(ReadADCSingleConversion(POT) > 512) //1023 -> 5V hence, 2.5 = 1024/2 = 512
  {
    if(overflowCounter == 31) //Timer0 overflows every 16.384 ms -> 500 ms = 31 overflows approx
    {
      PORTB ^= LED_BuiltIn; //toggles the built in LED
      overflowCounter = 0;  //resets our Timer 2 overflow counter
    }
    else
    overflowCounter++;
  }
}

void initADC(void) //initializing ADC 
{
  // internal Vcc (+5 V) as REF voltage
  ADMUX = (1<<REFS0);       // prescale divison factor 128, ADC clock: 125 kHz, sampling rate: 9.6 KHz
  ADCSRA = (1<<ADPS2) | (1<<ADPS1)| (1<<ADPS0);  // endable ADC
  ADCSRA |= (1<<ADEN);      // do first ADC run (initialisation) to warm up
  ADCSRA |= (1<<ADSC);      // start single conversion
  
  // Wait until first conversion is finished, ADSC bit resets to 0 on ADC complete
  do {} while (ADCSRA & (1<<ADSC) );
  // Read ADC register to clean ADC
  uint16_t garbage = ADC;
}

uint16_t ReadADCSingleConversion(uint8_t channel) //Function for ADC conversion of single value
{
  const uint8_t kMuxMask = ((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
  ADMUX &= ~kMuxMask;   // clear MUX
  ADMUX |= channel;     // set input channel
  ADCSRA |= (1<<ADSC);  // start single conversion
  
  // wait until it is finished
  do {} while( ADCSRA & (1<<ADSC) );
  // return result
  return ADC;
}

void init_Timer0() //initialization of Timer 0
{
  //No COM bits need to be initialised here since we aren't using the Pins connected to the timer directly
  //we need to use NORMAL mode
  TCCR0A = 0;
  //turning this Wave form generator bit ON sets our Timer/Counter operation mode to NORMAL mode
  TCCR0B |= ((1<<CS02) | (1<<CS00));
  //we must set CS02 = 1 and CS00 = 1 in order to get 1024 pre scaler division factor
  //this way, MAX value is reached every 16.38 ms
  TIMSK0 |= (1<<TOIE0);//this enables the Timer/Counter Overflow interrupt
}

void init_Timer1() //servo timer (Timer 1 initialization)
{//this is a 16 bit counter and hence we must limit it's top
  TCCR1A |= ((1<<WGM11)|(1<<COM1A1)) ;
  //WGM11=1, WGM12=1, WGM13 =1 -> Fast PWM Mode, sets it to fast pwm with ICR1 as top
  //COM1A0 -> Clear OC1A (Pin D9) [turn off Pin D9 when TOP value reached]
  TCCR1B |= ((1<<WGM12)|(1<<WGM13)|(1<<CS12)|(1<<CS10));
  //CS10=1,CS12=1 -> clk_IO/1024 (from prescaler) 
  //hence we are setting a pre scaler divion factor of 1024
  //ICR1 -> TOP value of Timer 1
  ICR1 = 313; 
  //running clock at 16000 Mhz/1024 = 15.625 Khz
  //15.625 Khz--> triggers an event every 16.38 ms
  //timer ticks 15,625 times per second
  //1 tick = 1 / 15625 = 0.000064 sec = 64 µs
  //we need servo pulses for a total time period of 20 ms(i.e., 50 Hz PWM)
  //hence, 20 ms/64 µs = 20,000 µs/64 µs = 312.5 -> 313 approx
  //hence, our TOP value must be 312 (total number of pulses within 20 ms)
  // (OCR1A/ICR1)*100% = Duty Cycle%
  //we will map our Potentiometer Readings to our OCR1A register which oscillates the Voltage to our SERVO pin at OC1A
}

void init_Timer2() //LED(P1) timer
{ //this is an 8 bit counter whose top is just 255,
  //so needn't be limited
  TCCR2A |= ((1<<COM2B1) | (1<<WGM21) | (1<<WGM20));
  //for fast PWM -> WGM21 = 1, WGM20 = 1  
  //for Top to be set to 255 (0xFF),
  //instead of a custom OCRA during Fast PWM, 
  //we must choose this waveform config
  //COM2B = [1 0] -> Clear OC2B on compare match
  TCCR2B |= ((1<<CS22) | (1<<CS21) | (1<<CS20));
}

void ServoMovement(uint16_t PotADC_values)
{
  //0.5 ms (2.5% Duty Cycle) is at one extreme position; 
  //1.5 ms (7.5% Duty Cycle) is at the center; 
  //2.5 ms (12.5% Duty Cycle) is at the other extreme position.
 
  //left extreme = 0.5 ms (2.5% Duty Cycle)--> (2.5/100)*312 = 8
  //right extreme = 2.5 ms (12.5% Duty Cycle)--> (12.5/100)*312 = 39
  //hence,ICR1 = 312
  //lower limit = 8, upper limit = 39
  OCR1A = AVRmap(PotADC_values, 0, 1023, 8, 39); 
  //OCR1A keeps counting till ICR1 value reached
  //OCR1A is our servo signal pulse pin PB1 (pin D9)
}

//our very own AVR MAP function (similar to Arduino Map)
//inputs: input_values, input range, output range
long AVRmap(uint16_t inVal,int inMin,int inMax,int outMin,int outMax)
{
  //Simple math
  return ((long)(inVal-inMin)*(outMax-outMin)/(inMax-inMin)) + outMin;
}
