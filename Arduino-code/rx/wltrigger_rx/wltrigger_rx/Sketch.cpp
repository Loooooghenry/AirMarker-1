// wireless trigger receiver project
//
// The IC sleeps mostly and consumes only 7 uA.
// When a switch connected to INT1 is asserted to low, the interrupt ISR is executed.
// Then, the output pin (pin #2) is set HIGH for 200 ms. The current consumed is 8 uA.
// When an LED is connected to the output pin, it will consume 13 mA.

#include <Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>


#define ledpin 13

#define IRSensorPin          3
#define triggerPin           2

#define IRFrequency          38000        // The IR emitter is operating at 38 kHz
#define IRDurationStart      325            // The IR emitter is on for 1 ms
#define IRDurationMicro      250          // bit duration of 0.5 ms
//#define IRDurationStart      1500            // The IR emitter is on for 1 ms
//#define IRDurationMicro      1000          // bit duration of 0.5 ms

#define set_bit(sfr, bit) (sfr |= (1 << bit))
#define clear_bit(sfr, bit) (sfr &= ~(1 << bit))


int prev_sensor_output = LOW;

// Define the output pulse variables
int out_pulse = LOW;
boolean out_on = false;
boolean signal_received = false;

void setup_timer2();
void fake_msdelay(int x);
void fake_usdelay(int x);
void send_pulse_signal(unsigned int pulse_width);
void send_osc_signal(unsigned int pulse_width);
byte IRread();
void loop_actual();
void loop_test();
void test_itr();

void setup() {
  // Set the pins
  for(int x = 1 ; x < 18 ; x++){
    pinMode(x, INPUT);
    digitalWrite(x, LOW);
  }

  pinMode(IRSensorPin, INPUT_PULLUP); //This is the main button, tied to INT1
  //digitalWrite(3, HIGH); //Enable internal pull up on button
  pinMode(triggerPin, OUTPUT);

  // Configure the sleep
  set_sleep_mode(SLEEP_MODE_STANDBY);
  //set_sleep_mode(SLEEP_MODE_PWR_SAVE);   // Need to use Timer2
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  //pinMode(ledpin, OUTPUT);
  
  // Shut down various parts
  ADCSRA &= ~(1<<ADEN); //Disable ADC
  ACSR = (1<<ACD); //Disable the analog comparator
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = (1<<AIN1D)|(1<<AIN0D); //Disable digital input buffer on AIN1/0

  power_twi_disable();
  power_spi_disable();
  power_usart0_disable();
  power_timer0_disable(); //Needed for delay_ms
  power_timer1_disable();
  power_timer2_disable(); //Needed for asynchronous 32kHz operation
  //power_adc_disable();    // Makes no difference to the current consumption


  //Setup TIMER2
  setup_timer2();
  //TCCR2A = (1<<COM2A0) | (1<<WGM21); // Toggle OC2A on compare match & use CTC mode
  //TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20); // Set CLK/1024
  //OCR2A = 96;
  //TIMSK2 = (1<<TOIE2); //Enable the timer 2 interrupt

  //TCCR2A = 0x00;
  ////TCCR2B = (1<<CS22)|(1<<CS20); //Set CLK/128 or overflow interrupt every 1s
  //TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20); //Set CLK/1024 or overflow interrupt every 8s
  //ASSR = (1<<AS2); //Enable asynchronous operation
  //TIMSK2 = (1<<TOIE2); //Enable the timer 2 interrupt
  
  //Setup external INT1 interrupt
  EICRA = (1<<ISC11); //Interrupt on falling edge
  EIMSK = (1<<INT1); //Enable INT1 interrupt

  //attachInterrupt(digitalPinToInterrupt(IRSensorPin), test_itr, FALLING);	

  sei(); //Enable global interrupts
}

ISR(TIMER2_COMPA_vect)
{
  // The interrupt subroutine does not need to do anything
  // The output at intCallPin is set to HIGH and LOW so that we can observe
  // the interrupt call on the oscilloscope.
  if (out_on)
  {
    out_pulse ^= HIGH;
  }
  else
  {
    out_pulse = LOW;
  }
  digitalWrite(triggerPin, out_pulse);
}
// No difference between SIGNAL and ISR
// There is a delay of about 2.2 ms between the onset of the interrupt (physical) signal and the start of the ISR
// The large delay may be due to the power down mechanism.
//SIGNAL(INT1_vect){
ISR(INT1_vect){
  //When you hit the button, we will need to display the time
 digitalWrite(triggerPin, HIGH);
 digitalWrite(triggerPin, LOW);

  if (signal_received == false) {
    signal_received = true;
  }
}

void test_itr()
{
  digitalWrite(triggerPin, HIGH);
  digitalWrite(triggerPin, LOW);
  if (signal_received == false) {
    signal_received = true;
  }

}

void loop()
{
  loop_actual();
 //loop_test();
}

void loop_test()
{
  power_timer0_enable();

  send_pulse_signal(500);
  delay(500);
}

void loop_actual() {
  // put your main code here, to run repeatedly:
  sleep_mode();

  //digitalWrite(ledpin, HIGH);
  //delay(1000);
  //digitalWrite(ledpin, LOW);
  //delay(1000);
  unsigned int pulse_width = 0;

  if (signal_received)
  {
//digitalWrite(triggerPin, HIGH);
//digitalWrite(triggerPin, LOW);
    
    //set_sleep_mode(SLEEP_MODE_PWR_SAVE); // Switch to power save to use TIMER2
    //set_sleep_mode(SLEEP_MODE_IDLE);
    EIMSK = (0<<INT1);                   // Disable the interrupt
    //detachInterrupt(digitalPinToInterrupt(IRSensorPin));

    power_timer0_enable();
    //send_pulse_signal(50);

    power_timer2_enable();
    //send_pulse_signal(50);

    pulse_width = IRread();
    //send_pulse_signal(50);

    setup_timer2();
    //send_pulse_signal(50);

    ////TIMSK2 = (1<<TOIE2);
    //
    ////send_pulse_signal(50);
    ////send_osc_signal(50);
    //send_pulse_signal(100);
    //send_pulse_signal(pulse_width/2);
    send_osc_signal(pulse_width/2);       // The actual delay is 2x than specified here for using 8 MHz internal RC OSC
    //
    power_timer2_disable();

    power_timer0_disable();
    
    //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    signal_received = false;
    EIMSK = (1<<INT1);
    sei(); //Enable global interrupts
  //attachInterrupt(digitalPinToInterrupt(IRSensorPin), test_itr, FALLING);

  }
}

void setup_timer2() {
  TCCR2A = (1<<COM2A0) | (1<<WGM21); // Toggle OC2A on compare match & use CTC mode
  TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20); // Set CLK/1024
  ASSR = (0<<AS2); //Disnable asynchronous operation
  OCR2A = 96;
  TIMSK2 = (1<<OCIE2A); //Enable the timer 2 interrupt
}

void fake_msdelay(int x){
  for( ; x > 0 ; x--)
  fake_usdelay(1000);
}

void fake_usdelay(int x){
  for( ; x > 0 ; x--) {
    __asm__("nop\n\t");
    //__asm__("nop\n\t");
    //__asm__("nop\n\t");
    //__asm__("nop\n\t");
    //__asm__("nop\n\t");
    //__asm__("nop\n\t");
    //__asm__("nop\n\t");
  }
}

void send_pulse_signal(unsigned int pulse_width)
{
  digitalWrite(triggerPin, HIGH);
  //fake_msdelay(pulse_width);
  delay(pulse_width);
  //delayMicroseconds(pulse_width*100);
  digitalWrite(triggerPin, LOW);
}

void send_osc_signal(unsigned int pulse_width)
{
  digitalWrite(triggerPin, HIGH);
  digitalWrite(triggerPin, LOW);
  out_pulse = LOW;
  out_on = true;
  delay(pulse_width);
  out_on = false;
  //digitalWrite(triggerPin, HIGH);
  //delay(1);
  //digitalWrite(triggerPin, LOW);
  digitalWrite(triggerPin, LOW);
}

/*
** IRread detects the state of the IR sensor and interprets it to bits.
** After reading 8 bits and detecting the stop bit, the data is returned.
** - Wait for 4 ms which is about at the middle of the first bit.
** - Read the first bit.
** - Wait for 3 ms and read the second bit.
** - And so forth
*/
byte IRread()
{
digitalWrite(triggerPin, HIGH);
digitalWrite(triggerPin, LOW);

  //  digitalWrite(IRreadtestPin, HIGH);
  //  delayMicroseconds(IRDuration*1000*1.5);
  delayMicroseconds(IRDurationStart);
  //  delay(IRDuration);
  byte data = 0;
  byte mask = 1;
  for (int i=0; i<8; ++i)
  {
    int inbit = digitalRead(IRSensorPin);
    //digitalWrite(triggerPin, inbit);
    if (inbit == HIGH)
    {
      data |= mask;
    }
    mask <<= 1;
    //delay(IRDuration);
    delayMicroseconds(IRDurationMicro);
  }
  //  digitalWrite(IRreadtestPin, LOW);
  return data;
}
