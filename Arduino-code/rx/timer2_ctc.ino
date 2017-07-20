// The aim is to produce an oscillation of around 40 Hz using Timer2.
// Timer2's prescaler is set to maximum (1024).
// Ftimer = 16M/1024/2 = 7812.5 Hz
// If OCR2A = 255, F_OC2A = 15625/2/256 = 30.52 Hz.
// Ttimer = 128 us.
// If OCR2A = 255, COMPA ISR is called every 256 * 128us = 32.768 ms.
// Then the output pulse's period is 65.536 ms (frequency = 15.3 Hz).
// So set OCR2A = 90 so that frequency = 41 Hz.
 
#define set_bit(sfr, bit) (sfr |= (1 << bit))
#define clear_bit(sfr, bit) (sfr &= ~(1 << bit))

//int output_pin[5] = {9, 10, 11, 12, 13};
/*
void setup() {
  for (int i=0; i<5; i++)
    pinMode(output_pin[i], OUTPUT);
  // put your setup code here, to run once:
  Setup_timer2();
  Serial.begin(115200);
  attachInterrupt(0, test, CHANGE);
  attachInterrupt(1, test2, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly: 
  if (digitalRead(2) == HIGH)
  {
    Serial.println("HIGH");
  }
}

void test()
{
  Serial.println("Interrupt");
}
void test2()
{
  Serial.println("Interrupt");
}
*/
/*
** If Timer 2 is set as CTC with prescaler = 1, the timer rate is 16 MHz.
** The frequency of the output signal is then 16MHz / ((OCR2A+1)*2).
*/
void Setup_timer2() {
  // Set the prescaler to 1024 -> CSS2:0 = b111
  set_bit (TCCR2B, CS20);
  set_bit (TCCR2B, CS21);
  set_bit (TCCR2B, CS22);
  
  // CTC timer operation
  // Compare Output Mode = Toggle OC2A on Compare Match for non-PWM mode
  // Set COM2A1:0 = b01 
  set_bit (TCCR2A, COM2A0);
  clear_bit (TCCR2A, COM2A1);
  clear_bit (TCCR2A, COM2B0);
  clear_bit (TCCR2A, COM2B1);
  
  // Waveform Generation Mode = CTC
  // Set WGM22:0 = 2 = b010
  clear_bit (TCCR2A, WGM20);  
  set_bit (TCCR2A, WGM21);
  clear_bit (TCCR2B, WGM22);

  // Set the counter limit
  OCR2A = 96;    
  
  // Turn on timer2
  set_bit (TIMSK2, OCIE2A);
}

/******************************************************************************************************************
** Definition of the Output Compare Match A Interrupt subroutine of Timer2
** The routine is called every 13 us given the timer configuration above.
** In practice, the ISR is called eery 18 us. 
** Hence, the OCR2A needs to be adjusted from 25 to 20.
** Refer to http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html for a list of interrupt vectors
******************************************************************************************************************/
ISR(TIMER2_COMPA_vect)
{
  // The interrupt subroutine does not need to do anything
  // The output at intCallPin is set to HIGH and LOW so that we can observe
  // the interrupt call on the oscilloscope. 
  if (out_on)
  {
    out_pulse ^= HIGH;
    digitalWrite(triggerPin, out_pulse);
  }
  else
  {
    digitalWrite(triggerPin, LOW);
  }
}
    

