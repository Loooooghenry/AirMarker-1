/*
** Transmitter unit
*/

#define set_bit(sfr, bit) (sfr |= (1 << bit))
#define clear_bit(sfr, bit) (sfr &= ~(1 << bit))

//#define intCallPin    8     // Pin for checking interrupt call
//#define outTimerPin   11    // Pin of the timer output
#define outPulsePin   13    // Pin of the output pulse

boolean output_transition = false;

/******************************************************************************************************************
** Definition of Setup_timer2() function
** The clock prescaler is set to 8.
** The waveform generation mode is set to CTC.
** According to the datasheet (page 148), fOC2A = fclk / (2*N*(1+OCR2A)
** where N is the prescaler and OCR2A the counter limit.
** The counter clock frequency is 16MHz/8 = 2 MHz. (Prescaler = 8)
** The corresponding clock period is then 0.5 us. 
** The desired signal frequency is 38 kHz and the corresponding period is 26 us.
** Then the toggle period needs to be 13 us which is the interval of the ISR call.
** Therefore, OCR2A must be 13/0.5 - 1 = 25. 
** This resets the counter every (25+1)*0.6 = 13 us. 
**
** Another option:
** The counter clock frequency is 16 MHz (Prescaler = 1).
** The corresponding clock period is then 1/16 us.
** In order to set the toggle period to 13 us, the counter must reset when it reaches 13*16-1 = 207.
** Therefore OCR2A is set to 207.
**
******************************************************************************************************************/
void Setup_timer2() {

//  pinMode(intCallPin, OUTPUT);
//  pinMode(outTimerPin, OUTPUT);
  pinMode(outPulsePin, OUTPUT);
  
  outPulse = LOW;
  toggleCount = 0;  
  
  // Set the prescaler to 8
  // Set CS22:0 = b010
  // Set the prescaler to 1 -> CSS2:0 = b001
//  clear_bit (TCCR2B, CS20);
//  set_bit (TCCR2B, CS21);
//  clear_bit (TCCR2B, CS22);
  set_bit (TCCR2B, CS20);
  clear_bit (TCCR2B, CS21);
  clear_bit (TCCR2B, CS22);
  
  // CTC timer operation
  // Compare Output Mode = Toggle OC2A on Compare Match for non-PWM mode
  // Set COM2A1:0 = b01 
  set_bit (TCCR2A, COM2A0);
  clear_bit (TCCR2A, COM2A1);
  clear_bit (TCCR2A, COM2B0);
  clear_bit (TCCR2A, COM2B1);
  
  // Waveform Generation Mode = CTC
  // Set WGM22:0 = b010
  clear_bit (TCCR2A, WGM20);  
  set_bit (TCCR2A, WGM21);
  clear_bit (TCCR2B, WGM22);

  // Set the counter limit
//  OCR2A = 25;
  OCR2A = 207;    
  
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
//  digitalWrite(intCallPin, HIGH); 

  if (toggleCount > 0)
  {
    outPulse ^= HIGH;
    --toggleCount;
    digitalWrite(outPulsePin, outPulse);
    output_transition = false;
  }
  else
  {
    if (output_transition == false)
    {
      digitalWrite(outPulsePin, HIGH);
      output_transition = true;
    }
  }

//  digitalWrite(intCallPin, LOW); 
}
    
