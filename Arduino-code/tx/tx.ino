#define DEBUG_MODE 1

// Define the pins
#define PIN_AUDIO                     A0
#define PIN_VISUAL_1                  2
#define PIN_VISUAL_2                  3
#define PIN_DIGITAL_1                 4
#define PIN_DIGITAL_2                 5
#define PIN_DIGITAL_3                  6
#define PIN_DIGITAL_4                 7
#define PIN_DIGITAL_5                 8
#define PIN_DIGITAL_6                 9
#define PIN_DIGITAL_7                 10
#define PIN_DIGITAL_8                 11
#define PIN_TEST                      12
#define PIN_IR_OUTPUT                 13

// Define the interrupts
#define ITR_VISUAL_1                  0
#define ITR_VISUAL_2                  1

// Define the audio parameters
#define ANAREF              511           // Analogue reference which corresponds to 2.5V (half of the power supply rail)
#define ANATH               560           // Analogue amplitude threshold
#define BUFSIZE             300           // Buffer size of the analogue samples
int audio_buffer[BUFSIZE];                // Buffer to store the analogue samples

// Define the ADC prescaler value
// Fadc = 16M/PS_xx. E.g. Fadc = 16M/PS_32 = 500,000 Hz
// Fsamp = Fadc/13.  E.g. Fsamp = 500,000/13 = 38 ksa/s, Tsamp = 26 us
const uint8_t PS_16 = (1 << ADPS2);
const uint8_t PS_32 = (1 << ADPS2) | (1 << ADPS0);
const uint8_t PS_64 = (1 << ADPS2) | (1 << ADPS1);
const uint8_t PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// Define the visual parameters
uint8_t visual_marker[2] = {100, 200};

// Define the digital input parameters
uint8_t digital_curr[8] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
uint8_t digital_prev[8] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
uint8_t digital_marker[8] = {40, 70, 100, 130, 160, 190, 220, 250};

// Define the output parameters
#define IRFrequency           38000        // The IR emitter is operating at 38 kHz
// Settings for achieving a bit duration of 0.5 ms
#define IRDurationMicro      250           // Ideal value = 500 ms
#define NUMCOUNT             40            // Ideal value = 38 (where 76 == 1 ms)

int outPulse = LOW;                        // Status of the output pulse at pin 13
volatile int toggleCount = 0;

// Define the switch modes
#define AUDIO_MODE_LEVEL      256
#define DIGITAL_MODE_LEVEL    512
#define SERIAL_MODE_LEVEL     768
#define INPUT_MODE_DEV        128

// Declare the variable type: InputTriggerMode and the functions associated with the variable type.
// If the function declaration is not included here, the compiler will return an error: InputTriggerMode does not name a type.
enum InputTriggerMode { AUDIO_INPUT, DIGITAL_INPUT, SERIAL_INPUT };
InputTriggerMode trigger_mode;
// Declare functions related to InputTriggerMode
InputTriggerMode read_trigger_mode();

// Define the variables to store the time when a stimulus is detected
// so that we can ignore any trailing part of the stimulus (ringing or bouncing)
uint32_t test_detected = 0;
const uint32_t test_wait = 500;
uint32_t audio_detected = 0;
const uint32_t audio_wait = 500;

uint8_t visual_on = 0;
uint32_t visual_detected = 0;
const uint32_t visual_wait = 500;


void setup()
{
  Serial.begin(115200);
  output_init();
  digital_init();
  visual_init();
  //  pinMode(PIN_DIGITAL_6, INPUT_PULLUP);
  audio_init();
  Setup_timer2();
}

void loop()
{

  // Check if the momentary button attached to pin D9 was depressed (pulled to LOW)
  // D9 is designated as PIN_DIGITAL_6
  // If the button is pressed, send 100 to the IR LED
  if ((digitalRead(PIN_DIGITAL_6) == LOW) && ((millis() - test_detected) > test_wait))
  {
    Serial.println("TEST");
    writeToIR(100);
    test_detected = millis();
  }

  //  if (Serial.available())
  //  {
  //    run_serial_handler();
  //  }

  run_visual();
  run_audio();
  //  run_digital();

  /*
    //**** PERHAPS THE TRIGGER MODE IS NOT NECESSARY - JUST COMBINE AUDIO AND DIGITAL
    //**** 8X DIGITALREAD TAKES 40 US - NOT ISSUE WITH AUDIO
    //**** AUDIO TAKES 10 MS FOR BUFFERING WHICH CAN BE ISSUE WITH DIGITAL BUT DIGITAL IS HARDLY USED.
    // Determine the input mode using the analogue input A1
    trigger_mode = read_trigger_mode();
    // Generate the pulse_width according to the stimulus type
    switch(trigger_mode)
    {
      case AUDIO_INPUT:
        run_audio();
        break;
      case DIGITAL_INPUT:
        run_digital();
        break;
    }
  */
}

InputTriggerMode read_trigger_mode()
{
  int inputModeRaw = analogRead(A1);                                   // Read the raw input mode from analogue input A1
  if ( abs(inputModeRaw - AUDIO_MODE_LEVEL) < INPUT_MODE_DEV )         // Check the input mode is audio mode
  {
    return AUDIO_INPUT;
  }
  else if ( abs(inputModeRaw - DIGITAL_MODE_LEVEL) < INPUT_MODE_DEV )  // Check the input mode is digital mode
  {
    return DIGITAL_INPUT;
  }
  else if ( abs(inputModeRaw - SERIAL_MODE_LEVEL) < INPUT_MODE_DEV )  // Check the input mode is serial mode
  {
    return SERIAL_INPUT;
  }
}

void visual_init()
{
  pinMode(PIN_VISUAL_1, INPUT);
  pinMode(PIN_VISUAL_2, INPUT);
  // Set the interrupt for the visual trigger
  attachInterrupt(ITR_VISUAL_1, visual_1_handler, CHANGE);
  attachInterrupt(ITR_VISUAL_2, visual_2_handler, CHANGE);
}

void visual_1_handler()
{
  //  Serial.println("Visual 1");
  visual_on = 1;
#if DEBUG_MODE == 1
  Serial.println("Visual 1");
#endif
}
void visual_2_handler()
{
  //  Serial.println("Visual 2");
  visual_on = 2;
#if DEBUG_MODE == 1
  Serial.println("Visual 2");
#endif
}

void run_visual()
{
  if ((visual_on > 0) && ((millis() - visual_detected) > visual_wait))
  {
    writeToIR(visual_marker[visual_on - 1]);
    visual_detected = millis();
    visual_on = 0;
  }
}
void audio_init()
{
  // Initialise the ADC
  pinMode(PIN_AUDIO, INPUT);
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  ADCSRA |= PS_32;    // set our own prescaler to 32 -> 31,250 sps (Tsamp = 32us)
  // Initialise the ADC buffer
  for (int i = 0; i < BUFSIZE; ++i)
  {
    audio_buffer[i] = 0;
  }
}

void run_audio()
{
  if ((millis() - audio_detected) > audio_wait)
  {
    // Store the most recent sample to the buffer
    //#if DEBUG_MODE == 1
    //  unsigned long ts = micros();
    //#endif
    audio_buffer[0] = analogRead(A0);
    //#if DEBUG_MODE == 1
    //  unsigned long tf = micros();
    //  Serial.println(tf-ts);
    //    Serial.println(audio_buffer[0]);
    //#endif
    // Check if the amplitude exceeds the threshold.
    // If so, an audio tone has been detected and it will start acquiring a frame of samples for analysis.
    if (audio_buffer[0] > ANATH)
    {
      //digitalWrite(testPin, HIGH);
      byte num_cross = 0;
      // Fill the buffer with the samples
      for (int n = 1; n < BUFSIZE; ++n)
      {
        audio_buffer[n] = analogRead(A0);
        // Analyse the samples to determine the type of tones
        // Evaluate the number of times the signal crosses the analogue reference as an estimate to the frequency.
        // Only use positive-going edge.
        // Assume that the sampling rate is 10 ksamp/sec. Hence 1kHz tone should have 10 positive-going edge.
        // In practice, 12 edges were returned for 1kHz tone and 14 edges for 1.2kHz.
        if ((audio_buffer[n] >= ANAREF) && (audio_buffer[n - 1] < ANAREF))
        {
          ++num_cross;
          //#if DEBUG_MODE == 1
          //    Serial.print("X: "); Serial.println(num_cross);
          //#endif
        }
      }
      //digitalWrite(testPin, LOW);
      // Return the pulse width value based on the number of oscillations
      writeToIR(num_cross * 10);
#if DEBUG_MODE == 1
      Serial.print("Marker: "); Serial.println(num_cross * 10);
#endif
      audio_detected = millis();
    }
  }
}


void run_serial_handler(void)
{
  read_from_serial_port();
  if (valid_serial_command())
  {
    // Read the command and parameters
    // Execute the command
  }
  else if (valid_number_from_serial())
  {
    // Write the number to the IR output
  }

}
void read_from_serial_port()
{
}

boolean valid_serial_command()
{
}
boolean valid_number_from_serial()
{
}

void output_init()
{
  pinMode(PIN_TEST, OUTPUT);
  pinMode(PIN_IR_OUTPUT, OUTPUT);
}
void digital_init()
{
  pinMode(PIN_DIGITAL_1, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_2, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_3, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_4, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_5, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_6, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_7, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_8, INPUT_PULLUP);
}
void run_digital()
{
  digital_curr[0] = digitalRead(PIN_DIGITAL_1);
  digital_curr[1] = digitalRead(PIN_DIGITAL_2);
  digital_curr[2] = digitalRead(PIN_DIGITAL_3);
  digital_curr[3] = digitalRead(PIN_DIGITAL_4);
  digital_curr[4] = digitalRead(PIN_DIGITAL_5);
  digital_curr[5] = digitalRead(PIN_DIGITAL_6);
  digital_curr[6] = digitalRead(PIN_DIGITAL_7);
  digital_curr[7] = digitalRead(PIN_DIGITAL_8);

  for (int i = 0; i < 8; ++i)
  {
    if ((digital_curr[i] == LOW) && (digital_prev[i] == HIGH))
    {
      writeToIR(digital_marker[i]);
    }
  }

  for (int i = 0; i < 8; ++i)
    digital_prev[i] = digital_curr[i];
}

/*
** writeToIREmitter sends a byte of data to the IR emitter.
** The LSB is sent out first.
** Bit 0 switches on the IR emitter while bit 1 switches it off. (active low)
** The duration of each bit is 3 ms.
** The start and stop bits are 0.
*/
void writeToIR(byte data)
{
  // Write the start bit = 0 -> IR emitter ON
  // The ISR is called every 13 us.
  // If the duration is 3 ms and the signal period is 26 us, there must be 115 pulses.
  // Therefore the number of toggles is 115*2 = 230.
  IRon();

  // Write the data bits
  for (int i = 0; i < 8; ++i)
  {
    if ((data & 0x01) == true)
    {
      IRoff();
    }
    else
    {
      IRon();
    }
    data >>= 1;
  }

  // Write the stop bit = 0
  //  IRon();
}

/*
** Switches on the IR emitter. The duration is determined by toggleCount.
*/
void IRon()
{
  toggleCount = NUMCOUNT;
  while (toggleCount > 0);
}

/*
** Switches off the IR emitter by setting toggleCount to zero and using delay(IRDuration).
*/
void IRoff()
{
  toggleCount = 0;
  delayMicroseconds(IRDurationMicro);
  //  delay(IRDuration);
}

