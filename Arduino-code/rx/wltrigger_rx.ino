/*
** Receiver unit
*/

#define IRFrequency          38000        // The IR emitter is operating at 38 kHz
#define IRDuration           1            // The IR emitter is on for 1 ms
#define IRDurationMicro      500          // bit duration of 0.5 ms 

#define IRSensorPin          3
#define triggerPin           2
//#define IRreadtestPin        13


int prev_sensor_output = LOW;

// Define the output pulse variables
int out_pulse = LOW;
boolean out_on = false;

void setup()
{
  Serial.begin(115200);
  pinMode(IRSensorPin, INPUT);
  pinMode(triggerPin, OUTPUT);
//  pinMode(IRreadtestPin, OUTPUT);
  Setup_timer2();
}

void loop()
{
  //out_on = true;
  // Check if the transmitter sends data
  byte pulse_width = 0;
  int sensor_output = digitalRead(IRSensorPin);
  if ((sensor_output == LOW) && (prev_sensor_output == HIGH))    // A start bit is detected
  {
    // If the start bit is detected, read the data
    pulse_width = IRread();
    Serial.println(pulse_width);
    // Send the trigger signal
    //send_pulse_signal(pulse_width);
    send_osc_signal(pulse_width);
  }
  // Keep track of the sensor output
  prev_sensor_output = sensor_output;
}

void send_pulse_signal(int pulse_width)
{
  digitalWrite(triggerPin, HIGH);
  delay(pulse_width);
  digitalWrite(triggerPin, LOW);
}

void send_osc_signal(int pulse_width)
{
  out_on = true;
  delay(pulse_width);
  out_on = false;
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
//  digitalWrite(IRreadtestPin, HIGH);
//  delayMicroseconds(IRDuration*1000*1.5);  
    delayMicroseconds(IRDuration*1000*0.75);  
//  delay(IRDuration);
  byte data = 0;
  byte mask = 1;
  for (int i=0; i<8; ++i)
  {
    int inbit = digitalRead(IRSensorPin);
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

