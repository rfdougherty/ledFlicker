/*
 * ledFlicker sketch for Arduino.
 * 
 * 
 * Six-channel LED oscillator for visual experiments. 
 * 
 *
 * Copyright 2010 Bob Dougherty.
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You might have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * TO DO:
 *   - store calibration data (emission spectra and gamma) in EEPROM and allow 
 *     the user to get it out to compute color transforms and gamma correction.
 *   - Add general-purpose commands to set/get some digital pins and ADC reads.
 *     This would be useful for, e.g., interfacing with EEG or MR scanner.
 *   - Keep track of the number of hours of service for the LEDs. This would be
 *     helpful in keeping a good calibration schedule. (E.g., if you wanted to
 *     calibrate every 10 hours of operation.)
 *
 * HISTORY:
 * 2010.03.19 Bob Dougherty (bobd@stanford.edu) finished a working version.
 * 2010.04.02 Bob Dougherty: Major overhaul to allow 6 channels on Arduino Mega.
 * I also changed the code to allow each waveform to play out on multiple channels.
 * 2010.06.30 Bob: Major overhaul to the hardware design to support high-power 
 * LEDs. We no longer use the Allegro A6280 for a constant current source, so
 * I've elimated all the code related to the current adjustment. I've also removed
 * support for non-mega Arduinos.
 * 2010.09.13 Bob: Removed old color transform code and increased resolution to
 * 12-bits. 
 */

#define VERSION "0.6"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>

// Flash library is available from http://arduiniana.org/libraries/Flash/
// We make extensive use of this so that we can be verbose in our messages
// without using up precious RAM. (Using Flash saved us over 2Kb of RAM!)
#include <Flash.h>
#include <Messenger.h>


// atmega 168  has  16k flash, 512 EEPROM, 1k RAM
// atmega 328  has  32k flash,  1k EEPROM, 2k RAM
// atmega 1280 has 128k flash,  4k EEPROM, 8k RAM
// The code below is very tight on RAM because of the wave table.

// Serial port baud rate. 115.2 kbs seems to work fine. That's ~12 bytes/msec, 
// which should keep latencies low for short (30-50 byte) commands. Data integrity
// shouldn't be a problem since we are effectively communicating over USB. But, 
// just to be sure, we'll use 57.6 kbs.
#define BAUD 57600

// F_CPU and __AVR_ATmega1280__ are defined for us by the arduino environment
// NOTE: we only support Arduino Mega!
#define PIN_LED1 11
#define PIN_LED2 12
#define PIN_LED3 13
#define PIN_LED4 2
#define PIN_LED5 3
#define PIN_LED6 5
#define NUM_WAVE_SAMPLES 600
#define NUM_ENV_SAMPLES 60
#define PIN_TEMP 0     // Analog input pin for temperture sensor
#define PIN_FAN  4     // Digital input for fan speed detector
// NUM_CHANNELS should be 6 for now.
// Values <6 should work, but only 2 and 6 have been tested.
#define NUM_CHANNELS 6

#define NUM_WAVES 2

// g_interruptFreq is the same as the INTERRUPT_FREQ, except for qunatization
// error. E.g., INTERRUPT_FREQ might be 3000, but the actual frequency achieved
// is 3003.4 Hz. All calculations should use this actual frequency.
static float g_interruptFreq;

#define PI 3.14159265358979
#define TWOPI 6.28318530717959

// The maxval determines the resolution of the PWM output. E.g.:
// 255 for 8-bit, 511 for 9-bit, 1023 for 10-bit, 2047 for 11-bit, 4095 for 12 bit.
// But remember that higher resolution means slower PWM frequency. I.e., 
// PWM freq = F_CPU/(PWM_MAXVAL+1) for fast PWM and = F_CPU/PWM_MAXVAL/2 for 
// phase/freq correct PWM. E.g., for 12-bit, PWM freq will be 1.95kHz for non-fast PWM.
#define PWM_MAXVAL 4095
#define PWM_MIDVAL (PWM_MAXVAL/2.0)
// Fast PWM goes twice as fast, but the pulses aren't as spectrally nice as
// the non-fast ("phase and frequency correct") PWM mode. Also, 'zero' is
// not really zero (there is a narrow spike that is visible onthe LEDs). All
// the spectral ugliness is way beyond what we can see, so fast PWM is fine if
// you don't need true zero. And, you can trade off some of the extra speed 
// for better resolution (see PWM_MAXVAL).
#define PWM_FAST_FLAG false

#define INTERRUPT_FREQ 2000
// The interrupt frequency is the rate at which the waveform samples will
// play out. A higher frequency will give better fidelity for high frequency
// wavforms. However, the maximum rate is limited by the complexity of the 
// ISR (interrupt function) that is run. If you set the INTERRUPT_FREQ too high, 
// the CPU will spend all its time servicing the interrupt, effectively locking 
// it up. For the current 6-channel ISR, we can easily go at 2kHz. If we tighten
// up the ISR code, we could go faster. See:
//   http://www.embedded.com/columns/15201575?_requestid=291362
//   http://www.cs.uiowa.edu/~jones/bcd/divide.html#fixed
//   http://www.piclist.com/techref/method/math/fixed.htm
// Also note that the waveforms can't be updated faster than the PWM period. 
// So, if PWM period is relatively slow (e.g., <=2kHz), it's probably a good
// idea to set the interrupt to be exactly the same as the PWM freq.

// We need some globals because of the interrupt service routine (ISR) that
// is used to play out the waveforms. ISRs can't take in parameters, but they
// can access globals.
// TODO: maybe bundle all the globals into a single struct to keep them neat?
static float g_sineWave[NUM_WAVE_SAMPLES];
static float g_envelope[NUM_ENV_SAMPLES];

static unsigned int g_envelopeTicsDuration;
static unsigned int g_envelopeStartEnd;
static unsigned int g_envelopeEndStart;
static unsigned int g_envelopeDwell;

volatile unsigned int g_envelopeTics;

static float g_amplitude[NUM_CHANNELS*NUM_WAVES];
static float g_mean[NUM_CHANNELS];
static float g_sineInc[NUM_WAVES];
unsigned int g_phase[NUM_WAVES];

//
// Define EEPROM variables for static configuration vars
//
// We don't bother setting defaults here; we'll check for valid values and set 
// the necessary defaults in the code.
//
char EEMEM gee_deviceId[16];

// Instantiate Messenger object
Messenger g_message = Messenger(',','[',']'); 

// Create the Message callback function. This function is called whener a complete 
// message is received on the serial port.
void messageReady() {
  //g_message.echoBuffer();
  float val[max(2*NUM_CHANNELS,12)];
  int i = 0;
  if(g_message.available()) {
    // get the command byte
    char command = g_message.readChar();
    switch(command) {
    
    case '?': // display help text
      Serial << F("All ledFlicker commands must be enclosed in square brackets ([]); all characters\n");
      Serial << F("outside of the brackets are ignored. Each command begins with a single letter\n");
      Serial << F("(defined below), followed by some parameters. Parameters are separated by a comma.\n\n");
      Serial << F("Commands (optional params are enclosed in parens with default value):\n\n");
      Serial << F("[?]\n");
      Serial << F("    Help (displays this text).\n");
      Serial << F("[m"); for(i=1; i<=NUM_CHANNELS; i++){ Serial << F(",val") << i; } Serial << F("]\n");
      Serial << F("    Set the mean outputs (0 - ") << PWM_MAXVAL << F(") for all channels.\n");
      Serial << F("[e,duration,riseFall]\n");
      Serial << F("    Set the envelope duration and rise/fall times (in seconds).\n");
      Serial << F("[w,waveNum,frequency,phase,amp0,amp1,...]\n");
      Serial << F("    Set waveform parameters for the specified waveform number (up to ") << NUM_WAVES << F("). Setting a\n");
      Serial << F("    waveform takes about half a millisecond for transformed color space; much less for native space.\n");      
      Serial << F("[p]\n");
      Serial << F("    Play the waveforms.\n");
      Serial << F("[h]\n");
      Serial << F("    Halt waveform playout.\n");
      Serial << F("[s]\n");
      Serial << F("    Status. Returns the time remaining for current playout (0 if no playout).\n");
      Serial << F("[i]\n");
      Serial << F("    Set the interrupt frequency that controls waveform sample rate (100 - 5000).\n");
      Serial << F("    A higher frequency will give better fidelity for high frequency wavforms. However, the maximum\n");
      Serial << F("    rate is limited by the complexity of the waveform code. If you set this value too high, the CPU\n");
      Serial << F("    spends all its time servicing the interrupt, effectively locking it up. \n");
      Serial << F("[v,waveNum]\n");
      Serial << F("    Validate the specified waveform. Prints some intenral variables and waveform stats.\n");
      Serial << F("[d,waveNum]\n");
      Serial << F("    Dump the specified wavform. (Dumps a lot of data to your serial port!\n\n"); 
      Serial << F("For example:\n");
      Serial << F("[e,10,0.2][w,0,3,0,.3,-.3,0,0,0,.9][p]\n\n");
      break;
      
    case 'm': // Set mean outputs
      while(g_message.available()) val[i++] = g_message.readInt();
      if(i!=1 && i!=NUM_CHANNELS){
        Serial << F("set outputs requires one param or ") << NUM_CHANNELS << F(" params.\n");
      }else{
        stopISR();
        if(i==1){
          setAllMeans((int)val[0]);
        }else{
          for(i=0; i<NUM_CHANNELS; i++)
            setMean(i, (int)val[i]);
        }
        applyMeans();
        Serial << F("Means set to ["); for(i=0; i<NUM_CHANNELS; i++) Serial << g_mean[i] << F(" "); Serial << F("]\n");
      }
      break;
      
    case 'e': // Set envelope params
      while(g_message.available()) val[i++] = g_message.readFloat();
      if(i<2) Serial << F("ERROR: envelope setup requires 2 parameters.\n");
      else{
        stopISR();
        float dur = setupEnvelope(val[0], val[1]);
        Serial << F("Envelope configured; actual duration is ") << dur << F(" seconds.\n");
      }
      break;

    case 'w': // setup waveforms
      while(g_message.available()) val[i++] = g_message.readFloat();
      if(i<3+NUM_CHANNELS){ // wave num, freq, phase, and amplitudes are mandatory
        Serial << F("ERROR: waveform setup requires at least 3 parameters.\n");
      }else{
        stopISR();
        // setup the waveform. params are: wave num, freq, phase, amp[]
        if(val[0]>=0&&val[0]<NUM_WAVES){
          setupWave((byte)val[0], val[1], val[2], &val[3]);
          applyMeans();
        }else{
          Serial << F("ERROR: waveform setup requires first param to be a valid wave number.\n");
        }
      }
      break;

    case 'p': // play waveforms
      // Reset state variables
      g_envelopeTics = 0;
      // NOTE: g_sineInc is not reset here. So, you must call setupWave before playing.
      startISR();
      break;

    case 'h': // halt waveform playout
      stopISR();
      applyMeans();
      break;

    case 's': // return status
      Serial.println(((float)g_envelopeTicsDuration-g_envelopeTics)/g_interruptFreq,3);
      Serial << F("LED temperature: ") << (float)getTemp() << F(" C");
      Serial << F(", Fan speed: ") << getFanSpeed() << F(" RPMs\n");
      break;
      
    case 'i': // set interrupt frequency
      if(g_message.available()){
        stopISR();
        val[0] = g_message.readInt();
        if(val[0]<100||val[0]>6000)
          Serial << F("ERROR: interrupt frequency must be >=100 and <=6000.\n");
        else{
            g_interruptFreq = SetupTimer2((int)val[0]);
            Serial << F("Interrupt Freq: ") << g_interruptFreq << F("; requested freq was: ") << (int)val[0] << F("\n");
        }
      }
      else Serial << F("ERROR: interrupt frequency command requires a frequency parameter.\n");
      break;

    case 'v':
      if(g_message.available()){
        val[0] = g_message.readInt();
        stopISR();
        Serial << F("amplitude: ") << g_amplitude[(unsigned int)val[0]] << F("\n");
        Serial << F("mean: ") << g_mean[(unsigned int)val[0]] << F("\n");
        Serial << F("sineInc: ") << g_sineInc[(unsigned int)val[0]] << F("\n");
        Serial << F("envTicsDuration: ") << g_envelopeTicsDuration << F("\n");
        Serial << F("envelopeStartEnd: ")<< g_envelopeStartEnd << F("\n");
        Serial << F("envelopeEndStart: ") << g_envelopeEndStart << F("\n");
        validateWave(val[0]);
      }
      else Serial << F("ERROR: validate command requires a channel parameter.\n");
      break;

    case 'd':
      if(g_message.available()){
        stopISR();
        val[0] = g_message.readInt();
        dumpWave((byte)val[0]);
      }
      else Serial << F("ERROR: dump command requires a channel parameter.\n");
      break;

    default:
      Serial << F("Unknown command: ") << command << F("\n");

    } // end switch
  } // end while
}

void setup(){
  Serial.begin(BAUD);
  Serial << F("*********************************************************\n");
  Serial << F("* ledFlicker firmware version ") << VERSION << F("\n");
  Serial << F("* Copyright 2010 Bob Dougherty <bobd@stanford.edu>\n");
  Serial << F("* For more info, see http://vistalab.stanford.edu/\n");
  Serial << F("*********************************************************\n\n");
  
  // Compute the wave and envelope LUTs. We could precompute and store them in 
  //flash, but they only take a few 10's of ms to compute when we boot up and it
  // simplifies the code. However, they do use much precious RAM. If the RAM 
  // limit becomes a problem, we might considerlooking into storing them in 
  // flash and using PROGMEM to force the compiler to read directly from flash.
  // However, the latency of such reads might be too long.
  Serial << F("Computing wave LUT: \n");
  unsigned long ms = millis();
  for(int i=0; i<NUM_WAVE_SAMPLES; i++)
    g_sineWave[i] = sin(TWOPI*i/NUM_WAVE_SAMPLES)*PWM_MIDVAL;
  for(int i=0; i<NUM_ENV_SAMPLES; i++)
    g_envelope[i] = 0.5 - cos(PI*i/(NUM_ENV_SAMPLES-1.0))/2.0;
  ms = millis()-ms;
  Serial << NUM_WAVE_SAMPLES << F(" samples in ") << ms << F(" miliseconds.");

  if(PWM_FAST_FLAG) Serial << F("Initializing fast PWM on timer 1.\n");
  else              Serial << F("Initializing phase/frequency correct PWM on timer 1.\n");
  float pwmFreq = SetupTimer1(PWM_MAXVAL, PWM_FAST_FLAG);
  Serial << F("PWM Freq: ") << pwmFreq << F(" Hz; Max PWM value: ") << PWM_MAXVAL << F("\n");

  Serial << F("Initializing waveform interrupt on timer 2.\n");
  if(pwmFreq < INTERRUPT_FREQ)
    g_interruptFreq = SetupTimer2(pwmFreq);
  else
    g_interruptFreq = SetupTimer2(INTERRUPT_FREQ);
  Serial << F("Interrupt Freq: ") << g_interruptFreq << F("; requested freq was: ") << INTERRUPT_FREQ << F("\n");
    
  // Set waveform defaults
  Serial << F("Initializing all waveforms to zero amplitude.\n");
  float amp[NUM_CHANNELS] = {0.0,0.0,0.0,0.0,0.0,0.0};
  for(int i=0; i<NUM_WAVES; i++) setupWave(i, 0.0, 0.0, amp);
  Serial << F("Initializing all means to ") << PWM_MAXVAL/100 << F("\n");
  setAllMeans(PWM_MAXVAL/100);
  applyMeans();
  setupEnvelope(3.0, 0.2);

  // Attach the callback function to the Messenger
  g_message.attach(messageReady);
  
  // Configure analog inputs to use the internal 1.1v reference.
  // See: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1264707156
  analogReference(2);
  //pinMode(PIN_TEMP, INPUT);  // Make sure temperature pin is set for input 
  pinMode(PIN_FAN, INPUT);
  
  Serial << F("ledFlicker Ready. Send the ? command ([?]) for help.\n");
  Serial << F("There are ") << g_message.FreeMemory() << F(" bytes of RAM free.\n\n");
}

void loop(){
  // The most effective way of using Serial and Messenger's callback:
  while ( Serial.available() )  g_message.process(Serial.read () );
}


float SetupTimer1(unsigned int topVal, bool fastPwm){
  // Set pwm clock divider for timer 1 (the 16-bit timer)
  // For CS12,CS11,CS10: 001 is /1 prescaler (010 is /8 prescaler)
  TCCR1B &= ~(1 << CS12); 
  TCCR1B &= ~(1 << CS11); 
  TCCR1B |= (1 << CS10);

  if(fastPwm){
    // mode 14 (fast, topVal is ICR1): 1,1,1,0
    TCCR1B |=  (1 << WGM13);
    TCCR1B |=  (1 << WGM12);
    TCCR1A |=  (1 << WGM11); 
    TCCR1A &= ~(1 << WGM10);
  }else{
    // mode 8 (phase & freq correct, topVal is ICR1): 1,0,0,0
    TCCR1B |=  (1 << WGM13);
    TCCR1B &=  ~(1 << WGM12);
    TCCR1A &=  ~(1 << WGM11); 
    TCCR1A &=  ~(1 << WGM10);
  }
  // Now load the topVal into the register. We can only do this after setting the mode:
  //   The ICRn Register can only be written when using a Waveform Generation mode that utilizes
  //   the ICRn Register for defining the counter’s TOP value. In these cases the Waveform Genera-
  //   tion mode (WGMn3:0) bits must be set before the TOP value can be written to the ICRn
  //   Register. (from the ATmega1280 data sheet)
  ICR1 = topVal;

  // Make sure all our pins are set for pwm
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  TCCR1A |= (1 << COM1A1); 
  TCCR1A &= ~(1 << COM1A0); 
  TCCR1A |=  (1 << COM1B1); 
  TCCR1A &= ~(1 << COM1B0);   
#if NUM_CHANNELS > 2
  // For arduino mega, we can use 4 more outputs
  pinMode(PIN_LED3, OUTPUT);
  TCCR1A |=  (1 << COM1C1); 
  TCCR1A &= ~(1 << COM1C0);
  
  TCCR3B &= ~(1 << CS32); 
  TCCR3B &= ~(1 << CS31); 
  TCCR3B |=  (1 << CS30);
  if(fastPwm){
    TCCR3B |=  (1 << WGM33);
    TCCR3B |=  (1 << WGM32);
    TCCR3A |=  (1 << WGM31); 
    TCCR3A &= ~(1 << WGM30);
  }else{
    TCCR3B |=  (1 << WGM33);
    TCCR3B &= ~(1 << WGM32);
    TCCR3A &= ~(1 << WGM31); 
    TCCR3A &= ~(1 << WGM30);
  }
  ICR3 = topVal;
  pinMode(PIN_LED4, OUTPUT);
  pinMode(PIN_LED5, OUTPUT);
  pinMode(PIN_LED6, OUTPUT);
  TCCR3A |=  (1 << COM3A1); 
  TCCR3A &= ~(1 << COM3A0); 
  TCCR3A |=  (1 << COM3B1); 
  TCCR3A &= ~(1 << COM3B0);   
  TCCR3A |=  (1 << COM3C1); 
  TCCR3A &= ~(1 << COM3C0);
#endif

  // for fast PWM, PWM_freq = F_CPU/(N*(1+TOP))
  // for phase-correct PWM, PWM_freq = F_CPU/(2*N*TOP)
  // F_CPU = CPU freq, N = prescaler = 1 and TOP = counter top value 
  float pwmFreq;
  if(fastPwm)
    pwmFreq = (float)F_CPU/(1.0+topVal)+0.5;
  else
    pwmFreq = (float)F_CPU/(2.0*topVal)+0.5;

  return(pwmFreq);
}

// 
// Timer 2 Configuration
// 
// See: http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
// 
// Configures the ATMega168 8-Bit Timer2 to generate an interrupt
// at the specified frequency. Returns the timer load value which 
// must be loaded into TCNT2 inside your ISR routine.
//
// Bit-primer:
//   Setting a bit: byte |= 1 << bit;
//   Clearing a bit: byte &= ~(1 << bit);
//   Toggling a bit: byte ^= 1 << bit;
//   Checking if a bit is set: if (byte & (1 << bit))
//   Checking if a bit is cleared: if (~byte & (1 << bit)) OR if (!(byte & (1 << bit)))
float SetupTimer2(float freq){
  unsigned int topVal;
  unsigned int ps;

  // Timer2 Prescalar setting: (0,0,0 to disable clock)
  // We want to use the fastest interrupt (smallest prescaler) possible
  // to minimize quantization error and ensure the closest match between 
  // requested and acutal frequency.
  // baseFreq is the slowest interrupt for a prescale of 1 (F_CPU/(1*(1+255))).
  float baseFreq = (float)F_CPU/256.0;
  if(freq>=baseFreq){
    ps = 1;
    TCCR2B &= ~(1<<CS22); TCCR2B &= ~(1<<CS21); TCCR2B |=  (1<<CS20);
  }else if(freq>=baseFreq/8.0){
    ps = 8;
    TCCR2B &= ~(1<<CS22); TCCR2B |=  (1<<CS21); TCCR2B &= ~(1<<CS20);
  }else if(freq>=baseFreq/32.0){
    ps = 32;
    TCCR2B &= ~(1<<CS22); TCCR2B |=  (1<<CS21); TCCR2B |=  (1<<CS20);
  }else if(freq>=baseFreq/64.0){
    ps = 64;
    TCCR2B |=  (1<<CS22); TCCR2B &= ~(1<<CS21); TCCR2B &= ~(1<<CS20);
  }else if(freq>=baseFreq/128.0){
    ps = 128;
    TCCR2B |=  (1<<CS22); TCCR2B &= ~(0<<CS21); TCCR2B |=  (1<<CS20);
  }else if(freq>=baseFreq/256.0){
    ps = 256;
    TCCR2B |=  (1<<CS22); TCCR2B |=  (1<<CS21); TCCR2B &= ~(1<<CS20);
  }else{
    ps = 1024;
    TCCR2B |=  (1<<CS22); TCCR2B |=  (1<<CS21); TCCR2B |=  (1<<CS20);
  }

  // Set up timer for CTC mode. It will fire our interrupt every time it reaches 'TOP'.
  // We just need to set the prescaler and the TOP value to achieve the desired freq.
  TCCR2A &= ~(1<<COM2A1) & ~(1<<COM2A0);         // Disconnect OC2A.
  TCCR2A &= ~(1<<COM2B1) & ~(1<<COM2B0);         // Disconnect OC2B.
  TCCR2A &= ~(1<<WGM20);  // Mode 2 - CTC (WGM2,1,0 = 0,1,0; WGM22 is in TCCR2B)
  TCCR2A |=  (1<<WGM21);
  TCCR2B &= ~(1<<WGM22);

  // Calculate the value that must be reloaded into the TOP register
  //   interruptFreq = F_CPU/(ps*(topVal+1))
  //   topVal = F_CPU/(interruptFreq*ps)-1
  // We also need to add 0.5 for proper rounding, thus:
  topVal = (unsigned int)((float)F_CPU/(ps*freq)-0.5);
  if(topVal>255) topVal = 255;
  // Now compute the exact frequency that we can achieve:
  freq = (float)F_CPU/(ps*(1.0+topVal));

  // Clear the counter
  TCNT2 = 0;

  // Set the top value in the register
  OCR2A = topVal;

  return(freq);
}

void startISR(){  // Starts the ISR
  TIMSK2 |= (1<<OCIE2A);                     // enable interrupt (calls ISR(TIMER2_COMPA_vect)
}

void stopISR(){    // Stops the ISR
  TIMSK2 &= ~(1<<OCIE2A);                    // disable interrupt
} 

void setupWave(byte wvNum, float freq, float ph, float *amp){
  static unsigned int maxWaveIndex = NUM_WAVE_SAMPLES-1;
  byte i;
  
  // Phase comes in as a relative value (0-1); convert to the index offset.
  g_phase[wvNum] = ph*maxWaveIndex;
  // Amplitude is -1 to 1 (negative inverts phase)
  if(amp!=NULL){
    // Now set the amplitudes in the global
    for(i=0; i<NUM_CHANNELS; i++)
      g_amplitude[wvNum*NUM_CHANNELS+i] = amp[i];
  }else{
    for(i=0; i<NUM_CHANNELS; i++)
      g_amplitude[wvNum*NUM_CHANNELS+i] = 0.0;
  }
  // the incremetor determines the output freq.
  // Wew scale by NUM_WAVE_SAMPLES/g_interruptFreq to convert freq in Hz to the incremeter value.
  g_sineInc[wvNum] = freq*NUM_WAVE_SAMPLES/g_interruptFreq;
}

void setOutput(byte chan, unsigned int val){
  // Set PWM output to the specified level
  if(val>PWM_MAXVAL) val = PWM_MAXVAL;
  switch(chan){
  case 0: 
    OCR1A = val; 
    break;
  case 1: 
    OCR1B = val; 
    break;
#ifdef __AVR_ATmega1280__
  case 2: 
    OCR1C = val; 
    break;
  case 3: 
    OCR3B = val; 
    break;
  case 4: 
    OCR3C = val; 
    break;
  case 5: 
    OCR3A = val; 
    break;
#endif
  }
}

void setAllMeans(int val){
  for(byte i=0; i<NUM_CHANNELS; i++)
    setMean(i,val);
}

void setMean(byte chan, int val){
  if(val<0)               val = 0;
  else if(val>PWM_MAXVAL) val = PWM_MAXVAL;
  g_mean[chan] = val;
}

void applyMeans(){
  // Set PWM output to mean level for all channels
  for(byte i=0; i<NUM_CHANNELS; i++)
    setOutput(i, (unsigned int)(g_mean[i]+0.5));
}

float setupEnvelope(float duration, float envRiseFall){
  // Configure the envelope global values
  // envelope rise/fall time is translated to the g_envelope incrementer
  if(duration*g_interruptFreq>65535.0)
    duration = 65535.0/g_interruptFreq;
  g_envelopeDwell = (unsigned int)(envRiseFall/((float)NUM_ENV_SAMPLES/g_interruptFreq)+0.5);
  g_envelopeTicsDuration = (unsigned int)(duration*g_interruptFreq);
  g_envelopeStartEnd = (NUM_ENV_SAMPLES-1)*g_envelopeDwell;
  g_envelopeEndStart = g_envelopeTicsDuration-g_envelopeStartEnd;
  // initialize the state variable
  g_envelopeTics = 0;
  return(duration);
}

unsigned int getEnvelopeIndex(unsigned int curTics){
  static const unsigned int maxEnvelopeIndex = NUM_ENV_SAMPLES-1;
  unsigned int envIndex;

  // Must be careful of overflow. For interrupt freq of 2kHz, max duation is ~ 32 secs. For 4kHz it is ~16.
  // We could switch to long ints if needed.

  if((curTics>g_envelopeStartEnd && curTics<g_envelopeEndStart) || g_envelopeDwell==0){
    envIndex = maxEnvelopeIndex;
  }
  else if(curTics<=g_envelopeStartEnd){
    envIndex = (unsigned int)((float)curTics/g_envelopeDwell+0.5);
  }
  else if(curTics>=g_envelopeEndStart){
    envIndex = (unsigned int)((float)(g_envelopeTicsDuration-curTics)/g_envelopeDwell+0.5);
  }
  // TO DO: replace floating point math with properly-rounded integer math.
  return(envIndex);
}


// This is the core function for waveform generation. It is called in the 
// ISR to output the waveform to the PWNM channels. 
void updateWave(unsigned int curTics, unsigned int envIndex, unsigned int *vals){
  byte wv, ch, ampInd;
  
  // Initialize each channel to the mean value, plus 0.5. The +0.5 is to do a proper rounding
  // when we convert from float to int below.
  for(ch=0; ch<NUM_CHANNELS; ch++) 
    vals[ch] = g_mean[ch]+0.5;

  // *** WORK HERE: the loops below can probably be optimized. E.g., we don't need
  // to clamp to the proper output range on each iteration.

  // Testing: mn=511.5;amp=1.0;env=1.0; w=floor(env.*amp.*sin([0:.01:2*pi]).*511.5+mn+0.5); [min(w) max(w) mean(w)]
  for(wv=0; wv<NUM_WAVES; wv++){
    unsigned int sineIndex = (unsigned long int)((g_sineInc[wv]*curTics+0.5)+g_phase[wv])%NUM_WAVE_SAMPLES;
    float envSine = g_envelope[envIndex]*g_sineWave[sineIndex];
    for(ch=0; ch<NUM_CHANNELS; ch++){
      ampInd = wv*NUM_CHANNELS+ch;
      vals[ch] += (unsigned int)(envSine*g_amplitude[ampInd]);
      if(vals[ch]<0) vals[ch] = 0;
      else if(vals[ch]>PWM_MAXVAL) vals[ch] = PWM_MAXVAL;
    }
  }
}

void validateWave(byte chan){
  unsigned int maxVal = 0.0;
  unsigned int minVal = 65535.0;
  float mnVal = 0.0;
  unsigned int val[NUM_CHANNELS];

  for(unsigned int i=0; i<g_envelopeTicsDuration; i++){
    updateWave(i, getEnvelopeIndex(i), val);
    if(val[chan]>maxVal) maxVal = val[chan];
    if(val[chan]<minVal) minVal = val[chan];
    mnVal += (float)val[chan]/g_envelopeTicsDuration;
  }
  Serial << F("Channel #") << chan << F(" [min,mean,max]: ") << minVal << F(",") << (int)(mnVal+0.5) << F(",") << maxVal << F("\n");
}

void dumpWave(byte chan){
  unsigned int val[NUM_CHANNELS];
  
  Serial << F("wave=[");
  for(int i=0; i<g_envelopeTicsDuration; i++){
    updateWave(i, getEnvelopeIndex(i), val);
    Serial << val[chan] << F(",");
  }
  Serial << F("];\n");
}

// Timer2 CTC interrupt vector handler
// (for overflow, use TIMER2_OVF_vect)
// see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1215675974/0
// and http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1216085233
ISR(TIMER2_COMPA_vect) {
  //g_shift.Enable(); // We can use the enable pin to test ISR timing
  static unsigned int envInd;
  static byte i;
  unsigned int val[NUM_CHANNELS];
  
  envInd = getEnvelopeIndex(g_envelopeTics);
  updateWave(g_envelopeTics, envInd, val);
  
  OCR1A = val[0];
  OCR1B = val[1];
  #if NUM_CHANNELS > 2
  OCR1C = val[2];
  OCR3B = val[3]; 
  OCR3C = val[4]; 
  OCR3A = val[5];
  #endif
  // Make the interrupt self-terminating
  if(g_envelopeTics>=g_envelopeTicsDuration){
    stopISR();
  }else{
    g_envelopeTics++;
  }
  // Note: there is no assurance that the PWMs will get set to their mean values
  // when the waveform play-out finishes. Thus, g_envelope must be designed to
  // provide this assurance; e.g., have 0 as it's first value and rise/fall >0 tics.
  
  //g_shift.Disable(); // We can use the enable pin to test ISR timing
}

float getTemp(){
  const byte numReadings = 2*-0;
  float reading;
  for(byte i=0; i<numReadings; i++){
    reading += analogRead(PIN_TEMP);
    delay(1);
  }
  reading /= numReadings;
  // converting that reading to voltage. We assume that we're using the internal 1.1v reference
  float voltage = reading * 1.1 / 1024; 
  // convert from 10 mv per degree with 500 mV offset to degrees ((volatge - 500mV) * 100)
  float temp = (voltage - 0.5) * 100;
  //if(fFlag) temp = (temp * 9 / 5) + 32;
  return(temp);
}

unsigned int getFanSpeed(){
  const unsigned long timeoutMicrosecs = 6e4; // timeout in 60 milliseconds; allows us to measure down to 1000 rpms
  unsigned long pulseDelta = pulseIn(PIN_FAN, HIGH, timeoutMicrosecs);
  //Serial << "pulseDelta=" << pulseDelta << "\n";
  unsigned int rpms = 60e6/pulseDelta;
  return(rpms);
}

