/*
 * ledFlicker sketch for Arduino.
 * 
 * 
 * Three-channel LED oscillator for visual experiments. 
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
 *
 * HISTORY:
 * 2010.03.19 Bob Dougherty (bobd@stanford.edu) finished a working version.
 *
 */

#define VERSION "0.2"

#include <avr/interrupt.h>
#include <avr/io.h>
// Flash library is available from http://arduiniana.org/libraries/Flash/
// We make extensive use of this so that we can be verbose in our messages
// without using up precious RAM. (Using Flash saved us over 2Kb of RAM!)
#include <Flash.h>
#include "Messenger.h"
#include "LedShift.h"


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

#ifdef __AVR_ATmega1280__
  #define led1Pin 11
  #define led2Pin 12
  #define led3Pin 13
  #define led4Pin 2
  #define led5Pin 3
  #define led6Pin 5
  #define shiftDataPin 8
  #define shiftLatchPin 7
  #define shiftEnablePin 6
  #define shiftClockPin 5
  #define NUM_WAVE_SAMPLES 600
  #define NUM_ENV_SAMPLES 60
  #define NUM_CHANNELS 6
#else
  #define led1Pin 9
  #define led2Pin 10
  #define shiftDataPin 8
  #define shiftLatchPin 7
  #define shiftEnablePin 6
  #define shiftClockPin 5
  #define NUM_WAVE_SAMPLES 200
  #define NUM_ENV_SAMPLES 30
  #define NUM_CHANNELS 2
#endif

#define INTERRUPT_FREQ 3000
// The interrupt frequency is the rate at which the waveform samples will
// play out. A higher frequency will give better fidelity for high frequency
// wavforms. However, the maximum rate is limited by the complexity of the 
// ISR (interrupt function) that is run. If you set the INTERRUPT_FREQ too high, 
// the CPU will spend all its time servicing the interrupt, effectively locking 
// it up. For the current 3-channel ISR, we can easily go at 1kHz. While 2kHz
// works, the serial port stops responding, so we can't process commands while 
// playing a wavform.  If we tighten up the ISR code, we could go faster.
// See http://www.embedded.com/columns/15201575?_requestid=291362
// http://www.cs.uiowa.edu/~jones/bcd/divide.html#fixed
// http://www.piclist.com/techref/method/math/fixed.htm


// g_interruptFreq is the same as the INTERRUPT_FREQ, except for qunatization
// error. E.g., INTERRUPT_FREQ might be 3000, but the actual frequency achieved
// is 3003.4 Hz. All calculations should use this actual frequency.
static float g_interruptFreq;

#define PI 3.14159265358979
#define TWOPI 6.28318530717959

// The maxval determines the resolution of the PWM output. E.g.:
// 255 for 8-bit, 511 for 9-bit, 1023 for 10-bit, 2047 for 11-bit, 4095 for 12 bit.
// But remember that higher resolution means slower PWM frequency.
#define PWM_MAXVAL 1023
#define PWM_MIDVAL (PWM_MAXVAL/2.0)
// Fast PWM goes twice as fast, but the pulses aren't as spectrally nice as
// the non-fast ("phase and frequency correct") PWM mode. Also, 'zero' is
// not really zero (there is a narrow spike that is visible onthe LEDs). All
// the spectral ugliness is way beyond what we can see, so fast PWM is fine if
// you don't need true zero. And, you can trade off some of the extra speed 
// for better resolution (see PWM_MAXVAL).
#define PWM_FAST_FLAG false

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

static float amplitude[NUM_CHANNELS];
static float mean[NUM_CHANNELS];
static float sineInc[NUM_CHANNELS];
unsigned int phase[NUM_CHANNELS];

// Instantiate Messenger object
Messenger message = Messenger(',','[',']'); 

// Instantiate LedShift object
// This is only used in setCurrents, but we get a weird compiler error when we put it in there.
LedShift shift = LedShift(shiftDataPin, shiftLatchPin, shiftEnablePin, shiftClockPin);

// Create the Message callback function. This function is called whener a complete 
// message is received on the serial port.
void messageReady() {
  message.echoBuffer();
  float val[10];
  int i = 0;
  if(message.available()) {
    // get the command byte
    char command = message.readChar();
    switch(command) {
    
    case '?': // display help text
      Serial << F("All ledFlicker commands must be enclosed in square brackets ([]); all characters\n");
      Serial << F("outside of the brackets are ignored. Each command begins with a single letter\n");
      Serial << F("(defined below), followed by some parameters. Parameters are separated by a comma.\n\n");
      Serial << F("Commands (optional params are enclosed in parens with default value):\n\n");
      Serial << F("[?]\n");
      Serial << F("    help (displays this text).\n");
      Serial << F("[o"); for(i=1; i<=NUM_CHANNELS; i++){ Serial << F(",val") << i; } Serial << F("]\n");
      Serial << F("    set the raw PWM outputs (0 - ") << PWM_MAXVAL << F(") for all channels.\n");
      Serial << F("[e,duration,riseFall]\n");
      Serial << F("    set the envelope duration and rise/fall times (in seconds).\n");
      Serial << F("[w,channel,frequency,amplitude,(phase=0),(mean=0.5)]\n");
      Serial << F("    set waveform params for the specified channel.\n");
      Serial << F("[p]\n");
      Serial << F("    play the waveforms.\n");
      Serial << F("[h]\n");
      Serial << F("    halt waveform playout.\n");
      Serial << F("[s]\n");
      Serial << F("    status. Returns the time remaining for current playout (0 if no playout).\n");
      Serial << F("[i]\n");
      Serial << F("    set the interrupt frequency that controls waveform sample rate (100 - 5000).\n");
      Serial << F("    A higher frequency will give better fidelity for high frequency wavforms. However, the maximum\n");
      Serial << F("    rate is limited by the complexity of the waveform code. If you set this value too high, the CPU\n");
      Serial << F("    spends all its time servicing the interrupt, effectively locking it up. \n");
      Serial << F("[c,chan1MaxValue,chan2MaxValue,chan3MaxValue]\n");
      Serial << F("    set the maximum current output for three channels. This assumes that you have an\n");
      Serial << F("    Allegro A6280 constant current source driving the LEDs and that it is connected top pins\n");
      Serial << F("    5 (data, SDI), 6 (latch, LI), 7 (enable, OEI), and 8 (clock, CI).\n"); 
      Serial << F("[v,channel]\n");
      Serial << F("    validate the specified waveform. Prints some intenral variables and waveform stats.\n");
      Serial << F("[d,channel]\n");
      Serial << F("    dump the specified wavform. (Dumps a lot of data to your serial port!\n\n");
      Serial << F("For example:\n");
      Serial << F("[e,10,0.2][w,0,2,1,0][w,1,2,1,0.334][w,2,2,1,0.667][p]\n\n");
      break;
      
    case 'o': // Set outputs
      // get incoming data
      while(message.available()) val[i++] = message.readFloat();
      if(i<NUM_CHANNELS) Serial << F("set outputs requires one parameter for each channel.\n");
      else{
        stopISR();
        for(i=0; i<NUM_CHANNELS; i++)
          setOutput(i, (unsigned int)val[i]);
      }
      break;
      
    case 'e': // Set envelope params
      // get incoming data
      while(message.available()) val[i++] = message.readFloat();
      if(i<2) Serial << F("ERROR: envelope setup requires 2 parameters.\n");
      else{
        stopISR();
        float dur = setupEnvelope(val[0], val[1]);
        Serial << F("Envelope configured; actual duration is ") << dur << F(" seconds.\n");
      }
      break;

    case 'w': // setup waveforms
      // get incoming data
      val[4] = 0.5; // default mean
      while(message.available()) val[i++] = message.readFloat();
      if(i<3){ // channel, freq and amplitude are mandatory
        Serial << F("ERROR: waveform setup requires at least 3 parameters.\n");
      }
      else{
        stopISR();
        // setup the waveform. params are: channel, freq, amp, phase, mean
        if(val[0]>=0&&val[0]<NUM_CHANNELS){
          setupWave((byte)val[0], val[1], val[2], val[3], val[4]);
          applyMeanLevel((byte)val[0]);
        }else{
          for(i=0; i<NUM_CHANNELS; i++){
            setupWave(i, val[1], val[2], val[3], val[4]);
            applyMeanLevel(i);
          }
        }
      }
      break;

    case 'p': // play waveforms
      // Reset state variables
      g_envelopeTics = 0;
      // NOTE: sineInc is not reset here. So, you must call setupWave before playing.
      startISR();
      break;

    case 'h': // halt waveform playout
      stopISR();
      break;

    case 's': // return playout status
      Serial.println(((float)g_envelopeTicsDuration-g_envelopeTics)/g_interruptFreq,3
      );
      break;
      
    case 'i': // set interrupt frequency
      if(message.available()){
        stopISR();
        val[0] = message.readInt();
        if(val[0]<100||val[0]>6000)
          Serial << F("ERROR: interrupt frequency must be >=100 and <=6000.\n");
        else{
            g_interruptFreq = SetupTimer2((int)val[0]);
            Serial << F("Interrupt Freq: ") << g_interruptFreq << F("; requested freq was: ") << (int)val[0] << F("\n");
        }
      }
      else Serial << F("ERROR: interrupt frequency command requires a frequency parameter.\n");
      break;

    case 'c': // Set LED max currents on A6280 constant current source chip
      // get incoming data
      while(message.available()) val[i++] = message.readInt();
      if(i<3)
        Serial << F("LED current requires 3 parameters.\n");
      if(val[0]<0||val[0]>127||val[1]<0||val[1]>127||val[2]<0||val[2]>127)
         Serial << F("LED current values must be >=0 and <=127.\n");
      else{
        setCurrents((byte)val[0], (byte)val[1], (byte)val[2]);
      }
      break;

    case 'v':
      if(message.available()){
        val[0] = message.readInt();
        stopISR();
        Serial << F("amplitude: ") << amplitude[(unsigned int)val[0]] << F("\n");
        Serial << F("mean: ") << mean[(unsigned int)val[0]] << F("\n");
        Serial << F("sineInc: ") << sineInc[(unsigned int)val[0]] << F("\n");
        Serial << F("envTicsDuration: ") << g_envelopeTicsDuration << F("\n");
        Serial << F("envelopeStartEnd: ")<< g_envelopeStartEnd << F("\n");
        Serial << F("envelopeEndStart: ") << g_envelopeEndStart << F("\n");
        validateWave(val[0]);
      }
      else Serial << F("ERROR: validate command requires a channel parameter.\n");
      break;

    case 'd':
      if(message.available()){
        stopISR();
        val[0] = message.readInt();
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
  Serial << F(  "* ledFlicker firmware version ") << VERSION << F("\n");
  Serial << F("* Copyright 2010 Bob Dougherty <bobd@stanford.edu>\n"); 
  Serial << F("* For more info, see http://vistalab.stanford.edu/\n");
  Serial << F("*********************************************************\n\n");
  
  // Compute the wave and envelope LUTs. We could precompute and store them in flash, but
  // they only take a a few 10's of ms to compute when we boot up and it simplifies the code.
  // However, they do cramp our use of RAM. If the RAM limit becomes a problem, we should store
  // them in flash and use PROGMEM to force the compiler to read directly from flash and not 
  // load them into RAM.
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
  unsigned int pwmFreq = SetupTimer1(PWM_MAXVAL, PWM_FAST_FLAG);
  Serial << F("PWM Freq: ") << pwmFreq << F(" Hz; Max PWM value: ") << PWM_MAXVAL << F("\n");   

  Serial << F("Initializing waveform interrupt on timer 2.\n");
  g_interruptFreq = SetupTimer2(INTERRUPT_FREQ);
  Serial << F("Interrupt Freq: ") << g_interruptFreq << F("; requested freq was: ") << INTERRUPT_FREQ << F("\n");
  
  Serial << F("Configuring constant current source shift registers.\n");
  // TO DO: get these from EEPROM!
  byte rc = 64;
  byte gc = 64;
  byte bc = 64;
  setCurrents(rc, gc, bc);

  // Set defaults
  Serial << F("Setting default waveform.\n");
  for(int i=0; i<NUM_CHANNELS; i++){
    setupWave(i, 2.0, 1.0, i/3.0, 0.5);
    applyMeanLevel(i);
  }
  setupEnvelope(3.0, 0.2);

  // Attach the callback function to the Messenger
  message.attach(messageReady);
  
  Serial << F("ledFlicker Ready.\n");
  Serial << F("freeMemory() reports ") << message.FreeMemory() << F(" bytes free.\n\n");
}

void loop(){
  // The following line is the most effective way of using Serial and Messenger's callback
  while ( Serial.available() )  message.process(Serial.read () );
}


unsigned int SetupTimer1(unsigned int topVal, bool fastPwm){
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
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  TCCR1A |= (1 << COM1A1); 
  TCCR1A &= ~(1 << COM1A0); 
  TCCR1A |=  (1 << COM1B1); 
  TCCR1A &= ~(1 << COM1B0);   
#if NUM_CHANNELS > 2
  // For arduino mega, we can use 4 more outputs
  pinMode(led3Pin, OUTPUT);
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
  pinMode(led4Pin, OUTPUT);
  pinMode(led5Pin, OUTPUT);
  pinMode(led6Pin, OUTPUT);
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
  unsigned int pwmFreq;
  if(fastPwm)
    pwmFreq = (unsigned int)((float)F_CPU/(1.0+topVal)+0.5);
  else
    pwmFreq = (unsigned int)((float)F_CPU/(2.0*topVal)+0.5);

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

void setupWave(byte chan, float freq, float amp, float ph, float mn){
  static unsigned int maxWaveIndex = NUM_WAVE_SAMPLES-1;
  
  // Phase comes in as a relative value (0-1); convert to the index offset.
  phase[chan] = ph*maxWaveIndex;
  // Amplitude is 0-1
  amplitude[chan] = amp;
  // Mean is in relative 0-1. If it is not set to 0.5, then you will get clipped waveforms at high amplitude.
  mean[chan] = mn*PWM_MAXVAL;
  // the incremetor determines the output freq.
  // Wew scale by NUM_WAVE_SAMPLES/g_interruptFreq to convert freq in Hz to the incremeter value.
  sineInc[chan] = freq*NUM_WAVE_SAMPLES/g_interruptFreq;
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

void applyMeanLevel(byte chan){
  // Set PWM output to mean level
  setOutput(chan, (unsigned int)(mean[0]+0.5));
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
unsigned int updateWave(byte chan, unsigned int curTics, unsigned int envIndex){
  unsigned int val;
  
  unsigned int sineIndex = (unsigned long int)((sineInc[chan]*curTics+0.5)+phase[chan])%NUM_WAVE_SAMPLES;

  // Testing: mn=511.5;amp=1.0;env=1.0; w=floor(env.*amp.*sin([0:.01:2*pi]).*511.5+mn+0.5); [min(w) max(w) mean(w)]
  val = (unsigned int)(g_envelope[envIndex]*amplitude[chan]*g_sineWave[sineIndex]+mean[chan]+0.5);
  if(val<0) val = 0;
  else if(val>PWM_MAXVAL) val = PWM_MAXVAL;
  return(val);
}

void validateWave(byte chan){
  unsigned int maxVal = 0.0;
  unsigned int minVal = 65535.0;
  float mnVal = 0.0;
  unsigned int val;

  for(unsigned int i=0; i<g_envelopeTicsDuration; i++){
    val = updateWave(chan, i, getEnvelopeIndex(i));
    if(val>maxVal) maxVal = val;
    if(val<minVal) minVal = val;
    mnVal += (float)val/g_envelopeTicsDuration;
  }
  Serial << F("Wave #") << chan << F(" [min,mean,max]: ") << minVal << F(",") << (int)(mnVal+0.5) << F(",") << maxVal << F("\n");
}

void dumpWave(byte chan){
  unsigned int val;
  Serial << F("wave=[");
  for(int i=0; i<g_envelopeTicsDuration; i++){
    val = updateWave(chan, i, getEnvelopeIndex(i));
    Serial << val << F(",");
  }
  Serial << F("];\n");
}

// Timer2 CTC interrupt vector handler
// (for overflow, use TIMER2_OVF_vect)
// see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1215675974/0
// and http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1216085233
ISR(TIMER2_COMPA_vect) {
  shift.Enable(); // We can use the enable pin to test ISR timing
  unsigned int envInd = getEnvelopeIndex(g_envelopeTics);
  OCR1A = updateWave(0, g_envelopeTics, envInd);
  OCR1B = updateWave(1, g_envelopeTics, envInd);
#if NUM_CHANNELS > 2
  OCR1C = updateWave(2, g_envelopeTics, envInd);
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
  
  shift.Disable(); // We can use the enable pin to test ISR timing
}

void setCurrents(byte r, byte g, byte b){
  long unsigned int p;
  
  // Not sure why I have to do this many times for it to 'take'.
  p = shift.BuildColorPacket(1023, 1023, 1023);
  for(int i=0; i<100; i++){
    shift.SendPacket(p);
    shift.Latch();
  }
  //Serial << F("Sent color packet: ") << p << F("\n");
    
  p = shift.BuildCommandPacket(r, g, b);
  shift.SendPacket(p);
  shift.Latch();
  //Serial << F("Sent command packet: ") << p << F("\n");
  //shift.SetCurrents(r, g, b);
  Serial << F("Current command packet sent: \n"); 
  Serial << F("   Red: ") << (int)r << F(" = ") << shift.GetCurrentPercent(r) << F("%\n");
  Serial << F(" Green: ") << (int)g << F(" = ") << shift.GetCurrentPercent(g) << F("%\n");
  Serial << F("  Blue: ") << (int)b << F(" = ") << shift.GetCurrentPercent(b) << F("%\n");
  
}

