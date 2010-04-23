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
 * 2010.04.02 Bob Dougherty: Major overhaul to allow 6 channels on Arduino Mega.
 * I also changed the code to allow each waveform to play out on multiple channels.
 */

#define VERSION "0.4"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>

// Flash library is available from http://arduiniana.org/libraries/Flash/
// We make extensive use of this so that we can be verbose in our messages
// without using up precious RAM. (Using Flash saved us over 2Kb of RAM!)
#include <Flash.h>
#include <Messenger.h>
#include <LedShift.h>


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
  #define PIN_LED1 11
  #define PIN_LED2 12
  #define PIN_LED3 13
  #define PIN_LED4 2
  #define PIN_LED5 3
  #define PIN_LED6 5
  #define PIN_SHIFTDATA 8
  #define PIN_SHIFTLATCH 7
  #define PIN_SHIFTENABLE 9
  #define PIN_SHIFTCLOCK 6
  #define NUM_WAVE_SAMPLES 600
  #define NUM_ENV_SAMPLES 60
  #define NUM_CHANNELS 6
#else
  #define PIN_LED1 9
  #define PIN_LED2 10
  #define PIN_SHIFTDATA 8
  #define PIN_SHIFTLATCH 7
  #define PIN_SHIFTENABLE 6
  #define PIN_SHIFTCLOCK 5
  #define NUM_WAVE_SAMPLES 200
  #define NUM_ENV_SAMPLES 30
  #define NUM_CHANNELS 2
#endif


#define NUM_WAVES 2

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


// g_interruptFreq is the same as the INTERRUPT_FREQ, except for qunatization
// error. E.g., INTERRUPT_FREQ might be 3000, but the actual frequency achieved
// is 3003.4 Hz. All calculations should use this actual frequency.
static float g_interruptFreq;

#define PI 3.14159265358979
#define TWOPI 6.28318530717959

// The maxval determines the resolution of the PWM output. E.g.:
// 255 for 8-bit, 511 for 9-bit, 1023 for 10-bit, 2047 for 11-bit, 4095 for 12 bit.
// But remember that higher resolution means slower PWM frequency. I.e., 
// PWM freq ~= F_CPU/PWM_MAXVAL for fast PWM and ~= F_CPU/PWM_MAXVAL/2 for 
// phase/freq correct PWM.
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

static float g_amplitude[NUM_CHANNELS*NUM_WAVES];
static float g_mean[NUM_CHANNELS];
static float g_sineInc[NUM_WAVES];
unsigned int g_phase[NUM_WAVES];

// The colorspace to use
static char g_colorSpace;

//
// Define EEPROM variables for static configuration vars
//
// We don't bother setting defaults here; we'll check for valid values and set 
// the necessary defaults in the code.
//
uint8_t EEMEM gee_currents[NUM_CHANNELS];
float EEMEM gee_rgb2lms[9];
char EEMEM gee_deviceId[16];

// Color transform matrices
float g_rgb2lms[9];
float g_lms2rgb[9];

// Instantiate Messenger object
Messenger g_message = Messenger(',','[',']'); 

// Instantiate LedShift object
// This is only used in setCurrents, but we get a weird compiler error when we put it in there.
LedShift g_shift = LedShift(PIN_SHIFTDATA, PIN_SHIFTLATCH, PIN_SHIFTENABLE, PIN_SHIFTCLOCK);

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
      Serial << F("    help (displays this text).\n");
      Serial << F("[m"); for(i=1; i<=NUM_CHANNELS; i++){ Serial << F(",val") << i; } Serial << F("]\n");
      Serial << F("    set the mean outputs (0 - 1.0) for all channels.\n");
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
      Serial << F("[l,m11,m12,m13,m21,m22,m23,m31,m32,m33]\n");
      Serial << F("    set the rgb2lms color transform matrix. Note the element order is row1, row2, row3.\n");
      Serial << F("    The lms2rgb matrix is also used, but it will be computed internally.\n");
      Serial << F("    Call this command with no args to see the current values.\n");      
      Serial << F("[x,l|r]\n");
      Serial << F("    set the color space transform to be used for subsequent commands. Currently supported\n");    
      Serial << F("    spaces are 'c' (human cone space) and 'n' (native LED reg,green,blue space). This will\n");
      Serial << F("    default to rgb when the firmware boots.\n");      
      Serial << F("For example:\n");
      Serial << F("[e,10,0.2][w,0,2,1,0][w,1,2,1,0.334][w,2,2,1,0.667][p]\n\n");
      break;
      
    case 'm': // Set mean outputs
      while(g_message.available()) val[i++] = g_message.readFloat();
      if(i!=1 && i!=NUM_CHANNELS){
        Serial << F("set outputs requires one param or ") << NUM_CHANNELS << F(" params.\n");
      }else{
        stopISR();
        if(i==1){
          setAllMeans(val[0]);
        }else{
          for(i=0; i<NUM_CHANNELS; i++)
            setMean(i, val[i]);
        }
        applyMeans();
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
      }
      else{
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

    case 's': // return playout status
      Serial.println(((float)g_envelopeTicsDuration-g_envelopeTics)/g_interruptFreq,3
      );
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

    case 'c': // Set LED max currents on A6280 constant current source chip
      while(g_message.available()) val[i++] = g_message.readInt();
      if(i<NUM_CHANNELS){
        Serial << F("LED current requires ") << NUM_CHANNELS << F(" parameters.\n");
      }else{
        for(i=0; i<NUM_CHANNELS; i++)
          if(val[i]<0||val[i]>127){
            Serial << F("LED current values must be >=0 and <=127.\n");
            val[0] = 255;
            break;
          }
        if(val[0]!=255){
          saveCurrents(val);
          setCurrents();
        }
      }
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

    case 'l': // Set lms2rgb color transform
      while(g_message.available()) val[i++] = g_message.readFloat();
      if(i==0){
        dumpLmsMatrix();
      }else if(i<9){
        Serial << F("rgb2lms requires 9 values!\n");
      }else{
        setLmsMatrix(val);
      }
      break;

    case 'x': // Set the current color spacem
      while(g_message.available()) val[i++] = g_message.readChar();
      if(i<1){
        Serial << F("color space Xform requires the color space code to be set!\n");
      }else{
        setColorSpace((char)val[0]);
      }
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
  unsigned int pwmFreq = SetupTimer1(PWM_MAXVAL, PWM_FAST_FLAG);
  Serial << F("PWM Freq: ") << pwmFreq << F(" Hz; Max PWM value: ") << PWM_MAXVAL << F("\n");

  Serial << F("Initializing waveform interrupt on timer 2.\n");
  g_interruptFreq = SetupTimer2(INTERRUPT_FREQ);
  Serial << F("Interrupt Freq: ") << g_interruptFreq << F("; requested freq was: ") << INTERRUPT_FREQ << F("\n");
  
  Serial << F("Configuring constant current source shift registers.\n");
  setCurrents();
  
  Serial << F("Setting color transform matrices from stored calibration data.\n");
  setLmsMatrix(NULL);

  // Set waveform defaults
  Serial << F("Setting default waveform.\n");
  float amp[NUM_CHANNELS] = {0.0,0.0,0.0,0.0,0.0,0.0};
  for(int i=0; i<NUM_WAVES; i++)
    setupWave(i, 0.0, 0.0, amp);
  setAllMeans(0.5);
  applyMeans();
  setupEnvelope(3.0, 0.2);
  
  Serial << F("Setting colorspace to native RGB. Use 'x' command to change it.\n");
  setColorSpace('n');

  // Attach the callback function to the Messenger
  g_message.attach(messageReady);
  
  Serial << F("ledFlicker Ready.\n");
  Serial << F("There are ") << g_message.FreeMemory() << F(" bytes of RAM free.\n\n");
}

void loop(){
  // The most effective way of using Serial and Messenger's callback:
  while ( Serial.available() )  g_message.process(Serial.read () );
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

void setupWave(byte wvNum, float freq, float ph, float *amp){
  static unsigned int maxWaveIndex = NUM_WAVE_SAMPLES-1;
  byte i;
  
  // Phase comes in as a relative value (0-1); convert to the index offset.
  g_phase[wvNum] = ph*maxWaveIndex;
  // Amplitude is -1 to 1 (negative inverts phase)
  if(amp!=NULL){
    if(g_colorSpace=='c'){
      // *** TODO: we can save some calcs here if we cache the backLMS values that get computed in lmsToRgb.
      float backRgb[3];
      if(NUM_CHANNELS>=3){
        for(i=0;i<3;i++) backRgb[i] = g_mean[i]/PWM_MAXVAL;
        lmsToRgb(&(amp[0]), backRgb);
      }
      if(NUM_CHANNELS>=6){
        for(i=0;i<3;i++) backRgb[i] = g_mean[i+3]/PWM_MAXVAL;
        lmsToRgb(&(amp[3]), backRgb);  
      }    
    }
    // Note that we do nothing for native ('n') colorspace.
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

void setAllMeans(float val){
  for(byte i=0; i<NUM_CHANNELS; i++)
    setMean(i,val);
}

void setMean(byte chan, float val){
  if(val<0.0)      val = 0.0;
  else if(val>1.0) val = 1.0;
  g_mean[chan] = val*PWM_MAXVAL;
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

void saveCurrents(float *newCurrents){
  byte cur[NUM_CHANNELS];
  for(int i=0; i<NUM_CHANNELS; i++)
    cur[i] = (byte)newCurrents[i];
  eeprom_write_block((void*)cur, (void*)gee_currents, NUM_CHANNELS);
}

void setCurrents(){
  byte cur[NUM_CHANNELS];
  bool err = false;
  byte i;
  
  // *** FIX ME: we should set all 6 channels!
  eeprom_read_block((void*)cur, (const void*)gee_currents, NUM_CHANNELS);
  for(i=0; i<NUM_CHANNELS; i++)
    if(cur[i]>127){
      err = true;
      break;
    }
  if(err){
    for(i=0; i<NUM_CHANNELS; i++) cur[i] = 64;
    Serial << F("Corrupt current spec in EEPROM- using defaults.\n");
  }
  g_shift.SetCurrents(cur[0], cur[1], cur[2]);
  Serial << F("Current command packet sent: \n"); 
  Serial << F("   Red: ") << (int)cur[0] << F(" = ") << g_shift.GetCurrentPercent(cur[0]) << F("%\n");
  Serial << F(" Green: ") << (int)cur[1] << F(" = ") << g_shift.GetCurrentPercent(cur[1]) << F("%\n");
  Serial << F("  Blue: ") << (int)cur[2] << F(" = ") << g_shift.GetCurrentPercent(cur[2]) << F("%\n");
}

void invertColorMatrix(float A[], float iA[]){
  // Quick-n-dirty 3x3 matrix inverse.
  // To do: check precision error (e.g., large determinant)
  float determinant = +A[0]*(A[4]*A[8]-A[7]*A[5])
                      -A[1]*(A[3]*A[8]-A[5]*A[6])
                      +A[2]*(A[3]*A[7]-A[4]*A[6]);
  float invdet = 1/determinant;
  iA[0] =  (A[4]*A[8]-A[7]*A[5])*invdet;
  iA[1] = -(A[1]*A[8]-A[2]*A[7])*invdet;
  iA[2] =  (A[1]*A[5]-A[2]*A[4])*invdet;
  iA[3] = -(A[3]*A[8]-A[5]*A[6])*invdet;
  iA[4] =  (A[0]*A[8]-A[2]*A[6])*invdet;
  iA[5] = -(A[0]*A[5]-A[3]*A[2])*invdet;
  iA[6] =  (A[3]*A[7]-A[6]*A[4])*invdet;
  iA[7] = -(A[0]*A[7]-A[6]*A[1])*invdet;
  iA[8] =  (A[0]*A[4]-A[3]*A[1])*invdet;
}

// Copies the global rgb2lms matrix from EEPROM and inverts it to also set lms2rgb.
// If rgb2lms is not null, then the EEPROM data is updated with the new matrix before setting the globals.
void setLmsMatrix(float rgb2lms[]){
  if(rgb2lms!=NULL)
    eeprom_write_block((void*)rgb2lms, (void*)gee_rgb2lms, 9*sizeof(float));
  eeprom_read_block((void*)g_rgb2lms, (const void*)gee_rgb2lms, 9*sizeof(float));
  invertColorMatrix(g_rgb2lms, g_lms2rgb); 
}

void dumpLmsMatrix(){
  Serial << F("rgb2lms: [ ") << g_rgb2lms[0] << F(",") << g_rgb2lms[1] << F(",") << g_rgb2lms[2] << F("\n");
  Serial << F("           ") << g_rgb2lms[3] << F(",") << g_rgb2lms[4] << F(",") << g_rgb2lms[5] << F("\n");
  Serial << F("           ") << g_rgb2lms[6] << F(",") << g_rgb2lms[7] << F(",") << g_rgb2lms[8] << F(" ]\n"); 
  Serial << F("lms2rgb: [ ") << g_lms2rgb[0] << F(",") << g_lms2rgb[1] << F(",") << g_lms2rgb[2] << F("\n");
  Serial << F("           ") << g_lms2rgb[3] << F(",") << g_lms2rgb[4] << F(",") << g_lms2rgb[5] << F("\n");
  Serial << F("           ") << g_lms2rgb[6] << F(",") << g_lms2rgb[7] << F(",") << g_lms2rgb[8] << F(" ]\n");
}

byte setColorSpace(char colorSpaceCode){
  if(colorSpaceCode!='n'&&colorSpaceCode!='c'){
    Serial << F("SetColorSpace: Invalid code: '") << colorSpaceCode << F("'\n");
    return(1);
  }else{
    g_colorSpace = colorSpaceCode;
  }
}

void lmsToRgb(float stim[], float backRgb[]){
  //Serial << F("lms: [ ") << stim[0] << F(",") << stim[1] << F(",") << stim[2] << F(" ]\n");
  float backLms[3];
  xformColor(backRgb, g_rgb2lms, backLms);
  float scaledStimLMS[3];
  scaledStimLMS[0] = 2.0*stim[0]*backLms[0];
  scaledStimLMS[1] = 2.0*stim[1]*backLms[1];
  scaledStimLMS[2] = 2.0*stim[2]*backLms[2];
  // Note that stim is transformed in-place
  xformColor(scaledStimLMS, g_lms2rgb, stim);
  // scale by the max so that it is physically realizable
  float maxVal = 0;
  for(byte i=0; i<3; i++){
    float tmp = fabs(stim[i]);
    if(tmp>maxVal) maxVal = tmp;
  }
  if(maxVal>1){
    Serial << F("rgb exceeds max! Scaling to fit within gamut:\n");
    stim[0] = stim[0]/maxVal;
    stim[1] = stim[1]/maxVal;
    stim[2] = stim[2]/maxVal;
    Serial << F("rgb: [ ") << stim[0] << F(",") << stim[1] << F(",") << stim[2] << F(" ]\n");
    float lms[3]; 
    xformColor(stim, g_rgb2lms, lms);
    for(int i=0;i<3;i++) lms[i] = lms[i] / backLms[i] / 2.0;
    Serial << F("Actual lms: [ ") << lms[0] << F(",") << lms[1] << F(",") << lms[2] << F(" ]\n");
  }
  //stimRGB = stimRGB/abs(stimRGB).max()*scale;
  // compute the actual LMS contrast
  //float actualLMS[3]; actualLMS = xformColor(stimRGB, g_rgb2lms, actualLMS) / backLms / 2.0;
}

void xformColor(float vecIn[], float tm[], float vecOut[]){
  vecOut[0] = vecIn[0]*tm[0] + vecIn[1]*tm[1] + vecIn[2]*tm[2];
  vecOut[1] = vecIn[0]*tm[3] + vecIn[1]*tm[4] + vecIn[2]*tm[5];
  vecOut[2] = vecIn[0]*tm[6] + vecIn[1]*tm[7] + vecIn[2]*tm[8];
}


