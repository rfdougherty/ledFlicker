/*
 * ledFlicker sketch for Arduino.
 * 
 * 
 * Twelve-channel, 12-bit, 2 kHz LED oscillator for visual experiments. 
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
 * Contributions by Tirthankar Chatterjee (Ty): tirth@stanford.edu 
 *
 * TO DO:
 *   - Store calibration emission spectra in EEPROM and allow the user to get 
 *     it out to compute color transforms.
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
 * 2010.10.13 Bob: Converted most of the waveform playout code to integer math
 * for better efficiency and added gamma correction (257 points per channel, 
 * with linear interpolation). 
 * 2010.12.11 Bob: Added the l (lobe flag) command to allow waveforms mono-phasic 
 * waveforms to be specified.
 * 2012.03 Ty: Changed the number of channels from 6 to 
 * NUM_CHANNELS currently set to 7.
 * 2012.05 Ty: Changed the number of points per channel to 129 instead of 257 so that
 * all the inverse gamma data from an increased number of channels can fit in RAM.
 * 2012.05 Ty: Changed the number of channels from 6 to 
 * NUM_CHANNELS currently set to 12.
 */

#define VERSION "0.9"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>

// Flash library is available from http://arduiniana.org/libraries/Flash/
// We make extensive use of this so that we can be verbose in our messages
// without using up precious RAM. (Using Flash saved us over 2Kb of RAM!)
#include <Flash.h>
#include <Messenger.h>

// atmega 168  	has  16k flash, 512 EEPROM, 1k RAM
// atmega 328	    	has  32k flash,  1k EEPROM, 2k RAM
// atmega 1280/2560     has 128k flash,  4k EEPROM, 8k RAM
// The code below is very tight on RAM because of the wave table.

// Serial port baud rate. 115.2 kbs seems to work fine. That's ~12 bytes/msec, 
// which should keep latencies low for short (30-50 byte) commands. Data integrity
// shouldn't be a problem since we are effectively communicating over USB. But, 
// just to be sure, we'll use 57.6 kbs.
#define BAUD 57600

// Waveform types
#define SQUAREWAVE 0
#define SINEWAVE 1

/************************************************
 *	Arduino to ATMEGA pin mappings	    
 *	Mappings wrt Arduino Mega 2560
 *	    
 *	The Digital Pins are used for PWM	
 *
 *	    Arduino	   | 	ATMEGA
 *	--------------------------------
 *	Digital Pin # 11   | 	OC1A
 *      Digital Pin # 12   | 	OC1B
 *      Digital Pin # 13   | 	OC1C
 *      Digital Pin # 5	   | 	OC3A
 *      Digital Pin # 2	   | 	OC3B
 *      Digital Pin # 3	   | 	OC3C
 *      Digital Pin # 6	   | 	OC4A
 *      Digital Pin # 7	   | 	OC4B
 *      Digital Pin # 8	   | 	OC4C
 *      Digital Pin # 46   | 	OC5A
 *      Digital Pin # 45   | 	OC5B
 *      Digital Pin # 44   | 	OC5C
 ************************************************/
#define PIN_LED1   11 	
#define PIN_LED2   12	
#define PIN_LED3   13	
#define PIN_LED4   5	
#define PIN_LED5   2	
#define PIN_LED6   3	
#define PIN_LED7   6	
#define PIN_LED8   7 	
#define PIN_LED9   8 	
#define PIN_LED10  46	 
#define PIN_LED11  45	
#define PIN_LED12  44	

// These must be powers of 2! (We use bit-shifting tricks to speed some calcualtions below.)
// See http://graphics.stanford.edu/~seander/bithacks.html#ModulusDivisionEasy
#define WAVE_SAMP_SHIFT 10
#define ENV_SAMP_SHIFT 7
#define NUM_WAVE_SAMPLES (1UL << WAVE_SAMP_SHIFT)   	
#define NUM_ENV_SAMPLES (1UL << ENV_SAMP_SHIFT)	

#define PIN_TEMP 0     // Analog input pin for temperture sensor
#define PIN_FAN  4     // Digital input for fan speed detector
// NUM_CHANNELS should be 6 for now.
// Values <6 should work, but only 2 and 6 have been tested.
#define NUM_CHANNELS 12

#define POINTS_PER_CHANNEL 129 // used to be 257
// This pin will be used for general digital output
#define DIGITAL_OUT_PIN 22

#define NUM_WAVES 2

// g_interruptFreq is the same as the INTERRUPT_FREQ, except for qunatization
// error. E.g., INTERRUPT_FREQ might be 3000, but the actual frequency achieved
// is 3003.4 Hz. All calculations should use this actual frequency.
static float g_interruptFreq;

#define PI 3.14159265358979
#define TWOPI 6.28318530717959

// The maxval determines the resolution of the PWM output. E.g.:
// 255 for 8-bit, 511 for 9-bit, 1023 for 10-bit, 2047 for 11-bit, 4095 for 12 bit.
// Higher resolution means slower PWM frequency. I.e., 
//    PWM freq = F_CPU/(PWM_MAXVAL+1) 
// for fast PWM and
//    PWM freq = F_CPU/PWM_MAXVAL/2 
// for phase/freq correct PWM. E.g., for non-fast 12-bit PWM freq will be 1.95kHz.
#define PWM_MAXVAL 4095
#define PWM_MIDVAL (PWM_MAXVAL/2.0)
// Fast PWM goes twice as fast, but the pulses aren't as spectrally nice as
// the non-fast ("phase and frequency correct") PWM mode. Also, 'zero' is
// not quite zero (there is a narrow spike that is visible on the LEDs). All
// the spectral ugliness is way beyond what we can see, so fast PWM is fine if
// you don't need true zero. And, you can trade off some of the extra speed 
// for better resolution (see PWM_MAXVAL).
#define PWM_FAST_FLAG false

// We use globals because of the interrupt service routine (ISR) that is used to
// play out the waveforms. ISRs can't take parameters, but can access globals.
// TODO: maybe bundle all the globals into a single struct to keep them neat?
static int g_sineWave[NUM_WAVE_SAMPLES];
static int g_envelope[NUM_ENV_SAMPLES];

static unsigned int g_envelopeTicsDuration;
static unsigned int g_envelopeStartEnd;
static unsigned int g_envelopeEndStart;
static unsigned int g_envelopeDwell;

// We'll make the main clock counter a long int so that we can run for many
// hours before wrapping. Users generally want to run for a few seconds or
// run forever. So, we keep the other tic counters as ints for efficiency.
// With this configuration, we run precisely timed, temporally enveloped stimuli
// up to ~30 sec duration, or we run forever with no temporal envelope.
volatile unsigned long int g_envelopeTics;

static int g_amplitude[NUM_WAVES][NUM_CHANNELS];
static unsigned long int g_mean[NUM_CHANNELS];
static float g_sineInc[NUM_WAVES];
static byte  g_waveType[NUM_WAVES];
static unsigned int g_phase[NUM_WAVES];

// A set of flags to allow positive or negative lobes of the waveform to be clipped.
static byte g_lobe[NUM_CHANNELS];
#define LOBE_BIP 0
#define LOBE_POS 1
#define LOBE_NEG 2

//
// Define EEPROM variables for static configuration vars
//
// We don't bother setting defaults here; we'll check for valid values and set 
// the necessary defaults in the code.
//
char EEMEM gee_deviceId[16];

/*************************************************************************
 * invGamma LUT maps
 * 128 values [0:512:65536] => 0-4095 PWM values(16 bit presicion)
 *   
 * Storage Requirement:
 * 12 channels* 128 points per channel * 2 bytes per point = 3072 bytes 
 *    
 * Arduino Mega 2560 EEPROM is 4096 bytes. The number of values (128)
 * is selected exactly for this reason, such that the entire LUT would
 * fit on the EEPROM and could be loaded into RAM
 *
 *************************************************************************/

unsigned int EEMEM gee_gammaMagic;
char EEMEM gee_gammaNotes[64];
unsigned int EEMEM gee_invGamma[POINTS_PER_CHANNEL*NUM_CHANNELS]; 
// The magic word ('LF' in ASCII) must be set to indicate that valid data has 
// been stored there. Anything else in that part of the EEPROM will trigger the 
// firmware to reload the default gamma when booting up.
#define EEMEM_MAGIC_WORD 0x4C46

// To use the inverse gamma params, we'll need to copy them to RAM. This uses a
// big chunk of our limited RAM.
unsigned int g_invGamma[POINTS_PER_CHANNEL][NUM_CHANNELS];

// Instantiate Messenger object used for serial port communication.
Messenger g_message = Messenger(',','[',']');

char g_errorMessage[128];
// 0 for very quiet, 1 for some stuff, 2 for more stuff, etc.
byte g_verboseMode;

// I can't figure out how to pass the PROGMEM stuff via a function call, so 
// we'll just use a macro.
#define ERROR(str) {strncpy_P(g_errorMessage,PSTR(str),128);error();}

void error(){
  Serial << F("ERR");
  if(g_verboseMode>0)
    Serial << F(": ") << g_errorMessage;
  Serial << F("\n");
}

void commandOk(){
  strcpy_P(g_errorMessage, PSTR("OK")); // Copy up to 128 characters
  Serial << F("OK\n");
}

// Create the Message callback function. This function is called whener a complete 
// message is received on the serial port.
void messageReady() {
  if(g_verboseMode>1) g_message.echoBuffer(); 
  float val[max(2*NUM_CHANNELS,14)]; //TODO: should 12 be 14
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
      Serial << F("    Set the envelope duration and rise/fall times (in seconds, max = ") << 65535.0/g_interruptFreq << F(" secs).\n");
      Serial << F("    Set duration=0 for no envelope and infinite duration. The envelope setting is preserved\n");
      Serial << F("    until the firmware is rebooted.\n");
      Serial << F("[l,l0,l1,l2,l3,l4,l5,l6,l7,l8,l9,l10,l11]\n");
      Serial << F("    Set twelve flags (one for each channel) that determine if the waveforms are played in full\n");
      Serial << F("    (0, the default), only the positive lobe is played (1), or only the negative lobe is played (-1).\n");
      Serial << F("    As with the envelope, this setting is preserved until the firmware is rebooted.\n");
      Serial << F("[w,waveNum,frequency,phase,amp0,amp1,...]\n");
      Serial << F("    Set waveform parameters for the specified waveform number (up to ") << NUM_WAVES << F(").\n");
      Serial << F("    Phase is 0-1, with 0.5 = pi radians. Amplitudes are -1 to 1.\n");
      Serial << F("[p]\n");
      Serial << F("    Play the waveforms. If the duration is infinite, then you only need to run\n");
      Serial << F("    this command once, even if waveform parmeters are changed.\n");
      Serial << F("[h]\n");
      Serial << F("    Halt waveform playout. This is especially useful with an infinite duration.\n");
      Serial << F("[s]\n");
      Serial << F("    Status information. If the previous command failed with an ERR, you can use this\n");
      Serial << F("    to see the error message. Also shows the time remaining for current playout.\n");
      Serial << F("[v,mode]\n");
      Serial << F("    Set verbosity mode. 0 for very quiet (just OK or ERR), 1 to show errors, 2 or higher for everything.\n");
      Serial << F("[c,waveNum]\n");
      Serial << F("    Check the currently specified waveform. Prints some internal variables and waveform stats.\n");
      Serial << F("[d,waveNum]\n");
      Serial << F("    Dump the specified wavform. (Dumps a lot of data to your serial port!\n\n"); 
      Serial << F("[g,lutSlice,ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11]\n");
      Serial << F("    Set the inverse gamma LUT values and store them in EEPROM. The gamma tables will \n");
      Serial << F("    be loaded automatically each time the board boots up. The computed (internal) modulation\n");
      Serial << F("    values (s) are in the range [0,65536]. The PWM value produced for internal modulation value s is:\n");
      Serial << F("       pwm = ((s%16)*lut[s>>4] + (16-s%16)*lut[s>>4+1]) / 16 \n");
      Serial << F("    There are 12 gamma tables (one for each output channel). Each gamma maps 129 internal modulation\n");
      Serial << F("    values ([0:512:65536]) to the 0-4095 PWM values according to the formula above, which includes\n");
      Serial << F("    linear interpolation for values between the 129 entries in the gamma LUTs. When setting the gamma\n");
      Serial << F("    tables, pass the 12 pwm values for one of the 129 slices through the 12 tables.\n");
      Serial << F("    Call this with no args to see the all the LUT values currently stored in EEPROM.\n"); 
      Serial << F("    Call this with just lutSlice to see the the LUT values for the specified slice of the gamma tables.\n");
      Serial << F("\nFor example:\n");
      Serial << F("[e,10,0.2][w,0,3,0,.3,-.3,0,0,0,.9,0,0,0,0,0,0][p]\n\n");
      break;
      
    case 'm': // Set mean outputs
      i = 0; // Resetting the value of i, TY
      while(g_message.available()) val[i++] = g_message.readFloat();
      if(i!=1 && i!=NUM_CHANNELS){
        ERROR("ERROR: Set outputs requires one param or 7 params.");
      }else{
        stopISR();
        if(i==1){
          setAllMeans(val[0]);
        }else{
          for(i=0; i<NUM_CHANNELS; i++)
            setMean(i, val[i]);
        }
        applyMeans();
        commandOk();
        if(g_verboseMode>1) 
          Serial << F("Means set to ["); for(i=0; i<NUM_CHANNELS; i++) Serial << (g_mean[i]>>19) << F(" "); Serial << F("]\n");
      }
      break;
      
    case 'e': // Set envelope params
      i = 0; 
      while(g_message.available()) val[i++] = g_message.readFloat();
      if(i<2){
        ERROR("ERROR: envelope setup requires 2 parameters.\n");
      }else{
        stopISR();
        float dur = setupEnvelope(val[0], val[1]);
        commandOk();
        if(g_verboseMode>1)
          Serial << F("Envelope configured; actual duration is ") << dur << F(" seconds.\n");
      }
      break;

    case 'l': // Set lobe-cutting flags
      i = 0; 
      while(g_message.available()) val[i++] = g_message.readFloat();
      if(i<NUM_CHANNELS){
        ERROR("ERROR: lobe setup requires 7 parameters.\n");
      }else{
        for(i=0; i<NUM_CHANNELS; i++){
          if(val[i]<0)      g_lobe[i] = LOBE_NEG;
          else if(val[i]>0) g_lobe[i] = LOBE_POS;
          else              g_lobe[i] = LOBE_BIP;
        }
        commandOk();
      }
      break;

    case 'w': // setup waveforms
      i = 0; 
      while(g_message.available()) val[i++] = g_message.readFloat();
      if(i<3+NUM_CHANNELS){ // wave num, freq, phase, and amplitudes are mandatory
        ERROR("ERROR: waveform setup requires at least 3 parameters.\n");
      }else{
        //stopISR();
        // setup the waveform. params are: wave num, freq, phase, amp[]
        if(val[0]>=0&&val[0]<NUM_WAVES){
          setupWave((byte)val[0], val[1], val[2], &val[3]);
          commandOk();
        }else{
          ERROR("ERROR: waveform setup requires first param to be a valid wave number.\n");
        }
      }
      break;

    case 'p': // play waveforms
      // Reset state variables
      g_envelopeTics = 0;
      // NOTE: g_sineInc is not reset here. So, you must call setupWave before playing.
      startISR();
      commandOk();
      break;

    case 'h': // halt waveform playout
      stopISR();
      applyMeans();
      commandOk();
      break;

    case 's': // return status
      Serial << F("Last status message: ") << g_errorMessage << F("\n");
      Serial.println(((float)g_envelopeTicsDuration-g_envelopeTics)/g_interruptFreq,3);
      Serial << F("LED temperature: ") << (float)getTemp() << F(" C");
      Serial << F(", Fan speed: ") << getFanSpeed() << F(" RPMs\n");
      commandOk();
      break;

    case 'v':
      if(g_message.available()){
        val[0] = g_message.readInt();
        if(val[0]<=0)      g_verboseMode = 0;
        else if(val[0]>10) g_verboseMode = 10;
        else               g_verboseMode = val[0];
        commandOk();
      }
      break;
      
    case 'c':
      if(g_message.available()){
        val[0] = g_message.readInt();
        validateWave(val[0]);
        commandOk();
      }
      else ERROR("ERROR: check command requires a channel parameter.\n");
      break;

    case 'd':
      if(g_message.available()){
        stopISR();
        val[0] = g_message.readInt();
        dumpWave((byte)val[0]);
        commandOk();
      }
      else ERROR("ERROR: dump command requires a channel parameter.\n");
      break;
      
    case 'g': 
      // Set inverse gamma LUT. We can't pass too much data at one time, so we set one slice at a time.
      // *** WORK HERE: allow a string to be sent in to set the gamma note string.
      while(g_message.available()) val[i++] = g_message.readFloat();
      if(i<1){
        dumpInvGamma();
        commandOk();
      }else if(i==1){
        dumpInvGammaSlice((int)val[0]);
        commandOk();
      }else if(i!=2+NUM_CHANNELS){
        ERROR("ERROR: g requires either 0 or 1 args (to dump current vals), or 7 values to set the inv gamma for a LUT entry!\n");
      }else if(val[0]<0 || val[0]>=POINTS_PER_CHANNEL){
        ERROR("ERROR: First argument is the LUT entry number and must be >=0 and <129.\n");
      }else{
        setInvGamma((int)val[0], &(val[1]));
        commandOk();
      }
      break;
    default:
      Serial << F("[") << command << F("]\n");
      ERROR("ERROR: Unknown command. ");
    } // end switch
  } // end while
}

// Precompute the digital output register and bitmask.
// TO DO: wrap this in a fast I/O class
#include "pins_arduino.h"
volatile uint8_t *g_digOutReg;
uint8_t g_digOutBit;

void setup(){
  Serial.begin(BAUD);
  Serial << F("*********************************************************\n");
  Serial << F("* ledFlicker firmware version ") << VERSION << F("\n");
  Serial << F("* Copyright 2010 Bob Dougherty <bobd@stanford.edu>\n");
  Serial << F("* http://vistalab.stanford.edu/newlm/index.php/LedFlicker\n");
  Serial << F("*********************************************************\n\n");
  
  pinMode(DIGITAL_OUT_PIN, OUTPUT);
  digitalWrite(DIGITAL_OUT_PIN, HIGH); 
  g_digOutReg =  portOutputRegister(digitalPinToPort(DIGITAL_OUT_PIN));
  g_digOutBit = digitalPinToBitMask(DIGITAL_OUT_PIN);
  
  // Compute the wave and envelope LUTs. We could precompute and store them in 
  // flash, but they only take a few 10's of ms to compute when we boot up and it
  // simplifies the code. However, they do use much precious RAM. If the RAM 
  // limit becomes a problem, we might considerlooking into storing them in 
  // flash and using PROGMEM to force the compiler to read directly from flash.
  // However, the latency of such reads might be too long.
  Serial << F("Computing wave LUT: \n");
  unsigned long ms = millis();
  // Sumary of waveform int32 integer math:
  // sinewave:  11 bits (10 bit value + sign bit)
  // envelope:   8 bits
  // amplitude: 13 bits (12 bit value + sign bit)
  //     total: 32 bits
  // The sinewave is represented by 11 bits (10 + sign bit):
  for(int i=0; i<NUM_WAVE_SAMPLES; i++)
    g_sineWave[i] = (int)round(sin(TWOPI*(float)i/NUM_WAVE_SAMPLES)*1024.0f);
  // The envelope is represented by 8 bits:
  // This will be a gaussian envelope with an SD of ~2
  float twoSigmaSquared = 2.0f*(NUM_ENV_SAMPLES/3)*(NUM_ENV_SAMPLES/3);
  for(int i=0; i<NUM_ENV_SAMPLES; i++)
    g_envelope[i] = (int)round(exp(-(NUM_ENV_SAMPLES-1.0f-i)*(NUM_ENV_SAMPLES-1.0f-i)/twoSigmaSquared) * 256.0f);
    //g_envelope[i] = (int)round((0.5f - cos(PI*i/(NUM_ENV_SAMPLES-1.0f))/2.0f) * 256.0f);
  ms = millis()-ms;
  Serial << NUM_WAVE_SAMPLES << F(" samples in ") << ms << F(" miliseconds.\n");

  if(PWM_FAST_FLAG) Serial << F("Initializing fast PWM on timer 1.\n");
  else              Serial << F("Initializing phase/frequency correct PWM on timer 1.\n");
  float pwmFreq = SetupTimer1(PWM_MAXVAL, PWM_FAST_FLAG);
  Serial << F("PWM Freq: ") << pwmFreq << F(" Hz; Max PWM value: ") << PWM_MAXVAL << F("\n");

  g_interruptFreq = pwmFreq;
  Serial << F("Interrupt Freq: ") << g_interruptFreq << F(" Hz\n");
  
  // Load inverse gamma from EEPROM into RAM
  Serial << F("Loading stored inverse gamma LUT.\n");
  const char *gammaNotes = loadInvGamma();
  if(gammaNotes!=NULL)
    Serial.println(gammaNotes);
  else
    Serial << F("Inv gama data is invalid- loaded default linear gamma.\n");

  // Set waveform defaults
  Serial << F("Initializing all waveforms to zero amplitude.\n");
  float amp[NUM_CHANNELS] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  for(int i=0; i<NUM_WAVES; i++) setupWave(i, 0.0, 0.0, amp);
  Serial << F("Initializing all means to ") << 0.5 << F("\n");
  setAllMeans(0.5f);
  applyMeans();
  setupEnvelope(3.0, 0.2);
  float amps[NUM_CHANNELS] = {0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9};
  setupWave(0, 1.0, 0, amps);
  // Default is bi-phaseic waveform
  for(int i=0; i<NUM_CHANNELS; i++) 
    g_lobe[i] = LOBE_BIP;
  
  // Attach the callback function to the Messenger
  g_message.attach(messageReady);
  
  // Configure analog inputs to use the internal 1.1v reference.
  // See: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1264707156
  analogReference(2);
  //pinMode(PIN_TEMP, INPUT);  // Make sure temperature pin is set for input 
  pinMode(PIN_FAN, INPUT);

  // Set to quiet
  Serial << F("Setting verbose mode to quiet; use [v,1] or [v,2] to increase verbosity.\n");
  g_verboseMode = 0;
  
  Serial << F("ledFlicker Ready. Send the ? command ([?]) for help.\n");
  Serial << F("There are ") << g_message.FreeMemory() << F(" bytes of RAM free.\n\n");
  digitalWrite(DIGITAL_OUT_PIN, LOW);
}

void loop(){
  // The most effective way of using Serial and Messenger's callback:
  while(Serial.available())  g_message.process(Serial.read());
}

bool isGammaValid(){
  unsigned int magicWord = eeprom_read_word(&gee_gammaMagic);
  if(magicWord==EEMEM_MAGIC_WORD) return(true);
  return(false);
}
void setGammaValid(){
  // EEPROM has a limited # of write cycles, so we avoid unecessary writes.
  if(!isGammaValid()) eeprom_write_word(&gee_gammaMagic, EEMEM_MAGIC_WORD);
}

const char * getGammaNotes(){
  static char notes[64];
  eeprom_read_block((void*)notes, (const void*)gee_gammaNotes, 64);
  return(notes);
}

/***************************************************************************
 *
 * Function to load the global inverse gamma LUT from EEPROM or in case
 * the Gamma data is invalid, generate default values.
 *
 * The inv gamma tables are arranged as a 2 dimensional int16 array of
 * total size POINTS_PER_CHANNEL (128) x  NUM_CHANNELS (12) 
 * Now which is     
 * 
 *
 ***************************************************************************/
const char * loadInvGamma(){
  if(isGammaValid()){
    for(int lutIndex=0; lutIndex<POINTS_PER_CHANNEL; lutIndex++)
      eeprom_read_block((void*)&g_invGamma[lutIndex][0], (const void*)&gee_invGamma[lutIndex*NUM_CHANNELS], NUM_CHANNELS*sizeof(unsigned int));
    return(getGammaNotes());
  }else{
    for(int i=0; i<POINTS_PER_CHANNEL; i++)
      for(byte j=0; j<NUM_CHANNELS; j++)
        g_invGamma[i][j] = (unsigned int)round(i*31.9921875); // less than 32 to get to 4095 rather than 4096.
    return(NULL);
  }
}

// Copies the global inverse gamma parameters from EEPROM.
// If invGamma is not null, then the EEPROM data is updated with the new matrix before setting the globals.
void setInvGamma(int lutIndex, float invGammaSlice[]){
  // The inv gamma tables is arranged as a [NUM_CHANNELS][129] uint16 array. 
  // We accept data as a float array and convert to an int array.
  if(lutIndex<POINTS_PER_CHANNEL){
    if(invGammaSlice!=NULL){
      unsigned int slice[NUM_CHANNELS];
      for(byte i=0; i<NUM_CHANNELS; i++) slice[i] = (unsigned int)round(invGammaSlice[i]);
      eeprom_write_block((void*)slice, (void*)&gee_invGamma[lutIndex*NUM_CHANNELS], NUM_CHANNELS*sizeof(unsigned int));
      // When the last slice is saved, set the gamma as valid
      if(lutIndex==POINTS_PER_CHANNEL-1){
        setGammaValid();
      }
    }
    eeprom_read_block((void*)&g_invGamma[lutIndex][0], (const void*)&gee_invGamma[lutIndex*NUM_CHANNELS], NUM_CHANNELS*sizeof(unsigned int));
  }
}

void setInvGammaNotes(char notes[]){
  // Load a notes string into the gamma notes EEMEM.
  // We want to save the null-terminator too, so we +1.
  unsigned int n = strlen(notes)+1;
  if(n>=64)
    eeprom_write_block((void*)notes, (void*)&gee_gammaNotes, 64);
  else
    eeprom_write_block((void*)notes, (void*)&gee_gammaNotes, n);
}

void dumpInvGamma(){
  for(int i=0; i<POINTS_PER_CHANNEL; i++){
    dumpInvGammaSlice(i);
  }
  Serial.println(getGammaNotes());
}

void dumpInvGammaSlice(int i){
    Serial << F("[g,") << i << F(",") << g_invGamma[i][0] << F(",") << g_invGamma[i][1] << F(",") << g_invGamma[i][2] << F(",")
                                      << g_invGamma[i][3] << F(",") << g_invGamma[i][4] << F(",") << g_invGamma[i][5] << F(",") 
				      << g_invGamma[i][6] << F(",") << g_invGamma[i][7] << F(",") << g_invGamma[i][8] << F(",") 
				      << g_invGamma[i][9] << F(",") << g_invGamma[i][10] << F(",") << g_invGamma[i][11] << F(",")
				      << F("];\n");
}


void setupWave(byte wvNum, float freq, float ph, float *amp){
  static unsigned int maxWaveIndex = NUM_WAVE_SAMPLES-1;
  byte i;
  
  // Phase comes in as a relative value (0-1); convert to the index offset.
  g_phase[wvNum] = (unsigned int)round(ph*maxWaveIndex);
  // Amplitude is -1 to 1 (negative inverts phase)
  if(amp!=NULL){
    // Now set the amplitudes in the global.
    // The amplitude is represented by 13 bits (12 + sign bit):
    for(i=0; i<NUM_CHANNELS; i++)
      g_amplitude[wvNum][i] = (int)round((amp[i]*4096.0f));
  }else{
    for(i=0; i<NUM_CHANNELS; i++)
      g_amplitude[wvNum][i] = 0;
  }
  // the incremetor determines the output freq.
  // Wew scale by NUM_WAVE_SAMPLES/g_interruptFreq to convert freq in Hz to the incremeter value.
  g_sineInc[wvNum] = fabs(freq)*NUM_WAVE_SAMPLES/g_interruptFreq;
  if(freq<0)
    g_waveType[wvNum] = SQUAREWAVE;
  else
    g_waveType[wvNum] = SINEWAVE;
    
}

/* WOKRK HERE */
//void initWaves(){
//  for(byte wv=0; wv<NUM_WAVES; wv++)
//    g_sineInd[wv] = g_phase[wv];
//}

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
  case 6: 
    OCR4A = val;
    break;
  case 7:
    OCR4B = val;
    break;
  case 8:
    OCR4C = val;
    break;
  case 9:
    OCR5A = val;
    break;
  case 10:
    OCR5B = val;
    break;
  case 11:
    OCR5C = val;
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
  // To save some ops during waveform playout, we save the means as scaled values (scale factor is 2^19).
  g_mean[chan] = ((unsigned long int)round(val*4095))<<19;
}

void applyMeans(){
  // Set PWM output to mean level for all channels
  // The means range from 0-4095 (2^12), but are stored as scaled by 2^19, so range from 0 - 2^31. 
  // getPwmFromLUT is expecting them to be 0 - 2^16, so we need to divide by 2^15.
  for(byte i=0; i<NUM_CHANNELS; i++)
    setOutput(i, getPwmFromLUT(i, g_mean[i]>>15));
}

float setupEnvelope(float duration, float envRiseFall){
  // Configure the envelope global values
  // envelope rise/fall time is translated to the g_envelope incrementer
  // g_envelopeTicsDuration is an unsigned int, so the max number of tics is 65535.
  // A duration of 0 means no envelope- run forever.
  if(duration*g_interruptFreq>65535.0)
    duration = 65535.0/g_interruptFreq;
  g_envelopeTicsDuration = (unsigned int)(duration*g_interruptFreq);
  if(g_envelopeTicsDuration==0){
    duration = 0.0f;
    g_envelopeDwell = 0;
  }else{
    g_envelopeDwell = (unsigned int)round(envRiseFall/((float)NUM_ENV_SAMPLES/g_interruptFreq));
    g_envelopeStartEnd = (NUM_ENV_SAMPLES-1)*g_envelopeDwell;
    g_envelopeEndStart = g_envelopeTicsDuration-g_envelopeStartEnd;
  }
  // initialize the state variable
  g_envelopeTics = 0;
  return(duration);
}

inline int getEnvelopeVal(unsigned long int curTics){
  static const unsigned int maxEnvelopeIndex = NUM_ENV_SAMPLES-1;
  unsigned int envIndex;

  // Must be careful of overflow. For interrupt freq of 2kHz, max duation is ~ 32 secs. For 4kHz it is ~16.
  // We could switch to long ints if needed.
  
  // TO DO: switch to interger arithmatic, make dwell a power of 2, and use bit-shifting for the division.

  if(g_envelopeDwell==0 || (curTics>g_envelopeStartEnd && curTics<g_envelopeEndStart)){
    return(256);
  }
  if(curTics<=g_envelopeStartEnd){
    envIndex = ((unsigned int)curTics/g_envelopeDwell);
  }else{
    envIndex = (g_envelopeTicsDuration-(unsigned int)curTics)/g_envelopeDwell;
  }
  // TO DO: replace with properly-rounded integer math.
  return(g_envelope[envIndex]);
}


// This is the core function for waveform generation. It is called in the 
// ISR to output the waveform to the PWNM channels. 
inline void updateWave(unsigned long int curTics, unsigned int envVal, unsigned int *vals){
  // This function takes about 280us to run (NUM_WAVES==1, at 16MHz).
  // Matlab code illustrating the principle behind the integer-math used here:
  // s = int32(round(sin(linspace(0,2*pi,1024))*2048)); % 12 bits for the sinewave (-2048 to 2048)
  // amp = int32(2048);                        % 12-bits for the amplitude (-2048 to 2048)
  // env = int32(256);                         % 8-bits for the envelope (0 to 256)
  // mean = bitshift(uint32(2048),19);         % actual mean PWM value is scaled by 2^19
  // % Note that we are using signed int32's, so we have 31 bits. 31-12 = 19.
  // The resulting waveform (in the range 0-4096) is given by:
  // w = bitshift(s*amp*env+mean,-19);
  // 
  byte wv, ch;
  long int envSine;
  unsigned long int lvals[NUM_CHANNELS];
  unsigned long int sineIndex;
  static byte lastWaveSign = 0;
  
  // Initialize each channel to the mean value
  for(ch=0; ch<NUM_CHANNELS; ch++)
    lvals[ch] = g_mean[ch];
  
  for(wv=0; wv<NUM_WAVES; wv++){
    //unsigned int sineIndex = (unsigned long int)((g_sineInc[wv]*curTics+0.5)+g_phase[wv])%NUM_WAVE_SAMPLES;
    sineIndex = (unsigned long int)((g_sineInc[wv]*curTics+0.5)+g_phase[wv]);
    // As long as NUM_WAVE_SAMPLES is a power of 2, this is equivalent to sineIndex = sineIndex % NUM_WAVE_SAMPLES
    // This should be a little faster than %.
    sineIndex = sineIndex & (NUM_WAVE_SAMPLES - 1);
    if(g_waveType[wv]==SQUAREWAVE){
      // squarewave (thresholded sine)
      if(g_sineWave[sineIndex]>0)      envSine =  1024L*(long int)envVal;
      else if(g_sineWave[sineIndex]<0) envSine = -1024L*(long int)envVal;
      else{
        // Special case for zero alternate so that we have a symmetric duty cycle (on average)
        if(lastWaveSign==0){
          envSine =  1024L*(long int)envVal;
          lastWaveSign = 1;
        }else{
          envSine = -1024L*(long int)envVal;
          lastWaveSign = 0;
        }
      }
    }else{
      // Only other option is a sinewave.
      envSine = (long int)envVal*g_sineWave[sineIndex];
    }
    for(ch=0; ch<NUM_CHANNELS; ch++){
      if(g_lobe[ch]==LOBE_BIP || (g_lobe[ch]==LOBE_NEG && envSine<0) || (g_lobe[ch]==LOBE_POS && envSine>0))
        lvals[ch] += (envSine*g_amplitude[wv][ch]);
    }
  }
  for(ch=0; ch<NUM_CHANNELS; ch++) {
    // Convert to 0-65536 for gamma correction:
    vals[ch] = getPwmFromLUT(ch, lvals[ch]>>15);
  }
  //Serial << vals[0] << F(",") << envSine<< F(",") << lvals[0] << F(";");
}

unsigned int getPwmFromLUT(byte ch, unsigned long int rawVal){
  // Input values should be 0-65536- run them through the invGamma LUT.
  unsigned int outVal;
  // Treat the ends as special cases:
  if(rawVal>65535){
    outVal = g_invGamma[POINTS_PER_CHANNEL-1][ch];
  }else if(rawVal==0){
    outVal = g_invGamma[0][ch];
  }else{
    // Get the index into the 129-entry gamma table that is at the lower bound of tmp:
    unsigned int lowerInd = (unsigned int)(rawVal>>9); // Edit TY: changed 8 to 9 bit shift since we have a 7 bit Gamma (from 2^16 to 2^7)
    // This is a fast way to compute tmp % 16:
    unsigned int mod = rawVal & (unsigned int)15;
    // The following calc should just fit into an unsigned int. The highest possible value is
    // 1*4095 + 15*4095. (That's assuming the invGamma has values in the proper range.)
    // We are doing a simple linear interpolation using pure integer math for speed. 
    // The +8 assures proper rounding.
    outVal = ((16-mod)*g_invGamma[lowerInd][ch] + mod*g_invGamma[lowerInd+1][ch] + 8)>>4;
  }
  return(outVal);
}

void validateWave(byte chan){
  unsigned int maxVal = 0;
  unsigned int minVal = 65535;
  float mnVal = 0.0;
  unsigned int val[NUM_CHANNELS];
  byte wvNum = 0;

  Serial << F("sineInc: ") << g_sineInc[wvNum] << F("\n");
  Serial << F("envTicsDwell: ") << g_envelopeDwell << F("\n");
  Serial << F("envTicsDuration: ") << g_envelopeTicsDuration << F("\n");
  Serial << F("envelopeStartEnd: ")<< g_envelopeStartEnd << F("\n");
  Serial << F("envelopeEndStart: ") << g_envelopeEndStart << F("\n");
  
  Serial << F("Channel ") << (int)chan << F(":\n");
  Serial << F("      amplitude: ") << (float)g_amplitude[0][chan]/4096.0f << F("\n");
  Serial << F("           mean: ") << (g_mean[chan]>>19) << F(" (PWM=") << getPwmFromLUT(chan,g_mean[chan]>>15) << F(")\n");
  for(unsigned int i=0; i<g_envelopeTicsDuration; i++){
    updateWave(i, getEnvelopeVal(i), val);
    if(val[chan]>maxVal) maxVal = val[chan];
    if(val[chan]<minVal) minVal = val[chan];
    mnVal += (float)val[chan]/g_envelopeTicsDuration;
  }
  Serial << F(" [min,mean,max]: ") << minVal << F(",") << (int)(mnVal+0.5) << F(",") << maxVal << F("\n");
}

void dumpWave(byte chan){
  unsigned int val[NUM_CHANNELS];
  
  Serial << F("wave=[");
  for(unsigned int i=0; i<g_envelopeTicsDuration; i++){
    updateWave(i, getEnvelopeVal(i), val);
    Serial << val[chan] << F(",");
  }
  Serial << F("];\n");
}


/********************************************************************************************* 
 * SETUP TIMERS:
 *  
 * Function to set up the timers 1,3,4 and 5. This is specific to Arduino Mega 2560
 *
 * Reference: http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
 * 
 * Bit-primer:
 *   Setting a bit: byte |= 1 << bit;
 *   Clearing a bit: byte &= ~(1 << bit);
 *   Toggling a bit: byte ^= 1 << bit;
 *   Checking if a bit is set: if (byte & (1 << bit))
 *   Checking if a bit is cleared: if (~byte & (1 << bit)) OR if (!(byte & (1 << bit)))
 *
 * In this program the fastPwm flag is set as FALSE, so we work with the PWM mode called
 * "Frequency and Phase correct PWM mode".
 *
 * Setting up the Timers involve modifying bits in their respective Configuration Registers.
 * Refer to the Datasheet : http://www.atmel.com/Images/doc2549.pdf
 * 
 * Operations that need to be done on the counters to set them up:
 *  
 *  1. Set the CS bits (Clock Select) to select the clock source. 
 *	We use the default CLK with a prescaler value of 1 (No prescaling)
 *	CSn2:0 = 001, n = 1,3,4,5 (Timer Counter number)
 *  
 *  2. Set the Timer Counter Mode.
 *	This is using the WGM (Waveform Generation Mode) bits.
 *	fastPWM mode is mode 14			    : WGMn3:0 = 1110
 *	Frequency and Phase correct mode is mode 8  : WGMn3:0 = 1000 ,for n = 1,3,4,5 
 *
 *  3. Since we use TC mode 8, the counter counts from 0 to the value in the register ICRn
 *	So we must set ICRn to the Maximum value we want to count till.
 *
 *  4. We must set the attribute of the pins that we use for PWM to outputs.
 *
 *  5. Since we are using the Output pins for a specific operation (PWM) which might not
 *	be its default operation we must set either one or both of the Timer's
 *	COM bits to override the normal port functionality of the pin it is connected to.
 *	Basically we set, the COMnx1:0 = 10, so that the output pins function like we
 *	want them to. The mapping of Arduino Pins to ATMEGA pins is defined in the 
 *      beginning section of the code.
 *	COMnx1:0 = 10; n = 1,3,4,5; x= A,B,C
 *
 *  6. When we do steps 1-5 for all the counters we are using (1,3,4,5) we have succesfully
 *     set up the Timers.
 ********************************************************************************************/
float SetupTimer1(unsigned int topVal, bool fastPwm){
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
  
  TCCR4B &= ~(1 << CS42); 
  TCCR4B &= ~(1 << CS41); 
  TCCR4B |=  (1 << CS40);
  if(fastPwm){
    TCCR4B |=  (1 << WGM43);
    TCCR4B |=  (1 << WGM42);
    TCCR4A |=  (1 << WGM41); 
    TCCR4A &= ~(1 << WGM40);
  }else{
    TCCR4B |=  (1 << WGM43);
    TCCR4B &= ~(1 << WGM42);
    TCCR4A &= ~(1 << WGM41); 
    TCCR4A &= ~(1 << WGM40);
  }
  ICR4 = topVal;
  pinMode(PIN_LED7, OUTPUT);
  pinMode(PIN_LED8, OUTPUT);
  pinMode(PIN_LED9, OUTPUT);
  TCCR4A |=  (1 << COM4A1); 
  TCCR4A &= ~(1 << COM4A0);
  TCCR4A |=  (1 << COM4B1); 
  TCCR4A &= ~(1 << COM4B0);
  TCCR4A |=  (1 << COM4C1); 
  TCCR4A &= ~(1 << COM4C0);

  TCCR5B &= ~(1 << CS52); 
  TCCR5B &= ~(1 << CS51); 
  TCCR5B |=  (1 << CS50);
  if(fastPwm){
    TCCR5B |=  (1 << WGM53);
    TCCR5B |=  (1 << WGM52);
    TCCR5A |=  (1 << WGM51); 
    TCCR5A &= ~(1 << WGM50);
  }else{
    TCCR5B |=  (1 << WGM53);
    TCCR5B &= ~(1 << WGM52);
    TCCR5A &= ~(1 << WGM51); 
    TCCR5A &= ~(1 << WGM50);
  }
  ICR5 = topVal;
  pinMode(PIN_LED10, OUTPUT);
  pinMode(PIN_LED11, OUTPUT);
  pinMode(PIN_LED12, OUTPUT);
  TCCR5A |=  (1 << COM5A1); 
  TCCR5A &= ~(1 << COM5A0);
  TCCR5A |=  (1 << COM5B1); 
  TCCR5A &= ~(1 << COM5B0);
  TCCR5A |=  (1 << COM5C1); 
  TCCR5A &= ~(1 << COM5C0);
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

void startISR(){  // Starts the ISR
  //TIMSK1 |= (1<<OCIE1A);        // enable output compare interrupt (calls ISR(TIMER1_COMPA_vect)
  TIMSK1 |= (1<<TOIE1);        // enable overflow interrupt (calls ISR(TIMER1_OVF_vect_vect)
}

void stopISR(){    // Stops the ISR
  //TIMSK1 &= ~(1<<OCIE1A);      // disable output compare interrupt 
  TIMSK1 &= ~(1<<TOIE1);      // disable overflow compare interrupt 
} 

// Timer1 interrupt vector handler
// (for overflow, use TIMER1_OVF_vect)
// see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1215675974/0
// and http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1216085233
//ISR(TIMER1_COMPA_vect) {
ISR(TIMER1_OVF_vect) {
  digitalOutHigh(); // test ISR timing
  static unsigned int envInd;
  static byte i;
  unsigned int val[NUM_CHANNELS];
  
  // We skip computing the envelope value when there is no envelope. This
  // should make the serial port more responsive during playout.
  if(g_envelopeTicsDuration>0){
    updateWave(g_envelopeTics, getEnvelopeVal(g_envelopeTics), val);
    // Make the interrupt self-terminating
    if(g_envelopeTicsDuration>0 && g_envelopeTics>=g_envelopeTicsDuration)
      stopISR();
  }else{
    updateWave(g_envelopeTics, 256, val);
  }
  OCR1A = val[0];
  OCR1B = val[1];
  #if NUM_CHANNELS > 2
  OCR1C = val[2];
  OCR3B = val[3]; 
  OCR3C = val[4]; 
  OCR3A = val[5];
  OCR4A = val[6];
  OCR4B = val[7];
  OCR4C = val[8];
  OCR5A = val[9];
  OCR5B = val[10];
  OCR5C = val[11];
  #endif

  g_envelopeTics++;
  // NOTE: this will wrap after ~500 hours!

  // Note: there is no assurance that the PWMs will get set to their mean values
  // when the waveform play-out finishes. Thus, g_envelope must be designed to
  // provide this assurance; e.g., have 0 as it's first value and rise/fall >0 tics.
  digitalOutLow(); // test ISR timing
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

inline void digitalOutLow(){
   *g_digOutReg &= ~g_digOutBit;
}

inline void digitalOutHigh(){
   *g_digOutReg |= g_digOutBit;
}

