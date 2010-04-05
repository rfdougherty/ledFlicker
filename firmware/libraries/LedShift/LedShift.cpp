/*
 * Allegro A6280/81 library for Arduino.
 * 
 * The Allegro A6280 and A6281 ICs are digitally controlled constant-current
 * sources useful for driving LEDs. Both chips have 3 channels of output 
 * (ideal for tricolor LEDs) and allow 7-bits of control over the current 
 * limit for each output. Both also have three 10-bit PWMs to modulate LED 
 * brightness rapidly; the 81 has an internal oscillator for the PWM while
 * the 80 does not. 
 *
 * SOme of the code here was borrowed from http://www.pololu.com/catalog/product/1240.
 * 
 * 2010.03.23 Bob Dougherty (bobd@stanford.edu)
 */
 
//ADDED FOR COMPATIBILITY WITH WIRING
extern "C" {
  #include <stdlib.h>
}

#include "WProgram.h"
#include "LedShift.h"
#include "pins_arduino.h"


LedShift::LedShift(uint8_t dataPin, uint8_t latchPin, uint8_t enablePin, uint8_t clockPin){
  
  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  
  // Precompute the clock and data register/bitmasks to speed up the ISR
  dataReg =  portOutputRegister(digitalPinToPort(dataPin));
  dataBit = digitalPinToBitMask(dataPin);
  latchReg =  portOutputRegister(digitalPinToPort(latchPin));
  latchBit = digitalPinToBitMask(latchPin);
  enableReg = portOutputRegister(digitalPinToPort(enablePin));
  enableBit = digitalPinToBitMask(enablePin);
  clockReg = portOutputRegister(digitalPinToPort(clockPin));
  clockBit = digitalPinToBitMask(clockPin);
  
  // Initialize pin states
  Latch();
  Enable();
}


// BuildColorPacket returns a 32-bit dataPacket for setting LED brightness.
//
// red, green, and blue are brightness values from 0 (off) to 1023 (full).
// 
long unsigned int LedShift::BuildColorPacket(unsigned int red, unsigned int green, unsigned int blue){
    // Make a packet and initialize all of the bits to zero.
    dataPacket dp = {value:0};

    dp.red   = red;
    dp.green = green;
    dp.blue  = blue;
    // command bit stays 0: dp.command = 0;
    return dp.value;
}

// SetColorPacket sets  the current dataPacket for LED brightness PWM.
//
// red, green, and blue are brightness values from 0 (off) to 1023 (full).
// 
void LedShift::SetColorPacket(unsigned int red, unsigned int green, unsigned int blue){
    // initialize all of the bits to zero.
    curDataPacket.value = 0;

    curDataPacket.red   = red;
    curDataPacket.green = green;
    curDataPacket.blue  = blue;
    // command bit stays 0: dp.command = 0;
}

// BuildCommandPacket returns a 32-bit dataPacket for sending commands to the A6280/1.
//
// redDotCorrect, greenDotCorrect, and blueDotCorrect lets you control what 
// percentage of current is flowing to each color diode. 
// Refer to page 8 of the datasheet for more information.
// clockMode lets you set the PWM frequency for the diodes (A6281 only). 
// Refer to page 7 of the datasheet for more information.  
//
long unsigned int LedShift::BuildCommandPacket(uint8_t red, uint8_t green, uint8_t blue, uint8_t clockMode){
    // Make a packet and initialize all of the bits to zero.
    dataPacket dp = {value:0};

    dp.redCurrent   = red;
    dp.greenCurrent = green;
    dp.blueCurrent  = blue;
    dp.clockMode = clockMode;
    dp.command = 1;

    return dp.value;
}

// Same as above, but don't bother setting clock mode
long unsigned int LedShift::BuildCommandPacket(uint8_t red, uint8_t green, uint8_t blue){
  // Make a packet and initialize all of the bits to zero.
  dataPacket dp = {value:0};
  
  dp.redCurrent   = red;
  dp.greenCurrent = green;
  dp.blueCurrent  = blue;
  // clock mode stays at 0
  dp.command = 1;
  
  return dp.value;
}

void LedShift::SendPacket(long unsigned int dp){
  // Loop over the 31 data bits
  for(signed char i=31; i>=0; i--){
    // Data is read on rising edge of the clock pin, so set clock low here
    // TO DO: if the clock and data pins are on the same port, we could save an instruction
    // by setting the clock low when we set the data bit.
    *clockReg &= ~clockBit;
    // Set the appropriate Data In value
    if ((dp >> i) & 1)
      *dataReg |= dataBit;
    else
      *dataReg &= ~dataBit;
    // Now set the clock high to send data
    *clockReg |= clockBit;
  }
 *latchReg |= latchBit;
 *latchReg &= ~latchBit;
}

void LedShift::SendPacket(){
  // Loop over the 31 data bits
  // Looping over all 31 bits would be simpler, but that code is much slower. Bit-shifting each 
  // of the four bytes is *much* faster.
  //for(signed char i=31; i>=0; i--){
  for(signed char curByte=3; curByte>=0; curByte--){
    for(signed char bitNum=7; bitNum>=0; bitNum--){
      // Data is read on rising edge of the clock pin, so set clock low here
      // TO DO: if the clock and data pins are on the same port, we could save an instruction
      // by setting the clock low when we set the data bit.
      *clockReg &= ~clockBit;
      // Set the appropriate Data In value
      if((curDataPacket.byte[curByte] >> bitNum) & 1)
        *dataReg |= dataBit;
      else
        *dataReg &= ~dataBit;
      // Now set the clock high to send data
      *clockReg |= clockBit;
    }
  }
 *latchReg |= latchBit;
 *latchReg &= ~latchBit;
}

void LedShift::SendABit(unsigned char bitNum){
  // TO DO: if the clock and data pins are on the same port, we could save an instruction
  // by setting the clock low when we set the data bit.
  *clockReg &= ~clockBit;
  // Set the appropriate Data In value
  unsigned char curByte = bitNum/8;
  bitNum = bitNum%8;
  //if((curDataPacket.byte[curByte] >> bitNum) & 1)
  if((curDataPacket.value >> bitNum) & 1)
    *dataReg |= dataBit;
  else
    *dataReg &= ~dataBit;
  // Clock in the data
  *clockReg |= clockBit;
}


void LedShift::SetCurrents(uint8_t red, uint8_t green, uint8_t blue){
  // Set pwms to full-on:
  SendPacket(BuildColorPacket(1023, 1023, 1023));
  Latch();
  // Now set the current source values
  SendPacket(BuildCommandPacket(red, green, blue));
  Latch();
}

float LedShift::GetCurrentPercent(uint8_t currentByte){
  float currentPercent = 0.5*currentByte+36.5;
  return(currentPercent);
}

void LedShift::Latch(){
   *latchReg |= latchBit;
   *latchReg &= ~latchBit;
}

void LedShift::Enable(){
   *enableReg &= ~enableBit;
}

void LedShift::Disable(){
   *enableReg |= enableBit;
}
