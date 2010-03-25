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

#ifndef LedShift_h
#define LedShift_h

#include <inttypes.h>

typedef union dataPacket{
    long unsigned int value;
    struct{
        unsigned greenCurrent:7;
        unsigned clockMode:2;
        unsigned :1;
        unsigned redCurrent:7;
        unsigned :3;
        unsigned blueCurrent:7;   
    };
    struct{
        unsigned green:10;
        unsigned red:10;
        unsigned blue:10;
        unsigned command:1;
        // the 31st bit is the same for the two aliases: command
    };
} dataPacket;


class LedShift{

public:
  //LedShift();
  LedShift(uint8_t dataPin, uint8_t latchPin, uint8_t enablePin, uint8_t clockPin);
  
  long unsigned int BuildColorPacket(unsigned int red, unsigned int green, unsigned int blue);
  long unsigned int BuildCommandPacket(uint8_t red, uint8_t green, uint8_t blue);
  long unsigned int BuildCommandPacket(uint8_t red, uint8_t green, uint8_t blue, uint8_t clockMode);
  void SendPacket(long unsigned int dp);
  void Latch();
  
  void SetCurrents(uint8_t red, uint8_t green, uint8_t blue);
  float GetCurrentPercent(uint8_t currentByte);
  
  
private:
  //void init(char separator, char prefix, char suffix);
  //  the clock and data register/bitmasks are precomputed to speed up the ISR
  volatile uint8_t *dataReg;
  uint8_t dataBit;
  volatile uint8_t *latchReg;
  uint8_t latchBit;
  volatile uint8_t *enableReg;
  uint8_t enableBit;
  volatile uint8_t *clockReg;
  uint8_t clockBit;
};

#endif

