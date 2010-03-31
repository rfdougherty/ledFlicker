/*
 * RobustEEPROM.h - Robust (error-correcting) EEPROM library
 *
 * Copyright 2010 Bob Dougherty.
 *
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
 * 2010.03.28 Bob Dougherty (bobd@stanford.edu) wrote it, based on code by Richard Soja.
 */

/* Functions to Generate hamming codes of distance 4, for data sizes in the range 1 bit to 11 bits.
The upper bound is limited by the encoded word type bit range (16 bits).
Corrects 1 bit error in any position (check or data), and detects 2 bit errors in any position.
After execution of the <Decode> function, the global variable <ErrFlag> is updated to indicate
level of error correction.
i.e.                    ErrFlag                              Condition
                            0                  No errors detected or corrected.
                            1                  One error detected and corrected.
                            2                  Two errors detected, but correction is erroneous.
When ErrFlag is 2, Decode will return a bad value because we can detect, but not correct two errors.
*/

#ifndef RobustEEPROM_h
#define RobustEEPROM_h

#include <inttypes.h>

class RobustEEPROM
{
  public:
    uint8_t readByte(int);
    void writeByte(int, uint8_t);

  private:
    byte DataSize;
    byte CodeSize;
    byte EncodedWord;
    byte ErrFlag;

    bool OddParity(word Code); // Returns true if Code is odd parity, otherwise returns false
    word Power2(byte e);
    byte InitEncode(byte DataLength);
    word MakeCheck(word Data);
    word Encode(word Data);
    word Decode(word Code);
};

#endif

