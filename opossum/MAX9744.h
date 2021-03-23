/*
 * MAX9744.h - Maxim MAX9744 Class-D Amplifier driver for Arduino
 * Copyright (c) 2017 Winry R. Litwa-Vulcu. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __MAX9744_H__
#define __MAX9744_H__

  #include <avr/pgmspace.h>
  #include <Wire.h>

  // MAX9744 amplifier gain levels (dB), multiplied
  // by 10x to allow PROGMEM storage as int16_t
  const int16_t MAX9744GainLevel[64] PROGMEM = 
  {
    95,    88,    82,    76,
    70,    65,    59,    54,
    49,    44,    39,    34,
    29,    24,    20,    16,
    12,    05,   -05,   -19,
    -34,   -50,   -60,   -71,
    -89,   -99,  -109,  -120,
    -131,  -144,  -154,  -164,
    -175,  -197,  -216,  -235,
    -252,  -272,  -298,  -315,
    -334,  -360,  -376,  -396,
    -421,  -437,  -456,  -481,
    -506,  -542,  -567,  -602,
    -627,  -662,  -687,  -722,
    -747,  -783,  -808,  -843,
    -868,  -903,  -929, -1095
  };

  class MAX9744 {
    public:
      MAX9744(uint8_t i2c_addr, uint8_t mute_p, uint8_t shutdown_n, bool init_twi) {
        // Use 'this->' to make the difference between the 'pin' 
        // attribute of the class and the local variable
        this->i2c_addr = i2c_addr;
        this->mute_p = mute_p;
        this->shutdown_n = shutdown_n;
        this->init_twi = init_twi;
      }

      void init(void) {
        // initialize the MAX9744 mute and shutdown signals
        pinMode(mute_p, OUTPUT);
        pinMode(shutdown_n, OUTPUT);

        // mute the MAX9744 then take it out of shutdown
        mute();
        enable();

        if (init_twi) {
          // do this check so that Wire only gets initialized once
          //if (TWCR ^ (0x01 << TWEN)) {
            // if TWI not enabled, initialize Wire library
            Wire.begin();
          //}
          // set TWI clock rate (default is 400000L)
          Wire.setClock(TWI_CLOCK_RATE);
        }
      }

      void enable(void) {
        // enable the MAX9744 by taking it out of shutdown (HIGH)
        digitalWrite(shutdown_n, HIGH);
      }

      void invertMuteLogic(bool invert_mute) {
        // invert the mute signal (needed for use with some MAX9744 kits)
        this->invert_mute = invert_mute;
      }

      void mute(void) {
        // mute the MAX9744 output (HIGH)
        if (invert_mute) {
          digitalWrite(mute_p, LOW);
        }
        else {
          digitalWrite(mute_p, HIGH);
        }
      }

      void shutdown(void) {
        // disable the MAX9744 by putting it into shutdown (LOW)
        digitalWrite(shutdown_n, LOW);
      }

      void unmute(void) {
        // unmute the MAX9744 output (LOW)
        if (invert_mute) {
          digitalWrite(mute_p, HIGH);
        }
        else {
          digitalWrite(mute_p, LOW);
        }
      }

      void volume(uint8_t value) {
        // initialize the MSGEQ7 reset and strobe signals
        // & (TWCR & (0x01 << TWEN)))
        if (value < 64) {
          Wire.beginTransmission(i2c_addr);
            Wire.write(value);
          Wire.endTransmission();
        }
      }


    private:
      #ifndef TWI_CLOCK_RATE
        #define TWI_CLOCK_RATE (int32_t)400000
      #endif

      bool init_twi;
      bool invert_mute = false;
      uint8_t i2c_addr;
      uint8_t mute_p;
      uint8_t shutdown_n;
  };

#endif
