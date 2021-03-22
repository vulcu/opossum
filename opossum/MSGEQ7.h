/*
 * MSGEQ7.h - MSI MSGEQ7 Seven Band Graphic Equalizer Driver for Arduino
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

#ifndef __MSGEQ7_H__
#define __MSGEQ7_H__

  class MSGEQ7 {
    public:
      MSGEQ7(uint8_t strobe_p, uint8_t output, uint8_t reset_p, bool isInputPullup) {
        // Use 'this->' to make the difference between the 'pin' 
        // attribute of the class and the local variable
        this->strobe_p = strobe_p;
        this->output = output;
        this->reset_p = reset_p;
        this->isInputPullup = isInputPullup;
      }


      void init(void) {
        // initialize the MSGEQ7 reset and strobe signals
        pinMode(strobe_p, OUTPUT);
        pinMode(reset_p,  OUTPUT);

        // set MSGEQ7 strobe low, and reset high (put device in standby)
        digitalWrite(strobe_p, LOW);
        digitalWrite(reset_p, HIGH);

        if (isInputPullup) {
          // initialize analog input 1 with internal pullup active
          digitalWrite(output, INPUT_PULLUP);
        }
        else {
          // initialize analog input 1 without internal pullup active
          digitalWrite(output, INPUT);
        }
      }


      void read(uint16_t *levelRead) {
        // read back spectral band data from the MSGEQ7
        // start by setting RESET pin low to enable output
        digitalWrite(reset_p, LOW);
        delayMicroseconds(100);

        // pulse STROBE pin to read all 7 frequency bands
        for(uint8_t k = 0; k < 7; k++) {
          // set STROBE pin low to enable output
          digitalWrite(strobe_p, LOW);
          delayMicroseconds(65);

          // read signal band level, account for later loudness adj.
          levelRead[k] = analogRead(output) << 3;

          // set STROBE pin high again to prepare for next band reading
          digitalWrite(strobe_p, HIGH);
          delayMicroseconds(35);
        }

        // set RESET high again to reset MSGEQ7 multiplexer
        digitalWrite(reset_p, HIGH);

        // spectral band adj. (very) loosely based on ISO 226:2003 [60 phons]
        levelRead[0] = levelRead[0] >> 3;   //   63 Hz
        levelRead[1] = levelRead[1] >> 1;   //  160 Hz
        levelRead[2] = levelRead[2] >> 0;   //  400 Hz
        levelRead[3] = levelRead[3] << 0;   // 1000 Hz
        levelRead[4] = levelRead[4] << 0;   // 2500 Hz
        levelRead[5] = levelRead[5] >> 1;   // 6250 Hz
        levelRead[6] = levelRead[6] >> 2;   //16000 Hz
      }


      uint16_t mean(uint16_t *levelRead) {
        // find mean of levelRead array data read from MSGEQ7
        // calculate the sum of levelRead array
        uint16_t sum = 0;
        for(uint8_t k = 0; k < 7; k++) {
          sum = sum + levelRead[k];
        }
        // much faster than divide-by-7, accurate to 7.00
        return (sum * 585L) >> 12;
      }


    private:
      bool isInputPullup;
      uint8_t output;
      uint8_t reset_p;
      uint8_t strobe_p;
  };

#endif
