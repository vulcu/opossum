/*
 * drivers.h - Bluetooth, Analog, & Amplifier drivers for Opossum BT Aplifier
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

#ifndef DRIVERS_H
#define DRIVERS_H

  #include "parameters.h"

 /*
  *  ########################
  *  BM62 Bluetooth Interface
  *  ########################
  */
  // BM62 UART commands for media playback control
  const uint8_t BM62_Play[7] =
  {
    0xAA, 0x00, 0x03, 0x04, 0x00, 0x05, 0xF4
  };
  const uint8_t BM62_Pause[7] =
  {
    0xAA, 0x00, 0x03, 0x04, 0x00, 0x06, 0xF3
  };
  const uint8_t BM62_Stop[7] =
  {
    0xAA, 0x00, 0x03, 0x04, 0x00, 0x08, 0xF1
  };
  const uint8_t BM62_PrevTrack[7] =
  {
    0xAA, 0x00, 0x03, 0x02, 0x00, 0x35, 0xC6
  };
  const uint8_t BM62_NextTrack[7] =
  {
    0xAA, 0x00, 0x03, 0x02, 0x00, 0x34, 0xC7
  };

  // check if the BM62 programming pin is pulled low
  void BM62_isProgramMode(void) {
    if (!digitalRead(PRGM_SENSE_N)) {
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down
      cli();                                // globally disable interrupts
      sleep_enable();                       // set sleep bit
      sleep_bod_disable();                  // disable brown out detection
      sleep_cpu();                          // go to sleep
      
      // interrupts are disabled, mcu will NOT wake up until next power cycle!
    }
  }

  // for calculating the checksum of a BM62 UART command:
  byte BM62_checksum(uint8_t a[], uint8_t numel) {
    // BM62 documentation is lacking but pretty sure
    uint16_t sum = 0;
    for (uint8_t k = 2; k < numel - 1; k++) {
      sum = sum + a[k];
    }

    // subtract sum from 0xFFFF and add one; use only the lower byte
    sum = ((uint16_t)0xFFFF - sum) + (uint16_t)0x0001;
    return(lowByte(sum));
  }



 /*  
  *  ##############################
  *  MSGEQ7 Spectrum Level Detector
  *  ##############################
  */
  // read back spectral band data from the MSGEQ7
  void MSGEQ7_read(uint16_t *levelRead) {
    // set RESET pin low to enable output
    digitalWrite(RESET, LOW);
    delayMicroseconds(100);

    // pulse STROBE pin to read all 7 frequency bands
    for(uint8_t k = 0; k < 7; k++) {
      // set STROBE pin low to enable output
      digitalWrite(STROBE, LOW);
      delayMicroseconds(65);

      // read signal band level, account for later loudness adj.
      levelRead[k] = analogRead(DCOUT) << 3;

      // set STROBE pin high again to prepare for next band reading
      digitalWrite(STROBE, HIGH);
      delayMicroseconds(35);
    }

    // set RESET high again to reset MSGEQ7 multiplexer
    digitalWrite(RESET, HIGH);

    // spectral band adj. (very) loosely based on ISO 226:2003 [60 phons]
    levelRead[0] = levelRead[0] >> 3;   //   63 Hz
    levelRead[1] = levelRead[1] >> 1;   //  160 Hz
    levelRead[2] = levelRead[2] >> 0;   //  400 Hz
    levelRead[3] = levelRead[3] << 0;   // 1000 Hz
    levelRead[4] = levelRead[4] << 0;   // 2500 Hz
    levelRead[5] = levelRead[5] >> 1;   // 6250 Hz
    levelRead[6] = levelRead[6] >> 2;   //16000 Hz
  }

  // find mean of levelRead array data read from MSGEQ7
  uint16_t MSGEQ7_mean(uint16_t *levelRead) {
    // calculate the sum of levelRead array
    uint16_t sum = 0;
    for(uint8_t k = 0; k < 7; k++) {
      sum = sum + levelRead[k];
    }
    // much faster than divide-by-7, accurate to 7.00
    return (sum * 585L) >> 12;
  }



 /*  
  *  #################
  *  MAX9744 Amplifier
  *  #################
  */
  // MAX9744 absolute gain levels (dB), multiplied
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

#endif
