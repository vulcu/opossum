/*
 * MSGEQ7.cpp - MSI MSGEQ7 Seven Band Graphic Equalizer Driver for Arduino
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

#include "MSGEQ7.h"

// class constructor for MSGEQ7 object
MSGEQ7::MSGEQ7(uint8_t strobe_p, uint8_t dc_out, uint8_t reset_p, bool isInputPullup) {
  this->strobe_p = strobe_p;
  this->dc_out = dc_out;
  this->reset_p = reset_p;
  this->isInputPullup = isInputPullup;
}

// initialize the MSGEQ7 reset and strobe signals
void MSGEQ7::init(void) {
  pinMode(strobe_p, OUTPUT);
  pinMode(reset_p,  OUTPUT);

  // set MSGEQ7 strobe low, and reset high (put device in standby)
  digitalWrite(strobe_p, LOW);
  digitalWrite(reset_p, HIGH);

  if (isInputPullup) {
    // initialize analog input 1 with internal pullup active
    digitalWrite(dc_out, INPUT_PULLUP);
  }
  else {
    // initialize analog input 1 without internal pullup active
    digitalWrite(dc_out, INPUT);
  }
}

// find mean of the `array_values` data read from MSGEQ7
uint16_t MSGEQ7::mean(uint16_t *read_array, size_t array_size) {
  uint16_t sum = 0;
  if ((uint16_t)array_size != (uint16_t)7) {
    // there are 7 spectral bands so the array size must be 7 elements
    // if this is not the case then don't do anything and return zero
    return sum;
  }
  else {
    // calculate the sum of levelRead array
    for(uint8_t k = 0; k < 7; k++) {
      sum = sum + read_array[k];
    }
    // much faster than divide-by-7, accurate to 7.00
    return (sum * 585L) >> 12;
  }
}

// read spectral band data from the MSGEQ7 and write to `read_array`
void MSGEQ7::read(uint16_t *read_array, size_t array_size) {
  // there are 7 spectral bands so the array size must be 7 elements
  if ((uint16_t)array_size != (uint16_t)7) {
    return;
  }
  else {
    // start by setting RESET pin low to enable output
    digitalWrite(reset_p, LOW);
    delayMicroseconds(100);

    // pulse STROBE pin to read all 7 frequency bands
    for(uint8_t k = 0; k < 7; k++) {
      // set STROBE pin low to enable output
      digitalWrite(strobe_p, LOW);
      delayMicroseconds(65);

      // read signal band level, account for later loudness adj.
      read_array[k] = analogRead(dc_out) << 3;

      // set STROBE pin high again to prepare for next band reading
      digitalWrite(strobe_p, HIGH);
      delayMicroseconds(35);
    }

    // set RESET high again to reset MSGEQ7 multiplexer
    digitalWrite(reset_p, HIGH);

    // spectral band adj. (very) loosely based on ISO 226:2003 [60 phons]
    read_array[0] = read_array[0] >> 3;   //   63 Hz
    read_array[1] = read_array[1] >> 1;   //  160 Hz
    read_array[2] = read_array[2] >> 0;   //  400 Hz
    read_array[3] = read_array[3] << 0;   // 1000 Hz
    read_array[4] = read_array[4] << 0;   // 2500 Hz
    read_array[5] = read_array[5] >> 1;   // 6250 Hz
    read_array[6] = read_array[6] >> 2;   //16000 Hz
  }
}
