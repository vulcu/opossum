/*
 * audiomath.cpp - Math library for Opossum BT Aplifier
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

#include <avr/pgmspace.h>

#include "audiomath.h"

// index for keeping track of the buffer used by decayBuffer32
static uint8_t buffer_indx_32 = 0;

// Coefficient table for fast dB approximations
static const uint16_t dBCoefTable[25] PROGMEM =
{
  2053, 2175, 2303, 2440, 2584,
  2738, 2900, 3072, 3254, 3446,
  3651, 3867, 4096, 4339, 4596,
  4868, 5157, 5462, 5786, 6129,
  6492, 6876, 7284, 7715, 8173
};

// update the relative dB level bands using currently defined volume level
void Audiomath::dBFastRelativeLevel(uint16_t *dBLevels, uint16_t baseLevel) {
  for(uint8_t k = 0; k < 25; k++) {
    dBLevels[k] = ((uint32_t)baseLevel * pgm_read_word(&(dBCoefTable[k]))) >> 12;
  }
}

// use a 32-value circular buffer to track audio levels, MUST be 32 elements
uint16_t Audiomath::decayBuffer32(uint16_t *data_buffer, size_t buffer_size,
                                  uint16_t data_mean, const uint16_t nominal_zero_signal_level) {
    if ((uint16_t)buffer_size != (uint16_t)32) {
      // this method will only return accurate values for arrays with 32 elements
      // if this is not the case then don't do anything and return zero
      return (uint16_t)0;
    }
    else {
      if (buffer_indx_32 >= 32) {
        buffer_indx_32 = 0;
      }
      if (data_mean > data_buffer[buffer_indx_32]) {
        // if the new value is greater, use value halfway between old and new
        data_buffer[buffer_indx_32] = data_buffer[buffer_indx_32] +
          ((data_mean - data_buffer[buffer_indx_32]) >> 1);
      }
      else if (data_mean < ((nominal_zero_signal_level * (uint16_t)18) >> 4)) {
        // if the latest level is less a small % over the nominal zero signal,
        // there's probs no significant audio signal so don't update buffer
      }
      else {
        // otherwise, decay the current value by approximately 3%
        data_buffer[buffer_indx_32] = (data_buffer[buffer_indx_32] * 31L) >> 5;
      }
      buffer_indx_32++;

      // calculate total sum of exponential buffer array
      uint32_t sum = 0;
      for (uint16_t k = 0; k < 32; k++) {
        sum = sum + data_buffer[k];
      }

      // divide the sum by 32 and return the array mean
      return (uint16_t)(sum >> 5);
    }
}
