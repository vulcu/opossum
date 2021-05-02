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
#include "MAX9744.h"

#ifndef AUDIOMATH_MODULE_PARAMETERS
#define AUDIOMATH_MODULE_PARAMETERS

  #define MAX9744_MAXIMUM_VOL_LEVEL  (uint8_t)63

  #define SIZE_DB_FAST_COEFFICIENT (uint8_t)25

  // AGC range bounds and analysis step size (in 1/100ths of a dB)
  // these values must be hardcoded to match `coeffecients_dB`
  #define MILLIBEL_BOUND_LOWER (int16_t) -600
  #define MILLIBEL_BOUND_UPPER (int16_t)  600
  #define MILLIBEL_STEP_SIZE   (int16_t)  50

#endif

// index for keeping track of the buffer used by decayBuffer32
static uint8_t buffer_indx_32 = 0;

// Coefficient table for fast dB approximations
static const uint16_t dB_fast_coefficient[SIZE_DB_FAST_COEFFICIENT] PROGMEM =
{
  2053, 2175, 2303, 2440, 2584,
  2738, 2900, 3072, 3254, 3446,
  3651, 3867, 4096, 4339, 4596,
  4868, 5157, 5462, 5786, 6129,
  6492, 6876, 7284, 7715, 8173
};

// update the relative dB level bands using currently defined volume level
void Audiomath::dBFastRelativeLevel(uint16_t *dBLevels, uint16_t baseLevel) {
  for(uint8_t k = 0; k < SIZE_DB_FAST_COEFFICIENT; k++) {
    dBLevels[k] = ((uint32_t)baseLevel * pgm_read_word(&(dB_fast_coefficient[k]))) >> 12;
  }
}

// return the dB gain values correllating amplifier volume settings
void Audiomath::convertVolumeToGain(uint8_t start, uint8_t stop, 
                                    int16_t *values, size_t size) {
  uint8_t index_minimum = ((start <= stop) ? start : stop);
  uint8_t index_maximum = ((start  > stop) ? start : stop);
  if ((index_maximum - index_minimum) > size) {
    return;   // the `values[]` array is not large enough to contain the requested range
  }
  else {
    for (uint8_t k = 0; k <= size; k++) {
      values[k] = MAX9744::getGainAtVolumeIndex(k);
    }
  }
}

// return the dB gain values correllating amplifier volume settings
void Audiomath::mapVolumeToBoundedRange(uint8_t volume, uint8_t *volumeMap, 
                                        int16_t lower_bound, int16_t upper_bound,
                                        int16_t step_size, uint8_t max_volume_value) {
  if (lower_bound == upper_bound) {
    volumeMap[0] = volume;
    return;   // if the range is zero, the volume map can only be one number
  }

  uint8_t a;
  uint8_t b;
  uint8_t index_minimum = ((lower_bound <= upper_bound) ? lower_bound : upper_bound);
  uint8_t index_maximum = ((lower_bound  > upper_bound) ? lower_bound : upper_bound);
  size_t map_size = (upper_bound - lower_bound) / step_size + 1;
  uint16_t offset_mB[map_size] = {0};
  uint8_t GmB_volume = MAX9744::getGainAtVolumeIndex(volume);

  for (uint8_t k = 0; k < map_size; k++) {
    offset_mB[k] = k * step_size;
  }

  if (volume == 1) {
    a = 0;
  }
  else {
    for(uint8_t k = volume - 1; k >= 0; k--) {
      if ((GmB_volume - MAX9744::getGainAtVolumeIndex(k)) > lower_bound) {
        a = k + 1;
        break;
      }
      else if (k == 0) {
        a = 0;
        break;
      }
    }
  }

  if (volume == max_volume_value) {
    b = max_volume_value;
  }
  else {
    for(uint8_t k = volume + 1; k <= max_volume_value; k++) {
      if ((MAX9744::getGainAtVolumeIndex(k) - GmB_volume) > upper_bound) {
        b = k - 1;
        break;
      }
      else if (k >= max_volume_value) {
        b = max_volume_value;
        break;
      }
    }
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
