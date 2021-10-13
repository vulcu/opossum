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

#ifndef AUDIOMATH_AGC_BOUND_VALUES
#define AUDIOMATH_AGC_BOUND_VALUES

  // AGC range bounds and analysis step size (in 1/100ths of a dB)
  // these values must be hardcoded to match `coeffecients_dB`
  #define MILLIBEL_BOUND_LOWER (int16_t) -600
  #define MILLIBEL_BOUND_UPPER (int16_t)  600
  #define MILLIBEL_STEP_SIZE   (int16_t)  50

#endif

// index for keeping track of the buffer used by decayBuffer32
uint8_t Audiomath::buffer_indx_32 = 0;

// index for keeping track of the most recent Volume Map index
uint8_t Audiomath::vm_index_previous = (DB_FAST_COEFFICIENT_COUNT >> 1);

// Coefficient table for fast dB approximations
const uint16_t Audiomath::dB_fast_coefficient[DB_FAST_COEFFICIENT_COUNT] PROGMEM =
{
  2053, 2175, 2303, 2440, 2584,
  2738, 2900, 3072, 3254, 3446,
  3651, 3867, 4096, 4339, 4596,
  4868, 5157, 5462, 5786, 6129,
  6492, 6876, 7284, 7715, 8173
};

// update the relative dB level bands using currently defined volume level
void Audiomath::dBFastRelativeLevel(uint16_t dBLevels[], const uint16_t baseLevel) {
  for(uint8_t k = 0; k < DB_FAST_COEFFICIENT_COUNT; k++) {
    dBLevels[k] = ((uint32_t)baseLevel * pgm_read_word(&(dB_fast_coefficient[k]))) >> 12;
  }
}

// return the dB gain values correllating amplifier volume settings
void Audiomath::convertVolumeToGain(const uint8_t start, const uint8_t stop, 
                                    int16_t values[], const size_t values_size) {
  uint8_t index_minimum = ((start <= stop) ? start : stop);
  uint8_t index_maximum = ((start  > stop) ? start : stop);
  if ((index_maximum - index_minimum) > values_size) {
    return;   // the `values[]` array is not large enough to contain the requested range
  }
  else {
    for (uint8_t k = 0; k <= values_size; k++) {
      values[k] = MAX9744::getGainAtVolumeIndex(index_minimum + k);
    }
  }
}

// use a 32-value circular buffer to track audio levels, MUST be 32 elements
uint16_t Audiomath::decayBuffer32(uint16_t data_buffer[], const size_t buffer_size,
                                  const uint16_t data_mean, 
                                  const uint16_t nominal_zero_signal_level) {
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

// return a map of volume thresholds based on gain-to-volume levels at present volume setting
void Audiomath::mapVolumeToBoundedRange(const uint8_t volume, uint8_t volumeMap[], 
                                        const size_t input_map_size) {
  if (input_map_size != (size_t)DB_FAST_COEFFICIENT_COUNT) {
    return; // volumeMap array isn't sized right so return without doing anything
  }
  uint8_t a = 0;
  uint8_t b = MAX9744_MAXIMUM_VOL_LEVEL;
  int16_t gain_current_volume = MAX9744::getGainAtVolumeIndex(volume);

  // reset the volume map index used by getVolumeMapIndx() to the default mid value
  vm_index_previous = (DB_FAST_COEFFICIENT_COUNT >> 1);

  // find the upper and lower bounds for volume values
  if (volume != 0) {
    for(int16_t k = volume - 1; k >= 0; k--) {
      if ((MAX9744::getGainAtVolumeIndex(k) - gain_current_volume) < MILLIBEL_BOUND_LOWER) {
        a = k + 1;
        break;
      }
    }
  }
  if (volume != MAX9744_MAXIMUM_VOL_LEVEL) {
    for(uint8_t k = volume + 1; k < MAX9744_MAXIMUM_VOL_LEVEL; k++) {
      if ((MAX9744::getGainAtVolumeIndex(k) - gain_current_volume) > MILLIBEL_BOUND_UPPER) {
        b = k - 1;
        break;
      }
    }
  }

  uint8_t v_range = (b - a) + 1;
  uint8_t v[v_range];
  for (int16_t k = b; k >= (int16_t)a; k--) {
    v[b - k] = k;
  }

  int16_t c_normal[v_range];
  //int16_t c_offset[v_range] = {0};
  for (uint8_t k = 0; k < v_range; k++) {
    c_normal[k] = MAX9744::getGainAtVolumeIndex(v[k]) - gain_current_volume;
  }

  uint8_t skip_zero_index = 0;
  uint8_t map_index = 0;
  for (uint8_t k = 0; k < input_map_size; k++) {
    if (c_normal[map_index] == 0) {
      skip_zero_index = 1;
    }
    if ((map_index + skip_zero_index) < v_range) {
      if ((MILLIBEL_BOUND_UPPER - c_normal[map_index + skip_zero_index]) 
          < (k * MILLIBEL_STEP_SIZE)) {
        map_index++;
      }
    }
    volumeMap[k] = v[map_index];
  }
}

// return a hysterisis-filtered volume map index value that corresponds to the mean audio level
uint8_t Audiomath::getVolumeMapIndx(const uint16_t audio_level, const uint16_t dBLevels[], 
                                    const size_t dBLevels_size) {
  // check dBLevels array is sized correctly, if not then return default mid-array index
  // bit shift to account for dBLevels being uint16_t and sizeof() returning byte count
  if (dBLevels_size != (size_t)(DB_FAST_COEFFICIENT_COUNT << 1)) {
    return (uint8_t)(DB_FAST_COEFFICIENT_COUNT >> 1);
  }
  for (int8_t k = (DB_FAST_COEFFICIENT_COUNT - 1); k >= 0; k--) {
    if ((dBLevels[k] <= audio_level) | (k < 1)) {
      if (abs(vm_index_previous - k) > 1) {
        vm_index_previous = ((vm_index_previous < k) ? 
                             (vm_index_previous + 1) : 
                             (vm_index_previous - 1));
      }
      else if (((vm_index_previous < 2) & (k < vm_index_previous)) |
               ((vm_index_previous > 22) & (k > vm_index_previous))){
        vm_index_previous = k;
      }
      break;
    }
  }
  return vm_index_previous;
}
