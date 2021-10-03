/*
 * audiomath.h - Math library for Opossum BT Aplifier
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

#ifndef OPOSSUM_AUDIOMATH_H
#define OPOSSUM_AUDIOMATH_H

  // array in PROGMEM for storing dB coefficients
  #define DB_FAST_COEFFICIENT_COUNT (uint8_t)25

  class Audiomath {
    public:
      static void     convertVolumeToGain(const uint8_t start, const uint8_t stop, 
                                          int16_t values[], const size_t size);
      static void     dBFastRelativeLevel(uint16_t dBLevels[], const uint16_t baseLevel);
      static uint16_t decayBuffer32(uint16_t data_buffer[], const size_t buffer_size, 
                                    uint16_t const data_mean, 
                                    const uint16_t nominal_zero_signal_level);
      static void     mapVolumeToBoundedRange(const uint8_t volume, uint8_t volumeMap[], 
                                              const size_t size_volumeMap);
      static uint8_t  getVolumeMapIndx(const uint16_t audio_level, const uint16_t dBLevels[], 
                                       const size_t dBLevels_size);
    
    private:
      static uint8_t buffer_indx_32;
      static uint8_t vm_index_previous;
      static const uint16_t dB_fast_coefficient[DB_FAST_COEFFICIENT_COUNT] PROGMEM;
  };

#endif
