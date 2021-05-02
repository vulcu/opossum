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

  class Audiomath {
    public:
      static void     convertVolumeToGain(uint8_t start, uint8_t stop, 
                                          int16_t *values, uint8_t size);
      static void     dBFastRelativeLevel(uint16_t *dBLevels, uint16_t baseLevel);
      static uint16_t decayBuffer32(uint16_t *data_buffer, size_t buffer_size, 
                                    uint16_t data_mean, const uint16_t nominal_zero_signal_level);
      static void     mapVolumeToBoundedRange(uint8_t volume, uint8_t *volumeMap, 
                                              uint8_t size_volumeMap);
  };

#endif
