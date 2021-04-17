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

#ifndef __OPOSSUM_AUDIO_MATH_H__
#define __OPOSSUM_AUDIO_MATH_H__

  class Audiomath {
    public:
      static void dBFastRelativeLevel(uint16_t *dBLevels, uint16_t baseLevel);
      static uint16_t decayBuffer32(uint16_t *levelBuf, uint16_t levelReadMean, 
                                    const uint16_t nominal_signal_level);
  };

#endif
