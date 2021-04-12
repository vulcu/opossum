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
