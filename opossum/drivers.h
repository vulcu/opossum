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
