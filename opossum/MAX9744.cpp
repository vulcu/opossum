/*
 * MAX9744.cpp - Maxim MAX9744 Class-D Amplifier driver for Arduino
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

#include "MAX9744.h"

// MAX9744 amplifier gain levels (dB), stored as milli-Bells 
// (1/100 of a dB) to allow PROGMEM storage as an int type
static const int16_t MAX9744Gain_milliBels[64] PROGMEM = 
{
  -10950,  -9290,   -9030,   -8680,
  -8430,   -8080,   -7830,   -7470,
  -7220,   -6870,   -6620,   -6270,
  -6020,   -5670,   -5420,   -5060,
  -4810,   -4560,   -4370,   -4210,
  -3960,   -3760,   -3600,   -3340,
  -3150,   -2980,   -2720,   -2520,
  -2350,   -2160,   -1970,   -1750,
  -1640,   -1540,   -1440,   -1310,
  -1200,   -1090,   -990,    -890,
  -710,    -600,    -500,    -340,
  -190,    -50,      50,      120,
   160,     200,     240,     290,
   340,     390,     440,     490,
   540,     590,     650,     700,
   760,     820,     880,     950
};

// class constructor for MAX9744 amplifier object
MAX9744::MAX9744(uint8_t i2c_addr, uint8_t mute_p, uint8_t shutdown_n, TwoWire* wire) :
  invert_mute(false) {
  this->i2c_addr = i2c_addr;
  this->mute_p = mute_p;
  this->shutdown_n = shutdown_n;
  this->wire = wire;
}

// initialize the MAX9744 and GPIO signals
void MAX9744::init(void) {
  pinMode(mute_p, OUTPUT);
  pinMode(shutdown_n, OUTPUT);

  // mute the MAX9744 then take it out of shutdown
  mute();
  enable();
}

// return the dB gain values correllating amplifier volume settings
void MAX9744::convertVolumeToGain(uint8_t start, uint8_t stop, int16_t *values, size_t size) {
  uint8_t index_minimum = ((start <= stop) ? start : stop);
  uint8_t index_maximum = ((start  > stop) ? start : stop);
  if ((index_maximum - index_minimum) > size) {
    return;   // the `values[]` array is not large enough to contain the requested range
  }
  else {
    for (uint8_t k = 0; k <= size; k++) {
      values[k] = pgm_read_word(&(MAX9744Gain_milliBels[k + index_minimum]));
    }
  }

  /* #ifndef AUDIOMATH_MODULE_PARAMETERS
  #define AUDIOMATH_MODULE_PARAMETERS
    #define MILLIBEL_BOUND_LOWER (int16_t) -600
    #define MILLIBEL_BOUND_UPPER (int16_t)  600
  #endif */

  // MATLAB code for determining which real amp gain values fall within +/- 800 mB
    /* if (volume == 1)
      a = 1;
    else
      for k = volume-1:-1:1
        if (((gain_mB(volume) - gain_mB(k)) >=  range_mB) || (k <= 1));
          a = k;
          break;
        end
      end
    end
    if (volume == numel(gain_mB))
      b = numel(gain_mB);
    else
      for k = volume+1:numel(gain_mB);
        if (((gain_mB(k) - gain_mB(volume)) >=  range_mB) || (k >= 64));
          b = k;
          break;
        end
      end
    end
    sizeof = b-a+1;
    c = zeros(sizeof, 1);
    c = gain_mB(a:b); */
}

// enable the MAX9744 by taking it out of shutdown (HIGH)
void MAX9744::enable(void) {
  digitalWrite(shutdown_n, HIGH);
}

// invert the mute signal (needed for use with Adafruit MAX9744 board)
void MAX9744::invertMuteLogic(bool invert_mute) {
  this->invert_mute = invert_mute;
}

// mute the MAX9744 amplifier via the MUTE pin
void MAX9744::mute(void) {
  // if the mute signal is inverted then set LOW to mute
  if (invert_mute) {
    digitalWrite(mute_p, LOW);
  }
  else {
    digitalWrite(mute_p, HIGH);
  }
}

// disable the MAX9744 via the GPIO shutdown signal
void MAX9744::shutdown(void) {
  digitalWrite(shutdown_n, LOW);
}

// unmute the MAX9744 amplifier via the MUTE pin
void MAX9744::unmute(void) {
  // if the mute signal is inverted then set HIGH to unmute
  if (invert_mute) {
    digitalWrite(mute_p, HIGH);
  }
  else {
    digitalWrite(mute_p, LOW);
  }
}

// set the amplifier volume to a value between 0 [min] and 63 [max]
void MAX9744::volume(uint8_t value) {
  // if the value is less than 64 and two-wire is configured
  if ((value < 64) & (TWCR != 0x00)) {
    wire->beginTransmission(i2c_addr);
      wire->write(value);
    wire->endTransmission();
  }
}

