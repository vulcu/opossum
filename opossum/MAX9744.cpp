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
    950,    880,    820,    760,
    700,    650,    590,    540,
    490,    440,    390,    340,
    290,    240,    200,    160,
    120,     50,    -50,   -190,
   -340,   -500,   -600,   -710,
   -890,   -990,  -1090,  -1200,
  -1310,  -1440,  -1540,  -1640,
  -1750,  -1970,  -2160,  -2350,
  -2520,  -2720,  -2980,  -3150,
  -3340,  -3600,  -3760,  -3960,
  -4210,  -4370,  -4560,  -4810,
  -5060,  -5420,  -5670,  -6020,
  -6270,  -6620,  -6870,  -7220,
  -7470,  -7830,  -8080,  -8430,
  -8680,  -9030,  -9290,  -10950
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
    for (uint8_t k = index_minimum; k <= index_maximum; k++) {
      values[k] = pgm_read_word(&(MAX9744Gain_milliBels[k]));
    }
  }
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

