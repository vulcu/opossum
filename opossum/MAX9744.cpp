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

// MAX9744 amplifier gain levels (dB), multiplied
// by 10x to allow PROGMEM storage as int16_t
static const int16_t MAX9744GainLevel[64] PROGMEM = 
{
  95,    88,    82,    76,
  70,    65,    59,    54,
  49,    44,    39,    34,
  29,    24,    20,    16,
  12,     5,    -5,    -19,
  -34,   -50,   -60,   -71,
  -89,   -99,   -109,  -120,
  -131,  -144,  -154,  -164,
  -175,  -197,  -216,  -235,
  -252,  -272,  -298,  -315,
  -334,  -360,  -376,  -396,
  -421,  -437,  -456,  -481,
  -506,  -542,  -567,  -602,
  -627,  -662,  -687,  -722,
  -747,  -783,  -808,  -843,
  -868,  -903,  -929,  -1095
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

