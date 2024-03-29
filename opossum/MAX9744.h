/*
 * MAX9744.h - Maxim MAX9744 Class-D Amplifier driver for Arduino
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

#ifndef MAX9744_H
#define MAX9744_H

  #include <avr/pgmspace.h>
  #include <Wire.h>

  // maximum volume level of the MAX9744 amplifier
  #define MAX9744_MAXIMUM_VOL_LEVEL  (uint8_t)63

  class MAX9744 {
    public:
      MAX9744(uint8_t i2c_addr, uint8_t mute_p, uint8_t shutdown_n, TwoWire* wire);

      void init(void);
      void enable(void);
      void invertMuteLogic(const bool invert_mute);
      void mute(void);
      void shutdown(void);
      void unmute(void);
      void volume(const uint8_t value);

      static inline int16_t getGainAtVolumeIndex(const uint8_t index);

    private:
      bool invert_mute;
      uint8_t i2c_addr;
      uint8_t mute_p;
      uint8_t shutdown_n;
      static const int16_t MAX9744Gain_milliBels[64] PROGMEM;
      TwoWire* wire;
  };

#endif
