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

#ifndef __MAX9744_H__
#define __MAX9744_H__

  #include <avr/pgmspace.h>
  #include <Wire.h>

  class MAX9744 {
    private:
      bool invert_mute;
      uint8_t i2c_addr;
      uint8_t mute_p;
      uint8_t shutdown_n;
      TwoWire* wire;

    public:
      MAX9744(uint8_t i2c_addr, uint8_t mute_p, uint8_t shutdown_n, TwoWire* wire);

      void init(void);
      void enable(void);
      void invertMuteLogic(bool invert_mute);
      void mute(void);
      void shutdown(void);
      void unmute(void);
      void volume(uint8_t value);
  };

#endif
