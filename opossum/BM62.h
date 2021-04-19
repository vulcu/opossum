/*
 * BM62.h - Microchip BM62 Audio Module Driver for Arduino
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

#ifndef __BM62_H__
#define __BM62_H__

  class BM62 {
    public:
      // These are the equalizer preset modes available on the BM62
      enum EQ_Preset_t
      {
        EQ_Off, 
        EQ_Soft, 
        EQ_Bass, 
        EQ_Treble, 
        EQ_Classical, 
        EQ_Rock, 
        EQ_Jazz, 
        EQ_Pop, 
        EQ_Dance,
        EQ_RnB, 
        EQ_Custom
      };

      BM62(uint8_t prgm_sense_n, uint8_t reset_n, uint8_t ind_a2dp_n, 
          HardwareSerial* hserial);

      void enable(void);
      void enterPairingMode(void);
      void init(void);
      bool isConnected(void);
      void reset(void);
      void setEqualizerPreset(EQ_Preset_t preset);
      void play(void);
      void pause(void);
      void playPauseToggle(void);
      void stop(void);
      void previous(void);
      void next(void);

    private:
      uint8_t prgm_sense_n;
      uint8_t reset_n;
      uint8_t ind_a2dp_n;
      HardwareSerial* hserial;

      uint8_t checksum(uint8_t command[], uint8_t command_length);
      void    isProgramMode(void);
      void    writeSerialCommand(const uint8_t *instruction, const uint16_t bytes_command);

  };

#endif
