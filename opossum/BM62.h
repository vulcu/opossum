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

  #include <avr/sleep.h>

  #ifndef BYTE_COUNT_MEDIA_COMMAND
    #define BYTE_COUNT_MEDIA_COMMAND (uint8_t)7
  #endif
  #ifndef BYTE_COUNT_MEDIA_PREFIX
    #define BYTE_COUNT_MEDIA_PREFIX (uint8_t)3
  #endif
  #ifndef BYTE_COUNT_MEDIA_INSTRUCTION
    #define BYTE_COUNT_MEDIA_INSTRUCTION (uint8_t)3
  #endif

  class BM62 {
    public:
      BM62(uint8_t prgm_sense_n, uint8_t reset_n, uint8_t ind_a2dp_n, 
          HardwareSerial* hserial) {
        // Use 'this->' to make the difference between the 'pin' 
        // attribute of the class and the local variable
        this->prgm_sense_n = prgm_sense_n;
        this->reset_n = reset_n;
        this->ind_a2dp_n = ind_a2dp_n;
        this->hserial = hserial;
      }


      void enable(void) {
        // set BM62 reset status, active-low signal so HIGH enables device
        digitalWrite(reset_n, HIGH);
      }


      void init(void) {
        // initialize the BM62 reset line and ensure reset is asserted
        pinMode(reset_n, OUTPUT);
        reset();

        // wait 10 ms, then take the BM62 out of reset
        delay(10);
        enable();
        
        // initialize the BM62 programming sense line
        pinMode(prgm_sense_n, INPUT);

        // determine if the BM62 is being programmed; if so, take a nap
        isProgramMode();

        // input for determining if a successful A2DP connection active
        pinMode(ind_a2dp_n, INPUT_PULLUP);
      }


      bool isConnected(void) {
        // query state of BM62 IND_A2DP_N pin and return FALSE if no 
        // active A2DP connection is available (indicated by a HIGH state)
        return (bool)!digitalRead(ind_a2dp_n);
      }


      void reset(void) {
        // set BM62 reset status, active-low signal so LOW puts device in reset
        digitalWrite(reset_n, LOW);
      }


      void play(void) {
        // start playback from bluetooth-connected media device
        if (isConnected()) {
          writeMediaCommand(BM62_Play);
        }
      }


      void pause(void) {
        // pause playback from bluetooth-connected media device
        if (isConnected()) {
          writeMediaCommand(BM62_Pause);
        }
      }


      void stop(void) {
        // stop playback from bluetooth-connected media device
        if (isConnected()) {
          writeMediaCommand(BM62_Stop);
        }
      }


      void prev(void) {
        // go to previous track on bluetooth-connected media device
        if (isConnected()) {
          writeMediaCommand(BM62_Prev_Track);
        }
      }


      void next(void) {
        // go to next track on bluetooth-connected media device
        if (isConnected()) {
          writeMediaCommand(BM62_Next_Track);
        }
      }

    private:
      uint8_t prgm_sense_n;
      uint8_t reset_n;
      uint8_t ind_a2dp_n;
      HardwareSerial* hserial;

      // BM62 UART commands for media playback control
      uint8_t BM62_Media_Command[BYTE_COUNT_MEDIA_COMMAND] =
      {
          0xAA, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00
      };
      const uint8_t BM62_Play[BYTE_COUNT_MEDIA_INSTRUCTION] =
      {
          0x04, 0x00, 0x05
      };
      const uint8_t BM62_Pause[BYTE_COUNT_MEDIA_INSTRUCTION] =
      {
          0x04, 0x00, 0x06
      };
      const uint8_t BM62_Stop[BYTE_COUNT_MEDIA_INSTRUCTION] =
      {
          0x04, 0x00, 0x08
      };
      const uint8_t BM62_Prev_Track[BYTE_COUNT_MEDIA_INSTRUCTION] =
      {
          0x02, 0x00, 0x35
      };
      const uint8_t BM62_Next_Track[BYTE_COUNT_MEDIA_INSTRUCTION] =
      {
          0x02, 0x00, 0x34
      };


      // build the BM62 UART media command array from its component parts:
      uint8_t buildMediaCommand(uint8_t mediaCommand[], uint8_t instruction[]) {
        memcpy(mediaCommand, BM62_Media_Command, BYTE_COUNT_MEDIA_COMMAND);
        memcpy(mediaCommand + BYTE_COUNT_MEDIA_PREFIX, instruction, BYTE_COUNT_MEDIA_INSTRUCTION);
        mediaCommand[BYTE_COUNT_MEDIA_COMMAND - 1] = checksum(mediaCommand, BYTE_COUNT_MEDIA_COMMAND);
      }


      // for calculating the checksum of a BM62 UART command:
      uint8_t checksum(uint8_t command[], uint8_t command_length) {
        // BM62 documentation is lacking but pretty sure this is right
        uint16_t chksum = 0;
        for (uint8_t k = 2; k < command_length - 1; k++) {
          chksum = chksum + command[k];
        }

        // subtract sum from 0xFFFF and add one; use only the lower byte
        chksum = ((uint16_t)0xFFFF - chksum) + (uint16_t)0x0001;
        return(lowByte(chksum));
      }


      // check if the BM62 programming pin is pulled low
      void isProgramMode(void) {
        if (!digitalRead(prgm_sense_n)) {
          set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down
          cli();                                // globally disable interrupts
          sleep_enable();                       // set sleep bit
          sleep_bod_disable();                  // disable brown out detection
          sleep_cpu();                          // go to sleep
          
          // interrupts are disabled, mcu will NOT wake up until next power cycle!
        }
      }


      // build the BM62 UART media command array w/ checksum and write over serial UART
      void writeMediaCommand(const uint8_t *instruction) {
          uint8_t mediaCommand[BYTE_COUNT_MEDIA_COMMAND];
          buildMediaCommand(mediaCommand, (uint8_t *)instruction);
          hserial->write(mediaCommand, BYTE_COUNT_MEDIA_COMMAND);
      }
  };

#endif
