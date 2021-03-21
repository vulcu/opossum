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

#include <avr/sleep.h>
#include "parameters.h"

class BM62 {
  private:
    #ifndef BYTE_COUNT_MEDIA_COMMAND
      #define BYTE_COUNT_MEDIA_COMMAND (uint8_t)7
    #endif
    #ifndef BYTE_COUNT_MEDIA_PREFIX
      #define BYTE_COUNT_MEDIA_PREFIX (uint8_t)3
    #endif
    #ifndef BYTE_COUNT_MEDIA_INSTRUCTION
      #define BYTE_COUNT_MEDIA_INSTRUCTION (uint8_t)3
    #endif
    #ifndef SERIAL_BAUD_RATE
      #define SERIAL_BAUD_RATE (uint16_t)57600
    #endif


    bool initSerialPort;


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
      if (!digitalRead(PRGM_SENSE_N)) {
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
        Serial.write(mediaCommand, BYTE_COUNT_MEDIA_COMMAND);
    }


  public:
    BM62(bool initSerialPort) {
      // Use 'this->' to make the difference between the 'pin' 
      // attribute of the class and the local variable
      this->initSerialPort = initSerialPort;
    }


    void enable(void) {
      // set BM62 reset status, active-low signal so HIGH enables device
      digitalWrite(RST_N, LOW);
    }


    void init(void) {
      // initialize the BM62 reset line and ensure reset is asserted
      pinMode(RST_N, OUTPUT);
      reset();

      // wait 10 ms, then take the BM62 out of reset
      delay(10);
      enable();
      
      // initialize the BM62 programming sense line
      pinMode(PRGM_SENSE_N, INPUT);

      // determine if the BM62 is being programmed; if so, take a nap
      isProgramMode();

      // input for determining if a successful A2DP connection active
      pinMode(IND_A2DP_N, INPUT_PULLUP);

      if (initSerialPort) {
        // initialize the UART port to talk to the BM62
        Serial.begin(SERIAL_BAUD_RATE, SERIAL_8N1);
      }
    }


    bool isConnected(void) {
      // query state of BM62 IND_A2DP_N pin and return FALSE if no active A2DP connection
      return (bool)!digitalRead(IND_A2DP_N);
    }


    void reset(void) {
      // set BM62 reset status, active-low signal so LOW puts device in reset
      digitalWrite(RST_N, HIGH);
    }


    void play(void) {
      // start playback from bluetooth-connected media device
      writeMediaCommand(BM62_Play);
    }
    void pause(void) {
      // pause playback from bluetooth-connected media device
      writeMediaCommand(BM62_Pause);
    }
    void stop(void) {
      // stop playback from bluetooth-connected media device
      writeMediaCommand(BM62_Stop);
    }
    void prev(void) {
      // go to previous track on bluetooth-connected media device
      writeMediaCommand(BM62_Prev_Track);
    }
    void next(void) {
      // go to next track on bluetooth-connected media device
      writeMediaCommand(BM62_Next_Track);
    }
};
