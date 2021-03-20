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
    #ifndef BYTE_COUNT_MEDIACONTROL
      #define BYTE_COUNT_MEDIACONTROL (uint8_t)7
    #endif
    #ifndef SERIAL_BAUD_RATE
      #define SERIAL_BAUD_RATE (uint16_t)57600
    #endif

    bool initSerialPort;

    // BM62 UART commands for media playback control
    const uint8_t BM62_Play[BYTE_COUNT_MEDIACONTROL] =
    {
        0xAA, 0x00, 0x03, 0x04, 0x00, 0x05, 0xF4
    };
    const uint8_t BM62_Pause[BYTE_COUNT_MEDIACONTROL] =
    {
        0xAA, 0x00, 0x03, 0x04, 0x00, 0x06, 0xF3
    };
    const uint8_t BM62_Stop[BYTE_COUNT_MEDIACONTROL] =
    {
        0xAA, 0x00, 0x03, 0x04, 0x00, 0x08, 0xF1
    };
    const uint8_t BM62_PrevTrack[BYTE_COUNT_MEDIACONTROL] =
    {
        0xAA, 0x00, 0x03, 0x02, 0x00, 0x35, 0xC6
    };
    const uint8_t BM62_NextTrack[BYTE_COUNT_MEDIACONTROL] =
    {
        0xAA, 0x00, 0x03, 0x02, 0x00, 0x34, 0xC7
    };

    // check if the BM62 programming pin is pulled low
    void isProgramMode(void) {
      if (!read(PRGM_SENSE_N)) {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down
        cli();                                // globally disable interrupts
        sleep_enable();                       // set sleep bit
        sleep_bod_disable();                  // disable brown out detection
        sleep_cpu();                          // go to sleep
        
        // interrupts are disabled, mcu will NOT wake up until next power cycle!
      }
    }

    // for calculating the checksum of a BM62 UART command:
    byte checksum(uint8_t a[], uint8_t numel) {
      // BM62 documentation is lacking but pretty sure
      uint16_t sum = 0;
      for (uint8_t k = 2; k < numel - 1; k++) {
        sum = sum + a[k];
      }

      // subtract sum from 0xFFFF and add one; use only the lower byte
      sum = ((uint16_t)0xFFFF - sum) + (uint16_t)0x0001;
      return(lowByte(sum));
    }

  public:
    BM62(bool initSerialPort) {
      // Use 'this->' to make the difference between the 'pin' 
      // attribute of the class and the local variable
      this->initSerialPort = initSerialPort;
    }
    void init(void) {
      // initialize the BM62 reset line and ensure reset is asserted
      pinMode(RST_N, OUTPUT);
      low(RST_N);

      // wait 10 ms, then take the BM62 out of reset
      delay(10);
      high(RST_N);
      
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
    void low(uint8_t pin) {
      digitalWrite(pin, LOW);
    }
    void high(uint8_t pin) {
      digitalWrite(pin, HIGH);
    }
    bool read(uint8_t pin) {
      return (bool)digitalRead(pin);
    }
    void play(void) {
      // start playback from bluetooth-connected media device
      Serial.write(BM62_Play, BYTE_COUNT_MEDIACONTROL);
    }
    void pause(void) {
      // pause playback from bluetooth-connected media device
      Serial.write(BM62_Pause, BYTE_COUNT_MEDIACONTROL);
    }
    void stop(void) {
      // stop playback from bluetooth-connected media device
      Serial.write(BM62_Stop, BYTE_COUNT_MEDIACONTROL);
    }
    void prev(void) {
      // go to previous track on bluetooth-connected media device
      Serial.write(BM62_PrevTrack, BYTE_COUNT_MEDIACONTROL);
    }
    void next(void) {
      // go to next track on bluetooth-connected media device
      Serial.write(BM62_NextTrack, BYTE_COUNT_MEDIACONTROL);
    }
};
