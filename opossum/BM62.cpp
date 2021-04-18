/*
* BM62.cpp - Microchip BM62 Audio Module Driver for Arduino
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

#include "BM62.h"

#ifndef BM62_MODULE_PARAMETERS
#define BM62_MODULE_PARAMETERS

  #define INIT_RESET_CYCLE_WAIT_TIME_MS (uint8_t)10
  #define TARGET_LENGTH_EQ_PRESET       (uint8_t)3

#endif

// BM62 UART syncword header for serial commands (probably 0xAA, may be 0x00AA)
// < EEPROM option (0xAE @ bit 4) adds “0x00” as wakeup byte in front of start byte >
static const uint8_t serial_uart_sync_header [1] = { 0xAA };

// BM62 UART commands for media playback control
// < Music_Control (0x04): AVRCP Commands for Music Control >
static const uint8_t BM62_Play        [3] = { 0x04, 0x00, 0x05 };
static const uint8_t BM62_Pause       [3] = { 0x04, 0x00, 0x06 };
static const uint8_t BM62_Play_Toggle [3] = { 0x04, 0x00, 0x07 };
static const uint8_t BM62_Stop        [3] = { 0x04, 0x00, 0x08 };
static const uint8_t BM62_Next_Track  [3] = { 0x04, 0x00, 0x09 };
static const uint8_t BM62_Prev_Track  [3] = { 0x04, 0x00, 0x0A };

// BM62 UART commands for audio equalization control
// < EQ_Mode_Setting 0x1C: Set EQ Mode of BTM for audio playback >
static const uint8_t BM62_EQ_Off       [3] = { 0x1C, 0x00, 0xFF };
static const uint8_t BM62_EQ_Soft      [3] = { 0x1C, 0x01, 0xFF };
static const uint8_t BM62_EQ_Bass      [3] = { 0x1C, 0x02, 0xFF };
static const uint8_t BM62_EQ_Treble    [3] = { 0x1C, 0x03, 0xFF };
static const uint8_t BM62_EQ_Classical [3] = { 0x1C, 0x04, 0xFF };
static const uint8_t BM62_EQ_Rock      [3] = { 0x1C, 0x05, 0xFF };
static const uint8_t BM62_EQ_Jazz      [3] = { 0x1C, 0x06, 0xFF };
static const uint8_t BM62_EQ_Pop       [3] = { 0x1C, 0x07, 0xFF };
static const uint8_t BM62_EQ_Dance     [3] = { 0x1C, 0x08, 0xFF };
static const uint8_t BM62_EQ_RnB       [3] = { 0x1C, 0x09, 0xFF };
static const uint8_t BM62_EQ_USER1     [3] = { 0x1C, 0x0A, 0xFF };

// BM62 UART commands for system status and control
// < MMI action 0x5D: fast enter pairing mode (from non-off mode) >
static const uint8_t BM62_EnterPairingMode [3] = { 0x02, 0x00, 0x5D };

BM62::BM62(uint8_t prgm_sense_n, uint8_t reset_n, uint8_t ind_a2dp_n, 
    HardwareSerial* hserial) {
  // GPIO for reading if the BM62 module was booted into flash reprogram mode
  this->prgm_sense_n = prgm_sense_n;
  // GPIO for resetting BM62 module
  this->reset_n = reset_n;
  // GPIO for reading A2DP profile connection status
  this->ind_a2dp_n = ind_a2dp_n;
  // HardwareSerial object for UART communication
  this->hserial = hserial;
}

// set BM62 reset status, active-low signal so HIGH enables device
void BM62::enable(void) {
  digitalWrite(reset_n, HIGH);
}

// put the BM62 back into pairing mode to permit pairing to new device
void BM62::enterPairingMode(void) {
  if (isConnected()) {
    writeSerialCommand(BM62_EnterPairingMode, (uint16_t)sizeof(BM62_EnterPairingMode));
  }
}

// set audio equalizer mode setting to specified preset mode
void BM62::setEqualizerPreset(EQ_Preset_t preset) {
  uint8_t BM62_EQ_Preset[TARGET_LENGTH_EQ_PRESET];
  switch (preset) {
    case EQ_Off: {
      memcpy(BM62_EQ_Preset, BM62_EQ_Off, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_Soft: {
      memcpy(BM62_EQ_Preset, BM62_EQ_Soft, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_Bass: {
      memcpy(BM62_EQ_Preset, BM62_EQ_Bass, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_Treble: {
      memcpy(BM62_EQ_Preset, BM62_EQ_Treble, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_Classical: {
      memcpy(BM62_EQ_Preset, BM62_EQ_Classical, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_Rock: {
      memcpy(BM62_EQ_Preset, BM62_EQ_Rock, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_Jazz: {
      memcpy(BM62_EQ_Preset, BM62_EQ_Jazz, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_Pop: {
      memcpy(BM62_EQ_Preset, BM62_EQ_Pop, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_Dance: {
      memcpy(BM62_EQ_Preset, BM62_EQ_Dance, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_RnB: {
      memcpy(BM62_EQ_Preset, BM62_EQ_RnB, TARGET_LENGTH_EQ_PRESET);
    } break;

    case EQ_Custom: {
      memcpy(BM62_EQ_Preset, BM62_EQ_USER1, TARGET_LENGTH_EQ_PRESET);
    } break;

    default: {
      return; // if we're here something went wrong so don't send a command
    } break;
  }
  if (isConnected()) {
    writeSerialCommand(BM62_EQ_Preset, (uint16_t)sizeof(BM62_EQ_Preset));
  }
}

// initialize the BM62 module and GPIO
void BM62::init(void) {
  // initialize the BM62 reset line and ensure reset is asserted
  pinMode(reset_n, OUTPUT);
  reset();

  // wait predefined number of ms, then take the BM62 out of reset
  delay(INIT_RESET_CYCLE_WAIT_TIME_MS);
  enable();
  
  // initialize the BM62 programming sense line
  pinMode(prgm_sense_n, INPUT);

  // determine if the BM62 is being programmed; if so, take a nap
  isProgramMode();

  // input for determining if a successful A2DP connection active
  pinMode(ind_a2dp_n, INPUT_PULLUP);
}

// query state of BM62 IND_A2DP_N pin and return FALSE if no 
// active A2DP connection is available (indicated by a HIGH state)
bool BM62::isConnected(void) {
  return (bool)!digitalRead(ind_a2dp_n);
}

// set BM62 reset status, active-low signal so LOW puts device in reset
void BM62::reset(void) {
  digitalWrite(reset_n, LOW);
}

// start playback from bluetooth-connected media device
void BM62::play(void) {
  if (isConnected()) {
    writeSerialCommand(BM62_Play, (uint16_t)sizeof(BM62_Play));
  }
}

// pause playback from bluetooth-connected media device
void BM62::pause(void) {
  if (isConnected()) {
    writeSerialCommand(BM62_Pause, (uint16_t)sizeof(BM62_Pause));
  }
}

// media playback play/pause toggle (pauses if playing, plays if paused)
void BM62::playPauseToggle(void) {
  if (isConnected()) {
    writeSerialCommand(BM62_Play_Toggle, (uint16_t)sizeof(BM62_Play_Toggle));
  }
}

// stop playback from bluetooth-connected media device
void BM62::stop(void) {
  if (isConnected()) {
    writeSerialCommand(BM62_Stop, (uint16_t)sizeof(BM62_Stop));
  }
}

// go to previous track on bluetooth-connected media device
void BM62::previous(void) {
  if (isConnected()) {
    writeSerialCommand(BM62_Prev_Track, (uint16_t)sizeof(BM62_Prev_Track));
  }
}

// go to next track on bluetooth-connected media device
void BM62::next(void) {
  if (isConnected()) {
    writeSerialCommand(BM62_Next_Track, (uint16_t)sizeof(BM62_Next_Track));
  }
}

// for calculating the checksum of a BM62 UART command:
uint8_t BM62::checksum(uint8_t command[], uint8_t command_length) {
  // BM62 documentation is lacking but pretty sure this is right
  uint16_t chksum = 0;
  for (uint8_t k = 2; k < command_length - 1; k++) {
    chksum = chksum + command[k];
  }

  // subtract sum from 0xFFFF and add one; use only the lower byte
  chksum = ((uint16_t)0xFFFF - chksum) + (uint16_t)0x0001;
  return(lowByte(chksum));
}

// check if the BM62 was booted into flash reprogram mode
void BM62::isProgramMode(void) {
  if (!digitalRead(prgm_sense_n)) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down
    cli();                                // globally disable interrupts
    sleep_enable();                       // set sleep bit
    sleep_bod_disable();                  // disable brown out detection
    sleep_cpu();                          // go to sleep
    
    // interrupts are disabled, mcu will NOT wake up until next power cycle!
  }
}

// build the BM62 UART command array w/ checksum and write over serial UART
void BM62::writeSerialCommand(const uint8_t *instruction, uint16_t bytes_command) {
  // allocation size in bytes of the serial syncword
  uint8_t bytes_syncword = sizeof(serial_uart_sync_header);

  // allocation size in bytes of the length value (2 bytes) plus the CRC value (1 byte)
  uint8_t bytes_length_and_crc = 3;

  // create the command buffer, the size will always be equal to
  // [START(1-2 bytes) + LENGTH(2 bytes) + INSTRUCTION + CRC(1 byte)]
  uint8_t command[bytes_syncword + bytes_length_and_crc + bytes_command];

  // copy the serial UART syncword (either 0xAA or 0x00AA) to the command buffer
  memcpy(command, serial_uart_sync_header, bytes_syncword);
  uint8_t indx = bytes_syncword;

  // copy 'target length' (total length minus syncword, length, and CRC) to command buffer
  uint8_t  target_length [2] = { highByte(bytes_command), lowByte(bytes_command) };
  memcpy(command + indx, target_length, sizeof(target_length));
  indx += sizeof(target_length);

  // copy the actual instruction to the command buffer
  memcpy(command + indx, instruction, bytes_command);
  indx += bytes_command;

  // calculate the checksum and write this value to the last element of the command buffer
  command[indx] = checksum(command, indx + 1);

  // write the command buffer to the serial output
  hserial->write(command, sizeof(command));

#ifdef BM62_DEBUG
  hserial->println("--");
  for (uint8_t k = 0; k < sizeof(command); k++) {
    hserial->println(command[k], HEX);
  }
  hserial->println("--");
#endif
}
