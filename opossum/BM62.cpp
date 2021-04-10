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

  #define BYTE_COUNT_UART_HEADER        (uint8_t)2

  #define BYTE_COUNT_A2DP_COMMAND       (uint8_t)7
  #define BYTE_COUNT_A2DP_PREFIX        (uint8_t)1
  #define BYTE_COUNT_A2DP_INSTRUCTION   (uint8_t)3

  #define BYTE_COUNT_SYSTEM_COMMAND     (uint8_t)7
  #define BYTE_COUNT_SYSTEM_PREFIX      (uint8_t)1
  #define BYTE_COUNT_SYSTEM_INSTRUCTION (uint8_t)3

  #define BYTE_COUNT_GET_DEVICE_INFO_COMMAND     (uint8_t)6
  #define BYTE_COUNT_GET_DEVICE_INFO_PREFIX      (uint8_t)1
  #define BYTE_COUNT_GET_DEVICE_INFO_INSTRUCTION (uint8_t)2

  #define BYTE_COUNT_GET_ELEMENT_ATTR_COMMAND     (uint8_t)23
  #define BYTE_COUNT_GET_ELEMENT_ATTR_PREFIX      (uint8_t)1
  #define BYTE_COUNT_GET_ELEMENT_ATTR_INSTRUCTION (uint8_t)19

#endif

// BM62 UART communication header bytes
static const uint8_t BM62_UART_Header [BYTE_COUNT_UART_HEADER] = 
{
    0xAA, 0x00
};

// BM62 UART commands for media playback control
static const uint8_t BM62_A2DP_Command_Prefix [BYTE_COUNT_A2DP_PREFIX] =
{
    0x03
};
static const uint8_t BM62_Play [BYTE_COUNT_A2DP_INSTRUCTION] =
{
    0x04, 0x00, 0x05
};
static const uint8_t BM62_Pause [BYTE_COUNT_A2DP_INSTRUCTION] =
{
    0x04, 0x00, 0x06
};
static const uint8_t BM62_Stop [BYTE_COUNT_A2DP_INSTRUCTION] =
{
    0x04, 0x00, 0x08
};
static const uint8_t BM62_Prev_Track [BYTE_COUNT_A2DP_INSTRUCTION] =
{
    0x02, 0x00, 0x35
};
static const uint8_t BM62_Next_Track [BYTE_COUNT_A2DP_INSTRUCTION] =
{
    0x02, 0x00, 0x34
};

// BM62 UART commands for audio equalization control
static const uint8_t BM62_EQ_OFF [BYTE_COUNT_A2DP_INSTRUCTION] =
{
    0x1C, 0x00, 0xFF
};
static const uint8_t BM62_EQ_CLASSICAL [BYTE_COUNT_A2DP_INSTRUCTION] =
{
    0x1C, 0x04, 0xFF
};
static const uint8_t BM62_EQ_JAZZ [BYTE_COUNT_A2DP_INSTRUCTION] =
{
    0x1C, 0x06, 0xFF
};
static const uint8_t BM62_EQ_DANCE [BYTE_COUNT_A2DP_INSTRUCTION] =
{
    0x1C, 0x08, 0xFF
};

// BM62 UART commands for system status control
static const uint8_t BM62_SYS_Command_Prefix [BYTE_COUNT_SYSTEM_PREFIX] =
{
    0x03
};
static const uint8_t BM62_EnterPairingMode [BYTE_COUNT_SYSTEM_INSTRUCTION] =
{
    0x02, 0x00, 0x5D
};

// BM62 UART commands for requesting device information
static const uint8_t BM62_GETDEVICE_Prefix [BYTE_COUNT_GET_DEVICE_INFO_PREFIX] =
{
    0x02
};
static const uint8_t BM62_GetDeviceInformation [BYTE_COUNT_GET_DEVICE_INFO_INSTRUCTION] =
{
    0x0E, 0x00
};

// BM62 UART commands for requesting element attributes
static const uint8_t BM62_GETATTRIBUTE_Prefix [BYTE_COUNT_GET_ELEMENT_ATTR_PREFIX] =
{
    0x13
};
static const uint8_t BM62_GetElementAttributes [BYTE_COUNT_GET_ELEMENT_ATTR_INSTRUCTION] =
{
    0x0B, 0x00, 0x20, 0x00, 0x00, 
    0x0D, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00
};

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
    writeMediaCommand(BM62_Play);
  }
}

// pause playback from bluetooth-connected media device
void BM62::pause(void) {
  if (isConnected()) {
    writeMediaCommand(BM62_Pause);
  }
}

// stop playback from bluetooth-connected media device
void BM62::stop(void) {
  if (isConnected()) {
    writeMediaCommand(BM62_Stop);
  }
}

// go to previous track on bluetooth-connected media device
void BM62::prev(void) {
  if (isConnected()) {
    writeMediaCommand(BM62_Prev_Track);
  }
}

// go to next track on bluetooth-connected media device
void BM62::next(void) {
  if (isConnected()) {
    writeMediaCommand(BM62_Next_Track);
  }
}

// build the BM62 UART media command array from its component parts:
uint8_t BM62::buildMediaCommand(uint8_t mediaCommand[], uint8_t instruction[]) {
  memcpy(mediaCommand, BM62_UART_Header, BYTE_COUNT_UART_HEADER);
  memcpy(mediaCommand + BYTE_COUNT_UART_HEADER, BM62_A2DP_Command_Prefix, BYTE_COUNT_A2DP_PREFIX);
  memcpy(mediaCommand + BYTE_COUNT_UART_HEADER +BYTE_COUNT_A2DP_PREFIX, instruction, BYTE_COUNT_A2DP_INSTRUCTION);
  mediaCommand[BYTE_COUNT_A2DP_COMMAND - 1] = checksum(mediaCommand, BYTE_COUNT_A2DP_COMMAND);
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

// build the BM62 UART media command array w/ checksum and write over serial UART
void BM62::writeMediaCommand(const uint8_t *instruction) {
  uint8_t mediaCommand[BYTE_COUNT_A2DP_COMMAND];
  buildMediaCommand(mediaCommand, (uint8_t *)instruction);
  hserial->write(mediaCommand, BYTE_COUNT_A2DP_COMMAND);
  //hserial->println("--");
  //for (uint8_t k = 0; k < sizeof(mediaCommand); k++) {
  //  hserial->println(mediaCommand[k], HEX);
  //}
  //hserial->println("--");
}

/*
// build the BM62 UART media command array w/ checksum and write over serial UART
void writeSerialCommand(const uint8_t *header, 
                        const uint8_t *prefix, 
                        const uint8_t *instruction) {
  uint8_t length = sizeof(header) + sizeof(prefix) + sizeof(instruction) + 1;
  uint8_t command[length];
  buildSerialCommand(command, (uint8_t *)header, (uint8_t *)prefix, (uint8_t *)instruction);
  hserial->write(command, length);
}
*/
