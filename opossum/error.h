/*
 * error.h - Error handling for Opossum BT Aplifier
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

#ifndef __OPOSSUM_ERROR_H__
#define __OPOSSUM_ERROR_H__

  #include <avr/pgmspace.h>
  #include "parameters.h"

  // put common error messages into Program memory to save SRAM space
  const char strErrSuccess[] PROGMEM = "Error: Success\n";

  // create error message buffer and a table to refer to common error messages
  static char ErrorMsgBuffer[ERROR_MSG_BUFFER_SIZE];
  static enum errors{Success} errorType;
  static const char *const strErrTable[] PROGMEM = {strErrSuccess};

  static void writeErrorMsgToSerialBuffer(char errorType, char *serialBuffer) {
    memset(serialBuffer, '\0', SERIAL_BUFFER_SIZE);
    strcpy_P(ErrorMsgBuffer, (char *)pgm_read_word(&(strErrTable[errorType])));
    strcpy(serialBuffer, ErrorMsgBuffer);
  }

  void error_SUCCESS_success(char *serialBuffer) {
    // Input buffer was empty
    Serial.println(F("An Error Has Occurred"));
    errorType = Success;
    writeErrorMsgToSerialBuffer(errorType, serialBuffer);
  }

#endif
