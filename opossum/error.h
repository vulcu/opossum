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

#ifndef ERROR_H
#define ERROR_H

  #include <avr/pgmspace.h>
  #include "parameters.h"

  // put common error messages into Program memory to save SRAM space
  const char strErrNoInput[] PROGMEM = "Error: No Input\n";
  const char strErrUnrecognizedInput[] PROGMEM = "Error: Unrecognized Input Character\n";
  const char strErrUnrecognizedProtocol[] PROGMEM = "Error: Unrecognized Protocol\n";
  const char strErrUnrecognizedSysRequest[] PROGMEM = "Error: Unrecognized SYSTEM request\n";
  const char strErrUnrecognizedSetRequest[] PROGMEM = "Error: Unrecognized SET request\n";
  const char strErrUnrecognizedGetRequest[] PROGMEM = "Error: Unrecognized GET request\n";
  const char strErrUnrecognizedOrgRequest[] PROGMEM = "Error: Unrecognized ORIGIN request\n";
  const char strErrNumericVal[] PROGMEM = "Input Error: Input value must be numeric\n";
  const char strErrInvalidLocMove[] PROGMEM = "Request Error: Invalid movement request\n";
  const char strErrLimitRegisterLock[] PROGMEM = "Write Failed: Limit Register Locked\n";

  // create error message buffer and a table to refer to common error messages
  static char ErrorMsgBuffer[ERROR_MSG_BUFFER_SIZE];
  static enum errors{NoInput, 
                     UnrecognizedInput, 
                     UnrecognizedProtocol, 
                     UnrecognizedSysRequest, 
                     UnrecognizedSetRequest, 
                     UnrecognizedGetRequest, 
                     UnrecognizedOrgRequest, 
                     NumericVal, 
                     InvalidLocMove, 
                     LimitRegisterLock} errorType;
  static const char *const strErrTable[] PROGMEM = {strErrNoInput, 
                                                    strErrUnrecognizedInput, 
                                                    strErrUnrecognizedProtocol, 
                                                    strErrUnrecognizedSysRequest, 
                                                    strErrUnrecognizedSetRequest, 
                                                    strErrUnrecognizedGetRequest, 
                                                    strErrUnrecognizedOrgRequest, 
                                                    strErrNumericVal, 
                                                    strErrInvalidLocMove, 
                                                    strErrLimitRegisterLock};

  static void writeErrorMsgToSerialBuffer(char errorType, char *serialInput) {
    memset(serialInput, '\0', SERIAL_INPUT_BUFFER_SIZE);
    strcpy_P(ErrorMsgBuffer, (char *)pgm_read_word(&(strErrTable[errorType])));
    strcpy(serialInput, ErrorMsgBuffer);
  }

  void error_INPUT_NoInput(char *serialInput) {
    // Input buffer was empty
    errorType = NoInput;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_INPUT_UnrecognizedInput(char *serialInput) {
    // Input character was unrecognized and couldn't be parsed
    errorType = UnrecognizedInput;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_INPUT_UnrecognizedProtocol(char *serialInput) {
    // Input protocol was unrecognized and couldn't be parsed
    errorType = UnrecognizedProtocol;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_INPUT_UnrecognizedSysRequest(char *serialInput) {
    // System request was unrecognized and couldn't be parsed
    errorType = UnrecognizedSysRequest;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_INPUT_UnrecognizedSetRequest(char *serialInput) {
    // Set request was unrecognized and couldn't be parsed
    errorType = UnrecognizedSetRequest;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_INPUT_UnrecognizedGetRequest(char *serialInput) {
    // Get request was unrecognized and couldn't be parsed
    errorType = UnrecognizedGetRequest;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_INPUT_UnrecognizedOrgRequest(char *serialInput) {
    // Origin request was unrecognized and couldn't be parsed
    errorType = UnrecognizedOrgRequest;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_INPUT_NumericVal(char *serialInput) {
    // Input value was non-numeric and couldn't be parsed
    errorType = NumericVal;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_YAW_MaxAngleStepSize(char *serialInput) {
    // Yaw movement request exceeded the maximum allowed for a single request
    Serial.println(F("Request exceeds maximum YAW angle step size"));
    Serial.print(F("Maximum angle step size is "));
    Serial.print(MAX_ANGLE_STEP_SIZE);
    Serial.println(F(" degrees"));

    errorType = InvalidLocMove;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_YAW_LimitExceeded(char *serialInput, int16_t *yawLimits) {
    // Yaw movement request exceeded the global yaw axis limits
    Serial.println(F("Request exceeds YAW axis limit"));
    Serial.print(F("Maximum YAW limit relative to ORIGIN is : "));
    Serial.print((int16_t)((yawLimits[1] * MIN_ANGLE_STEP_SIZE) / PULSES_PER_MIN_ANGLE_STEP));
    Serial.println(F(" degrees"));
    Serial.print(F("Minimum YAW limit relative to ORIGIN is : "));
    Serial.print((int16_t)((yawLimits[0] * MIN_ANGLE_STEP_SIZE) / PULSES_PER_MIN_ANGLE_STEP));
    Serial.println(F(" degrees"));

    errorType = InvalidLocMove;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void error_SYSTEM_LimitRegisterLock(char *serialInput) {
    // Global limit register was locked and couldn't be written to
    errorType = LimitRegisterLock;
    writeErrorMsgToSerialBuffer(errorType, serialInput);
  }

  void warning_INPUT_FractionalValIgnored(int16_t numericValue) {
    // Input data value was not an integer
    Serial.println(F("Warning: Only integer data values are accepted"));
    Serial.print(F("Requested value rounded towards zero, new value is "));
    Serial.println(numericValue);
  }

  void warning_YAW_MinimumAngleMultiple(int16_t updateAngle) {
    // Yaw movement request was not a multiple of the minimum angle step size
    Serial.print(F("Warning: YAW value request must be a multiple of "));
    Serial.print(MIN_ANGLE_STEP_SIZE);
    Serial.println(F(" degrees"));
    Serial.print(F("Request has been rounded to next-nearset location at "));
    Serial.print(updateAngle);
    Serial.println(F(" degrees"));
  }

#endif
