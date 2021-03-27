/*
 * parameters.h - System paraeters for Opossum BT Aplifier
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

#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

  // define the MAX9744 I2C address, default is 0x4B
  #define MAX9744_I2CADDR ((uint8_t)0x4B)

  // define the digitalOut pins according to the schematic net name
  #define S1_INT       (int8_t) -1
  #define S2_INT       (uint8_t)3
  #define IND_A2DP_N   (uint8_t)4
  #define S2_LEDPWM    (int8_t) 5
  #define S1_LEDPWM    (int8_t) 6
  #define RST_N        (uint8_t)8
  #define STROBE       (uint8_t)9
  #define RESET        (uint8_t)10
  #define MUTE         (uint8_t)12
  #define SHDN         (uint8_t)13
  #define PRGM_SENSE_N (uint8_t)17

  // define the analogRead pins according to the schematic net name
  #define VOLUME (uint8_t)0
  #define DC_OUT (uint8_t)A1

  // time in ms between audio input level reads
  #define AUDIO_READ_INTERVAL (uint16_t)100

  // min, max, and default 'on' LED brightness values for each switch LED
  #define S1_PWM_MIN (uint8_t)20          // S1 minimum PWM level
  #define S1_PWM_MAX (uint8_t)255         // S1 maximum PWM level
  #define S1_PWM_DEF (uint8_t)200         // S1 default PWM level
  #define S2_PWM_MIN (uint8_t)20          // S2 minimum PWM level
  #define S2_PWM_MAX (uint8_t)255         // S2 maximum PWM level
  #define S2_PWM_DEF (uint8_t)200         // S2 default PWM level

  // define two wire interface (I2C) clock rate
  #define TWI_CLOCK_RATE (int32_t)400000

  // serial UART baud rate, must be 57600 for communication with BM62
  #define SERIAL_BAUD_RATE (uint16_t)57600

  // serial I/O buffer sizes (error reporting only)
  #define SERIAL_BUFFER_SIZE (uint8_t)32
  #define ERROR_MSG_BUFFER_SIZE (uint8_t)32

#endif
