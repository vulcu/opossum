/*
 * pushbutton.h - UI Push-button Driver w/ LED Support for Arduino
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

#ifndef __PUSHBUTTON_H__
#define __PUSHBUTTON_H__

  class PUSHBUTTON {
    public:
      PUSHBUTTON(uint8_t pinNumber) {
        // Use 'this->' to make the difference between the 'pin' 
        // attribute of the class and the local variable
        this->pinNumber = pinNumber;
      }


      void init(void) {
        // initialize the pushbutton
        
      }

    private:
        uint8_t pinNumber;
  };

#endif
