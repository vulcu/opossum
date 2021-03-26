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

#ifndef __LEDBUTTON_H__
#define __LEDBUTTON_H__

  class BUTTON {
    private:    
      class LED {
        public:
          LED(void) :
            // class constructor
            pinAttachment(-1)
          {
          }

          void init(int8_t pinAttachment) {
            // configure the port and set initial LED state to OFF
            this->pinAttachment = pinAttachment;
            pinMode(pinAttachment, OUTPUT);
            off();
          }

          void reset(void) {
            // turn the LED off and clear the pin attachment
            off();
            this->pinAttachment = -1;
          }

          void off(void) {
            // set the LED to OFF
            if (pinAttachment >= 0 ) {
              // don't write anything if no valid LED pin is defined
              digitalWrite(pinAttachment, LOW);
            }
          }

          void write(uint8_t value) {
            // write LED PWM value (0-255)
            if (pinAttachment >= 0 ) {
              // don't write anything if no valid LED pin is defined
              analogWrite(pinAttachment, value);
            }
          }

        private:
          int8_t pinAttachment;
      };
      
      const uint8_t pin;
      uint8_t input_mode = INPUT;
      LED led;

    public:
      BUTTON(uint8_t pinAttachment) :
        // class constructor
        pin(pinAttachment),
        led()
      {
      }

      void attach(uint8_t led_pin) {
        // attach an LED pin and configure the LED for use
        led.init((int8_t)led_pin);
      }

      void brightness(uint8_t value) {
        // write a PWM brightness to the LED
        led.write(value);
      }

      void unattach(void) {
        // attach and LED pin and configure the LED for use
        led.reset();
      }

      void init(void) {
        // configure the port and input pullup mode
        pinMode(pin, input_mode);
      }

      bool read(void) {
        // configure the port and input pullup mode
        return digitalRead(pin);
      }

      void disableInputPullup(void) {
        // disable the input pullup
        input_mode = INPUT;
        init();
      }

      void enableInputPullup(void) {
        // enable the input pullup
        input_mode = INPUT_PULLUP;
        init();
      }

  };

#endif
