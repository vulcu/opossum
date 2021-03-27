/*
 * userinterface.h - UI Push-button Driver w/ LED Support for Arduino
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

#ifndef __LED_H__
#define __LED_H__

  class LED {
    public:
      LED(int8_t led_pinAttachment) :
        led_pin(led_pinAttachment) {
        // class constructor
      }

      void attach(int8_t led_pinAttachment) {
        // attach the LED to a hardware pin
        this->led_pin = led_pinAttachment;
        init();
      }

      void brightness(uint8_t value) {
        // write LED PWM value (0-255)
        if (led_pin >= 0 ) {
          // don't write anything if no valid LED pin is defined
          analogWrite(led_pin, value);
        }
      }

      void init(void) {
        // configure the port and set initial LED state to OFF
        pinMode(led_pin, OUTPUT);
        off();
      }

      void off(void) {
        // set the LED to OFF
        if (led_pin >= 0 ) {
          // don't write anything if no valid LED pin is defined
          digitalWrite(led_pin, LOW);
        }
      }

      void reset(void) {
        // turn the LED off and clear the pin attachment
        off();
        this->led_pin = -1;
      }

    private:
      int8_t led_pin;
  };

#endif

#ifndef __BUTTON_H__
#define __BUTTON_H__

  class BUTTON {
    private:    
      const int8_t button_pin;
      uint8_t button_port_input_mode = INPUT;

    public:
      BUTTON(uint8_t button_pinAttachment) :
        button_pin(button_pinAttachment) {
        // class constructor
      }

      void disableInputPullup(void) {
        // disable the input pullup
        button_port_input_mode = INPUT;
        init();
      }

      void enableInputPullup(void) {
        // enable the input pullup
        button_port_input_mode = INPUT_PULLUP;
        init();
      }

      void init(void) {
        // configure the port and input pullup mode
        pinMode(button_pin, button_port_input_mode);
      }

      bool read(void) {
        // configure the port and input pullup mode
        return digitalRead(button_pin);
      }
  };

#endif

#ifndef __LEDBUTTON_H__
#define __LEDBUTTON_H__

  class LEDBUTTON: public BUTTON {
    LED &led;

    public:
      LEDBUTTON(int8_t button_pinAttachment, LED &led_Attachment) :
        BUTTON(button_pinAttachment),
        led(led_Attachment){
        // class constructor
      }
    
      void attach(int8_t led_pin) {
        // attach an LED pin and configure the LED for use
        led.attach(led_pin);
      }

      void brightness(uint8_t value) {
        // write a PWM brightness to the LED
        led.brightness(value);
      }

      void off(void) {
        // set the LED to OFF
        led.off();
      }
      
      void unattach(void) {
        // reset the LED pin attachment (disables control of LED)
        led.reset();
      }
  };

#endif
