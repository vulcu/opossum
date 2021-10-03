/*
 * ledbutton.h - UI Push-button Driver w/ LED Support for Arduino
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

#ifndef LED_BUTTON_H
#define LED_BUTTON_H

  class LED {
    public:
      LED(int8_t led_pinAttachment);

      void attach(const int8_t led_pinAttachment);
      void brightness(const uint8_t value);
      void init(void);
      void off(void);
      void on(void);
      void reset(void);

      private:
      int8_t led_pin;
  };


  class Button {
    public:
      Button(uint8_t button_pinAttachment);

      void disableInputPullup(void);
      void enableInputPullup(void);
      void init(void);
      bool read(void);

      private:    
      const int8_t button_pin;
      uint8_t button_port_input_mode;
  };


  class LED_Button: public Button {
    public:
      LED_Button(int8_t button_pinAttachment, LED &led_Attachment);
    
      void attach(const int8_t led_pin);
      void brightness(const uint8_t value);
      void disableInputPullup(void);
      void enableInputPullup(void);
      void init(void);
      void off(void);
      void unattach(void);

    private:
      LED led;
  };

#endif
