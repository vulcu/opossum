/*
 * ledbutton.cpp - UI Push-button Driver w/ LED Support for Arduino
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

#include "ledbutton.h"

/**************************************************************************/
// class constructor for LED class
LED::LED(int8_t led_pinAttachment) :
  led_pin(led_pinAttachment) {
}

// attach the LED to a hardware pin
void LED::attach(int8_t led_pinAttachment) {
  this->led_pin = led_pinAttachment;
  init();
}

// write LED PWM value (0-255)
void LED::brightness(uint8_t value) {
  if (led_pin >= 0 ) {
      // don't write anything if no valid LED pin is defined
      analogWrite(led_pin, value);
  }
}

// configure the port and set initial LED state to OFF
void LED::init(void) {
  pinMode(led_pin, OUTPUT);
  off();
}

// set the LED to OFF
void LED::off(void) {
  if (led_pin >= 0 ) {
    // don't write anything if no valid LED pin is defined
    digitalWrite(led_pin, LOW);
  }
}

// set the LED to ON
void LED::on(void) {
  if (led_pin >= 0 ) {
    // don't write anything if no valid LED pin is defined
    digitalWrite(led_pin, HIGH);
  }
}

// turn the LED off and clear the pin attachment
void LED::reset(void) {
  off();
  this->led_pin = -1;
}
/**************************************************************************/

/**************************************************************************/
// class constructor for Button class
Button::Button(uint8_t button_pinAttachment) :
  button_pin(button_pinAttachment),
  button_port_input_mode(INPUT) {
}

// disable the input pullup for the attached pin
void Button::disableInputPullup(void) {
  button_port_input_mode = INPUT;
  init();
}

// enable the input pullup for the attached pin
void Button::enableInputPullup(void) {
  button_port_input_mode = INPUT_PULLUP;
  init();
}

void Button::init(void) {
  // configure the port and input pullup mode
  pinMode(button_pin, button_port_input_mode);
}

// configure the port and input pullup mode
bool Button::read(void) {
  return digitalRead(button_pin);
}
/**************************************************************************/

/**************************************************************************/
// class constructor for LED_Button class
LED_Button::LED_Button(int8_t button_pinAttachment, LED &led_Attachment) :
  Button(button_pinAttachment),
  led(led_Attachment){
}

// attach an LED pin and configure the LED for use
void LED_Button::attach(int8_t led_pin) {
  led.attach(led_pin);
}

// write a PWM brightness to the Button LED
void LED_Button::brightness(uint8_t value) {
  led.brightness(value);
}

// disable the input pullup for the Button input
void LED_Button::disableInputPullup(void) {
  Button::disableInputPullup();
  led.init();
}

// enable the input pullup for the Button input
void LED_Button::enableInputPullup(void) {
  Button::enableInputPullup();
  led.init();
}

// initialize the hardware button and LED
void LED_Button::init(void) {
  Button::init();
  led.init();
}

// set the Button LED to OFF
void LED_Button::off(void) {
  led.off();
}

// reset the button's LED pin attachment (disables control of LED)
void LED_Button::unattach(void) {
  led.reset();
}
/**************************************************************************/
