/*
 * Opossum.ino - Bluetooth Audio Amplifier w/ Automatic Gain Control
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

// comment to deactivate UART debug mode
#define DEBUG
//#define BM62_DEBUG

// include libraries for PROGMEM, SLEEP, & I2C
#include <avr/pgmspace.h>

#include "opossum/opossum.h"

#include "opossum/ledbutton.h"
#include "opossum/ledbutton.cpp"

#include "opossum/BM62.h"
#include "opossum/BM62.cpp"

#include "opossum/audiomath.h"
#include "opossum/audiomath.cpp"

#include "opossum/MAX9744.h"
#include "opossum/MAX9744.cpp"

#include "opossum/MSGEQ7.h"
#include "opossum/MSGEQ7.cpp"

// buffer index, volume, filtered volume, base level, present level
uint8_t  volumeRange[2];
int16_t  vol      = 0;
int16_t  volOut   = 0;
uint16_t levelOut = MSGEQ7_ZERO_SIGNAL_LEVEL;

// audio levels, audio level buffer, and approx. relative dB levels
uint16_t levelRead[MSGEQ7_SIGNAL_BAND_COUNT];
uint16_t levelBuf[LEVEL_TRACK_BUFFER_SIZE];
uint16_t dBLevels[LEVEL_TRACK_BUFFER_SIZE];

// for tracking automatic gain control and EQ mode feature states
bool    feature_AGC_mode = false;
uint8_t feature_EQ_mode  = 0;

// correlates to the number of switch state transitions registered [0, 1, 2, 3, or 4]
enum feature 
{
  feature_null,
  feature_pairing,
  feature_playback,
  feature_equalizer,
  feature_autovolume
};

// time of most recent audio level read (rolls over after about 50 days)
uint32_t previousMillis = 0;

// S2 interrupt debounce and timer values
volatile bool     S2_button_read_ACTIVE       = false;
volatile uint8_t  S2_interrupt_state_COUNT    = 0;
volatile uint32_t S2_button_read_START        = 0;
volatile uint32_t S2_interrupt_debounce_START = 0;

// create BM62 driver object
BM62 bluetooth(PRGM_SENSE_N, RST_N, IND_A2DP_N, &Serial);

// create MSGEQ7 driver object
MSGEQ7 spectrum(STROBE, DC_OUT, RESET, MSGEQ7_INPUT_PULLUP_ON_DC_OUT);

// create MAX9744 driver object
MAX9744 amplifier(MAX9744_I2CADDR, MUTE, SHDN, &Wire);

// create LED and LED+button objects for S1 and S2 user interface switches
LED led_SW1(S1_LEDPWM);
LED led_SW2(S2_LEDPWM);
LED_Button ledbutton_SW2(S2_PIN, led_SW2);


void ISR_BLOCK_S2_FALLING(void) {
  // execute interrupt code here
  uint32_t currentMillis = millis();
  if ((currentMillis - S2_interrupt_debounce_START) > S2_DEBOUNCE_MILLISECONDS) {
    detachInterrupt(S2_INTERRUPT_VECTOR);
    S2_interrupt_debounce_START = currentMillis;
    if (S2_interrupt_state_COUNT == 0) {
      S2_button_read_START = currentMillis;
      S2_button_read_ACTIVE = true;
      S2_interrupt_state_COUNT += 1;
    }
    else if ((S2_interrupt_debounce_START - S2_button_read_START) <= S2_READTIME_MILLISECONDS) {
      S2_interrupt_state_COUNT += 1;
    }
    attachInterrupt(S2_INTERRUPT_VECTOR, ISR_BLOCK_S2_RISING, RISING);
  }
}


void ISR_BLOCK_S2_RISING(void) {
  // execute interrupt code here
  uint32_t currentMillis = millis();
  if ((currentMillis - S2_interrupt_debounce_START) > S2_DEBOUNCE_MILLISECONDS) {
    detachInterrupt(S2_INTERRUPT_VECTOR);
    S2_interrupt_debounce_START = currentMillis;
    if (S2_interrupt_state_COUNT != 0) {
      if ((S2_interrupt_debounce_START - S2_button_read_START) <= S2_READTIME_MILLISECONDS) {
        S2_interrupt_state_COUNT += 1;
      }
    }
    attachInterrupt(S2_INTERRUPT_VECTOR, ISR_BLOCK_S2_FALLING, FALLING);
  }
}


// wait for BM62 to indicate a successful A2DP connection
void waitForConnection(void) {
  // keep track of PWM level and direction for each switch LED
  bool S1_PWM_DIR = HIGH;  // HIGH = rising, LOW = falling 
  bool S2_PWM_DIR = HIGH;  // HIGH = rising, LOW = falling
  uint16_t S1_PWM_VAL = 0;     // PWM analogWrite value for S1
  uint16_t S2_PWM_VAL = 0;     // PWM analogWrite value for S2

  while (!bluetooth.isConnected()) {
    // get the elapsed time, in milliseconds, since power-on
    uint32_t currentMillis = millis();
    
    // update LED brightness only every 10 ms
    if (currentMillis - previousMillis >= 10) {
      // save the time of most recent PWM value update
      previousMillis = currentMillis;
      if (S1_PWM_DIR & (S1_PWM_VAL <= (uint16_t)S1_PWM_MAX)) {
        if (S1_PWM_VAL >= (uint16_t)S1_PWM_MAX){
          // if at max brightness, reverse direction
          S1_PWM_DIR = LOW;
        }
        else {
          // otherwise, set LED brightness and increment
          led_SW1.brightness(S1_PWM_VAL);
          S1_PWM_VAL += 5;
        }
      }
      else {
        if (S1_PWM_VAL <= (uint16_t)S1_PWM_MIN){
          // if at min brightness, reverse direction
          S1_PWM_DIR = HIGH;
        }
        else {
          // otherwise, set LED brightness and decrement
          led_SW1.brightness(S1_PWM_VAL);
          S1_PWM_VAL -= 5;
        }
      }
    }
  }

  // turn off the LED and wait, helps distinguish next section
  led_SW1.off();
  delay(100);

  // pulse quickly twice to indicate a successful connection
  S1_PWM_DIR = HIGH;          // reset the S1 PWM direction
  for (uint8_t k = 0; k < 2; k++) {
    for (uint16_t m = S1_PWM_MIN; m <= S1_PWM_MAX; m += 4) {
      led_SW1.brightness(m);
      delay(2);
    }
    for (uint16_t m = S1_PWM_MAX; m >= S1_PWM_MIN; m -= 4) {
      led_SW1.brightness(m);
      delay(2);
    }
  }
  delay(250);

  // set the S1 LED brightness to the default 'on' value
  led_SW1.brightness(S1_PWM_DEF);

  // wait 200 ms to help avoid BM62 missing UART reads
  delay(200);
}


// calculate valid amplifier volume adjustment range (assumes range of 0-63)
void updateVolumeRange64(void) {
  uint8_t CurrentVolumeLevel = lowByte(volOut >> 4);
  // if the volume control value is zero set min/max to zero to mute output
  if (CurrentVolumeLevel == (uint8_t)0) {
    volumeRange[0] = 0;
    volumeRange[1] = 0;
  }
  // if the volume control value is less than 7, set lower-bound to 0
  else if (CurrentVolumeLevel < (uint8_t)7) {
    volumeRange[0] = 0;
    volumeRange[1] = CurrentVolumeLevel + 6;
  }
  // if the volume control value is less than 7, set lower-bound to 0
  else if (CurrentVolumeLevel > (uint8_t)57) {
    volumeRange[0] = CurrentVolumeLevel - 6;
    volumeRange[1] = 63;
  }
  // otherwise set lower bound to (volume - 6), upper bound to (volume + 6)
  else {
    volumeRange[0] = CurrentVolumeLevel - 6;
    volumeRange[1] = CurrentVolumeLevel + 6;
  }
}


// set up and configure the MCU, BM62, and MSGEQ7
void setup() {
  // initialize the BM62 bluetooth device
  Serial.begin(SERIAL_BAUD_RATE, SERIAL_8N1);
  bluetooth.init();

  // initialize the MMAX9744
  Wire.begin();
  Wire.setClock(TWI_CLOCK_RATE);
  amplifier.invertMuteLogic(true);
  amplifier.init();

  // initialize the MSGEQ7
  spectrum.init();

  // initialize S1 and S2 switches and attach their LEDs to them
  led_SW1.init();
  ledbutton_SW2.enableInputPullup();
  attachInterrupt(S2_INTERRUPT_VECTOR, ISR_BLOCK_S2_FALLING, FALLING);

  // wait for the BM62 to indicate a successful A2DP connection
  waitForConnection();
  bluetooth.stop();
  
  // set initial MAX9744 amplifier volume parameter and unmute
  vol = analogRead(VOLUME);             // read Volume Control
  amplifier.volume(lowByte(vol >> 4));
  volOut = vol;
  updateVolumeRange64();
  amplifier.unmute();
  
  // initialize levelBuf to nominal 'zero-signal' value
  for (uint8_t k = 0; k < 32; k++) {
    levelBuf[k] = MSGEQ7_ZERO_SIGNAL_LEVEL;
  }
  
  // read weighted audio level data, find mean, calculate buffer value
  spectrum.read(levelRead, MSGEQ7_SIGNAL_BAND_COUNT);
  levelOut = Audiomath::decayBuffer32(levelBuf, LEVEL_TRACK_BUFFER_SIZE,
                                      spectrum.mean(levelRead, 
                                      MSGEQ7_SIGNAL_BAND_COUNT),
                                      MSGEQ7_ZERO_SIGNAL_LEVEL);

  // initialize base volume level and relative dB values
  Audiomath::dBFastRelativeLevel(dBLevels, levelOut);

  // interrupt will often glitch and count when first enabled, so reset this here
  // (after establishing BT connection which gives us a delay) without checking it
  S2_interrupt_state_COUNT = 0;
}


void loop() {
  if (!bluetooth.isConnected()) {
    // if A2DP connection is lost, halt and wait for reconnection
    waitForConnection();
    bluetooth.stop();
  }

  // get the elapsed time, in millisecionds, since power-on
  uint32_t currentMillis = millis();

  // read audio levels from MSGEQ7 only if enough time has passed
  if (currentMillis - previousMillis >= AUDIO_READ_INTERVAL_MILLISECONDS) {
    // save the time of most recent transmission
    previousMillis = currentMillis;

    // read weighted audio level data, find mean, calculate buffer value
    spectrum.read(levelRead, MSGEQ7_SIGNAL_BAND_COUNT);
    levelOut = Audiomath::decayBuffer32(levelBuf, LEVEL_TRACK_BUFFER_SIZE,
                                        spectrum.mean(levelRead, 
                                        MSGEQ7_SIGNAL_BAND_COUNT),
                                        MSGEQ7_ZERO_SIGNAL_LEVEL);

    if (S2_button_read_ACTIVE) {
      if ((currentMillis - S2_button_read_START) >= (S2_READTIME_MILLISECONDS)) {
        cli();
        uint8_t S2_buttonStateCount = S2_interrupt_state_COUNT;
        S2_interrupt_state_COUNT = 0;
        S2_button_read_ACTIVE = false;
        sei();

        // switch 2 feature [1=pairing mode, 2=play/pause, 3=EQ mode, 4=autovolume on/off]
        feature S2_feature = ((S2_buttonStateCount == 1) ? feature_pairing    :
                             ((S2_buttonStateCount == 2) ? feature_playback   :
                             ((S2_buttonStateCount == 3) ? feature_equalizer  : 
                             ((S2_buttonStateCount == 4) ? feature_autovolume : feature_null))));
        switch (S2_feature) {
          case feature_pairing: {
            // MMI action, fast enter pairing mode (from non-off mode)
            bluetooth.enterPairingMode();
          } break;

          case feature_playback: {
            // media playback play/pause toggle (pauses if playing, plays if paused)
            bluetooth.playPauseToggle();
          } break;

          case feature_equalizer: {
            if (!feature_EQ_mode) {
              bluetooth.setEqualizerPreset(bluetooth.EQ_Classical);
              feature_EQ_mode = true;
            }
            else {
              bluetooth.setEqualizerPreset(bluetooth.EQ_Off);
              feature_EQ_mode = false;
            }
          } break;

          case feature_autovolume: {
            // automatic gain control enable/disable code goes here
          } break;

          default: {
            // too many button presses or something went wrong, so do nothing
          } break;
        }
      }
    } 

    #if defined DEBUG
      uint16_t levelDebug[2] = {(uint16_t)(lowByte(volOut >> 4)), levelOut};
      Serial.print(levelDebug[0]);
      Serial.print(" ");
      Serial.print(levelDebug[1]);
      Serial.print(" \n");
    #endif
  }
 
  vol = analogRead(VOLUME); // read Channel A0 to get volume control position
  
  // ignore three LSB to filter noise and prevent output level oscillations
  if (abs(volOut - vol) > 8) {
    amplifier.volume(lowByte(
      volOut >> 4));
    volOut = vol;
    updateVolumeRange64();
    Audiomath::dBFastRelativeLevel(dBLevels, levelOut);

    #if defined DEBUG
      Serial.print((uint16_t)(lowByte(volOut >> 4)));
      Serial.print(" ");
      Serial.print(levelOut);
      Serial.print(" ");
      for(uint8_t k = 0; k < 2; k++) {
        Serial.print(volumeRange[k]);
        Serial.print(" ");
      }
      size_t values_size = volumeRange[1] - volumeRange[0] + 1;
      int16_t values [values_size] ={0};
      Audiomath::convertVolumeToGain(volumeRange[0], volumeRange[1], 
                                    values, values_size);
      for(uint16_t k = 0; k < values_size; k++) {
        Serial.print(values[k]);
        Serial.print(" ");
      }
      for(uint8_t k = 0; k < 25; k++) {
        Serial.print(dBLevels[k]);
        Serial.print(" ");
      }
      Serial.print("\n");
    #endif
  }
}
