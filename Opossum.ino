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

// comment to deactivate various debug modes
//#define DEBUG_BM62_SERIAL
//#define DEBUG_LEVELOUT
//#define DEBUG_VOLUME
//#define DEBUG_EEPROM_RESET

// include libraries for PROGMEM, SLEEP, & I2C
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// include *.cpp files here b/c Arduino IDE won't find them unless installed as a library
#include "opossum/opossum.h"

#include "opossum/ledbutton.h"
#include "opossum/ledbutton.cpp"

#include "opossum/BM62.h"
#include "opossum/BM62.cpp"

#include "opossum/MAX9744.h"
#include "opossum/MAX9744.cpp"

#include "opossum/audiomath.h"
#include "opossum/audiomath.cpp"

#include "opossum/MSGEQ7.h"
#include "opossum/MSGEQ7.cpp"

// declare built-in reset fuction at memory address 0
void(* resetFunc) (void) = 0;

// for tracking automatic gain control and EQ mode feature states
bool feature_AGC_mode = false;
bool feature_EQ_mode  = false;

// use these for pulsing the AGC LED when making automated volume changes
int8_t  agc_ledpulse_counter = 0;
uint8_t agc_vm_index_previous = (DB_FAST_COEFFICIENT_COUNT >> 1);

// volume map index, filtered volume, raw volume, mean audio buffer level
int16_t  volume_out  = 0;
int16_t  volume_raw  = 0;
uint16_t audio_level = MSGEQ7_ZERO_SIGNAL_LEVEL;

// audio levels, audio level buffer, and approx. relative dB levels
uint8_t  volumeMap[DB_FAST_COEFFICIENT_COUNT];
uint16_t levelRead[MSGEQ7_SIGNAL_BAND_COUNT];
uint16_t levelBuf[LEVEL_TRACK_BUFFER_SIZE];
uint16_t dBLevels[DB_FAST_COEFFICIENT_COUNT];

// time of most recent audio level read (rolls over after about 50 days)
uint32_t previousMillis = 0;

// correlates to the number of switch state transitions registered [0, 1, 2, 3, or 4]
enum feature 
{
  feature_null,
  feature_pairing,
  feature_playback,
  feature_equalizer,
  feature_autovolume
};

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

// interrup service routine triggered by the watchdog timer exceeding set period
ISR(WDT_vect, ISR_BLOCK) {
  // this shouldn't ever happen so if we're here then something went wrong
  amplifier.mute();     // mute the audio amplifier
  amplifier.shutdown(); // then put it in shutdown
  bluetooth.reset();    // place the bluetooth module into reset/standby
  resetFunc();          // reset the MCU
}

// interrup service routine triggered by S2 button state falling from HIGH to LOW
void ISR_BLOCK_S2_FALLING(void) {
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

// interrup service routine triggered by S2 button state rising from LOW to HIGH
void ISR_BLOCK_S2_RISING(void) {
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
  bool S1_PWM_DIR = HIGH;   // HIGH = rising, LOW = falling
  uint16_t S1_PWM_VAL = 0;  // PWM analogWrite value for S1

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

    wdt_reset();  // reset the watchdog to prevent accidental system reboot
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

  wdt_reset();  // reset the watchdog to prevent accidental system reboot
  delay(250);   // adding a delay here help distiguish a visual pattern

  // set the S1 LED brightness to the default 'on' value
  led_SW1.brightness(S1_PWM_DEF);
}

// configure and initialize the WDT along with the system hardware
void setup() {
  // configure and enable the watchdog timer interrupt to shutdown and reset system
  cli();  // start by clearing global interrupt enable bit to disable interrupts

  // Clear watchdog system reset flag (WRDF) per 32u4 datasheet recommendation (p. 57)
  MCUSR &= ~(0x01 << WDRF);
    
  // Write logical 1 to WDCE and WDE at once to allow alteration of WDT mode (p. 56)
  WDTCSR = (0x01 << WDCE) | (0x01 << WDE);

  // configure WDT for interrupt not reset and set the WDT timeout period (pp. 55-61)
  WDTCSR = (0x01 << WDIE) | (WDTO_1S << WDP0);  // enable interrupt and timer period
  sei();  // set global interrupt enable bit and reenable interrupts

  // initialize the serial UART communication then initialize the bluetooth module
  Serial.begin(SERIAL_BAUD_RATE, SERIAL_8N1);
  bluetooth.init();

  // initialize I2C communication then initialize the audio amplifier
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
  
  // set initial MAX9744 amplifier volume parameter and unmute
  volume_out = analogRead(VOLUME);             // read Volume Control
  amplifier.volume(lowByte(volume_out >> 4));
  amplifier.unmute();

  // read weighted audio level data from spectrum analyzer
  spectrum.read(levelRead, sizeof(levelRead));

  // initialize levelBuf to initial audio level value
  for (uint8_t k = 0; k < LEVEL_TRACK_BUFFER_SIZE; k++) {
    uint16_t initial_signal_level = spectrum.mean(levelRead, sizeof(levelRead));
    levelBuf[k] = initial_signal_level;
  }

  // calculate mean audio level from weighted data and update the level buffer
  audio_level = Audiomath::decayBuffer32(levelBuf, LEVEL_TRACK_BUFFER_SIZE,
                                         spectrum.mean(levelRead, sizeof(levelRead)), 
                                         MSGEQ7_ZERO_SIGNAL_LEVEL);

  // initialize base volume level and relative dB values
  Audiomath::dBFastRelativeLevel(dBLevels, audio_level);

  // use this to reset the EEPROM registers to their default value (debug only!)
  #ifdef DEBUG_EEPROM_RESET
    for (int k = 0; k <= 255; k++) {
      EEPROM.update(k, 0xFF);
    }
  #endif

  // By default all EEPROM registers are initialized to 0xFF, so if the address in question is
  // equal to 0xFF then initialize it to 0x00 and modify the init register at 0x00 to reflect this
  if (((uint8_t)EEPROM.read(EEPROM_ADDR_INIT_REG_0) & BM_INIT_REG_FEATURE) == BM_INIT_REG_FEATURE){
    EEPROM.update(EEPROM_ADDR_FEATURE_STATE, (int16_t)0x00);
    EEPROM.update(EEPROM_ADDR_INIT_REG_0, 
                 ((uint8_t)EEPROM.read(EEPROM_ADDR_INIT_REG_0) & (uint8_t)(~BM_INIT_REG_FEATURE)));
  }

  if ((bool)(EEPROM.read(EEPROM_ADDR_FEATURE_STATE) & BM_EQ_STATE)) {
    feature_EQ_mode = true;
    bluetooth.setEqualizerPreset(bluetooth.EQ_Classical);
  }
  if ((bool)(EEPROM.read(EEPROM_ADDR_FEATURE_STATE) & BM_AGC_STATE)) {
    feature_AGC_mode = true;

    // recalculate the the relative dB levels based on most recent level reading
    Audiomath::dBFastRelativeLevel(dBLevels, audio_level);
    Audiomath::mapVolumeToBoundedRange(lowByte(volume_out >> 4), volumeMap, sizeof(volumeMap));

    // set SW2 LED to default brightness to indicate that the AGC function is enabled
    ledbutton_SW2.brightness(S2_PWM_DEF);
  }

  // SW2 interrupt will often glitch and count when first enabled, so reset this here
  // (after establishing BT connection which gives us a delay) without checking it
  S2_interrupt_state_COUNT = 0;
}

void loop() {
  // reset the watchdog timer before beginning the next loop iteration
  wdt_reset();

  // if A2DP connection is lost, halt and wait for reconnection
  if (!bluetooth.isConnected()) {
    amplifier.mute();
    waitForConnection();  // wdt gets reset multiple times during function call
    amplifier.unmute();
  }

  // get the elapsed time, in millisecionds, since power-on
  uint32_t currentMillis = millis();

  // check button interrupt status and execute corresponding feature functions
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
        // MMI action, enter fast pairing mode (from non-off mode)
        case feature_pairing: {
          bluetooth.enterPairingMode();
        } break;

        // media playback play/pause toggle (pauses if playing, plays if paused)
        case feature_playback: {
          bluetooth.playPauseToggle();
        } break;

        // enable/disable an equalizer preset within the bluetooth module DSP
        case feature_equalizer: {
          if (!feature_EQ_mode) {
            feature_EQ_mode = true;
            bluetooth.setEqualizerPreset(bluetooth.EQ_Classical);

            // update EQ enable/disable state stored in EEPROM
            uint8_t feature_state = (uint8_t)EEPROM.read(EEPROM_ADDR_FEATURE_STATE);
            EEPROM.update(EEPROM_ADDR_FEATURE_STATE, (int16_t)(feature_state | BM_EQ_STATE));
          }
          else {
            feature_EQ_mode = false;
            bluetooth.setEqualizerPreset(bluetooth.EQ_Off);

            // update EQ enable/disable state stored in EEPROM
            uint8_t feature_state = (uint8_t)EEPROM.read(EEPROM_ADDR_FEATURE_STATE);
            EEPROM.update(EEPROM_ADDR_FEATURE_STATE, (int16_t)(feature_state & ~BM_EQ_STATE));
          }
        } break;

        // toggle enabled/disabled state of the automatic gain control feature
        case feature_autovolume: {
          if (!feature_AGC_mode) {
            feature_AGC_mode = true;

            // update AGC enable/disable state stored in EEPROM
            uint8_t feature_state = (uint8_t)EEPROM.read(EEPROM_ADDR_FEATURE_STATE);
            EEPROM.update(EEPROM_ADDR_FEATURE_STATE, (int16_t)(feature_state | BM_AGC_STATE));

            // recalculate the the relative dB levels based on most recent level reading
            Audiomath::dBFastRelativeLevel(dBLevels, audio_level);
            Audiomath::mapVolumeToBoundedRange(lowByte(volume_out >> 4), volumeMap, sizeof(volumeMap));

            // set SW2 LED to default brightness to indicate that the AGC function is enabled
            ledbutton_SW2.brightness(S2_PWM_DEF);
          }
          else {
            feature_AGC_mode = false;

            // update AGC enable/disable state stored in EEPROM
            uint8_t feature_state = (uint8_t)EEPROM.read(EEPROM_ADDR_FEATURE_STATE);
            EEPROM.update(EEPROM_ADDR_FEATURE_STATE, (int16_t)(feature_state & ~BM_AGC_STATE));

            // set SW2 LED to off to indicate that the AGC function is disabled
            ledbutton_SW2.off();
          }
        } break;

        default: {
          // too many button presses or something went wrong, so do nothing
        } break;
      }
    }
  }

  // read Channel A0 to get volume control position as a 10-bit value
  volume_raw = analogRead(VOLUME); 
  
  // ignore four LSB to filter noise and prevent output level oscillations
  if (abs(volume_out - volume_raw) >= 16) {
    volume_out = volume_raw;
    amplifier.volume(lowByte(volume_raw >> 4));

    // recalculate the the relative dB levels based on most recent level reading
    if (feature_AGC_mode) {
      ledbutton_SW2.brightness(S2_PWM_DEF);
      agc_ledpulse_counter = 0;
      agc_vm_index_previous = (DB_FAST_COEFFICIENT_COUNT >> 1);
      Audiomath::dBFastRelativeLevel(dBLevels, audio_level);
      Audiomath::mapVolumeToBoundedRange(lowByte(volume_raw >> 4), volumeMap, sizeof(volumeMap));
    }

    #ifdef DEBUG_VOLUME
      Serial.print((uint16_t)(volume_out >> 4));
      Serial.print(" ");
      Serial.print(audio_level);
      Serial.print(" ");

      for(uint8_t k = 0; k < 25; k++) {
        Serial.print(dBLevels[k]);
        Serial.print(" ");
      }
      
      Audiomath::mapVolumeToBoundedRange(lowByte(volume_out >> 4), volumeMap, sizeof(volumeMap));
      for(uint8_t k = 0; k < 25; k++) {
        Serial.print(volumeMap[k]);
        Serial.print(" ");
      }

      Serial.print("\n");
    #endif
  }

  // read audio levels from MSGEQ7 only if enough time has passed
  if (currentMillis - previousMillis >= AUDIO_READ_INTERVAL_MILLISECONDS) {
    // save the time of most recent transmission
    previousMillis = currentMillis;

    // read weighted audio level data, find mean, calculate buffer value
    spectrum.read(levelRead, sizeof(levelRead));
    audio_level = Audiomath::decayBuffer32(levelBuf, LEVEL_TRACK_BUFFER_SIZE,
                                           spectrum.mean(levelRead, sizeof(levelRead)),
                                           MSGEQ7_ZERO_SIGNAL_LEVEL);

    // if AGC enabled, map the audio_level to the current volume and adjust accordingly
    if (feature_AGC_mode) {
      uint8_t vm_index = Audiomath::getVolumeMapIndx(audio_level, dBLevels, sizeof(dBLevels));

      // if the volume map index has changed then modify the amplifier volume
      if (volumeMap[vm_index] != volumeMap[agc_vm_index_previous]) {
        ledbutton_SW2.brightness(S2_PWM_DEF);
        amplifier.volume(volumeMap[vm_index]);

        // turn the LED on for 5 read intervals (defined by AUDIO_READ_INTERVAL_MILLISECONDS)
        if (volumeMap[vm_index] > volumeMap[agc_vm_index_previous]) {
          ledbutton_SW2.brightness(S2_PWM_MAX);
        }
        else {
          ledbutton_SW2.brightness(S2_PWM_MIN);
        }
        agc_vm_index_previous = vm_index;
        agc_ledpulse_counter = 5;
      }
      // if the volume map index hasn't changed, check to see if the LED timing counter is active
      else if (agc_ledpulse_counter > 0) {
        if (agc_ledpulse_counter > 1) {
          agc_ledpulse_counter -= 1;
        }
        else {
          ledbutton_SW2.brightness(S2_PWM_DEF); 
          agc_ledpulse_counter = 0;
        }
      }
    }

    #ifdef DEBUG_LEVELOUT
      uint16_t levelDebug[2] = {(uint16_t)(volume_out >> 4), audio_level};
      Serial.print(levelDebug[0]);
      Serial.print(" ");
      Serial.print(levelDebug[1]);
      Serial.print(" ");
      if (feature_AGC_mode) {
        uint8_t vm_index = Audiomath::getVolumeMapIndx(audio_level, dBLevels, sizeof(dBLevels));
        Serial.print(volumeMap[vm_index]);
      }
      else {
        Serial.print(0);
      }
      Serial.print(" \n");
    #endif
  }
}
