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

// include libraries for PROGMEM, SLEEP, & I2C
#include <avr/pgmspace.h>
#include <TimerOne.h>

#include "opossum/parameters.h"
#include "opossum/BM62.h"
#include "opossum/userinterface.h"
#include "opossum/MAX9744.h"
#include "opossum/MSGEQ7.h"

// comment to deactivate UART debug mode
#define DEBUG

// Coefficient table for fast dB approximations
const uint16_t dBCoefTable[25] PROGMEM =
{ 
  2053, 2175, 2303, 2440, 2584,
  2738, 2900, 3072, 3254, 3446,
  3651, 3867, 4096, 4339, 4596,
  4868, 5157, 5462, 5786, 6129,
  6492, 6876, 7284, 7715, 8173
};



// buffer index, volume, filtered volume, base level, present level
uint8_t  bufferIndx = 0;
uint8_t volumeRange[2];
int16_t  vol, volOut = 0;
uint16_t zeroSignal, baseLevel, levelOut = 251;

// audio levels, audio level buffer, and approx. relative dB levels
uint16_t levelRead[7];
uint16_t levelBuf[32];
uint16_t dBLevs[25];

// time of most recent audio level read (rolls over after about 50 days)
uint32_t previousMillis = 0;

// value of S2 function button, set/cleared by ISR
volatile bool functionFeatureIsEnabled = LOW;
volatile bool S2LedIsOn = LOW;

// S2 interrupt debounce and timer values
volatile bool S2_interruptEnabled, S2_isResetInterrupt = false;
volatile bool S2_buttonReadComplete = false;
volatile uint8_t S2_buttonStateCount = 0;
volatile uint32_t S2_debounceStart, S2_debounceStop = 0;

// define if serial UART port should be initialized when BM62 is init
bool BM62_initSerialPort = true;

// create BM62 driver object
BM62 bluetooth(PRGM_SENSE_N, RST_N, IND_A2DP_N, &Serial);

// define if analog input pullup should be set active when MSGEQ7 is init
bool MSGEQ7_isInputPullup = false;

// create MSGEQ7 driver object
MSGEQ7 spectrum(STROBE, DC_OUT, RESET, MSGEQ7_isInputPullup);

// define if analog input pullup should be set active when MSGEQ7 is init
bool MAX9744_init_TWI = true;

// create MAX9744 driver object
MAX9744 amplifier(MAX9744_I2CADDR, MUTE, SHDN, &Wire);

// create LED and LED+button objects for S1 and S2 user interface switches
LED led_SW1(S1_LEDPWM);
LED led_SW2(S2_LEDPWM);
LED_Button ledbutton_SW2(S2_PIN, led_SW2);


void ISR_BLOCK_S2_FALLING(void) {
  // execute interrupt code here
  if (S2_interrupt_is_ENABLED) {
    detachInterrupt(S2_INTERRUPT_VECTOR);
    S2_interrupt_is_ENABLED = false;
    S2_interrupt_debounce_START = millis();
    if ((S2_interrupt_stateCounter_BUTTON == 0) && (!S2_interrupt_read_COMPLETE)) {
      S2_interrupt_read_START = true;
      S2_interrupt_stateCounter_BUTTON += 1;
    }
    else {
      S2_interrupt_stateCounter_BUTTON += 1;
    }
    S2_interrupt_trigger_DIRECTION = HIGH;
    attachInterrupt(S2_INTERRUPT_VECTOR, ISR_BLOCK_S2_RISING, RISING);
  }
}


void ISR_BLOCK_S2_RISING(void) {
  // execute interrupt code here
  if (S2_interrupt_is_ENABLED) {
    detachInterrupt(S2_INTERRUPT_VECTOR);
    S2_interrupt_debounce_START = millis();
    if ((S2_interrupt_stateCounter_BUTTON != 0) && (!S2_interrupt_read_COMPLETE)) {
      S2_interrupt_stateCounter_BUTTON += 1;
    }
    S2_interrupt_trigger_DIRECTION = LOW;
    attachInterrupt(S2_INTERRUPT_VECTOR, ISR_BLOCK_S2_FALLING, FALLING);
  }
}


void ISR_BLOCK_TIMER1_S2(void) {
  Timer1.detachInterrupt();
  S2_interrupt_read_COMPLETE = true;
}


// wait for BM62 to indicate a successful A2DP connection
void waitForConnection(void) {
  // keep track of PWM level and direction for each switch LED
  bool     S1_PWM_DIR, S2_PWM_DIR = HIGH;  // HIGH = rising, LOW = falling
  uint16_t S1_PWM_VAL, S2_PWM_VAL = 0;     // PWM analogWrite value

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

  // wait 200 ms to avoid BM62 missing UART reads
  delay(200);
}


// use a 32-value circular buffer to track audio levels
uint16_t expDecayBuf(uint16_t levelReadMean) {
  if (bufferIndx >= 32) {
    bufferIndx = 0;
  }
  if (levelReadMean > levelBuf[bufferIndx]) {
    // if the new value is greater, use value halfway between old and new
    levelBuf[bufferIndx] = levelBuf[bufferIndx] +
      ((levelReadMean - levelBuf[bufferIndx]) >> 1);
  }
  else {
    if (levelReadMean < ((zeroSignal * 3) >> 1)) {
      // if the latest level is less than 150% of the zero signal value,
      // there's probs no significant audio signal so don't update buffer
    }
    else {
      // otherwise, decay the current value by approximately 3%
      levelBuf[bufferIndx] = (levelBuf[bufferIndx] * 31L) >> 5;
    }
  }
  bufferIndx++;

  // calculate total sum of exponential buffer array
  uint32_t sum = 0;
  for (int k = 0; k < 32; k++) {
    sum = sum + levelBuf[k];
  }

  // divide by 32 and return array mean
  return sum >> 5; 
}


// update the relative dB level bands using currently defined volume level
void dBFastRelativeLevel(void) {
  for(uint8_t k = 0; k < 25; k++) {
    dBLevs[k] = ((uint32_t)baseLevel * pgm_read_word(&(dBCoefTable[k]))) >> 12;
  }
}


// calculate allowable MAX9744 volume adjustment range
void updateVolumeRange(void) {
  uint8_t CurrentVolumeLevel = lowByte(volOut >> 4);
  if (CurrentVolumeLevel == (uint8_t)0) {
    volumeRange[0] = 0;
    volumeRange[1] = 0;
  }
  else if (CurrentVolumeLevel < (uint8_t)7) {
    volumeRange[0] = 0;
    volumeRange[1] = CurrentVolumeLevel + 6;
  }
  else if (CurrentVolumeLevel > (uint8_t)57) {
    volumeRange[0] = CurrentVolumeLevel - 6;
    volumeRange[1] = 63;
  }
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

  // wait for the BM62 to indicate a successful A2DP connection
  waitForConnection();
  bluetooth.stop();
  
  // set initial MAX9744 amplifier volume parameter and unmute
  vol = analogRead(VOLUME);             // read Volume Control
  amplifier.volume(lowByte(vol >> 4));
  volOut = vol;
  void updateVolumeRange();
  amplifier.unmute();
  
  // initialize levelBuf to 'zero-signal' value
  for (uint8_t k = 0; k < 32; k++) {
    levelBuf[k] = baseLevel;
  }
  
  // read weighted audio level data, find mean, calculate buffer value
  spectrum.read(levelRead);
  levelOut = expDecayBuf(spectrum.mean(levelRead));

  // initialize base volume level and relative dB values
  baseLevel = levelOut;
  dBFastRelativeLevel();

  Timer1.initialize(S2_READTIME_MICROSECONDS);

  cli();
  S2_interrupt_trigger_DIRECTION = LOW;
  attachInterrupt(S2_INTERRUPT_VECTOR, ISR_BLOCK_S2_FALLING, FALLING);
  S2_interrupt_is_ENABLED = true;
  sei();
}


void loop() {
  if (!bluetooth.isConnected()) {
    // if A2DP connection is lost, halt and wait for reconnection
    waitForConnection();
    bluetooth.stop();
  }

  // get the elapsed time, in millisecionds, since power-on
  uint32_t currentMillis = millis();

  if (!S2_interrupt_is_ENABLED) {
    cli();
    if ((currentMillis - S2_interrupt_debounce_START) > S2_DEBOUNCE_MILLISECONDS) {
      S2_interrupt_is_ENABLED = true;
    }
    sei();
  }

  if (S2_interrupt_read_START) {
    Timer1.restart();
    Timer1.attachInterrupt(ISR_BLOCK_TIMER1_S2);
    S2_interrupt_read_START = false;
  }

  if (S2_interrupt_read_COMPLETE) {
    cli();
    uint8_t S2_buttonStateCount = S2_interrupt_stateCounter_BUTTON;
    sei();
    S2_interrupt_stateCounter_BUTTON = 0;
    S2_interrupt_read_COMPLETE = false;
  }

  // read audio levels from MSGEQ7 only if enough time has passed
  if (currentMillis - previousMillis >= AUDIO_READ_INTERVAL_MILLISECONDS) {
    // save the time of most recent transmission
    previousMillis = currentMillis;

    // read weighted audio level data, find mean, calculate buffer value
    spectrum.read(levelRead);
    levelOut = expDecayBuf(spectrum.mean(levelRead));

    #if defined DEBUG
      uint16_t levelDebug[2] = {(uint16_t)(lowByte(volOut >> 4)), levelOut};
      Serial.print(levelDebug[0]);
      Serial.print(" ");
      Serial.print(levelDebug[1]);
      Serial.print(" ");
      Serial.print(S2_debounceStop);
      Serial.print(" \n");
    #endif
  }
 
  vol = analogRead(VOLUME); // read Channel A0
  
  // ignore two LSB to filter noise and prevent output level oscillations
  if (abs(volOut - vol) > 4) {
    amplifier.volume(lowByte(volOut >> 4));
    volOut = vol;
    updateVolumeRange();
    baseLevel = levelOut;
    dBFastRelativeLevel();

    #if defined DEBUG
      Serial.print((uint16_t)(lowByte(volOut >> 4)));
      Serial.print(" ");
      Serial.print(levelOut);
      Serial.print(" ");
      Serial.print(S2_debounceStop);
      Serial.print(" ");
      for(uint8_t k = 0; k < 2; k++) {
        Serial.print(volumeRange[k]);
        Serial.print(" ");
      }
      for(uint8_t k = 0; k < 25; k++) {
        Serial.print(dBLevs[k]);
        Serial.print(" ");
      }
      Serial.print("\n");
    #endif
  }
}
