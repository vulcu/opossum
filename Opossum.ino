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
#include <avr/sleep.h>
#include <Wire.h>

// comment to deactivate UART debug mode
#define DEBUG

// define the MAX9744 I2C address, default is 0x4B
#define MAX9744_I2CADDR ((uint8_t)0x4B)

// define the digitalOut pins according to the schematic net name
const uint8_t       S2_INT = 3;
const uint8_t   IND_A2DP_N = 4;
const uint8_t    S2_LEDPWM = 5;
const uint8_t    S1_LEDPWM = 6;
const uint8_t        RST_N = 8;
const uint8_t       STROBE = 9;
const uint8_t        RESET = 10;
const uint8_t         MUTE = 12;
const uint8_t         SHDN = 13;
const uint8_t PRGM_SENSE_N = 17;

// define the analogRead pins according to the schematic net name
const uint8_t VOLUME = 0;
const uint8_t  DCOUT = 1;

// time in ms between audio input level reads
const uint16_t audioReadInterval = 100;

// Coefficient table for fast dB approximations
const PROGMEM uint16_t dBCoefTable[25] =
{ 
  2053, 2175, 2303, 2440, 2584,
  2738, 2900, 3072, 3254, 3446,
  3651, 3867, 4096, 4339, 4596,
  4868, 5157, 5462, 5786, 6129,
  6492, 6876, 7284, 7715, 8173
};

// MAX9744 absolute gain levels (dB), multiplied
// by 10x to allow PROGMEM storage as int16_t
const PROGMEM int16_t MAX9744GainLevel[64] = 
{
    95,    88,    82,    76,
    70,    65,    59,    54,
    49,    44,    39,    34,
    29,    24,    20,    16,
    12,    05,   -05,   -19,
   -34,   -50,   -60,   -71,
   -89,   -99,  -109,  -120,
  -131,  -144,  -154,  -164,
  -175,  -197,  -216,  -235,
  -252,  -272,  -298,  -315,
  -334,  -360,  -376,  -396,
  -421,  -437,  -456,  -481,
  -506,  -542,  -567,  -602,
  -627,  -662,  -687,  -722,
  -747,  -783,  -808,  -843,
  -868,  -903,  -929, -1095
};

// BM62 UART commands for media playback control
const uint8_t BM62_Play[7] =
{
  0xAA, 0x00, 0x03, 0x04, 0x00, 0x05, 0xF4
};
const uint8_t BM62_Pause[7] =
{
  0xAA, 0x00, 0x03, 0x04, 0x00, 0x06, 0xF3
};
const uint8_t BM62_Stop[7] =
{
  0xAA, 0x00, 0x03, 0x04, 0x00, 0x08, 0xF1
};
const uint8_t BM62_PrevTrack[7] =
{
  0xAA, 0x00, 0x03, 0x02, 0x00, 0x35, 0xC6
};
const uint8_t BM62_NextTrack[7] =
{
  0xAA, 0x00, 0x03, 0x02, 0x00, 0x34, 0xC7
};

// BM62 UART Commands for DSP processing control

// min, max, and default 'on' LED brightness values for each switch LED
uint8_t S1_PWM_MIN = 20;                // S1 minimum PWM level
uint8_t S1_PWM_MAX = 255;               // S1 maximum PWM level
uint8_t S1_PWM_DEF = 200;               // S1 default PWM level
uint8_t S2_PWM_MIN = 20;                // S2 minimum PWM level
uint8_t S2_PWM_MAX = 255;               // S2 maximum PWM level
uint8_t S2_PWM_DEF = 200;               // S2 default PWM level

// keep track of PWM level and direction for each switch LED
bool     S1_PWM_DIR, S2_PWM_DIR = HIGH;  // HIGH = rising, LOW = falling
uint16_t S1_PWM_VAL, S2_PWM_VAL = 0;     // PWM analogWrite value

// buffer index, volume, filtered volume, base level, present level
uint8_t  bufferIndx = 0;
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
volatile uint8_t S2ButtonPressCount = 0;
volatile uint32_t S2DebounceStart, S2DebounceStop = 0;


// check if the BM62 programming pin is pulled low
void isProgramMode(void) {
  if (!digitalRead(PRGM_SENSE_N)) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set sleep mode to power down
    cli();                                // globally disable interrupts
    sleep_enable();                       // set sleep bit
    sleep_bod_disable();                  // disable brown out detection
    sleep_cpu();                          // go to sleep
    
    // interrupts are disabled, mcu will NOT wake up until next power cycle!
  }
}


// wait for BM62 to indicate a successful A2DP connection
void waitForConnection(void) {
  while (digitalRead(IND_A2DP_N)) {
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
          analogWrite(S1_LEDPWM, S1_PWM_VAL);
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
          analogWrite(S1_LEDPWM, S1_PWM_VAL);
          S1_PWM_VAL -= 5;
        }
      }
    }
  }

  // turn off the LED and wait, helps distinguish next section
  digitalWrite(S1_LEDPWM, LOW);
  delay(100);

  // pulse quickly twice to indicate a successful connection
  S1_PWM_DIR = HIGH;          // reset the S1 PWM direction
  for (uint8_t k = 0; k < 2; k++) {
    for (uint16_t m = S1_PWM_MIN; m <= S1_PWM_MAX; m += 4) {
      analogWrite(S1_LEDPWM, m);
      delay(2);
    }
    for (uint16_t m = S1_PWM_MAX; m >= S1_PWM_MIN; m -= 4) {
      analogWrite(S1_LEDPWM, m);
      delay(2);
    }
  }
  delay(250);

  // set the S1 LED brightness to the default 'on' value
  analogWrite(S1_LEDPWM, S1_PWM_DEF);

  // reset previousMillis to the initial value
  previousMillis = 0;

  // wait 200 ms to avoid BM62 missing UART reads
  delay(200);
}


// read back spectral band data from the MSGEQ7
void readMSGEQ7(void) {
  // set RESET pin low to enable output
  digitalWrite(RESET, LOW);
  delayMicroseconds(100);

  // pulse STROBE pin to read all 7 frequency bands
  for(uint8_t k = 0; k < 7; k++) {
    // set STROBE pin low to enable output
    digitalWrite(STROBE, LOW);
    delayMicroseconds(65);

    // read signal band level, account for later loudness adj.
    levelRead[k] = analogRead(DCOUT) << 3;

    // set STROBE pin high again to prepare for next band reading
    digitalWrite(STROBE, HIGH);
    delayMicroseconds(35);
  }

  // set RESET high again to reset MSGEQ7 multiplexer
  digitalWrite(RESET, HIGH);

  // spectral band adj. (very) loosely based on ISO 226:2003 [60 phons]
  levelRead[0] = levelRead[0] >> 3;   //   63 Hz
  levelRead[1] = levelRead[1] >> 1;   //  160 Hz
  levelRead[2] = levelRead[2] >> 0;   //  400 Hz
  levelRead[3] = levelRead[3] << 0;   // 1000 Hz
  levelRead[4] = levelRead[4] << 0;   // 2500 Hz
  levelRead[5] = levelRead[5] >> 1;   // 6250 Hz
  levelRead[6] = levelRead[6] >> 2;   //16000 Hz
}


// find mean of levelRead array data read from MSGEQ7
uint16_t msgeq7Mean(void) {
  // calculate the sum of levelRead array
  uint16_t sum = 0;
  for(uint8_t k = 0; k < 7; k++) {
    sum = sum + levelRead[k];
  }
  // much faster than divide-by-7, accurate to 7.00
  return (sum * 585L) >> 12;
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
  uint8_t volumeRange[2];
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


// for calculating the checksum of a BM62 UART command:
byte checksum(uint8_t a[], uint8_t numel) {
  // BM62 documentation is lacking but pretty sure
  uint16_t sum = 0;
  for (uint8_t k = 2; k < numel - 1; k++) {
    sum = sum + a[k];
  }

  // subtract sum from 0xFFFF and add one; use only the lower byte
  sum = ((uint16_t)0xFFFF - sum) + (uint16_t)0x0001;
  return(lowByte(sum));
}


// set up and configure the MCU, BM62, and MSGEQ7
void setup() {
  // initialize the BM62 reset line and ensure reset is asserted
  pinMode(RST_N, OUTPUT);
  digitalWrite(RST_N, LOW);
  
  // wait 10 ms, then take the BM62 out of reset
  delay(10);
  digitalWrite(RST_N, HIGH);
  
  // determine if the BM62 is being programmed; if so, take a nap
  pinMode(PRGM_SENSE_N, INPUT);
  isProgramMode();
  
  // initialize remaining digital pin modes
  pinMode(S2_INT,     INPUT);
  pinMode(IND_A2DP_N, INPUT_PULLUP);
  pinMode(S2_LEDPWM,  OUTPUT);
  pinMode(S1_LEDPWM,  OUTPUT);
  pinMode(STROBE,     OUTPUT);
  pinMode(RESET,      OUTPUT);
  pinMode(MUTE,       OUTPUT);
  pinMode(SHDN,       OUTPUT);  

  // ensure both LEDs are turned off
  digitalWrite(S1_LEDPWM, LOW);
  digitalWrite(S2_LEDPWM, LOW);

  // set analog input pullup to minimize glitchy MSGEQ7 reads
  digitalWrite(A1, INPUT);

  // set MSGEQ7 strobe low, and reset high
  digitalWrite(STROBE, LOW);
  digitalWrite(RESET, HIGH);

  // mute the MAX9744 then take it out of shutdown
  digitalWrite(MUTE, LOW);
  digitalWrite(SHDN, HIGH);

  // wait for the BM62 to indicate a successful A2DP connection
  waitForConnection();

  // initialize the UART port to talk to the BM62
  Serial.begin(57600, SERIAL_8N1);

  // initialize Wire library and set clock rate to 400 kHz
  Wire.begin();
  Wire.setClock(400000L);
  

  // unmute MAX9744 and configure initial amplifier volume parameters
  Wire.beginTransmission(MAX9744_I2CADDR);
  Wire.write(lowByte(vol >> 4));
  Wire.endTransmission();
  volOut = vol;
  updateVolumeRange();
  digitalWrite(MUTE, HIGH);
  
  // initialize levelBuf to 'zero-signal' value
  for (uint8_t k = 0; k < 32; k++) {
    levelBuf[k] = baseLevel;
  }
  
  // read weighted audio level data, find mean, calculate buffer value
  readMSGEQ7();
  levelOut = expDecayBuf(msgeq7Mean());

  // initialize base volume level and relative dB values
  baseLevel = levelOut;
  dBFastRelativeLevel();
}


void loop() {
  if (digitalRead(IND_A2DP_N)) {
    // if A2DP connection is lost, halt and wait for reconnection
    waitForConnection();
    
    // start playback from media device
    Serial.write(BM62_Play, 7);
  }
  
  // get the elapsed time, in millisecionds, since power-on
  uint32_t currentMillis = millis();
  
  // read audio levels from MSGEQ7 only if enough time has passed
  if (currentMillis - previousMillis >= audioReadInterval) {
    // save the time of most recent transmission
    previousMillis = currentMillis;

    // read weighted audio level data, find mean, calculate buffer value
    readMSGEQ7();
    levelOut = expDecayBuf(msgeq7Mean());

    #if defined DEBUG
      uint16_t levelDebug[2] = {(uint16_t)(lowByte(volOut >> 4)), levelOut};
      Serial.print(levelDebug[0]);
      Serial.print(" ");
      Serial.print(levelDebug[1]);
      Serial.print(" \n");
    #endif
  }
 
  
  // ignore two LSB to filter noise and prevent output level oscillations
  if (abs(volOut - vol) > 4) {
    Wire.beginTransmission(MAX9744_I2CADDR);
    Wire.write(lowByte(volOut >> 4));
    Wire.endTransmission();
    volOut = vol;
    updateVolumeRange();
    baseLevel = levelOut;
    dBFastRelativeLevel();

    #if defined DEBUG
      Serial.print((uint16_t)(lowByte(volOut >> 4)));
      Serial.print(" ");
      Serial.print(levelOut);
      Serial.print(" ");
      for(uint8_t k = 0; k < 25; k++) {
        Serial.print(dBLevs[k]);
        Serial.print(" ");
      }
      Serial.print("\n");
    #endif
  }
}
