# Opossum - Bluetooth Stereo Amplifier #
A bluetooth-based amplifier with automatic gain control, written in C++ and Arduino/AVR C for a specific hardware platform comprised of:
  * [ATmega328-compatible microcontroller](https://www.microchip.com/wwwproducts/en/ATmega328)
  * [Microchip BM62 Bluetooth module](https://www.microchip.com/wwwproducts/en/BM62)
  * [MSGEQ7 spectrum level detector](https://www.sparkfun.com/datasheets/Components/General/MSGEQ7.pdf)
  * [Max9744 I2C-Controllable Class-D Amplifier](https://www.maximintegrated.com/en/products/analog/audio/MAX9744.html)



## Table of Contents ##
* [General Info](#general-info)
* [Features](#features)
* [Remaining Work](#remaining-work)

## General Info ##
This project is a basis for a bluetooth stereo audio amplifier with a spetral-based automatic gain control. It was concieved and started at a point before I had access to an online streaming service such as Spotify, who normalizes track levels across albums and artists. When playing back music without that normalization it often frustrated me when a quiet track would follow a loud track, or vice versa, and the playback volume would thus jump substantially in one direction or the other.

A bluetooth-connected amplifier utilizing a microcontroller continuously analyzing the audio spectrum and adjusting the playback amplifier volume accordingly, solves the issue!

## Features ##
At present, the system will automatically connect to a device via bluetooth, play audio from that device, reestablish a lost connection, control the amplifier volume manually, read and process audio levels from the MSGEQ7, determine relative dB levels for audio data and automatically recalculate those levels when the volume is manually adjusted. It will use that data to build a volume map of audio levels against nearby amplifier volume steps, make automated gain adjustments, and indicate via the front panel when and in what direction an adjustment was made.

#### Remaining Work ####
* ~~move some remaining functions out of main file~~ _complete_
  * ~~volume range update~~ _removed_
  * ~~circular level tracking buffer~~ _done_
* ~~the button interrupts are working and accurately debounce and count _**N**_ switch transitions within period _**T**_, now that count needs to be used to:~~ _done_
  1) ~~[single button press] play/pause media playback~~ _done_
  2) ~~[double button press] turn automatic gain control on/off~~ _done_
  3) ~~[press and hold down] disconnect bluetooth and enter pairing mode~~ _done_
  4) ~~[press once, then press again and hold] enable/disable EQ preset~~ _done_
* ~~connect up the spectrum level detection algorithm to the amplifier volume control such that the first controls the second~~ _done_
  * ~~the 'adjustment range' needs to be mapped to the available gain steps which exist relative to the present gain setting, since this varies~~ _done_
  * ~~this is also going to need some kind of hysterysis~~ _done_
* ~~automated gain adjustments should pulse the secondary front panel LED in the direction of the adjustment when they occur (`bright >> baseline >> bright` or `dim >> baseline >> dim`)~~ _done_
* ~~add EEPROM storage functionality for maintaining enabled/disabled state of AGC and EQ between power cycles~~

#### Status: This project is maintained but considered complete. Additional features will be incorporated into future designs, but are unlikely to be added to Opossum. ####

## ##
(C) 2017-2021, Winry R. Litwa-Vulcu
