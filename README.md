# Opossum - Bluetooth Stereo Amplifier #
A bluetooth-based amplifier with automatic gain control, written primarily in Arduino/AVR C for a specific hardware platform comprised of:
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
At present, the system will automatically connect to a device via bluetooth, play audio from that device, reestablish a lost connection, control the amplifier volume, read and process audio levels from the MSGEQ7, determine relative dB levels for audio data and automatically recalculate those levels when the volume is manually adjusted. Other than the fact it's not completely plumbed together yet, a lot of the code is in there and working reliably.

#### Remaining Work ####
* move some remaining functions out of main file and into related classes
  * ~~volume range update~~ _won't do, makes sense to leave where it is_
  * ~~circular level tracking buffer~~ _done_
* the button interrupts are working and accurately debounce and count _**N**_ switch transitions within period _**T**_, now that count needs to be used to:
  1) ~~[single button press] play/pause media playback~~ _done_
  2) [double button press] turn automatic gain control on/off
  3) ~~[press and hold down] disconnect bluetooth and enter pairing mode~~ _done_
  4) ~~[press once, then press again and hold] enable/disable EQ preset~~ _done_
* connect up the spectrum level detection algorithm to the amplifier volume control such that the first controls the second
  * the 'adjustment range' needs to be mapped to the available gain steps which exist relative to the present gain setting, since this varies
  * this is also going to need some kind of hysterysis
* automated gain adjustments should pulse the secondary front panel LED in the direction of the adjustment when they occur (`bright >> baseline >> bright` or `dim >> baseline >> dim`)

#### Status: This project is active development ####

## ##
(C) 2017-2021, Winry R. Litwa-Vulcu
