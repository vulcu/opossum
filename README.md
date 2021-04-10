# Opossum - Bluetooth Stereo Amplifier #
A bluetooth-based amplifier with automatic gain control, written primarily in Arduino/AVR C for a specific hardware platform comprised of:
  * [ATmega328-compatible microcontroller](https://www.microchip.com/wwwproducts/en/ATmega328)
  * [Microchip BM62 Bluetooth module](https://www.microchip.com/wwwproducts/en/BM62)
  * [MSGEQ7 spectrum level detector](https://www.sparkfun.com/datasheets/Components/General/MSGEQ7.pdf)
  * [Max9744 I2C-Controllable Class-D Amplifier](https://www.maximintegrated.com/en/products/analog/audio/MAX9744.html)



## Table of Contents ##
* [General Info](#general-info)
* [Features](#features)
* [Installation](#installation)
* [Algorithms](#algorithms)
* [References](#references)

## General Info
This project is a basis for a bluetooth stereo audio amplifier with a spetral-based automatic gain control. It was concieved and started at a point before I had access to an online streaming service such as Spotify, who normalizes track levels across albums and artists. When playing back music without that normalization it often frustrated me when a quiet track would follow a loud track, or vice versa, and the playback volume would thus jump substantially in one direction or the other.

A bluetooth-connected amplifier utilizing a microcontroller continuously analyzing the audio spectrum and adjusting the playback amplifier volume accordingly, solves the issue!

## Features ##
At present, the system will automatically connect to a device via bluetooth, play audio from that device, reestablish a lost connection, control the amplifier volume, read and process audio levels from the MSGEQ7, determine relative dB levels for audio data and automatically recalculate those levels when the volume is manually adjusted. Other than the fact it's not completely plumbed together yet, a lot of the code is in there and working reliably.

#### To Do ####
* Connect up the spectrum level detection algorithm to the amplifier volume control and integrate them

#### Status: This project is active development ####

## Installation ##
This repo will eventually be updated to contain a validated circuit schematic, PCBA design, and BM62 firmware load. Installation instructions will follow at that time.

## Algorithms ##
This project uses the following algorithms for analysis and controller feedback:
```
 1) 
```

## References: ##
I need to verify that there either are, or aren't, any!

## ##
(C) 2017-2021, Winry R. Litwa-Vulcu
