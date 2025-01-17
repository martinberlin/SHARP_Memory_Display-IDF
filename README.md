# Adafruit SHARP Memory Display [![Build Status](https://github.com/adafruit/Adafruit_SHARP_Memory_Display/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_SHARP_Memory_Display/actions)

This is a fork of the Adafruit SHARP Memory Display Arduino Library for Arduino, that is now working when compiled by **idf.py build**

IMPORTANT: this fork works with the 128x128 LCD but not with the 240x400 still didn't found why.If you know why please open an ossue or make a pull-request to fix it
 thanks for your help!
 
Tested and works great with the Adafruit SHARP Memory Display Breakout Board. Pick one up today in the adafruit shop!
 http://www.adafruit.com/products/1393

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

# Dependencies

  These displays use SPI to communicate, 3 pins are required to interface
 
  Additionally please add in your components folder also my fork of Adafuit GFX as a submodule:

* [Adafruit GFX Library for ESP-IDF](https://github.com/martinberlin/Adafruit-GFX-Library-ESP-IDF)

    git submodule add https://github.com/martinberlin/Adafruit-GFX-Library-ESP-IDF.git components/adafruit-gfx

# Contributing

Contributions are welcome! Please read our [Code of Conduct](https://github.com/adafruit/Adafruit_SHARP_Memory_Display/blob/master/CODE_OF_CONDUCT.md>)
before contributing to help this project stay welcoming.

## Documentation and doxygen
Documentation is produced by doxygen. Contributions should include documentation for any new code added.

Some examples of how to use doxygen can be found in these guide pages:

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen-tips

Written by Limor Fried & Kevin Townsend for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution

To install, use the Arduino Library Manager and search for "Adafruit SHARP Memory Display" and install the library.
