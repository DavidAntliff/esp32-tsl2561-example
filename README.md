# esp32-tsl2561-example

[![Platform: ESP-IDF](https://img.shields.io/badge/ESP--IDF-v3.0%2B-blue.svg)](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/)
[![Build Status](https://travis-ci.org/DavidAntliff/esp32-tsl2561-example.svg?branch=master)](https://travis-ci.org/DavidAntliff/esp32-tsl2561-example)
[![license](https://img.shields.io/github/license/mashape/apistatus.svg)]()

## Introduction

This is an example application for the TAOS TSL2561 Light-to-Digital Converter device.

It is written and tested for v3.3 of the [ESP-IDF](https://github.com/espressif/esp-idf) environment, using the xtensa-esp32-elf toolchain (gcc version 5.2.0).

Ensure that submodules are cloned:

    $ git clone --recursive https://github.com/DavidAntliff/esp32-tsl2561-example.git

Build the application with:

    $ cd esp32-tsl2561-example.git
    $ idf.py menuconfig    # set your serial configuration and the I2C GPIO - see below
    $ idf.py -p (PORT) flash monitor

The program should detect your connected device and periodically obtain a light reading from it, displaying it on the console.

## Dependencies

This application makes use of the following components (included as submodules):

 * components/[esp32-smbus](https://github.com/DavidAntliff/esp32-smbus)
 * components/[esp32-tsl2561](https://github.com/DavidAntliff/esp32-tsl2561)

## Hardware

To run this example, connect one TSL2561 device to two GPIOs on the ESP32 (I2C SDA and SCL). If external pull-up resistors are not provided with the sensor, add a 10 KOhm resistor from each GPIO to the 3.3V supply.

`idf.py menuconfig` can be used to set the I2C GPIOs and TSL2561 device I2C address.

The TSL2561 can be set to one of three I2C addresses depending on the connection state of the ADDR SEL pin:

 * If ADDR SEL is tied to ground, the address is 0x29.
 * If ADDR SEL is floating (not connected), the address is 0x39.
 * If ADDR SEL is tied to VCC (3.3V), the address is 0x49.

## Features

This example provides:

 * Visible, InfraRed and Full Spectrum light level measurements.
 * Lux calculation .
 * Compile-time selection of integration time and gain.

## Source Code

The source is available from [GitHub](https://www.github.com/DavidAntliff/esp32-tsl2561-example).

## License

The code in this project is licensed under the MIT license - see LICENSE for details.

## Links

 * [TSL2560, TSL2561 datasheet (via Adafruit)](https://cdn-shop.adafruit.com/datasheets/TSL2561.pdf)
 * [Espressif IoT Development Framework for ESP32](https://github.com/espressif/esp-idf)

## Acknowledgements

"I2C" is a registered trademark of Phillips Corporation.

"SMBus" is a trademark of Intel Corporation. 
