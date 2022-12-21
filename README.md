# Introduction 
WARA-PS agent code examples. The agents are connecting to the WARA-PS Core System.

## Arduino
- Level 1 (sensor) agent => WARA-PS_L1_Agent_ESP8266 or a
- Level 2 (direct execution) agent => WARA-PS_L2_Agent_ESP8266

These agents are using ESP-based development boards and programmed by the Arduino IDE.
The code has been tested on a Lolin/WeMos D1 Mini but should also work on ESP12/ESP32/ESPx with WiFi connectivety.

### Getting Started/Build/etc.
Instructions are in the preamble of each main project file, e.g. WARA-PS_Lx_Agent_ESP8266.ino.

## Python
A level 2+ python agent has been developed by Tommy Persson at LiU.
This agent is using Docker Compose, python scripts and evironment variables and will run on virtually any platform.
Pleas visit https://gitlab.liu.se/lrs/lrs_devenv_json for more information.



MIT License

Copyright (c) 2022 WARA-PS

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.