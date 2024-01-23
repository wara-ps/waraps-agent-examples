# Introduction 
WARA-PS agent code examples. The agents are connecting to the WARA-PS Core System.

## Arduino
- Level 1 (sensor) agent => WARA-PS_L1_Agent_ESP8266 or a
- Level 2 (direct execution) agent => WARA-PS_L2_Agent_ESP8266

These agents are using ESP-based development boards and programmed by the Arduino IDE.
The code has been tested on a Lolin/WeMos D1 Mini but should also work on ESP12/ESP32/ESPx with WiFi connectivety.

### Getting Started/Build/etc.
Instructions are in the preamble of each main project file, e.g. WARA-PS_Lx_Agent_ESP8266.ino.

## C++
The cpp folder contains examples of L1 and L2 agents written in C++11. Read cpp/README.md for instructions on running the C++ examples.

## Python
This folder contains three subfolders each containing code to run an example python agent.
- WARA-PS_L1_Python_Agent - L1 Python agent example, sends position. Intended to practice adding new sensors to increase agent values.
- WARA-PS_L2_Python_Agent - L2 Python agent example, sends position, speed, course and heading as well as can act on commands 'move-to', 'move-path', 'search-area' and 'go-home'.
- WARA-PS_L2_Workshop_Agent - L2 Python agent example, sends position, speed, course and heading as well as can act on command 'go-home'. Intended to practice adding own commands and agent functionality.

### Getting Started/Build/etc.
Please read the README found in the main pyhthon folder for setup and run instructions.

Note! Each folder contains its own .env file that need to be configured to the intended broker.

## ROS 2
This folder contains a L1 agent written as two python nodes.
One node publishes the position of the agent. The other node connects to mqtt, subscribes to the position from the first node, and publishes it to mqtt. The second node also publishes heartbeat and sensor_info messages to mqtt, making it a complete L1 agent.

### Getting Started/Build/etc.
See the README.md file inside the ros2 folder.

## MIT License

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