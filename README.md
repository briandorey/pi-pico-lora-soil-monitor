# Allotment LoRa Wireless Moisture Monitor

This repository contains the hardware and software files for the allotment LoRa wireless moisture monitoring system that can be found on https://www.briandorey.com/post/pi-pico-lora-remote-soil-monitor

The system consists of a sender and receiver, both using a Raspberry Pi Pico and a LoRa wireless module. The sensor reads moisuture, temperature and humitidy values from a series of sensors and sends the data wirelessly to the receiver module using the LoRa wireless protocol.

## Components
###  1. AllotmentSender
   - Hardware: Includes the design and implementation of the sender's physical components.
   - Firmware: Software running on the sender hardware to read sensor data, update an e-paper display and send the data to the home receiver via a LoRa module.


### 2. Home Receiver
   - Hardware: Details about the receiver's physical components.
   - Firmware: Software running on the receiver hardware to handle incoming LoRa data and relay it to an MQTT broker

## License



MIT License

Copyright (c) 2025 Brian Dorey

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.