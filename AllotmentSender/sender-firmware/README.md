# sender-firmware

This directory contains the Raspberry Pi Pico firmware the Sender module. Full details for the allotment LoRa wireless moisture monitoring system that can be found on https://www.briandorey.com/post/pi-pico-lora-remote-soil-monitor

The allotment sender consists of the following components.

[Raspberry Pi Pico](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html#pico-1-family) microcontroller  
[Waveshare Pico-LoRa-SX1262-868M](https://www.waveshare.com/wiki/Pico-LoRa-SX1262-868M) LoRa module  
[AB Electronics UK ADC Pi](https://www.abelectronics.co.uk/p/69/adc-pi) analogue to digital converter  
[Microchip MCP9803](https://www.microchip.com/en-us/product/MCP9803) temperature sensor  
AHT20 temperature and humidity sensor  
[WeAct 2.9 Inch E-paper Module](https://www.aliexpress.com/item/1005004644515880.html)

## Configuration

config.py contains all of the sender program configuration options.