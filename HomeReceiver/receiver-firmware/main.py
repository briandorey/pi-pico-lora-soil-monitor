
# This is a simple LoRa receiver that connects to a Wi-Fi network and publishes messages to an MQTT broker.
# It uses the SX1262 LoRa module for communication and handles errors gracefully.
# It also includes a message queue to store messages before sending them to the MQTT broker.
# The code is designed to run on a Raspberry Pi Pico 2 W and uses the MicroPython framework.
#
# MIT License
# 
# Copyright (c) 2025 Andrew Dorey
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from lib.sx1262 import SX1262
from lib.umqtt.simple import MQTTClient
from machine import Pin, WDT
import network
import socket
import struct
import time
import machine
from config import * # Import all variables from config


# Global Variables
last_day = 0
wifi_retries = 0
wlan = network.WLAN(network.STA_IF)
message_received = False 
led = machine.Pin("LED", machine.Pin.OUT)
wdt = WDT(timeout=8000)  # Watchdog timer set to 5 seconds
lora_module = SX1262(spi_bus=LORA_SPIBUS, clk=LORA_CLK, mosi=LORA_MOSI, miso=LORA_MISO, cs=LORA_CS, irq=LORA_IRQ, rst=LORA_RST, gpio=LORA_GPIO)

class MessageQueue:
    """
    A simple message queue to store messages before sending them to the MQTT broker.
    """

    def __init__(self, max_size=100):
        self.queue = []
        self.max_size = max_size
    
    def add(self, message, topic):
        if len(self.queue) < self.max_size:
            self.queue.append((message, topic))
            return True
        return False
    
    def process_queue(self):
        wdt.feed()  # Feed the watchdog timer
        while self.queue:
            message, topic = self.queue[0]
            try:
                publish_mqtt(message, topic)
                self.queue.pop(0)
            except:
                break  # Stop processing if publication fails


def connect_wifi(max_retries=3, timeout=20):
    """
    Connect to the WiFi network with retry logic
    """
    global wlan, wifi_retries
    attempts = 0
    
    # Add initial delay to allow WiFi hardware to stabilize
    time.sleep(1)

    while attempts < max_retries:
        wdt.feed()  # Feed the watchdog timer
        print(f"Connecting to Wi-Fi: {WIFI_SSID} (Attempt {attempts + 1}/{max_retries})")

        try:
            wlan.active(False)
            time.sleep(1)
            wlan.active(True)
            wlan.config(pm=0xa11140)  # Disable power-saving mode
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)
            
            start_time = time.time()
            while not wlan.isconnected():
                wdt.feed()  # Feed the watchdog timer
                if time.time() - start_time > timeout:
                    raise Exception("WiFi connection timeout")
                print(".", end="")
                time.sleep(1)

                
            print("\nConnected to Wi-Fi:", wlan.ifconfig())
            return True
            
        except Exception as e:
            attempts += 1
            print(f"\nWiFi connection attempt {attempts} failed: {str(e)}")
            if attempts >= max_retries:
                print("Max retries reached. Could not connect to WiFi")
                return False


def publish_mqtt(message, topic):
    """
    Publish a message to the MQTT broker with error handling
    """
    wdt.feed()  # Feed the watchdog timer

    try:
        client = MQTTClient(
            CLIENT_ID, MQTT_BROKER, port=MQTT_PORT, 
            user=MQTT_USERNAME, password=MQTT_PASSWORD
        )
        
        client.connect()
        print("Connected to MQTT Broker")

        client.publish(topic, message)
        print(f"Published message: {message}")
        
    except Exception as e:
        print(f"MQTT Error: {str(e)}")
    finally:
        try:
            client.disconnect()
        except:
            pass

def get_ntp_time():
    """
    Get time from NTP server with retry logic
    """
    NTP_DELTA = 2208988800  # Difference between NTP and Unix epoch (1970)
    host = NTP_SERVER
    max_retries = 10

    for attempt in range(max_retries):
        wdt.feed()  # Feed the watchdog timer

        try:
            # Send request
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            addr = socket.getaddrinfo(host, 123)[0][-1]
            msg = b'\x1b' + 47 * b'\0'
            s.sendto(msg, addr)

            # Receive response
            s.settimeout(1)
            msg = s.recv(48)
            s.close()

            # Unpack time
            val = struct.unpack("!I", msg[40:44])[0]
            return val - NTP_DELTA
        except Exception as e:
            print(f"NTP request attempt {attempt + 1} failed: {str(e)}")
            time.sleep(1)  # Wait before retrying

    print("NTP request failed after 10 attempts. Setting clock to 1st January 2000.")
    return time.mktime((2000, 1, 1, 0, 0, 0, 0, 0))



# Set internal RTC
def set_rtc():
    """
    Set the time on the internal RTC from the NTP server
    """
    wdt.feed()  # Feed the watchdog timer

    try:
        # Get the current date and time from the NTP server
        ntp_time = get_ntp_time()
        if ntp_time:
            tm = time.localtime(ntp_time)
            rtc = machine.RTC()
            rtc.datetime((tm[0], tm[1], tm[2], tm[6]+1, tm[3], tm[4], tm[5], 0))
            
            print("RTC set to:", datetime_str(get_rtc()))
        else:
            raise ValueError("NTP time could not be retrieved")
    except Exception as e:
        error_message = f"RTC Error: Could not set RTC. {str(e)}"
        publish_mqtt(error_message, MQTT_MESSAGE_TOPIC)
        print(error_message)

def get_rtc():
    """
    Get the time from the internal RTC
    """
    rtc = machine.RTC()
    return rtc.datetime()

def get_day() -> type[int]:
    rtc = machine.RTC()
    return rtc.datetime()[3]

def datetime_str(timestamp):  
    """
    Format the date as an ISO Date string
    """  
    datestring="%04d-%02d-%02dT%02d:%02d:%02dZ"%(timestamp[0:3] + timestamp[4:7])
    return datestring

def lora_callback(events):
    """
    Callback function for LoRa events
    """

    global message_received
    wdt.feed()  # Feed the watchdog timer

    if events & SX1262.RX_DONE:
        response, err = lora_module.recv()
        if len(response) > 0:
            error = SX1262.STATUS[err]
            msg = response.decode('utf-8')
            message_received = True
            print(msg)
            print(error)
            if msg == "ping": # ping request from sender
                response_string = "pong;" + datetime_str(get_rtc()) + ";" + str(lora_module.getSNR())
                response: bytes = response_string.encode('utf-8')
                lora_module.send(response)
                print("Ping Received! Sending Response")
                print(response)
            elif msg.startswith("{"): # JSON response from sender
                publish_mqtt(msg, MQTT_DATA_TOPIC)
                print("JSON Received! Sending Response")
                response_string = "ok"
                response: bytes = response_string.encode('utf-8')
                lora_module.send(response)
            else: # Message received was not expected
                error_message = "Receive Error: Uknown Command Received at " + datetime_str(get_rtc())
                publish_mqtt(error_message, MQTT_MESSAGE_TOPIC)
                print(error_message)
    elif events & SX1262.TX_DONE:
        print('TX done.')

def blink_led(count=1) -> None:
    """
    Blink the LED to indicate a message was received
    """
    wdt.feed()  # Feed the watchdog timer

    for _ in range(count):
        led.on()
        time.sleep(0.2)  # Keep LED on for 200 ms
        led.off()
        time.sleep(0.2)  # Keep LED off for 200 ms

def main():
    """
    Main program function
    """
    global wlan, message_received, last_day
    message_queue = MessageQueue()
    wdt.feed()  # Feed the watchdog timer

    try:
        blink_led(1)  # Blink once on startup

        # Initial setup
        # Hardware reset at startup
        wlan.active(False)

        # Initialize LoRa
        print("Initializing LoRa module...")
        try:
            lora_module.begin(freq=LORA_FREQUENCY, bw=LORA_BANDWIDTH, sf=LORA_SPREADINGFACTOR, 
                    cr=LORA_CODINGRATE, syncWord=LORA_SYNCWORD,
                    power=LORA_POWER, currentLimit=LORA_CURRENTLIMIT, 
                    preambleLength=LORA_PREAMBLELENGTH,
                    implicit=False, implicitLen=0xFF,
                    crcOn=True, txIq=False, rxIq=False,
                    tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)
            print("LoRa module initialized successfully")
        except Exception as e:
            print(f"LoRa initialization failed: {str(e)}")
            raise

        if not connect_wifi():
            raise Exception("Initial WiFi connection failed")

        blink_led(3)  # Blink three times to indicate Wi-Fi connection

        set_rtc()
        global last_day
        last_day = get_day()

        
        
        # Set LoRa to non-blocking mode
        lora_module.setBlockingCallback(False, lora_callback)

        print("Waiting for messages")
        publish_mqtt("Device Reset at " + datetime_str(get_rtc()), MQTT_MESSAGE_TOPIC)

        while True:
            wdt.feed()  # Feed the watchdog timer
            try:
                if not wlan.isconnected():
                    connect_wifi()
                
                message_queue.process_queue()  # Try to send queued messages

                if message_received:
                    message_received = False
                    blink_led()

                if last_day != get_day():
                    set_rtc()
                    last_day = get_day()
                    
                time.sleep(1)  # Prevent tight loop
                
            except Exception as e:
                print(f"Error in main loop: {str(e)}")
                time.sleep(5)  # Wait before retrying
                
    except Exception as e:
        print(f"Fatal error: {str(e)}")
        machine.reset()  # Reset the device on fatal errors

if __name__ == "__main__":
    main()