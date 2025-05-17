import ubinascii
import network
# Configuration file for the LoRa and Wi-Fi settings

# LORA Pin Definitions
LORA_SPIBUS = 1 # SPI bus number
LORA_MOSI = 11 # GPIO11 on Raspberry Pi Pico
LORA_MISO = 12 # GPIO12 on Raspberry Pi Pico
LORA_CLK = 10 # GPIO10 on Raspberry Pi Pico
LORA_CS = 3 # GPIO3 on Raspberry Pi Pico
LORA_RST = 15 # GPIO15 on Raspberry Pi Pico
LORA_GPIO = 2 # GPIO2 on Raspberry Pi Pico 
LORA_IRQ = 20 # GPIO20 on Raspberry Pi Pico

# LoRa Configuration Constants
# These constants are used to configure the LoRa module and should match the values used in the sender configuration.
LORA_FREQUENCY=868 # Frequency in MHz
LORA_BANDWIDTH=500.0 # Bandwidth in kHz
LORA_SPREADINGFACTOR=12 # Spreading factor (7-12)
LORA_CODINGRATE=8 # Coding rate (5-8)
LORA_SYNCWORD=0x18 # Sync word for private networks
LORA_POWER=10 # Transmit power in dBm
LORA_CURRENTLIMIT=60.0 # Current limit in mA
LORA_PREAMBLELENGTH=8 # Preamble length in symbols

# Wi-Fi Credentials
# Replace with your Wi-Fi credentials
WIFI_SSID = "myssid"  # Wi-Fi SSID
WIFI_PASSWORD = "mypassword"  # Wi-Fi password

# NTP Server Settings
NTP_SERVER = "0.uk.pool.ntp.org"  # NTP server address

# MQTT Broker Settings
MQTT_BROKER = "192.168.0.5"  # Replace with your broker's address
MQTT_PORT = 1883 # MQTT port
MQTT_DATA_TOPIC = "allotment/data" # Topic for data packets
MQTT_MESSAGE_TOPIC = "allotment/message" # Topic for message packets
MQTT_USERNAME = "mqttuser"  # Set your MQTT username
MQTT_PASSWORD = "mqttpassword"  # Set your MQTT password
CLIENT_ID: bytes = ubinascii.hexlify(network.WLAN(network.AP_IF).config("mac")[-3:])  # Unique ID for the client