# Constants
SLEEP_TIME: int = 600 # Sleep time in seconds (10 minutes)


# Reference voltages based on your calibration
V_DRY = 0.80   # Sensor voltage in dry soil (0%)
V_WET = 0.46   # Sensor voltage in wet soil (100%)
V_OPEN = 1.00  # Sensor miniumum voltage in open air
V_DISCONNECT = 0.2  # Sensor maximum voltage when disconnected
MAX_MOISTURE = 34  # Maximum calibrated moisture percentage

# Moisture Sensor Min/Max Values
MOISTURE_SENSOR_DRY = 15 # Dry moisture sensor value
MOISTURE_SENSOR_IDEAL_DRY = 20 # Ideal dry moisture sensor value
MOISTURE_SENSOR_IDEAL_WET = 35 # Ideal wet moisture sensor value
MOISTURE_SENSOR_WET = 40 # Saturated moisture sensor value

# RPI Header Pin Definitions
RPIHEADER_GPIO17 = 27 # GPIO 17 on the Raspberry Pi header - connects to GPIO27 on Raspberry Pi Pico
RPIHEADER_UART_TX = 4 # UART TX on the Raspberry Pi header - connects to GPIO4 on Raspberry Pi Pico
RPIHEADER_UART_RX = 5 # UART RX on the Raspberry Pi header - connects to GPIO5 on Raspberry Pi Pico

# E-paper Display Pin Definitions
DISPLAY_SPIBUS = 0 # SPI bus number
DISPLAY_SDA = 19 # GPIO19 on Raspberry Pi Pico
DISPLAY_SCK = 18 # GPIO18 on Raspberry Pi Pico
DISPLAY_CS = 17 # GPIO17 on Raspberry Pi Pico
DISPLAY_DC = 28 # GPIO28 on Raspberry Pi Pico
DISPLAY_RST = 22 # GPIO22 on Raspberry Pi Pico
DISPLAY_BUSY = 21 # GPIO21 on Raspberry Pi Pico

# LORA Module Pin Definitions
LORA_SPIBUS = 1 # SPI bus number
LORA_MOSI = 11 # GPIO11 on Raspberry Pi Pico
LORA_MISO = 12 # GPIO12 on Raspberry Pi Pico
LORA_CLK = 10 # GPIO10 on Raspberry Pi Pico
LORA_CS = 3 # GPIO3 on Raspberry Pi Pico
LORA_RST = 15 # GPIO15 on Raspberry Pi Pico
LORA_GPIO = 2 # GPIO2 on Raspberry Pi Pico 
LORA_IRQ = 20 # GPIO20 on Raspberry Pi Pico

# I2C Bus Pin Definitions
# The I2C bus is used for communication with the AHT20 and MCP9803 sensors.
I2C_BUS = 0 # I2C bus number
I2C_SDA = 8 # GPIO8 on Raspberry Pi Pico
I2C_SCL = 9 # GPIO9 on Raspberry Pi Pico
I2C_FREQ = 100000 # I2C frequency

# LoRa Configuration Constants
# These constants are used to configure the LoRa module and should be set according to your requirements.
LORA_FREQUENCY=868 # Frequency in MHz
LORA_BANDWIDTH=500.0 # Bandwidth in kHz
LORA_SPREADINGFACTOR=12 # Spreading factor (7-12)
LORA_CODINGRATE=8 # Coding rate (5-8)
LORA_SYNCWORD=0x18 # Sync word for private networks
LORA_POWER=-9 # Transmit power in dBm
LORA_MAX_POWER=22 # Maximum transmit power in dBm
LORA_CURRENTLIMIT=60.0 # Current limit in mA
LORA_PREAMBLELENGTH=8 # Preamble length in symbols

# Define minimum acceptable SNR for communication
# The SNR (Signal-to-Noise Ratio) is a measure of signal quality.
# The LoRa power output will be increased until the SNR is above this threshold.
# This value should be adjusted based on your specific requirements
# and the environment in which the LoRa module operates.
# A higher value means a stronger signal is required for successful communication.
MIN_SNR = -20