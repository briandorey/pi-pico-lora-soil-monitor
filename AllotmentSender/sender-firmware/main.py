
# LoRa Sensor Node
# This script reads data from various sensors, sends the data via LoRa, and updates a display.
# It uses the SX1262 LoRa module, an ADC Pi for moisture sensors, a MCP9803 for board temperature
# and an AHT20 for temperature and humidity.
# The script is designed to run on a Raspberry Pi Pico and uses the EPD_2in9_B_V4_Portrait display.

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
import time
import ujson
from machine import Pin, reset, I2C, WDT
from lib.ADCPi import ADCPi
from lib.mcp9803 import MCP9803
from lib.aht20 import AHT20
from lib.display import EPD_2in9_B_V4_Portrait
from config import * # Import all variables from config


# Pico Pin Connections
led = Pin(25, Pin.OUT) # LED on Raspberry Pi Pico
psu = Pin(6, Pin.OUT) # 3.3V Regulator enable pin
temperature_alert = Pin(7, Pin.IN) # MCP9803 Temperature Alert Pin
rpi_gpio17 = Pin(27, Pin.OUT) # Raspberry Pi Header GPIO17 pin

# Watchdog Timer
wdt = WDT(timeout=8000)  # 8 second timeout

# Flags
lora_initialized = False
adc_initialized = False
mcp9803_initialized = False
aht20_initialized = False
send_complete = False
send_timeout = False

#Global variables
moisture_values: list[float] = [0.0] * 8
temperature, humidity = 0.0, 0.0
board_temperature = 0.0
date: str = "01/01/2000 00:00:00"   
snr: float = 0.0

# Initialise communication buses
i2c = I2C(I2C_BUS, sda=I2C_SDA, scl=I2C_SCL, freq=I2C_FREQ)
lora_module = SX1262(spi_bus=LORA_SPIBUS, clk=LORA_CLK, mosi=LORA_MOSI, miso=LORA_MISO, cs=LORA_CS, irq=LORA_IRQ, rst=LORA_RST, gpio=LORA_GPIO)


def initialize_lora() -> int:
    """
    Initialize the LoRa module.
    """


    global lora_initialized, power
    try:
        # Load the LoRa power level from flash memory
        power: int = read_lora_power_from_flash()

        if power is not None:
            print("Loaded LoRa power level from flash:", power)
        else:
            power = LORA_POWER
            print("No LoRa power level found in flash, using default:", power)

        if int(power) >= LORA_MAX_POWER:
            power = LORA_POWER
            print("LoRa power level in flash is 10, using default:", power)

        # SX1262 LoRa module configuration
        lora_module.begin(freq=LORA_FREQUENCY, bw=LORA_BANDWIDTH, sf=LORA_SPREADINGFACTOR, cr=LORA_CODINGRATE, 
                syncWord=LORA_SYNCWORD, power=power, currentLimit=LORA_CURRENTLIMIT, 
                preambleLength=LORA_PREAMBLELENGTH, implicit=False, implicitLen=0xFF, 
                crcOn=True, txIq=False, rxIq=False, tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)
        lora_initialized = True
        print("LoRa module initialized successfully.")
        return power
    except Exception as e:
        print("Error initializing LoRa module:", e)
        lora_initialized = False
        return 0

def save_lora_power_to_flash(value):
    """Save a value to flash memory"""
    with open('lora.txt', 'w') as f:
        f.write(str(value))

def read_lora_power_from_flash() -> int:
    """Read a value from flash memory"""
    try:
        with open('lora.txt', 'r') as f:
            return int(f.read())
    except OSError:  # File doesn't exist
        return 0

def wakeup_devices() -> None:
    """
    Wake up the devices
    """
    psu.on() # Turn on the 3.3V regulator
    time.sleep(0.1) # Wait for the regulator to stabilize
    return

def sleep_devices() -> None:
    """
    Put the devices to sleep
    """
    psu.off() # Turn off the 3.3V regulator
    return

def adc_to_moisture_percent(adc_value) -> int:
    """
    Convert ADC value to moisture percentage
    :param adc_value: ADC value
    :return: Moisture percentage
    """
    if adc_value >= V_OPEN:
        return -1
    elif adc_value <= V_DISCONNECT:
        return -2
    else:
        # Calculate the moisture percentage using the formula
        moisture = 100 * (V_DRY - adc_value) / (V_DRY - V_WET)
        if moisture < 0:
            return 0
        elif moisture > 100:
            return 100
        else:
            return int(moisture)


def get_moisture() -> list[float]:
    """
    Get moisture readings from the ADC Pi
    """
    try:
        # Initialise ADC inputs using an ADC Pi from AB Electronics UK
        adc = ADCPi(0x68, 0x69, 12,sda=I2C_SDA,scl=I2C_SCL) 

        moisture: list[float] = [0.0] * 8
        # Read the moisture values from the ADC Pi
        for i in range(8):
            moisture[i] = adc_to_moisture_percent(adc.read_voltage(i + 1))
    except Exception as e:
        print("Error reading moisture values:", e)
        moisture = [0.0] * 8

    return moisture

def get_board_temperature() -> float:
    """
    Get the board temperature from the MCP9803
    """
    # Initialise MCP9803 temperature sensor
    
    try:
        mcp9803 = MCP9803(address=0x48, sda=I2C_SDA, scl=I2C_SCL, freq=I2C_FREQ, i2cbus=I2C_BUS)
        board_temperature: float = mcp9803.read_temperature()
    except Exception as e:
        print("Error reading board temperature:", e)
        board_temperature = 0.0
    return board_temperature

def get_temperature_and_humidity() -> tuple[float, float]:
    """
    Get the polytunnel temperature and humidity from the AHT20
    """
    try:
        # Initialise AHT20 temperature and humidity sensor
        i2c = I2C(I2C_BUS, sda=I2C_SDA, scl=I2C_SCL, freq=I2C_FREQ)
        aht20 = AHT20(i2c)
        aht20.initialize_sensor()
        aht20.soft_reset()

        # Get the temperature and humidity values
        temperature, humidity = aht20.get_temperature_and_humidity()
    except Exception as e:
        print("Error reading temperature and humidity:", e)
        temperature = 0.0
        humidity = 0.0
    return temperature, humidity

def get_data() -> bool:
    """
    Create a JSON object with the moisture and temperature data.
    :param date: Date string
    :return: JSON object as bytes
    """
    # Wake up the devices
    wakeup_devices()

    global date, moisture_values, board_temperature, temperature, humidity

    # Get the moisture and temperature data
    moisture_values: list[float] =  get_moisture()
    temperature, humidity = get_temperature_and_humidity()
    board_temperature = get_board_temperature()
    
    # Sleep the devices
    sleep_devices()   

    return True

def get_json_string(data) -> bytes:
    """
    Convert the data to JSON and encode it as bytes.
    :param data: Data to convert
    :return: JSON object as bytes
    """

    global date, moisture_values, board_temperature, temperature, humidity

    # Create a JSON object with the data
    data = {
        "date": date,
        "moisture1": moisture_values[0],
        "moisture2": moisture_values[1],
        "moisture3": moisture_values[2],
        "moisture4": moisture_values[3],
        "moisture5": moisture_values[4],
        "moisture6": moisture_values[5],
        "moisture7": moisture_values[6],
        "moisture8": moisture_values[7],
        "board_temperature": board_temperature,
        "temperature": temperature,
        "humidity": humidity
    }

    json_string: str = ujson.dumps(data)
    return json_string.encode('utf-8')

def send_data(sx, lora_power) -> bool:
    """
    Send the data to the LoRa module.
    """
    global send_complete, send_timeout, date
    send_timeout = False
    wdt.feed()

    print(f"Trying with power {power} dBm...")
    sx.setOutputPower(lora_power)
    sx.send(b"ping")
    print("Waiting for response...")
    # Now wait for response
    response, err = sx.recv(timeout_en=True, timeout_ms=2000)

    print(response)
    if response:
        print("Received:", response)
        print("Errors:", err)
        try:
            if response[:5] == b'pong;':
                # Parse SNR from message
                date = response[5:24]
                snr = float(response[26:])
                print("date: " + str(date))
                print("snr: " + str(snr))

                if snr >= MIN_SNR:
                    print("SNR acceptable. Done.")
                    save_lora_power_to_flash(lora_power)
                else:
                    print("SNR too low. Adjusting power.")
                    return False
            else:
                print("Unexpected response.")
                return False
        except Exception as e:
            print("Error decoding response:", e)

    else:
        print("No response received.")
        return False

    # Send the data to the LoRa module
    lora_module.send(get_json_string(date))
    # Now wait for response
    response, err = sx.recv(timeout_en=True, timeout_ms=2000)

    if response:
        print("Received:", response)
        print("Errors:", err)
        if response[:5] == b'ok':
            print("Data sent successfully.")
            send_complete = True
            return True
        else:
            print("Error sending data.")
            send_timeout = True
            return False

    return False
   

def update_display() -> None:
    """
    Update the display with the given data.
    :param data: Data to display
    """
    global date, moisture_values, board_temperature, temperature, humidity, send_timeout, wdt

    try:
        # Wake up the devices
        wakeup_devices()

        display = EPD_2in9_B_V4_Portrait(DISPLAY_SPIBUS, DISPLAY_SDA, DISPLAY_SCK, DISPLAY_CS, DISPLAY_DC, DISPLAY_RST, DISPLAY_BUSY, wdt)

        if send_timeout:
            date_string = "LoRa TX Failed..."
        else:
            date_string: str = date

        # Draw the display
        draw_display(display, date_string, moisture_values, board_temperature, temperature, humidity)   

    except Exception as e:
        print("Error updating display:", e)

    sleep_devices()

    return

def draw_display(epd, date_string, moisture_values, board_temperature, temperature, humidity):
    
    epd.Clear()
    epd.imageblack.fill(0xFF)

    epd.imageblack.text(date_string, 0, 3, 0x00)
    epd.imageblack.text("TEMP:     " + "{:.1f}".format(temperature) + "C", 3, 146, 0x00)
    epd.imageblack.text("HUMIDITY: " + "{:.1f}".format(humidity) + "%", 3, 158, 0x00)
    epd.imageblack.fill_rect(0, 16, 50, 2, 0x00) # black outline
    epd.imageblack.fill_rect(78, 16, 50, 2, 0x00) # black outline
        
    # Moisture Boxes

    draw_moisture_box(epd, 3, 21, moisture_values[0])
    draw_moisture_box(epd, 3, 81, moisture_values[1])
    draw_moisture_box(epd, 85, 21, moisture_values[2])
    draw_moisture_box(epd, 85, 81, moisture_values[3])
    draw_moisture_box(epd, 3, 169, moisture_values[4])
    draw_moisture_box(epd, 85, 169, moisture_values[5])
    draw_moisture_box(epd, 3, 233, moisture_values[6])
    draw_moisture_box(epd, 85, 233, moisture_values[7])

    epd.display()
    epd.delay_ms(2000)
    epd.sleep()

def draw_moisture_box(epd, x: int, y: int, moisture: int) -> None:
    epd.imageblack.fill_rect(x, y, 40, 58, 0x00) # black outline
    epd.imageblack.fill_rect(x + 2, y + 2, 36, 54, 0xFF) # white box

    if moisture < 0: # If the moisture is invalid
        if moisture == -1: # If the moisture is open circuit
            epd.imageblack.text("Open", x + 5, y + 40, 0x00) # black text            
        elif moisture == -2: # If the moisture is disconnected
            epd.imageblack.text("NC", x + 12, y + 40, 0x00) # black text
    
    fill_height = int(moisture / 2)
    if moisture >= MOISTURE_SENSOR_IDEAL_DRY: # If the moisture is above the dry value
        epd.imageblack.fill_rect(x + 3, y + 4 + (50 - fill_height), 34, fill_height, 0x00) # black fill
        if moisture >= MOISTURE_SENSOR_IDEAL_WET: # If the moisture is above the wet value
            epd.imageblack.text("WET", x + 8, y + 38, 0xFF) # black text

    else: # If the moisture is below the dry value
        epd.imagered.fill_rect(x + 3, y + 4 + (50 - fill_height), 34, fill_height, 0xFF) # red fill
        epd.imagered.ellipse(x + 20, y + 8, 3, 3, 0xFF, True) # red circle
        epd.imagered.ellipse(x + 20, y + 20, 3, 3, 0xFF, True) # red circle
        epd.imagered.ellipse(x + 20, y + 30, 3, 3, 0xFF, True) # red circle
        epd.imagered.fill_rect(x + 17, y + 10, 7, 10, 0xFF) # red fill
    return

def blink_led(count = 1) -> None:
    """
    Blink the LED a given number of times.
    :param count: Number of times to blink
    """
    for _ in range(count):
        led.on()
        time.sleep(0.1)
        led.off()
        time.sleep(0.1)
    return

def main() -> None:
    """
    Main function to initialize the LoRa module and start the main loop.
    """
    # Initialize watchdog at the start
    #wdt = WDT(timeout=10000)  # 10 second timeout

    global lora_initialized, power, send_complete, send_timeout

    wdt.feed()

    blink_led(1) # Blink the LED once to indicate startup

    try:
        while True:
            data_success: bool = get_data() # Get the data from the sensors

            if not data_success:
                print("Failed to get data from sensors.")
                time.sleep(SLEEP_TIME) # Go to sleep
                reset() # Reset the device
            
            power = initialize_lora()

            wdt.feed()

            if lora_initialized:
                blink_led(2) # Blink the LED two times to indicate LoRa initialization

                if power is None:
                    power = LORA_POWER

                # Send the data to the LoRa module
                while not send_complete and not send_timeout:
                    send_data(lora_module, power)
                    power = power + 1
                    if power > LORA_MAX_POWER:
                        print("Max power reached, stopping.")
                        break                    
                if not send_complete:                    
                    print("Failed to send data to LoRa module.")
                    send_timeout = True
                    blink_led(6) # Blink the LED six times to indicate failure
                else:
                    blink_led(3) # Blink the LED three times to indicate success


            wdt.feed()
            
            print("Updating display with data...")
            update_display()

            print("Sleeping...")
            time.sleep(SLEEP_TIME)
            reset() # Reset the device

    except Exception as e:
        print("Fatal error:", e)
        # Reset on error
        reset()


if __name__ == "__main__":
    main()