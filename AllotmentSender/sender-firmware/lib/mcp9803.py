import machine
import struct

class MCP9803:
    """
    Class to interface with the MCP9803 temperature sensor.
    This class provides methods to read the temperature and configure the sensor.
    """
    # Constants for the MCP9803

    # Internal Variables
    __i2c = machine.I2C(0, sda=20, scl=21, freq=100000)
    __address = 0x48

    def __init__(self, address=0x48, sda=20, scl=21, freq=100000, i2cbus=0):
        """
        Initialize the MCP9803 sensor.
        
        :param address: I2C address of the MCP9803 sensor (default is 0x48).
        :param sda: SDA pin number (default is GPIO20).
        :param scl: SCL pin number (default is GPIO21).
        :param freq: I2C frequency (default is 100000 Hz).
        :param i2cbus: I2C bus number (default is 0).
        """
        if not (0 <= sda <= 40):
            raise ValueError("SDA pin must be between 0 and 40.")
        if not (0 <= scl <= 40):
            raise ValueError("SCL pin must be between 0 and 40.")
        if not (0 <= address <= 0x7F):
            raise ValueError("I2C address must be between 0x00 and 0x7F.")

        sda_pin = machine.Pin(sda)
        scl_pin = machine.Pin(scl)

        try:
            self.__i2c = machine.I2C(i2cbus, sda=sda_pin, scl=scl_pin, freq=freq)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize I2C: {e}")

        self.__address = address

    def read_temperature(self):
        """
        Read the temperature from the MCP9803 sensor.
        
        :return: The temperature in Celsius as a float.
        """
        try:
            # Read two bytes from the temperature register (0x00)
            temp_data = self.__i2c.readfrom_mem(self.__address, 0x00, 2)
        except Exception as e:
            raise RuntimeError(f"Failed to read temperature: {e}")

        # Convert the data to a temperature value
        temp_raw = struct.unpack('>h', temp_data)[0] >> 4
        if temp_raw & 0x800:  # Check if the temperature is negative
            temp_raw -= 1 << 12
        return temp_raw * 0.0625

    def set_alert(self, alert_temp, hysteresis_temp):
        """
        Set the alert temperature and hysteresis temperature.
        
        :param alert_temp: The temperature at which the alert is triggered (in Celsius).
        :param hysteresis_temp: The hysteresis temperature (in Celsius).
        """
        if not (-55 <= alert_temp <= 125):
            raise ValueError("Alert temperature must be between -55 and 125 Celsius.")
        if not (-55 <= hysteresis_temp <= 125):
            raise ValueError("Hysteresis temperature must be between -55 and 125 Celsius.")

        try:
            # Convert temperatures to raw values
            alert_raw = int(alert_temp / 0.0625) << 4
            hysteresis_raw = int(hysteresis_temp / 0.0625) << 4

            # Write to the TSET (alert temperature) register (0x02)
            self.__i2c.writeto_mem(self.__address, 0x02, struct.pack('>h', alert_raw))
            # Write to the THYST (hysteresis temperature) register (0x03)
            self.__i2c.writeto_mem(self.__address, 0x03, struct.pack('>h', hysteresis_raw))
        except Exception as e:
            raise RuntimeError(f"Failed to set alert temperatures: {e}")

    def configure(self, config):
        """
        Configure the MCP9803 sensor.
        
        :param config: A 16-bit configuration value.
        """
        if not (0 <= config <= 0xFFFF):
            raise ValueError("Configuration value must be a 16-bit integer.")

        try:
            # Write the configuration to the configuration register (0x01)
            self.__i2c.writeto_mem(self.__address, 0x01, struct.pack('>h', config))
        except Exception as e:
            raise RuntimeError(f"Failed to configure the sensor: {e}")