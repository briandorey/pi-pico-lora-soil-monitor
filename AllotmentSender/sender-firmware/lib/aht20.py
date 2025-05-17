from machine import I2C
import time

class AHT20:
    AHT20_ADDRESS = 0x38
    CMD_INITIALIZE = b'\xBE\x08\x00'
    CMD_TRIGGER_MEASUREMENT = b'\xAC\x33\x00'
    CMD_SOFT_RESET = b'\xBA'

    def __init__(self, i2c: I2C) -> None:
        self.i2c: I2C = i2c
        self.initialize_sensor()

    def initialize_sensor(self) -> None:
        """Initialize the AHT20 sensor."""
        self.i2c.writeto(self.AHT20_ADDRESS, self.CMD_INITIALIZE)
        time.sleep(0.02)  # Wait for initialization

    def soft_reset(self) -> None:
        """Perform a soft reset of the sensor."""
        self.i2c.writeto(self.AHT20_ADDRESS, self.CMD_SOFT_RESET)
        time.sleep(0.02)  # Wait for reset

    def read_raw_data(self) -> bytes:
        """Trigger a measurement and read raw data from the sensor."""
        self.i2c.writeto(self.AHT20_ADDRESS, self.CMD_TRIGGER_MEASUREMENT)
        time.sleep(0.08)  # Wait for measurement
        data: bytes = self.i2c.readfrom(self.AHT20_ADDRESS, 6)
        return data

    def get_temperature_and_humidity(self) -> tuple[float, float]:
        """Get temperature (°C) and humidity (%) from the sensor."""
        data: bytes = self.read_raw_data()

        if len(data) != 6 or (data[0] & 0x80):  # Check if data is valid
            raise RuntimeError("Sensor data is invalid or not ready")

        # Parse humidity
        humidity_raw: int = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4
        humidity: float = (humidity_raw / (1 << 20)) * 100.0

        # Parse temperature
        temperature_raw: int = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        temperature: float = ((temperature_raw / (1 << 20)) * 200.0) - 50.0

        return temperature, humidity