"""
test_moisture_sensor.py

Simply print the soil moisture to the shell every 100ms so you can experiment with
the sensor placement and make sure the sensor is working properly
"""

from os import uname
from plant_io import PlantIO

if uname().sysname == "Linux":
    from time import sleep

    def sleep_ms(duration_ms):
        """
        Block for duration_ms milliseconds
        """
        sleep(float(duration_ms) / 1000.0)

else:
    from utime import sleep_ms


def main():
    """
    Every 100ms print the measured soil moisture
    """
    plantIO = PlantIO()
    while True:
        print(f"Moisture {plantIO.measure_soil():5.2f}%")
        sleep_ms(100)


if __name__ == "__main__":
    main()
