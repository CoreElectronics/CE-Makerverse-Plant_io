"""
test_moisture_sensor.py

Simply print the soil moisture to the shell every 100ms so you can experiment with
the sensor placement and make sure the sensor is working properly
"""

from time import sleep_ms
from Plant_io import Plant_io


def main():
    """
    Every 100ms print the measured soil moisture
    """
    plant = Plant_io()
    while True:
        print(f"Moisture {plant.measure_soil():5.2f}%")
        sleep_ms(100)


if __name__ == "__main__":
    main()
