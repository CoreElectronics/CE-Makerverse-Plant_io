# Test the Soil Moisture Sensor
# This script simply prints the soil moisture to the shell so you can experiment with
# the sensor placement and make sure the sensor is working properly.

from time import sleep_ms
from Plant_io import Plant_io

# Initialise the Plant_io controller
plant = Plant_io() 

while True:
    current_soil_moisture = plant.measure_soil()
    print(f'moisture {current_soil_moisture:5.2f}%')
    sleep_ms(100)