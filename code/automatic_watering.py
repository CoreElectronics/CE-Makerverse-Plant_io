"""
automatic_watering.py

Requires PlantIO.py module for interfacing with the pump,
moisture sensor and performing file datalogging

This module performs simple automatic irrigation based on
soil moisture
"""

from os import uname
from plant_io import PlantIO, DataLogger

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
    Every 20-minutes read and print and log the soil moisture, pump time, and voltage
    """

    PERIOD_MINUTES = 20  # The chosen interval time on the Makerverse Nano Power Timer
    PERIOD_MS = round(1000 * 60 * PERIOD_MINUTES)
    LOG_FILENAME = "log.csv"

    HEADING = {
        "TIME": "Time [minutes]",
        "MOISTURE": "Moisture [%]",
        "PUMP": "Pump Run [seconds]",
        "VOLTAGE": "Supply Voltage [V]",
    }

    # Step 1: Initialise the logfile and Plant
    logfile = DataLogger(
        filename=LOG_FILENAME, title_row=list(HEADING.values())
    )  # Open the log file, this will only write the heading if the file was just created

    plantIO = PlantIO()

    # Change this to tune how moist the growing media should be.
    # Use the results from test_moisture_sensor.py
    plantIO.moisture_setpoint = 32

    while True:
        # Step 2: Collect some data to log

        soil_moisture = plantIO.measure_soil()
        voltage = plantIO.measure_system_voltage()

        # Step 3: Run the pump if plant requires water.
        # This function uses soil moisture to decide whether to run the pump or not.

        pump_running_seconds = plantIO.run_pump_control()

        # Step 4: Print, then log the data to a file

        print(f"Moisture {soil_moisture:5.2f}%")
        print(f"Pump Time {pump_running_seconds:5.2f}s")

        logfile.log_data(
            {
                HEADING["TIME"]: logfile.last_timestamp + PERIOD_MINUTES,
                HEADING["MOISTURE"]: soil_moisture,
                HEADING["PUMP"]: pump_running_seconds,
                HEADING["VOLTAGE"]: voltage,
            }
        )

        # Step 5: Signal to the Makerverse Nano Power Timer that we are DONE!
        # This removes power from the project until the next timer interval
        # unless USB power is supplied
        plantIO.sleep()

        # Step 6: If we are running from USB power then power will never be removed
        # by the Nano Power Timer. Instead we can just insert a delay. When powered
        # only by batteries, this code will never run.
        sleep_ms(PERIOD_MS)


if __name__ == "__main__":
    main()
