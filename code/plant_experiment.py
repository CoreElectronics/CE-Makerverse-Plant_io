"""
plant_experiment.py

Requires PlantIO.py module for interfacing with the pump,
moisture sensor, and performing file datalogging

This module performs simple automatic irrigation based on
soil moisture, and provides logging of any of the other sensors
configured in this file such as the:

Atmospheric Sensor
Air-Quality Sensor
Colour Sensor
Ambient Light Sensor
Laser Distance Sensor
3-Axis Accelerometer
3-Axis Magnetometer

PlantIO removes a lot of setup complexity from PiicoDev
but is only compatible with select modules.
"""

from time import sleep_ms
from plant_io import PlantIO, DataLogger


def main():
    """
    Every 20-minutes read and print and log the soil moisture, pump time,
    and voltage, as well as any other configured sensors, then
    print and log this data
    """

    # Valid options to add:
    # 'BME280' - Atmospheric Sensor
    # 'ENS160' - Air-Quality Sensor,
    # 'VEML6040' - Colour Sensor
    # 'VEML6030' - Ambient Light Sensor
    # 'VL53L1X' - Laser Distance Sensor
    # 'LIS3DH' -  3-Axis Accelerometer
    #  'QMC6310' - 3-Axis Magnetometer
    CONNECTED_SENSORS = []

    PERIOD_MINUTES = 20  # The chosen interval time on the Makerverse Nano Power Timer
    PERIOD_MS = round(1000 * 60 * PERIOD_MINUTES)
    LOG_FILENAME = "log.csv"

    HEADING = {
        "TIME": "Time [minutes]",
        "MOISTURE": "Moisture [%]",
        "PUMP": "Pump Run [seconds]",
        "VOLTAGE": "Supply Voltage [V]",
    }

    logfile = DataLogger(
        filename=LOG_FILENAME, title_row=(list(HEADING.values()) + CONNECTED_SENSORS)
    )  # Open the log file, this will only write the heading if the file was just created

    plantIO = PlantIO()

    # Change this to tune how moist the growing media should be.
    # Use the results from test_moisture_sensor.py
    plantIO.moisture_setpoint = 32

    if CONNECTED_SENSORS:
        print("Initialising PiicoDev modules")
        for CONNECTED_SENSOR in CONNECTED_SENSORS:
            plantIO.attach_piicodev_module(CONNECTED_SENSOR)
        print("")

    while True:
        data = {HEADING["TIME"]: logfile.last_timestamp + PERIOD_MINUTES}

        soil_moisture = plantIO.measure_soil()
        print(f"Moisture {soil_moisture:5.2f}%", end="")
        data[HEADING["MOISTURE"]] = soil_moisture

        voltage = plantIO.measure_system_voltage()
        print(f"    Voltage {voltage:5.2f}%", end="")
        data[HEADING["VOLTAGE"]] = voltage

        pump_running_seconds = plantIO.run_pump_control()
        print(f"    Pump Time {pump_running_seconds:5.2f}s", end="")
        data[HEADING["PUMP"]] = pump_running_seconds

        if "VEML6030" in CONNECTED_SENSORS:
            lux = plantIO.VEML6030_light()
            print(f"    Light Level {lux:5.2f} lux")
            data["Light Level [lux]"] = lux

        if "BME280" in CONNECTED_SENSORS:
            temperature_c, pressure_pa, humidity_rh = plantIO.BME280_weather()
            print(f"    Temperature {temperature_c:5.2f} Celsius")
            data["Temperature [C]"] = temperature_c
            print(f"    Pressure {pressure_pa:5.2f} Pascals")
            data["Pressure [Pa]"] = pressure_pa
            print(f"    Humidity {humidity_rh:5.2f} Relative Humidity")
            data["Humidity [RH]"] = humidity_rh

        if "ENS160" in CONNECTED_SENSORS:
            _, aqi, tvoc, eco2 = plantIO.ENS160_air_quality()
            print(f"    AQI {aqi:5.2f}")
            data["AQI"] = aqi
            print(f"    TVOC {tvoc:5.2f} ppb")
            data["TVOC [ppb]"] = tvoc
            print(f"    eCO2 {eco2:5.2f} ppm")
            data["eCO2 [ppm]"] = eco2

        if "VEML6040" in CONNECTED_SENSORS:
            hsv = plantIO.VEML6040_HSV()
            rgb = plantIO.VEML6040_RGB()
            print(f"    HSV {hsv}")
            data["HSV"] = hsv
            print(f"    RGB {rgb}")
            data["RGB"] = rgb

        if "VL53L1X" in CONNECTED_SENSORS:
            distance_mm = plantIO.VL53L1X_distance()
            print(f"    Distance {distance_mm:5.2f} mm")
            data["Distance [mm]"] = distance_mm

        if "LIS3DH" in CONNECTED_SENSORS:
            acceleration = plantIO.LIS3DH_acceleration()
            print(f"    Acceleration {acceleration} m/s/s")
            data["Acceleration [m/s/s]"] = acceleration

        if "QMC6310" in CONNECTED_SENSORS:
            # Should only need to run once. Can be commented out once calibration is complete.
            plantIO.QMC6310_calibrate()
            flux = plantIO.QMC6310_flux()
            polar = plantIO.QMC6310_polar()
            print(f"    Flux {flux}")
            data["Flux"] = flux
            print(f"    Polar {polar}")
            data["Polar"] = polar

        logfile.log_data(data)

        # Signals to the Makerverse Nano Power Timer that we are DONE!
        # This removes power from the project until the next timer interval
        # unless USB power is supplied
        plantIO.sleep()

        # If we are running from USB power then power will never be removed
        # by the Nano Power Timer. Instead we can just insert a delay. When powered
        # only by batteries, this code will never run.
        sleep_ms(PERIOD_MS)


if __name__ == "__main__":
    main()
