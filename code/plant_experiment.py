# This project requires the Plant_io module which handles
# - Interfacing with electronics (pump, moisture sensor, PiicoDev modules)
# - Datalogging to a file
#
# Plant_io removes a lot of setup complexity from PiicoDev but is only compatible with select modules.

from time import sleep_ms
from Plant_io import Plant_io, DataLogger


while True:
    ### Step 1: Initialise the Plant
    plant = Plant_io()
    plant.moisture_setpoint = 50 # change this to tune how moist the growing media should be. Use the results from test_moisture_sensor.py
    
    # Attach sensors. Comment-out any sensors you are not using.
    print("Initialising PiicoDev modules")
#     plant.attach('BME280')            # Atmospheric Sensor
#     plant.attach('ENS160')            # Air-Quality Sensor
#     plant.attach('VEML6040')          # Colour Sensor
#     plant.attach('VEML6030')          # Ambient Light Sensor
#     plant.attach('VL53L1X')           # Laser Distance Sensor
#     plant.attach('LIS3DH')            # 3-Axis Accelerometer
#     plant.attach('QMC6310')           # 3-Axis Magnetometer

    print("")

    ### Step 2: Collect some data to log
    soil_moisture = plant.measure_soil()
    voltage = plant.measure_system_voltage()

#     Ambient Light Sensor VEML6030
#     lux = plant.VEML6030_light()
# 
#     Atmospheric Sensor BME280
#     temperature_C, pressure_Pa, humidity_RH = plant.BME280_weather()
# 
#     Air-Quality Sensor ENS160
#     ENS160_status, AQI, TVOC, eCO2 = plant.ENS160_air_quality()
# 
#     Colour Sensor VEML6040
#     hsv = plant.VEML6040_HSV()
#     rgb = plant.VEML6040_RGB()
# 
#     Laser Distance Sensor VL53L1X
#     distance_mm = plant.VL53L1X_distance()
# 
#     3-Axis Accelerometer LIS3DH
#     acceleration = plant.LIS3DH_acceleration()
# 
#     Magnetometer QMC6310
#     plant.QMC6310_calibrate() # should only need to run once. Comment out once calibration is complete.
#     flux = plant.QMC6310_flux()
#     polar = plant.QMC6310_polar()



    ### Step 3: Run the pump if plant requires water. This function uses soil moisture to decide whether to run the pump or not.
    pump_running_seconds = plant.run_pump_control()

    # Print some debugging information
    print(f'Moisture {soil_moisture:5.2f}%    Pump Time {pump_running_seconds:5.2f}s')


    ### Step 4: Log the data to a file
    period_minutes = 20 # The chosen interval time on the Makerverse Nano Power Timer
    file_name = 'log.txt'

    # These are the labels that appear at the top of each data column in the log.txt file
    # TODO: Add your heading titles here
    heading_time = 'Time [minutes]'
    heading_moisture = 'Moisture [%]'
    heading_pump = 'Pump Run [seconds]'
    heading_voltage = 'Supply Voltage [V]'
    data_heading = [heading_time,heading_moisture, heading_pump, heading_voltage] # The heading that will appear at the top of the log file


    logfile = DataLogger(file_name, data_heading) # Open the log file, and write the data_heading if the file was just created.
    timestamp = logfile.last_timestamp + period_minutes # get the most recent timestamp

    # Construct a data dictionary - dictionary keys match the data headings eg. {heading string : data to log}
    # TODO: Add your sampled data here
    data = {heading_time        : timestamp,
            heading_moisture    : soil_moisture,
            heading_pump        : pump_running_seconds,
            heading_voltage     : voltage,
        }

    logfile.log_data(data)
    
    
    ### Step 5: Signal to the Makerverse Nano Power Timer that we are DONE!
    # This removes power from the project until the next timer interval
    plant.sleep()
    
    ### Step 6: If we are running from USB power then power will never be removed by the Nano Power Timer.
    # Instead we can just insert a delay. When powered only by batteries, this code will never run.
    sleep_ms(round(1000*60*period_minutes))
    
