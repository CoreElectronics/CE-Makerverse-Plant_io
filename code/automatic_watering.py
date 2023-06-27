# This project requires the Plant_io module which handles
# - Interfacing with electronics (pump, moisture sensor)
# - Datalogging to a file
#
# This code will run simple automatic irrigation based on soil moisture

from time import sleep_ms
from Plant_io import Plant_io, DataLogger


while True:
    ### Step 1: Initialise the Plant
    plant = Plant_io()
    plant.moisture_setpoint = 32 # change this to tune how moist the growing media should be. Use the results from test_moisture_sensor.py

    ### Step 2: Collect some data to log
    soil_moisture = plant.measure_soil()
    voltage = plant.measure_system_voltage()

    ### Step 3: Run the pump if plant requires water. This function uses soil moisture to decide whether to run the pump or not.
    pump_running_seconds = plant.run_pump_control()

    # Print some debugging information
    print(f'Moisture {soil_moisture:5.2f}%    Pump Time {pump_running_seconds:5.2f}s')




    ### Step 4: Log the data to a file
    period_minutes = 20 # The chosen interval time on the Makerverse Nano Power Timer
    file_name = 'log.txt'

    # These are the labels that appear at the top of each data column in the log.txt file
    heading_time = 'Time [minutes]'
    heading_moisture = 'Moisture [%]'
    heading_pump = 'Pump Run [seconds]'
    heading_voltage = 'Supply Voltage [V]'
    data_heading = [heading_time, heading_moisture, heading_pump, heading_voltage] # The heading that will appear at the top of the log file

    logfile = DataLogger(file_name, data_heading, period_minutes) # Open the log file, and write the data_heading if the file was just created.
    timestamp = logfile.last_timestamp + period_minutes # get the most recent timestamp

    # Construct a data dictionary - dictionary keys match the data headings eg. {heading string : data to log}
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
    

