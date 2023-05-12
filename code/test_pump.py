# Test the Peristaltic Pump
# This script simply prints the pump for 3 seconds

from Plant_io import Plant_io

# Initialise the Plant_io controller (pump)
plant = Plant_io()

# Run the pump for a short period
plant.drive_pump_for_seconds(3)

