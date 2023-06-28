"""
test_pump.py

Run the pump for 3s
"""

from plant_io import PlantIO


def main():
    """
    Calls drive_pump_for_seconds(3) on a new PlantIO instance
    """
    PlantIO().drive_pump_for_seconds(3)


if __name__ == "__main__":
    main()
