"""
test_pump.py

Run the pump for 3s
"""

from plant_io import PlantIO


def main():
    """
    Calls run_pump(3000.0) on a new PlantIO instance to run the pump
    for 3000ms or 3s
    """
    PlantIO().run_pump(3000.0)


if __name__ == "__main__":
    main()
