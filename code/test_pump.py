"""
test_pump.py

Run the pump for 3s
"""

from Plant_io import Plant_io


def main():
    """
    Calls drive_pump_for_seconds(3) on a new Plant_io instance
    """
    Plant_io().drive_pump_for_seconds(3)


if __name__ == "__main__":
    main()
