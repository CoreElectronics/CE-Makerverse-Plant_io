"""
plant_io.py

TODO: Describe plant_io
"""

from math import nan
from os import statvfs, uname
from ucollections import namedtuple
from machine import Pin, ADC, PWM, I2C
from ansi_colours import Colours, colour_print

if uname().sysname == "Linux":
    from time import sleep

    def sleep_ms(duration_ms):
        """
        Block for duration_ms milliseconds
        """
        sleep(float(duration_ms) / 1000.0)

else:
    from utime import sleep_ms

try:
    from PiicoDev_BME280 import PiicoDev_BME280
except ImportError:
    pass
try:
    from PiicoDev_VEML6030 import PiicoDev_VEML6030
except ImportError:
    pass
try:
    from PiicoDev_ENS160 import PiicoDev_ENS160
except ImportError:
    pass
try:
    from PiicoDev_VEML6040 import PiicoDev_VEML6040
except ImportError:
    pass
try:
    from PiicoDev_VL53L1X import PiicoDev_VL53L1X
except ImportError:
    pass
try:
    from PiicoDev_LIS3DH import PiicoDev_LIS3DH
except ImportError:
    pass
try:
    from PiicoDev_QMC6310 import PiicoDev_QMC6310
except ImportError:
    pass


def map_range(
    x: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    """
    TODO: Describe the behaviour of the map_range function
    """
    return max(
        min(
            (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min,
            out_max,
        ),
        out_min,
    )


def get_csv_lines(filename: str) -> list(str):
    """
    Reads a CSV file at filename and returns a list
    of strings if the file exists, or returns an empty list
    """
    try:
        with open(filename, "r", encoding="utf-8") as file:
            return [line for line in file.read().split(",\n") if line]
    except OSError:
        colour_print(f"{filename} not found!", Colours.RED)
        return []


def append_csv_value(filename: str, value: any):
    """
    Appends a value with a comma and newline
    to the end of the filename specified, creating
    the file if it doesn't exist
    """
    with open(filename, "a", encoding="utf-8") as file:
        file.write(f"{value:.2f},\n")


def csv_lines_to_dict(filename: str) -> dict:
    """
    Wrapper class to get_csv_lines which automatically
    converts the CSV into a dict based on : values for
    each line
    """
    lines = get_csv_lines(filename)
    if not lines:
        return {}
    return {line.split(":")[0]: line.split(":")[1] for line in lines}


def normalised(value: float, value_min: float, value_max: float) -> float:
    """
    Wrapper class to map_range normalising the
    value with a custom function based on min-max values
    """
    return 100 - map_range(value, value_min, value_max, 0, 100)


def peristaltic_wrapper(
    value: float,
    debug=False,
    min_servo_duty=1000.0,
    max_servo_duty=8700.0,
    duty_deadband_min=-70.0,
    duty_deadband_max=50.0,
) -> int:
    """
    Validates value for the peristaltic motor based on the map_range function
    """
    if duty_deadband_min < value < duty_deadband_max:
        if debug:
            colour_print("Stopping! Inside deadband!", Colours.RED)
        return 0
    return int(map_range(value, -100, 100, min_servo_duty, max_servo_duty))


def last_line_of_logfile(log_filename: str) -> str:
    """
    Returns the last line of the logfile or an empty string
    """
    lines = get_csv_lines(log_filename)
    if not lines:
        return ""
    return float(lines[-1])


class DataModel:
    """
    A custom class which maintains an Moving Average Window to smooth values
    """

    def __init__(self):
        self.size = 5
        self.moving_average_sum = 0
        self.measurement_window = [0] * self.size

    def moving_average_window_measurements(
        self, callable_measurement_function, delay_ms=100
    ):
        """
        Flushes the moving average window by making the measurement multiple times ,
        note that you should be passing the callable function as a parameter,
        not the measured value.
        """
        for _ in range(len(self.measurement_window) - 1):
            self.moving_average_single_measurement(callable_measurement_function())
            sleep_ms(delay_ms)
        return self.moving_average_single_measurement(callable_measurement_function())

    def moving_average_single_measurement(self, callable_measurement_function):
        """
        Makes a single measurement pushing it into the moving average window
        note that you should be passing the callable function as a parameter,
        not the measured value
        """
        measurement = callable_measurement_function()
        self.moving_average_sum -= self.measurement_window.pop(0)
        self.measurement_window.append(measurement)
        self.moving_average_sum += measurement
        return int(self.moving_average_sum / self.size)


def pump_ctrl(
    x_norm: float, set_point: float, control_gain=1.0, max_time=10.0
) -> float:
    """
    TODO: Describe the behaviour of this controller method
    """
    u = control_gain * (set_point - x_norm)
    # If the system doesn't require much adjustment (<=1) don't make any
    return 0.0 if u <= 1 else min(u, max_time)


def pump_run(pump, duration_ms, speed=100):
    """
    Attempts to run the pump at the specified
    speed for the provided duration in milliseconds
    """
    pump.duty_u16(peristaltic_wrapper(speed))
    sleep_ms(int(duration_ms))
    pump.duty_u16(peristaltic_wrapper(0))


class PlantIO:
    """
    This is the main class for the Plant_io project.
    This class is responsible for the management of data logging,
    sensor reading, and pump control
    """

    def __init__(self, pump=12, soil=28, voltage_pin=27, moisture_setpoint=22):
        # Give the sensor a chance to collect valid data
        sleep_ms(3000)

        # Pin objects
        i2c = I2C(0, sda=Pin(8), scl=Pin(9))
        self.soil = ADC(soil)
        self.voltage_pin = ADC(voltage_pin)
        self.pump = PWM(Pin(pump))
        self.pump.freq(50)
        self.power_timer_done_pin = Pin(22, Pin.OUT)  # Makerverse Power Timer

        # Instantiate a new empty DataModel to take measurements
        # against a moving average window auto-updating on
        # each measurement
        self.data_model = DataModel()

        # Data files
        self.control_file = "lib/plant_sys.txt"

        # Control parameters
        self.moisture_setpoint = moisture_setpoint
        self.u = 0

        self.current_soil_reading = 0

        self.discovered_addresses = i2c.scan()
        self.attached_addresses = []
        self.attached_modules = {}

    def sleep(self):
        """
        Turns on the power_timer_done_pin for 250ms
        """
        self.power_timer_done_pin.value(1)
        sleep_ms(250)
        self.power_timer_done_pin.value(0)

    def is_address_collision(self, address: int):
        """
        Returns True when the address already exists in attached_addresses
        """
        return address in self.attached_addresses

    def attach_piicodev_module(self, piicodev_module: str, asw=0):
        """
        Provided a module and the value of the address switch as an integer
        (defaults 0), this will attempt to connect the module to this instance
        of PlantIO based on the default addresses
        """
        PIICODEV_MODULES = {
            "VEML6030": {
                "name": "PiicoDev Ambient Light Sensor",
                "asw": {0: 0x10, 1: 0x48},
            },
            "BME280": {
                "name": "PiicoDev Atmospheric Sensor",
                "asw": {0: 0x77, 1: 0x76},
            },
            "ENS160": {
                "name": "PiicoDev Air-Quality Sensor",
                "asw": {0: 0x53, 1: 0x52},
            },
            "VEML6040": {
                "name": "PiicoDev Colour Sensor",
                "asw": {0: 0x10},
            },
            "VL53L1X": {
                "name": "PiicoDev Laser Distance Sensor",
                "asw": {0: 0x29},
            },
            "LIS3DH": {
                "name": "PiicoDev 3-Axis Accelerometer",
                "asw": {0: 0x19, 1: 0x18},
            },
            "QMC6310": {
                "name": "PiicoDev Magnetometer",
                "asw": {0: 0x1C},
            },
        }
        module_dict = PIICODEV_MODULES[piicodev_module]
        name = module_dict["name"]
        address = module_dict["asw"][asw]
        if self.is_address_collision(address=address):
            colour_print(
                f"""Warning: Initialising {name} failed!
A device is already initialised at the address {hex(address)}""",
                Colours.BOLD,
                Colours.RED,
            )
            return
        elif not address in self.discovered_addresses:
            colour_print(
                f"    Not Attached: {name} {piicodev_module}. Skipping this device",
                Colours.LIGHT_RED,
            )
        else:
            self.attached_addresses.append(address)

            new_module = None

            if piicodev_module == "VEML6030":
                new_module = PiicoDev_VEML6030(addr=address)
                new_module.setGain(0.125)
            elif piicodev_module == "BME280":
                new_module = PiicoDev_BME280(address=address)
            elif piicodev_module == "ENS160":
                new_module = PiicoDev_ENS160(address=address)
            elif piicodev_module == "VEML6040":
                new_module = PiicoDev_VEML6040()
            elif piicodev_module == "VL53L1X":
                new_module = PiicoDev_VL53L1X()
            elif piicodev_module == "LIS3DH":
                new_module = PiicoDev_LIS3DH(address=address)
            elif piicodev_module == "QMC6310":
                new_module = PiicoDev_QMC6310()

            self.attached_modules[piicodev_module] = new_module

            colour_print(
                f"    Attached: {name} {piicodev_module} at address {hex(address)}",
                Colours.LIGHT_BLUE,
            )

    def is_module_attached(self, piicodev_module: str):
        """
        Returns true if a module has been attached to this PlantIO instance
        """
        return piicodev_module in self.attached_modules

    def VEML6030_light(self):
        """
        Returns the read() method from the attached VEML6030 if it exists,
        otherwise returns math.nan
        """
        if self.is_module_attached("VEML6030"):
            return self.attached_modules["VEML6030"].read()
        colour_print(
            "Warning. VEML6030_light() not available: VEML6030 not initialised/connected",
            Colours.LIGHT_RED,
        )
        return nan

    def BME280_weather(self):
        """
        Returns the values() method from the attached BME280 if it exists,
        otherwise returns (math.nan, math.nan, math.nan)
        """
        if self.is_module_attached("BME280"):
            return self.attached_modules["BME280"].values()
        colour_print(
            "Warning. BME280_weather() not available: BME280 not initialised/connected",
            Colours.LIGHT_RED,
        )
        return (nan, nan, nan)

    def ENS160_air_quality(self):
        """
        Returns the air quality metrics if the module exists
        """
        if self.is_module_attached("ENS160"):
            return (
                self.attached_modules["ENS160"].operation,
                self.attached_modules["ENS160"].aqi,
                self.attached_modules["ENS160"].tvoc,
                self.attached_modules["ENS160"].eco2,
            )

        colour_print(
            "Warning. ENS160_air_quality() not available: ENS160 not initialised/connected",
            Colours.LIGHT_RED,
        )
        AQI_Tuple = namedtuple("AQI", ("value", "rating"))
        ECO2_Tuple = namedtuple("eCO2", ("value", "rating"))
        return (
            "unknown",
            AQI_Tuple(nan, "unknown"),
            nan,
            ECO2_Tuple(nan, "unknown"),
        )

    def VEML6040_RGB(self):
        """
        Returns the RGB values from the connected sensor if it exists
        """
        if self.is_module_attached("VEML6040"):
            return self.attached_modules["VEML6040"].readRGB()
        colour_print(
            "Warning. VEML6040_RGB() not available: VEML6040 not initialised/connected",
            Colours.LIGHT_RED,
        )
        return {"red": nan, "green": nan, "blue": nan}

    def VEML6040_HSV(self):
        """
        Returns the HSV values from the connected sensor if it exists
        """
        if self.is_module_attached("VEML6040"):
            return self.attached_modules["VEML6040"].readHSV()
        colour_print(
            "Warning. VEML6040_HSV() not available: VEML6040 not initialised/connected",
            Colours.LIGHT_RED,
        )
        return {"hue": nan, "sat": nan, "val": nan}

    def VL53L1X_distance(self):
        """
        Returns the measured distance from the connected sensor if it exists
        """
        if self.is_module_attached("VL53L1X"):
            return self.attached_modules["VL53L1X"].read()
        colour_print(
            "Warning. VL53L1X_distance() not available: VL53L1X not initialised/connected",
            Colours.LIGHT_RED,
        )
        return nan

    def LIS3DH_acceleration(self):
        """
        Returns the acceleration in m/s/s for each axis from the connected sensor if it exists
        """
        if self.is_module_attached("LIS3DH"):
            return self.attached_modules["LIS3DH"].acceleration
        colour_print(
            "Warning. LIS3DH_acceleration() not available: LIS3DH not initialised/connected",
            Colours.LIGHT_RED,
        )
        AccelerationTuple = namedtuple("acceleration", ("x", "y", "z"))
        return AccelerationTuple(nan, nan, nan)

    def QMC6310_calibrate(self):
        """
        Calibrates the QMC6310 if it is connected
        """
        if self.is_module_attached("QMC6310"):
            self.attached_modules["QMC6310"].calibrate()
        else:
            colour_print(
                "Warning. QMC6310_calibrate() not available: QMC6310 not initialised/connected",
                Colours.LIGHT_RED,
            )

    def QMC6310_flux(self):
        """
        Returns the magnetic flux on each axis of the QMC6310 if it is connected
        """
        if self.is_module_attached("QMC6310"):
            return self.attached_modules["QMC6310"].read()
        colour_print(
            "Warning. QMC6310_flux() not available: QMC6310 not initialised/connected",
            Colours.LIGHT_RED,
        )
        return {"x": nan, "y": nan, "z": nan}

    def QMC6310_polar(self):
        """
        Returns the magnetic flux from the QMC6310 if it is connected as a dict
        of polar, Gauss, and micro-telsas uT
        """
        if self.is_module_attached("QMC6310"):
            return self.attached_modules["QMC6310"].readPolar()
        colour_print(
            "Warning. QMC6310_polar() not available: QMC6310 not initialised/connected",
            Colours.LIGHT_RED,
        )
        return {"polar": nan, "Gauss": nan, "uT": nan}

    def measure_soil(self):
        """
        Makes a single measurement against the ADC connected to the soil
        moisture sensor pushing it into the moving average window in the
        data_model
        """
        self.current_soil_reading = normalised(
            self.data_model.moving_average_single_measurement(self.soil.read_u16),
            27500.0,
            50000.0,
        )
        return self.current_soil_reading

    def run_pump_control(self, debug=False):
        """
        Runs and sets u using the pump_ctrl method and runs the pump
        """
        self.u = pump_ctrl(
            x_norm=self.current_soil_reading, set_point=self.moisture_setpoint
        )
        if debug:
            colour_print(f"Control value: {self.u}")
        pump_run(self.pump, self.u)
        return self.u

    def measure_system_voltage(self):
        """Voltage is measured through a 0.5x voltage divider"""
        return 2 * self.voltage_pin.read_u16() * 3.3 / 65535

    def run_pump(self, duration_ms: float):
        """
        Calls the pump_run method on the connected pump
        """
        pump_run(self.pump, duration_ms)

    @property
    def last_u_value(self):
        """
        Getter property for the u value of this instance
        """
        return self.u


def get_free_bytes():
    """
    Returns the free bytes in the filesystem from the root directory /
    """
    status_of_filesystem = statvfs("/")
    return status_of_filesystem[1] * status_of_filesystem[4]


class DataLogger:
    """
    This class is responsible for the writing and interacting
    with the data logging file
    """

    def __init__(self, filename, title_row):
        self.filename = filename
        self.title_row = title_row
        self.last_timestamp = 0.0
        try:
            with open(filename, "r", encoding="utf-8") as file:
                last_line = file.readlines()[-1]
                self.last_timestamp = float(last_line.split(",")[0])
        except OSError:
            with open(filename, "w", encoding="utf-8") as file:
                file.write(",".join(self.title_row) + "\n")
                self.last_timestamp = 0.0

    def log_data(self, data: dict, debug=False):
        """
        Appends to the initialised file (or creates the file) with a new
        line of CSVs (comma seperated values)
        """
        line = ",".join([str(data[key]) for key in self.title_row]) + "\n"
        with open(self.filename, "a", encoding="utf-8") as file:
            file.write(line)
        if debug:
            colour_print(
                f"Storage: Approx. {get_free_bytes() // len(line)} Lines remaining",
                Colours.UNDERLINE,
                Colours.GREEN,
            )
