from machine import Pin, ADC, PWM, I2C
from utime import sleep
from ucollections import namedtuple
from math import nan
from os import statvfs


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


class Colour:
    """ANSI color codes"""

    BLACK = "\033[0;30m"
    LIGHT_RED = "\033[0;31m"
    LIGHT_GREEN = "\033[0;32m"
    BROWN = "\033[0;33m"
    LIGHT_BLUE = "\033[0;34m"
    PURPLE = "\033[0;35m"
    CYAN = "\033[0;36m"
    LIGHT_GRAY = "\033[0;37m"
    DARK_GRAY = "\033[1;30m"
    RED = "\033[1;31m"
    GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[1;34m"
    LIGHT_PURPLE = "\033[1;35m"
    LIGHT_CYAN = "\033[1;36m"
    LIGHT_WHITE = "\033[1;37m"
    BOLD = "\033[1m"
    FAINT = "\033[2m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"
    BLINK = "\033[5m"
    NEGATIVE = "\033[7m"
    CROSSED = "\033[9m"
    END = "\033[0m"


def print_coloured(val, colour=Colour.BLACK):
    """
    Calls print wrapped by ANSI color codes starting
    with the specified color (default black)
    """
    print(colour + str(val) + Colour.END)


class manager_funcs:
    def map_range(self, x, in_min, in_max, out_min, out_max, ret_int):
        mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        if ret_int:
            return int(max(min(mapped, out_max), out_min))
        else:
            return max(min(mapped, out_max), out_min)

    # Returns an list of
    def read_file(self, filename):
        try:
            f = open(filename, "r")
            cont = f.read()
            f.close()
            cont_lst = cont.split(",\n")
            return [item for item in cont_lst if item]  # Removes any empty list items
        except:
            print("File not found, returning 0")
            return 0

    def log_and_create_file(self, filename, val):
        with open(filename, "a") as f:
            f.write("{:.2f},\n".format(val))
        # f = open(filename, "a") # Creates or appends a file
        # f.write(str(val) + ",\n")
        # f.close()

    def file2dict(self, filename):
        read_lst = self.read_file(filename)
        dict_ret = dict()
        if read_lst == 0:  # If there was an error
            return 0
        else:
            for item in read_lst:
                str_splt = item.split(":")
                dict_ret[str_splt[0]] = str_splt[1]
            return dict_ret

    def normalise_x(self, x, x_min, x_max):
        return 100 - self.map_range(x, x_min, x_max, 0, 100, ret_int=False)

    def peristaltic_wrapper(
        self,
        val,
        debug=False,
        min_servo_duty=1000,
        max_servo_duty=8700,
        duty_deadband_min=-70,
        duty_deadband_max=50,
    ):
        ret = 0
        if (val > duty_deadband_min) and (val < duty_deadband_max):
            if debug:
                print("Stopping, Inside deadband")
            ret = 0
        else:
            ret = self.map_range(
                val, -100, 100, min_servo_duty, max_servo_duty, ret_int=True
            )
        return ret

    def last_sens_wrapper(self, sens_log_filename):
        sens_lst = self.read_file(sens_log_filename)
        if type(sens_lst) == int:
            return 0
        return float(sens_lst[-1])


class data_funcs:
    def __init__(self):
        self.mov_ave_wnd_size = 5
        self.sum_mov_ave = 0

        self.meas_window = [0] * self.mov_ave_wnd_size

    def run_mov_ave(self, function2ave, delay=0.1):
        for mov_ave_i in range(len(self.meas_window) - 1):
            self.moving_average_single_measurement(function2ave)
            sleep(delay)
        return self.moving_average_single_measurement(function2ave)

    def moving_average_single_measurement(self, measurement):
        self.sum_mov_ave -= self.meas_window.pop(0)
        self.meas_window.append(measurement)
        self.sum_mov_ave += measurement
        return int(self.sum_mov_ave / self.mov_ave_wnd_size)


class controller:
    def ctrl(x_norm, setpoint, max_time=10):
        K = 1  # control gain
        u = K * (setpoint - x_norm)

        if u <= 1:  # if the system doesnt need much adjustment
            u = 0
        u = min(u, max_time)  # just in case

        return u

    def run_pump(mf_instance, pump, duration, speed=100):
        pump.duty_u16(mf_instance.peristaltic_wrapper(speed))
        sleep(duration)
        pump.duty_u16(mf_instance.peristaltic_wrapper(0))


class Plant_io:
    def __init__(
        self, pump=12, soil=28, voltage_pin=27, moisture_setpoint=22, max_no_succ=10
    ):
        sleep(3)  # Give the sensor a chance to collect valid data
        self.done_pin = Pin(22, Pin.OUT)  # From the Makerverse Power Timer

        # Pin objects
        self.soil = ADC(soil)

        self.pump = PWM(Pin(pump))
        self.pump.freq(50)

        self.voltage_pin = ADC(voltage_pin)

        # Library links
        self.df = data_funcs()
        self.mf = manager_funcs()

        # Data files
        self.control_file = "lib/plant_sys.txt"

        # Control parameters
        self.moisture_setpoint = moisture_setpoint
        self.max_no_succ = max_no_succ
        self.u = 0

        # Run the prelim error check
        self.error_scale = 0  # Major: <-100, minor >-100
        #          self.check_sys_error_readout()

        # PiicoDev module addresses
        i2c = I2C(0, sda=Pin(8), scl=Pin(9))
        self.discovered_addresses = i2c.scan()
        self.attached_addresses = []

    def sleep(self):
        self.done_pin.value(1)
        sleep(0.25)
        self.done_pin.value(0)

    def is_address_collision(self, address, name):
        if address in self.attached_addresses:
            print_coloured(
                f"Warning: Initialising {name} failed! A device is already initialised at the address {hex(address)}"
            )
            return True
        return False

    def attach_BME280(self, asw=0):
        if not (0x77 in self.discovered_addresses or 0x76 in self.discovered_addresses):
            print_coloured(
                "    Not Attached: BME280. Skipping this device", Colour.LIGHT_RED
            )
            return
        try:
            address = 0x77 if asw == 0 else 0x76
            self.bme280 = PiicoDev_BME280(address=address)
            self.attached_addresses.append(address)
            print_coloured(
                f"    Attached: PiicoDev Atmospheric Sensor BME280 at address {hex(self.bme280.addr)}",
                Colour.LIGHT_BLUE,
            )
        except:
            pass

    def attach_ENS160(self, asw=0):
        if not (0x53 in self.discovered_addresses or 0x52 in self.discovered_addresses):
            print_coloured(
                "    Not Attached: ENS160. Skipping this device", Colour.LIGHT_RED
            )
            return
        try:
            address = 0x53 if asw == 0 else 0x52
            if self.is_address_collision(address, "ENS160"):
                return
            self.ens160 = PiicoDev_ENS160(address=address)
            self.attached_addresses.append(address)
            print_coloured(
                f"    Attached: PiicoDev Air-Quality Sensor ENS160 at address {hex(self.ens160.address)}",
                Colour.LIGHT_BLUE,
            )
        except:
            pass

    def attach_VEML6030(self, asw=0):
        if not (0x48 in self.discovered_addresses or 0x10 in self.discovered_addresses):
            print_coloured(
                "    Not Attached: VEML6030. Skipping this device", Colour.LIGHT_RED
            )
            return
        try:
            if asw == 0:
                addr = 0x10
            else:
                addr = 0x48
            if self.is_address_collision(addr, "VEML6030"):
                return
            self.veml6030 = PiicoDev_VEML6030(addr=addr)
            self.veml6030.setGain(0.125)
            self.attached_addresses.append(addr)
            print_coloured(
                f"    Attached: PiicoDev Ambient Light Sensor VEML6030 at address {hex(addr)}",
                Colour.LIGHT_BLUE,
            )
        except:
            pass

    def attach_VEML6040(self, **kwargs):
        address = 0x10
        if not (address in self.discovered_addresses):
            print_coloured(
                "    Not Attached: VEML6040. Skipping this device", Colour.LIGHT_RED
            )
            return
        try:
            if self.is_address_collision(address, "VEML6040"):
                return
            self.veml6040 = PiicoDev_VEML6040()
            self.attached_addresses.append(address)
            print_coloured(
                f"    Attached: PiicoDev Colour Sensor VEML6040 at address {hex(address)}",
                Colour.LIGHT_BLUE,
            )
        except:
            pass

    def attach_VL53L1X(self, asw=None):
        address = 0x29
        if not (address in self.discovered_addresses):
            print_coloured(
                "    Not Attached: VL53L1X. Skipping this device", Colour.LIGHT_RED
            )
            return
        try:
            if self.is_address_collision(address, "VL53L1X"):
                return
            self.vl53l1x = PiicoDev_VL53L1X()
            self.attached_addresses.append(address)
            print_coloured(
                f"    Attached: PiicoDev Laser Distance Sensor VL53L1X at address {hex(address)}",
                Colour.LIGHT_BLUE,
            )
        except:
            pass

    def attach_LIS3DH(self, asw=0):
        if not (0x19 in self.discovered_addresses or 0x18 in self.discovered_addresses):
            print_coloured(
                "    Not Attached: LIS3DH. Skipping this device", Colour.LIGHT_RED
            )
            return
        try:
            if asw == 0:
                addr = 0x19
            else:
                addr = 0x18
            if self.is_address_collision(addr, "LIS3DH"):
                return
            self.lis3dh = PiicoDev_LIS3DH(address=addr)
            self.attached_addresses.append(addr)
            print_coloured(
                f"    Attached: PiicoDev 3-Axis Accelerometer LIS3DH at address {hex(addr)}",
                Colour.LIGHT_BLUE,
            )
        except Exception as e:
            print(e)

    def attach_QMC6310(self, asw=None):
        address = 0x1C
        if not (address in self.discovered_addresses):
            print_coloured(
                "    Not Attached: QMC6310. Skipping this device", Colour.LIGHT_RED
            )
            return
        try:
            if self.is_address_collision(address, "QMC6310"):
                return
            self.qmc6310 = PiicoDev_QMC6310()
            self.attached_addresses.append(address)
            print_coloured(
                f"    Attached: PiicoDev Magnetometer QMC6310 at address {hex(address)}",
                Colour.LIGHT_BLUE,
            )
        except:
            pass

    def attach(self, part_ID, asw=0):
        candidates = {
            "VEML6030": self.attach_VEML6030,
            "BME280": self.attach_BME280,
            "ENS160": self.attach_ENS160,
            "VEML6040": self.attach_VEML6040,
            "VL53L1X": self.attach_VL53L1X,
            "LIS3DH": self.attach_LIS3DH,
            "QMC6310": self.attach_QMC6310,
        }
        try:
            initialisation_function = candidates[part_ID]
        except KeyError as e:
            print_coloured(
                '.attach() was given an unrecognised ID: "{}"'.format(part_ID),
                Colour.LIGHT_RED,
            )
            raise (e)
        initialisation_function(asw=asw)

    def VEML6030_light(self):
        if hasattr(self, "veml6030"):
            return self.veml6030.read()
        else:
            print_coloured(
                "Warning. VEML6030_light() not available: VEML6030 not initialised/connected",
                Colour.LIGHT_RED,
            )
            return nan

    def BME280_weather(self):
        if hasattr(self, "bme280"):
            return self.bme280.values()
        else:
            print_coloured(
                "Warning. BME280_weather() not available: BME280 not initialised/connected",
                Colour.LIGHT_RED,
            )
            return (nan, nan, nan)

    def ENS160_air_quality(self):
        if hasattr(self, "ens160"):
            return (
                self.ens160.operation,
                self.ens160.aqi,
                self.ens160.tvoc,
                self.ens160.eco2,
            )
        else:
            print_coloured(
                "Warning. ENS160_air_quality() not available: ENS160 not initialised/connected",
                Colour.LIGHT_RED,
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
        if hasattr(self, "veml6040"):
            return self.veml6040.readRGB()
        else:
            print_coloured(
                "Warning. VEML6040_RGB() not available: VEML6040 not initialised/connected",
                Colour.LIGHT_RED,
            )
            return {"red": nan, "green": nan, "blue": nan}

    def VEML6040_HSV(self):
        if hasattr(self, "veml6040"):
            return self.veml6040.readHSV()
        else:
            print_coloured(
                "Warning. VEML6040_HSV() not available: VEML6040 not initialised/connected",
                Colour.LIGHT_RED,
            )
            return {"hue": nan, "sat": nan, "val": nan}

    def VL53L1X_distance(self):
        if hasattr(self, "vl53l1x"):
            return self.vl53l1x.read()
        else:
            print_coloured(
                "Warning. VL53L1X_distance() not available: VL53L1X not initialised/connected",
                Colour.LIGHT_RED,
            )
            return nan

    def LIS3DH_acceleration(self):
        if hasattr(self, "lis3dh"):
            return self.lis3dh.acceleration
        else:
            print_coloured(
                "Warning. LIS3DH_acceleration() not available: LIS3DH not initialised/connected",
                Colour.LIGHT_RED,
            )
            AccelerationTuple = namedtuple("acceleration", ("x", "y", "z"))
            return AccelerationTuple(nan, nan, nan)

    def QMC6310_calibrate(self):
        if hasattr(self, "qmc6310"):
            self.qmc6310.calibrate()
        else:
            print_coloured(
                "Warning. () not available: QMC6310 not initialised/connected",
                Colour.LIGHT_RED,
            )

    def QMC6310_flux(self):
        if hasattr(self, "qmc6310"):
            return self.qmc6310.read()
        else:
            print_coloured(
                "Warning. QMC6310_flux() not available: QMC6310 not initialised/connected",
                Colour.LIGHT_RED,
            )
            return {"x": nan, "y": nan, "z": nan}

    def QMC6310_polar(self):
        if hasattr(self, "qmc6310"):
            return self.qmc6310.readPolar()
        else:
            print_coloured(
                "Warning. QMC6310_polar() not available: QMC6310 not initialised/connected",
                Colour.LIGHT_RED,
            )
            return {"polar": nan, "Gauss": nan, "uT": nan}

    #     def measure_soil(self):
    #         soil_adc_reading = self.soil.read_u16()
    #         moving_ave = self.df.run_mov_ave(soil_adc_reading)
    #         self.curr_sens = self.mf.normalise_x(moving_ave, 27500, 50000)
    #
    #         prev_sens = self.mf.last_sens_wrapper(self.log_filename)
    #         return self.curr_sens, prev_sens
    def measure_soil(self):
        for _ in range(5):
            self.soil.read_u16()
            sleep(0.05)
        soil_adc_reading = self.soil.read_u16()
        moving_ave = self.df.run_mov_ave(soil_adc_reading)
        self.curr_sens = self.mf.normalise_x(moving_ave, 27500, 50000)
        return self.curr_sens

    def run_pump_control(self, debug=False):
        self.u = controller.ctrl(self.curr_sens, self.moisture_setpoint)
        if debug:
            print("Control value: ", self.u)
        controller.run_pump(self.mf, self.pump, self.u)
        return self.u

    def measure_system_voltage(self):
        """Voltage is measured through a 0.5x voltage divider"""
        return 2 * self.voltage_pin.read_u16() * 3.3 / 65535

    @property
    def last_u_value(self):
        return self.u

    def drive_pump_for_seconds(self, duration):
        controller.run_pump(self.mf, self.pump, duration)


def get_free_space_Bytes():
    status_of_filesystem = statvfs("/")
    return status_of_filesystem[1] * status_of_filesystem[4]


class DataLogger:
    def __init__(self, filename, title_row, period=1):
        self.filename = filename
        self.title_row = title_row
        self.period = period
        self.last_timestamp = -self.period  # initialises to zero after the first sample

        # Check if the file exists or not
        try:
            with open(filename, "r") as file:
                # If the file already exists, get the last timestamp
                last_line = file.readlines()[-1]
                self.last_timestamp = float(last_line.split(",")[0])
        except:
            # If the file does not exist, write the title row
            with open(filename, "w") as file:
                file.write(",".join(self.title_row) + "\n")

    def log_data(self, data):
        data_list = [str(data[key]) for key in self.title_row]
        data_string = ",".join(data_list) + "\n"
        timestamp = str(self.last_timestamp + self.period)

        with open(self.filename, "a") as file:
            file.write(data_string)
        sleep(0.25)
        free_space_Bytes = get_free_space_Bytes()
        line_size = len(data_string)
        lines_remaining = free_space_Bytes // line_size
        print("Storage: Approx. {} Lines remaining".format(lines_remaining))
