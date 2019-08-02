import os
import time
import math
import spidev

from RPi import GPIO
from typing import Tuple, List
from smbus2 import SMBusWrapper
from abc import abstractmethod, ABC


class Interrupt(ABC):
    @abstractmethod
    def wait_for(self, timeout: int) -> bool:
        """Returns True if the interrupt happened and false
        if timeout milliseconds passed without an interrupt"""
        pass

    @abstractmethod
    def close(self):
        """Releases any resources held by the interrupt"""
        pass


class AbstractTransport(ABC):
    @abstractmethod
    def close(self):
        """Releases any resources held by the transport"""
        pass

    @abstractmethod
    def write_byte(self, address: int, value: int) -> int:
        """Writes a single byte to the given address
        :param address: the address to write to
        :param value: the byte to write
        """
        pass

    @abstractmethod
    def read_byte(self, address: int) -> int:
        """Reads a single byte
        :param address: the address to read
        """
        pass

    @abstractmethod
    def read_bytes(self, address: int, length: int) -> List[int]:
        """
        Reads 'length' bytes starting at 'address'
        :param address: the address to read
        :param length: number of bytes to read
        """
        pass

    @abstractmethod
    def data_ready(self, timeout: int) -> bool:
        """Waits for data to be ready."""
        pass


class AGStatus:
    def __init__(self, status: int):
        self.status = status

    @property
    def accelerometer_interrupt(self) -> bool:
        return (self.status & 0x40) != 0

    @property
    def gyroscope_interrupt(self) -> bool:
        return (self.status & 0x20) != 0

    @property
    def inactivity_interrupt(self) -> bool:
        return (self.status & 0x10) != 0

    @property
    def boot_status(self) -> bool:
        return (self.status & 0x08) != 0

    @property
    def temperature_data_available(self) -> bool:
        return (self.status & 0x04) != 0

    @property
    def gyroscope_data_available(self) -> bool:
        return (self.status & 0x02) != 0

    @property
    def accelerometer_data_available(self) -> bool:
        return (self.status & 0x01) != 0


class MagnetometerStatus:
    def __init__(self, status: int):
        self.status = status

    @property
    def overrun(self) -> bool:
        """data overrun on all axes"""
        return (self.status & 0x80) != 0

    @property
    def z_overrun(self) -> bool:
        """Z axis data overrun"""
        return (self.status & 0x40) != 0

    @property
    def y_overrun(self) -> bool:
        """Y axis data overrun"""
        return (self.status & 0x20) != 0

    @property
    def x_overrun(self) -> bool:
        """X axis data overrun"""
        return (self.status & 0x10) != 0

    @property
    def data_available(self) -> bool:
        """There's new data available for all axes."""
        return (self.status & 0x08) != 0

    @property
    def z_axis_data_available(self) -> bool:
        return (self.status & 0x04) != 0

    @property
    def y_axis_data_available(self) -> bool:
        return (self.status & 0x02) != 0

    @property
    def x_axis_data_available(self) -> bool:
        return (self.status & 0x01) != 0


class Driver:
    AG_ID = 0b01101000
    MAG_ID = 0b00111101

    # from LSM9DS1_Datasheet.pdf
    ACC_SENSOR_SCALE = 0.061 / 1000.0
    GAUSS_SENSOR_SCALE = 0.14 / 1000.0
    DPS_SENSOR_SCALE = 8.75 / 1000.0
    TEMP_SENSOR_SCALE = 59.5 / 1000.0
    TEMPC_0 = 25

    def __init__(self, ag_protocol: AbstractTransport, magnetometer_protocol: AbstractTransport,
                 high_priority: bool = False):
        self.ag = ag_protocol
        self.mag = magnetometer_protocol
        # Needs to be a high priority process or it'll drop samples
        # when other processes are under heavy load.
        if high_priority:
            priority = os.sched_get_priority_max(os.SCHED_FIFO)
            param = os.sched_param(priority)
            os.sched_setscheduler(0, os.SCHED_FIFO, param)

    def configure(self):
        """Resets the device and configures it"""
        ###################################################
        #   - Bit 0 - SW_RESET - software reset for accelerometer and gyro - default 0
        #   - Bit 2 - IF_ADD_INC - automatic register increment for multibyte access - default 1
        #   - Bit 6 - BDU - Block data update . Ensures high and low bytes come from
        #             the same sample. Not necessary if waiting for data ready - default 0
        self.ag.write_byte(Register.CTRL_REG8, 0x05)
        # 0x08 - reboot magnetometer, +/- 4 Gauss full scale - fixes occasional magnetometer hang
        # 0x04 - soft reset magnetometer, +/- 4 Gauss full scale
        self.mag.write_byte(Register.CTRL_REG2_M, 0x08)
        time.sleep(0.01)    # Wait for reset
        ###################################################
        # Confirm that we're connected to the device
        if self.ag.read_byte(Register.WHO_AM_I) != Driver.AG_ID:
            raise RuntimeError('Could not find LSM9DS1 Acceleromter/Gyro. Check wiring and port numbers.')
        if self.mag.read_byte(Register.WHO_AM_I_M) != Driver.MAG_ID:
            raise RuntimeError('Could not find LSM9DS1 Magnetometer. Check wiring and port numbers.')
        ###################################################
        # Set up output data rate for Accelerometer and Gyro if using both
        # Use CTRL_REG2_G and CTRL_REG3_G to control the optional additional filters
        # 0x6A - 500 dps, 119 Hz ODR, 38Hz cut off (31 Hz Cut off if HP filter is enabled)
        # 0x8A - 500 dps, 238 Hz ODR, 76Hz cut off (63 Hz Cut off if HP filter is enabled)
        # 0xAA - 500 dps, 476 Hz ODR, 100Hz cut off (57 Hz Cut off if HP filter is enabled)
        # 0x00 - disabled
        self.ag.write_byte(Register.CTRL_REG1_G, 0x8A)
        # 0x03 - Enable LPF2 - Frequency set by REG1 (2nd Low Pass Filter)
        # self.ag.write_byte(Register.CTRL_REG2_G, 0x03)
        # 0x45   - Enable High Pass Filter at (0.2 Hz @ 119 ODR) or (0.5 Hz @ 238 ODR)
        # self.ag.write_byte(Register.CTRL_REG3_G, 0x45)
        ###################################################
        # Set up Accelerometer
        # 0xC0 - Set to +- 2G, 119 Hz ODR, 50 Hz BW (Frequency is ignored if Gryo is enabled)
        # 0x87 - Set to +- 2G, 238 Hz ODR, 50 Hz BW (Frequency is ignored if Gryo is enabled)
        self.ag.write_byte(Register.CTRL_REG6_XL, 0x87)
        # 0x01 INT1_A/G pin set by accelerometer data ready
        self.ag.write_byte(Register.INT1_CTRL, 0x01)
        ###################################################
        # Set up magnetometer
        # MSB enables temperature compensation
        # 0x98 - 40 Hz ODR, Enable Temp Comp
        # 0x9C - 80 Hz ODR, Enable Temp Comp
        # 0x18 - 40 Hz ODR, Disable Temp Comp
        # 0x1C - 80 Hz ODR, Enable Temp Comp
        # 0xE2 - 155 Hz ODR, Enable Temp Comp, x and y ultra high performance
        # 0xC2 - 300 Hz ODR, Enable Temp Comp, x and y high performance
        # 0xA2 - 560 Hz ODR, Enable Temp Comp, x and y medium performance
        # 0x82 - 1000 Hz ODR, Enable Temp Comp, x and y low performance
        self.mag.write_byte(Register.CTRL_REG1_M, 0xC2)
        # 0x00 - Magnetometer continuous operation - I2C enabled
        # 0x80 - Magnetometer continuous operation - I2C disabled
        self.mag.write_byte(Register.CTRL_REG3_M, 0x00)
        # 0x08 - z axi high performance mode - doesn't seem to do anything
        self.mag.write_byte(Register.CTRL_REG4_M, 0x08)
        # Enable BDU (block data update) to ensure high and low bytes come from the same sample
        self.mag.write_byte(Register.CTRL_REG5_M, 0x40)

    def close(self):
        """Closes the I2C/SPI connection. This must be called on shutdown."""
        self.ag.close()
        self.mag.close()

    def ag_data_ready(self, timeout_millis: int) -> bool:
        return self.ag.data_ready(timeout_millis)

    def read_ag_status(self) -> AGStatus:
        """Returns the status byte for the accelerometer and gyroscope."""
        data = self.ag.read_byte(Register.STATUS_REG)
        return AGStatus(data)

    def read_ag_data(self) -> Tuple[int, List[int], List[int]]:
        """Returns the current temperature, acceleration and angular velocity
        values in one go. This is faster than fetching them independently.
        These values can be invalid unless they're read when the data is ready."""
        data = self.ag.read_bytes(Register.OUT_TEMP_L, 14)
        temp = Driver.to_int16(data[0:2])
        gyro = Driver.to_vector_left_to_right_hand_rule(data[2:8])
        acc = Driver.to_vector_left_to_right_hand_rule(data[8:14])
        return temp, acc, gyro

    def read_values(self):
        temp, acc, gyro = self.read_ag_data()
        tempc = Driver.TEMPC_0 + temp * Driver.TEMP_SENSOR_SCALE
        tempf = (tempc * 9/5) + 32
        acc = [c * Driver.ACC_SENSOR_SCALE for c in acc]
        gyro = [g * Driver.DPS_SENSOR_SCALE for g in gyro]
        return tempf, acc, gyro

    def read_temperature(self) -> int:
        """Reads the temperature. See also read_ag_data()"""
        data = self.ag.read_bytes(Register.OUT_TEMP_L, 2)
        return Driver.to_int16(data)

    def read_acceleration(self) -> List[int]:
        """Reads the accelerations. See also read_ag_data()"""
        data = self.ag.read_bytes(Register.OUT_X_XL, 6)
        return Driver.to_vector_left_to_right_hand_rule(data)

    def read_gyroscope(self) -> List[int]:
        """Reads the angular velocities. See also read_ag_data()"""
        data = self.ag.read_bytes(Register.OUT_X_G, 6)
        return Driver.to_vector_left_to_right_hand_rule(data)

    def magnetometer_data_ready(self, timout_millis: int) -> bool:
        return self.mag.data_ready(timout_millis)

    def read_magnetometer_status(self) -> MagnetometerStatus:
        """Returns the status byte for the magnetometer"""
        data = self.mag.read_byte(Register.STATUS_REG_M)
        return MagnetometerStatus(data)

    def mag_values(self):
        mag = self.read_magnetometer()
        mag = [m * Driver.GAUSS_SENSOR_SCALE for m in mag]
        return mag

    def mag_heading(self):
        values = self.mag_values()
        y = values[1]
        x = values[0]
        # z = values[0]
        heading = math.atan2(y, x) * 180 / math.pi
        if heading < 0:
            heading += 360.0
        return heading

    def read_magnetometer(self) -> List[int]:
        """Reads the magnetometer field strengths"""
        data = self.mag.read_bytes(Register.OUT_X_L_M, 6)
        return Driver.to_vector(data)

    @staticmethod
    def to_vector(data: List[int]) -> List[int]:
        return [Driver.to_int16(data[0:2]), Driver.to_int16(data[2:4]), Driver.to_int16(data[4:6])]

    @staticmethod
    def to_vector_left_to_right_hand_rule(data: List[int]) -> List[int]:
        """Like to_vector except it converts from the left to the right hand rule
        by negating the x-axis."""
        return [-Driver.to_int16(data[0:2]), Driver.to_int16(data[2:4]), Driver.to_int16(data[4:6])]

    @staticmethod
    def to_int16(data: List[int]) -> int:
        """
        Converts little endian bytes into a signed 16-bit integer
        :param data: 16bit int in little endian, two's complement form
        :return: an integer
        """
        return int.from_bytes(data, byteorder='little', signed=True)


class Register:
    """Register constants"""
    WHO_AM_I = 0x0F
    CTRL_REG1_G = 0x10
    CTRL_REG2_G = 0x11
    CTRL_REG3_G = 0x12
    OUT_TEMP_L = 0x15
    STATUS_REG = 0x17
    OUT_X_G = 0x18
    CTRL_REG4 = 0x1E
    CTRL_REG5_XL = 0x1F
    CTRL_REG6_XL = 0x20
    CTRL_REG7_XL = 0x21
    CTRL_REG8 = 0x22
    CTRL_REG9 = 0x23
    CTRL_REG10 = 0x24
    OUT_X_XL = 0x28
    REFERENCE_G = 0x0B
    INT1_CTRL = 0x0C
    INT2_CTRL = 0x0D
    WHO_AM_I_M = 0x0F
    CTRL_REG1_M = 0x20
    CTRL_REG2_M = 0x21
    CTRL_REG3_M = 0x22
    CTRL_REG4_M = 0x23
    CTRL_REG5_M = 0x24
    STATUS_REG_M = 0x27
    OUT_X_L_M = 0x28


class GPIOInterrupt(Interrupt):
    def __init__(self, gpio_pin: int):
        self.gpio_pin = gpio_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN)

    def close(self):
        """This method must be called to release the pin for use by other processes"""
        GPIO.cleanup(self.gpio_pin)

    def wait_for(self, timeout: int) -> bool:
        """
        Returns true if when the pin transitions from low to high.
        Assumes some other process will reset the pin to low.
        :param timeout: time to wait for the interrupt in milliseconds
        :return: True if the interrupt happened. False if it timed out.
        """
        ready = False
        # Dividing sleep time by 300 instead of 30 double CPU load but cuts
        # IMU timestamp variation from about 20% to less than 1%
        sleep_time = (timeout / 1000.0) / 30
        stop_time = time.monotonic_ns() + (timeout * 1000_000.0)
        while not ready and time.monotonic_ns() < stop_time:
            ready = GPIO.input(self.gpio_pin)
            time.sleep(sleep_time)
        return ready


class I2CTransport(AbstractTransport):
    data_ready_interrupt: GPIOInterrupt
    I2C_AG_ADDRESS = 0x6B
    I2C_MAG_ADDRESS = 0x1E

    def __init__(self, port: int, i2c_address: int, data_ready_pin: int = None):
        super().__init__()
        self.port = port
        self.i2c_device = i2c_address
        self.data_ready_interrupt = None
        if data_ready_pin:
            self.data_ready_interrupt = GPIOInterrupt(data_ready_pin)

    def close(self):
        if self.data_ready_interrupt:
            self.data_ready_interrupt.close()

    def write_byte(self, address: int, value: int):
        with SMBusWrapper(self.port) as bus:
            bus.write_byte_data(self.i2c_device, address, value)

    def read_byte(self, address: int) -> int:
        with SMBusWrapper(self.port) as bus:
            bus.write_byte(self.i2c_device, address)
            return bus.read_byte(self.i2c_device)

    def read_bytes(self, address: int, length: int) -> List[int]:
        with SMBusWrapper(self.port) as bus:
            bus.write_byte(self.i2c_device, address)
            result = bus.read_i2c_block_data(self.i2c_device, address, length)
            return result

    def data_ready(self, timeout: int) -> bool:
        if self.data_ready_interrupt:
            return self.data_ready_interrupt.wait_for(timeout)
        else:
            raise RuntimeError('I2CTransport needs a GPIO pin to support data_ready().')


class SPITransport(AbstractTransport):
    __READ_FLAG = 0x80
    __MAGNETOMETER_READ_FLAG = 0xC0
    __DUMMY = 0xFF
    data_ready_interrupt: GPIOInterrupt

    def __init__(self, spi_device: int, magnetometer: bool, data_ready_pin: int = None):
        super().__init__()
        self.magnetometer = magnetometer
        self.spi = spidev.SpiDev()
        self._init_spi(spi_device)
        self.data_ready_interrupt = None
        if data_ready_pin:
            self.data_ready_interrupt = GPIOInterrupt(data_ready_pin)

    def _init_spi(self, spi_device: int):
        self.spi.open(0, spi_device)
        self.spi.mode = 0b00
        self.spi.max_speed_hz = 8_000_000

    def close(self):
        self.spi.close()
        if self.data_ready_interrupt:
            self.data_ready_interrupt.close()

    def write_byte(self, address: int, value: int):
        self.spi.writebytes([address, value])

    def read_byte(self, address: int) -> int:
        return self.spi.xfer([address | self.__READ_FLAG, self.__DUMMY])[1]

    def read_bytes(self, reg_address, length):
        request = [self.__DUMMY] * (length + 1)
        if self.magnetometer:
            # Need to set bit 1 for multi-byte reads by the magnetometer or we
            # just keep reading the same byte
            request[0] = reg_address | self.__MAGNETOMETER_READ_FLAG
        else:
            request[0] = reg_address | self.__READ_FLAG
        response = self.spi.xfer(request)
        return response[1:]

    def data_ready(self, timeout: int) -> bool:
        if self.data_ready_interrupt:
            return self.data_ready_interrupt.wait_for(timeout)
        else:
            raise RuntimeError('SPITransport needs a GPIO pin to support data_ready().')
