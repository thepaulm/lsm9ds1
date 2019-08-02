import time

from lsm9ds1_rjg import Driver, I2CTransport, SPITransport


class SimpleExample:
    X_IND = 1
    Y_IND = 2
    Z_IND = 0

    PITCH_IND = 1
    ROLL_IND = 0
    YAW_IND = 2

    """This example shows how to poll the sensor for new data.
    It queries the sensor to discover when the accelerometer/gyro
    has new data and then reads all the sensors."""
    def __init__(self):
        # self.driver = self._create_spi_driver()
        self.driver = self._create_i2c_driver()
        self.driver.configure()

    @staticmethod
    def _create_i2c_driver() -> Driver:
        return Driver(
            I2CTransport(1, I2CTransport.I2C_AG_ADDRESS),
            I2CTransport(1, I2CTransport.I2C_MAG_ADDRESS))

    @staticmethod
    def _create_spi_driver() -> Driver:
        return Driver(
            SPITransport(0, False),
            SPITransport(1, True))

    def main(self):
        try:
            count = 0
            while True:
                ag_data_ready = self.driver.read_ag_status().accelerometer_data_available
                if ag_data_ready:
                    self.read_ag()
                    self.read_magnetometer()
                    count += 1
                time.sleep(0.05)
        finally:
            self.driver.close()

    def read_ag(self):
        temp, acc, gyro = self.driver.read_values()
        print("Temp: %.1f °f" % temp)
        print("Gyro Roll: %.4f, Pitch: %.4f, Yaw: %.4f" % (gyro[SimpleExample.ROLL_IND],
                                                           gyro[SimpleExample.PITCH_IND],
                                                           gyro[SimpleExample.YAW_IND]))
        print("X: %.4f, Y: %.4f, Z: %.4f" % (acc[SimpleExample.X_IND],
                                             acc[SimpleExample.Y_IND],
                                             acc[SimpleExample.Z_IND]))

    def read_magnetometer(self):
        hdg = self.driver.mag_heading()
        print("Heading: %.2f" % hdg)


if __name__ == '__main__':
    SimpleExample().main()
