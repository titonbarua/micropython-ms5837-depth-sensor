"""
This module implements a micropython class to read data from MS5837 pressure sensors.
"""
import asyncio
from array import array
import time

from micropython import const
from machine import I2C

_I2C_ADDR = const(0x76)

# Command to reset the sensor.
_CMD_RESET = const(0x1E)

# Supported oversampling rates.
_OSR_RATES = (256, 512, 1024, 2048, 4096)

# We are just defining the base commands here as the rest can be derived using
# a simple formula:
#
# target_command = base_command + osr_rate_index * 2
#
_CMD_CONV_P_BASE = const(0x40)
_CMD_CONV_T_BASE = const(0x50)

# Command to read converted pressure and temperature.
_CMD_READ_ADC = const(0x00)

# Command to read the built-in PROM data. There are 7 addresses, each address
# being offset by 2. The addresses span from 0xA0 to 0xAC.
_CMD_READ_PROM_START = const(0xA0)

_RX_BUFF_SIZE = const(3)
_PROM_DATA_SIZE = const(7)


class MS5837Error(Exception):
    """Exception related to MS5837 sensor."""


class MS5837OsrUnsupported(MS5837Error):
    """Unsupported oversampling rate."""


class MS5837CommFailure(MS5837Error):
    """Failed to communicate with sensor in expected manner."""


class MS5837PromCRCError(MS5837Error):
    """CRC verification failed for PROM data."""


class MS5837SensorIDMismatch(MS5837Error):
    """Detected sensor is not supported."""


class MS5837SensorBase(object):
    """Class to communicate with MS5837 sensor."""

    # Sensor I2C address.
    _I2C_ADDR = 0x76

    # Recommended wait time (in microseconds) for ADC conversions.
    _OSR_ADC_READ_DELAYS_US = (10000, 10000, 10000, 10000, 10000)

    # Sensor IDs supported by this class.
    _SUPPORTED_SENSOR_IDS = ()

    # _osr_adc_read_delays_us = (600, 1170, 2280, 4540, 9040)

    def __init__(
            self,
            i2c_obj,
            pressure_osr=256,
            temperature_osr=256,
    ):
        """Instantiate MS5837Sensor class.

        Args:
        - i2c_obj: Micropython I2C object.
        - pressure_osr: (int) Pressure oversampling rate.
        - temperature_osr: (int) Temperature oversampling rate.
        """
        try:
            p_osr_idx = _OSR_RATES.index(pressure_osr)
        except KeyError:
            raise MS5837OsrUnsupported()

        try:
            t_osr_idx = _OSR_RATES.index(temperature_osr)
        except KeyError:
            raise MS5837OsrUnsupported()

        # Determine conversion command and read delays for given osr.
        self._p_conv_cmd = _CMD_CONV_P_BASE + 2 * p_osr_idx
        self._t_conv_cmd = _CMD_CONV_T_BASE + 2 * t_osr_idx
        self._p_read_delay_us = self._OSR_ADC_READ_DELAYS_US[p_osr_idx]
        self._t_read_delay_us = self._OSR_ADC_READ_DELAYS_US[t_osr_idx]

        # Buffer to read calibration data out of chip PROM.
        self._prom_data = array('H', [0] * _PROM_DATA_SIZE)

        # Buffers used to read and write data.
        self._rx_buff_1 = array('B', [0] * _RX_BUFF_SIZE)
        self._rx_buff_2 = array('B', [0] * _RX_BUFF_SIZE)
        self._tx_buff = array('B', [0])

        self._init()

    def reset(self, max_tries=5):
        """Reset the device.

        Returns `True` if sensor acknowledged command.

        """
        for _ in range(max_tries):
            # Send reset command.
            self._tx_buff[0] = _CMD_RESET
            n_ack = self._i2c_obj.writeto(
                self._i2c_addr, self._tx_buff)

            # If reset command is properly acknowledged, return True.
            if n_ack == 1:
                return True

            # If reset command was not acknowledged, the data-sheet recommends
            # sending several SCL-s and trying reset again. We can send SCL-s
            # by trying to read a byte.
            self._i2c_obj.readfrom(self._i2c_addr, 1)
        else:
            return False

    def _adc_convert_p(self):
        """Initiate pressure data conversion in ADC.

        Returns `True` if sensor acknowledged command.

        """
        self._tx_buff[0] = self._p_conv_cmd
        n_ack = self._i2c_obj.writeto(
            self._i2c_addr, self._tx_buff)

        return n_ack == 1

    def _adc_convert_t(self):
        """Initiate temperature data conversion in ADC.

        Returns `True` if sensor acknowledged command.

        """
        self._tx_buff[0] = self._t_conv_cmd
        n_ack = self._i2c_obj.writeto(
            self._i2c_addr, self._tx_buff)

        return n_ack == 1

    def _fetch_adc_into(self, buff, max_tries):
        """Read ADC data and store it in the buffer.

        Returns `True` if ADC read command was acknowledged.

        """
        # Try to send ADC read command.
        self._tx_buff[0] = _CMD_READ_ADC
        for _ in range(max_tries):
            n_ack = self._i2c_obj.writeto(
                self._i2c_addr, self._tx_buff)

            if n_ack == 1:
                break
        else:
            return False

        # Read ADC data into the given buffer.
        self._i2c_obj.readfrom_into(
            self._i2c_addr, buff)

        return True

    def _read_prom(self):
        """Read builtin PROM data into the designated buffer."""
        base_addr = _CMD_READ_PROM_START
        rx_buff = array('B', [0, 0])
        for i in range(len(self._prom_data)):
            # Set address.
            self._tx_buff[0] = base_addr + i * 2

            # Send read command.
            n_ack = self._i2c_obj.writeto(
                self._i2c_addr, self._tx_buff)
            if n_ack != 1:
                raise MS5837CommFailure()

            # Read back prom data.
            self._i2c_obj.readfrom_into(self._i2c_addr, rx_buff)
            print(rx_buff)
            self._prom_data[i] = (
                rx_buff[0] << 8 | rx_buff[1])

    def _check_crc(self):
        """Check CRC4 value of the prom data."""
        expected_crc = (self._prom_data[0] & 0xF000) >> 12

        # Verbatim copy of the algorithm from data-sheet.
        # ----------------------------------------------,
        n_rem = 0
        n_prom = list(self._prom_data) + [0]

        n_prom[0] = n_prom[0] & 0x0FFF
        n_prom[7] = 0
        for cnt in range(16):
            if cnt % 2 == 1:
                n_rem ^= (n_prom[cnt >> 1] & 0x00FF)
            else:
                n_rem ^= (n_prom[cnt >> 1] >> 8)

            for n_bit in range(8, 0, -1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = n_rem << 1

        n_rem = (n_rem >> 12) & 0x000F
        crc = n_rem ^ 0x00
        # ----------------------------------------------'

        print(f"CRC: {crc}, Expected CRC: {expected_crc}")
        if not crc == expected_crc:
            raise MS5837PromCRCError()

    def _check_sensor_id(self):
        """Check sensor ID from prom data."""
        sid = (self._prom_data[0] >> 5) & 0x7f
        if sid not in self._SUPPORTED_SENSOR_IDS:
            raise MS5837SensorIDMismatch()

    def _init(self):
        """Initialize sensor."""
        if not self.reset():
            raise MS5837CommFailure()

        # Wait for device to recover from reset.
        time.sleep_us(100)

        # Read and verify PROM data.
        self._read_prom()
        self._check_crc()
        self._check_sensor_id()

    def read_raw(self, max_tries):
        """Read raw pressure and temperature ADC values from sensor.

        Args:
        - `max_tries`: (int) Max number of times each ADC read command is
          tried, if an ACK is not received.

        Returns a tuple of format: (<pressure>, <temperature>).

        """
        # Reset rx buffers.
        for i in range(_RX_BUFF_SIZE):
            self._rx_buff_1[i] = 0
            self._rx_buff_2[i] = 0

        # We convert and read temperature first, as that is the auxiliary
        # measurement and is much slower to change.
        if not self._adc_convert_t():
            raise MS5837CommFailure()
        time.sleep_us(self._t_read_delay_us)
        if not self._fetch_adc_into(self._rx_buff_1, max_tries):
            raise MS5837CommFailure()

        # Next, we read pressure.
        if not self._adc_convert_p():
            raise MS5837CommFailure()
        time.sleep_us(self._p_read_delay_us)
        if not self._fetch_adc_into(self._rx_buff_2, max_tries):
            raise MS5837CommFailure()

        # Infer temperature value.
        t = (self._rx_buff_1[0] << 16 |
             self._rx_buff_1[1] << 8 |
             self._rx_buff_1[2])

        # Infer pressure value.
        p = (self._rx_buff_2[0] << 16 |
             self._rx_buff_2[1] << 8 |
             self._rx_buff_2[2])

        return (p, t)

    async def read_raw_async(self, max_tries):
        """Asynchronous version of `read_raw` function."""
        # Reset rx buffers.
        for i in range(_RX_BUFF_SIZE):
            self._rx_buff_1[i] = 0
            self._rx_buff_2[i] = 0

        # We convert and read temperature first as that is the auxiliary
        # measurement and is much slower to change.
        if not self._adc_convert_t():
            raise MS5837CommFailure()
        await asyncio.sleep_us(self._t_read_delay_us)
        if not self._fetch_adc_into(self._rx_buff_1, max_tries):
            raise MS5837CommFailure()

        # Next, we read pressure.
        if not self._adc_convert_p():
            raise MS5837CommFailure()
        await asyncio.sleep_us(self._p_read_delay_us)
        if not self._fetch_adc_into(self._rx_buff_2, max_tries):
            raise MS5837CommFailure()

        # Infer temperature value.
        t = (self._rx_buff_1[0] << 16 |
             self._rx_buff_1[1] << 8 |
             self._rx_buff_1[2])

        # Infer pressure value.
        p = (self._rx_buff_2[0] << 16 |
             self._rx_buff_2[1] << 8 |
             self._rx_buff_2[2])

        return (p, t)

    def _calc_pres_KPa_and_temp_C(
            self,
            raw_p,
            raw_t,
            second_order_compensation):
        """Convert raw pressure and temperature into compensated physical units.

        Args:
        - raw_p: (unsigned int) Raw pressure value from ADC.
        - raw_t: (unsigned int) Raw temperature value from ADC.
        - second_order_compensation: (bool) If `True`, an expensive, second
          order compensation is calculated for optimum accuracy.

        Returns a tuple of format:
          (<pressure-KPa>, <temperature-°C>).

        """
        raise NotImplementedError()

    def read(
            self,
            second_order_compensation=True,
            max_tries=3):
        """Read pressure and temperature in physical units.

        Args:
        - second_order_compensation: (bool) If `True`, an expensive, second
          order compensation is applied for optimum accuracy.
        - max_tries: (int) Maximum retries for I2C communication.

        Returns a tuple of format:
          (<pressure-KPa>, <temperature-°C>)

        """
        p_raw, t_raw = self.read_raw(max_tries)
        return self._calc_pres_KPa_and_temp_C(
            p_raw, t_raw, second_order_compensation)

    async def async_read(
            self,
            second_order_compensation=True,
            max_tries=3):
        """Asynchronously read pressure and temperature.

        This is the asynchronous version of `read` method.

        """
        p_raw, t_raw = await self.read_raw(max_tries)
        return self._calc_pres_KPa_and_temp_C(
            p_raw, t_raw, second_order_compensation)
