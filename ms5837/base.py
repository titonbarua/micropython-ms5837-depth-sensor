"""This module implements a base class to read data from MS5837 sensors.

Author: Titon Barua <baruat@email.sc.edu, titon@vimmaniac.com>

"""
import asyncio
from array import array
from math import log2
import time

from micropython import const


# Command to reset the sensor.
_CMD_RESET = const(0x1E)

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

    # List of recommended wait time (in microseconds) for ADC conversions.
    _OSR_ADC_READ_DELAYS_US = NotImplemented

    # List of sensor IDs supported by this class.
    _SUPPORTED_SENSOR_IDS = NotImplemented

    def __init__(
            self,
            i2c_obj,
            pressure_osr=256,
            temperature_osr=256,
            enable_sensor_id_check=False,
            debug=False
    ):
        """Instantiate MS5837Sensor class.

        Args:
        - i2c_obj: Micropython I2C object.
        - pressure_osr: (int) Pressure oversampling rate.
        - temperature_osr: (int) Temperature oversampling rate.
        - enable_sensor_id_check: (bool) If True, sensor version is read and
          matched from bits[11:5] of first PROM word. Set to False by default
          as some of the 30BA sensors don't properly have their versions set
          par data-sheet.
        - debug: (bool) If True, reports communication retry attempts.

        """
        self._i2c_obj = i2c_obj
        self._debug = debug
        self._enable_sensor_id_check = enable_sensor_id_check

        # Supported oversampling rates are: 256, 512, 1024, 2048, 4096
        p_osr_idx = round(log2(pressure_osr)) - 8
        t_osr_idx = round(log2(temperature_osr)) - 8
        n_osr_rates = len(self._OSR_ADC_READ_DELAYS_US)
        if not ((0 <= p_osr_idx < n_osr_rates) and
                (0 <= t_osr_idx < n_osr_rates)):
            raise MS5837OsrUnsupported()

        # Determine conversion command and read delays for given osr.
        self._p_conv_cmd = _CMD_CONV_P_BASE + 2 * p_osr_idx
        self._t_conv_cmd = _CMD_CONV_T_BASE + 2 * t_osr_idx
        self._p_read_delay_us = self._OSR_ADC_READ_DELAYS_US[p_osr_idx]
        self._p_read_delay_s = round(self._p_read_delay_us / 1.0e6, 6)
        self._t_read_delay_us = self._OSR_ADC_READ_DELAYS_US[t_osr_idx]
        self._t_read_delay_s = round(self._t_read_delay_us / 1.0e6, 6)

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
                self._I2C_ADDR, self._tx_buff)

            # If reset command is properly acknowledged, return True.
            if n_ack == 1:
                return True

            # If reset command was not acknowledged, the data-sheet recommends
            # sending several SCL-s and trying reset again. We can send SCL-s
            # by trying to read a byte.
            self._i2c_obj.readfrom(self._I2C_ADDR, 1)
        else:
            return False

    def _adc_convert_p(self):
        """Initiate pressure data conversion in ADC.

        Returns `True` if sensor acknowledged command.

        """
        self._tx_buff[0] = self._p_conv_cmd
        n_ack = self._i2c_obj.writeto(
            self._I2C_ADDR, self._tx_buff)

        return n_ack == 1

    def _adc_convert_t(self):
        """Initiate temperature data conversion in ADC.

        Returns `True` if sensor acknowledged command.

        """
        self._tx_buff[0] = self._t_conv_cmd
        n_ack = self._i2c_obj.writeto(
            self._I2C_ADDR, self._tx_buff)

        return n_ack == 1

    def _fetch_adc_into(self, buff):
        """Read ADC data and store it in the buffer.

        Returns `True` if read was successful.

        """
        # Try to send ADC read command.
        self._tx_buff[0] = _CMD_READ_ADC
        n_ack = self._i2c_obj.writeto(
            self._I2C_ADDR, self._tx_buff)

        if not n_ack == 1:
            return False

        # Read ADC data into the given buffer.
        try:
            self._i2c_obj.readfrom_into(self._I2C_ADDR, buff)
            return True
        except OSError:
            return False

    def _read_prom(self):
        """Read builtin PROM data into the designated buffer."""
        base_addr = _CMD_READ_PROM_START
        rx_buff = array('B', [0, 0])
        for i in range(len(self._prom_data)):
            # Set address.
            self._tx_buff[0] = base_addr + i * 2

            # Send read command.
            n_ack = self._i2c_obj.writeto(
                self._I2C_ADDR, self._tx_buff)
            if n_ack != 1:
                raise MS5837CommFailure()

            # Read back prom data.
            self._i2c_obj.readfrom_into(self._I2C_ADDR, rx_buff)
            self._prom_data[i] = (
                (rx_buff[0] << 8) | rx_buff[1])

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

        # print(f"crc: {crc}, expected crc: {expected_crc}")
        if not crc == expected_crc:
            raise MS5837PromCRCError()

    def _check_sensor_id(self):
        """Check sensor ID from prom data."""
        sid = (self._prom_data[0] >> 5) & 0x7F
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
        self._init_constants()

        # If enabled, do and ID check on the sensor.
        if self._enable_sensor_id_check:
            self._check_sensor_id()

        # We no longer need prom data.
        del self._prom_data

    def read_raw(self, max_tries):
        """Read raw pressure and temperature ADC values from sensor.

        Args:
        - `max_tries`: (int) Max number of times each ADC read command is
          tried, if an ACK is not received.

        Returns a tuple of format: (<pressure>, <temperature>).

        """
        # Copy buffer references locally.
        rx_buff1 = self._rx_buff_1
        rx_buff2 = self._rx_buff_2

        # Reset rx buffers. Intentionally avoiding 'for' loop.
        i = 0
        while i < _RX_BUFF_SIZE:
            rx_buff1[i] = 0
            rx_buff2[i] = 0
            i += 1

        # We convert and read temperature first, as that is the auxiliary
        # measurement and is much slower to change.
        if not self._adc_convert_t():
            raise MS5837CommFailure()

        i = 0
        while i < max_tries:
            time.sleep_us(self._t_read_delay_us)
            if self._fetch_adc_into(rx_buff1):
                break
            i += 1
        else:
            raise MS5837CommFailure()

        # Next, we read pressure.
        if not self._adc_convert_p():
            raise MS5837CommFailure()

        i = 0
        while i < max_tries:
            time.sleep_us(self._p_read_delay_us)
            if self._fetch_adc_into(rx_buff2):
                break
            i += 1
            if self._debug:
                print("[DEBUG] ADC read failed!")
        else:
            raise MS5837CommFailure()

        # Infer temperature value.
        t = (rx_buff1[0] << 16 |
             rx_buff1[1] << 8 |
             rx_buff1[2])

        # Infer pressure value.
        p = (rx_buff2[0] << 16 |
             rx_buff2[1] << 8 |
             rx_buff2[2])

        return (p, t)

    async def async_read_raw(self, max_tries):
        """Asynchronous version of `read_raw` function."""
        # Copy buffer references locally.
        rx_buff1 = self._rx_buff_1
        rx_buff2 = self._rx_buff_2

        # Reset rx buffers.
        i = 0
        while i < _RX_BUFF_SIZE:
            rx_buff1[i] = 0
            rx_buff2[i] = 0
            i += 1

        # We convert and read temperature first as that is the auxiliary
        # measurement and is much slower to change.
        if not self._adc_convert_t():
            raise MS5837CommFailure()

        i = 0
        while i < max_tries:
            await asyncio.sleep(self._t_read_delay_s)
            if self._fetch_adc_into(rx_buff1):
                break
            i += 1
        else:
            raise MS5837CommFailure()

        # Next, we read pressure.
        if not self._adc_convert_p():
            raise MS5837CommFailure()

        i = 0
        while i < max_tries:
            await asyncio.sleep(self._p_read_delay_s)
            if self._fetch_adc_into(rx_buff2):
                break
            i += 1
            if self._debug:
                print("[DEBUG] ADC read failed!")
        else:
            raise MS5837CommFailure()

        # Infer temperature value.
        t = (rx_buff1[0] << 16 |
             rx_buff1[1] << 8 |
             rx_buff1[2])

        # Infer pressure value.
        p = (rx_buff2[0] << 16 |
             rx_buff2[1] << 8 |
             rx_buff2[2])

        return (p, t)

    def _init_constants(self):
        """Prepare constants from prom data."""
        raise NotImplementedError()

    def _calc_pressure_temp(self, raw_p, raw_t):
        """Convert raw pressure and temperature into compensated physical units.

        Args:
        - raw_p: (unsigned int) Raw pressure value from ADC.
        - raw_t: (unsigned int) Raw temperature value from ADC.

        Returns a tuple of format:
          (<pressure-KPa>, <temperature-degree-C>).

        """
        raise NotImplementedError()

    def read(self, max_tries=3):
        """Read pressure and temperature in physical units.

        Args:
        - max_tries: (int) Maximum retries for I2C communication.

        Returns a tuple of format:
          (<pressure-KPa>, <temperature-degree-C>)

        """
        p_raw, t_raw = self.read_raw(max_tries)
        return self._calc_pressure_temp(p_raw, t_raw)

    async def async_read(self, max_tries=3):
        """Asynchronously read pressure and temperature.

        This is the asynchronous version of `read` method.

        """
        p_raw, t_raw = await self.async_read_raw(max_tries)
        return self._calc_pressure_temp(p_raw, t_raw)
