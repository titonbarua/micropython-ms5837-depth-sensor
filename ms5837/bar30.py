"""This module contains a class to interface with MS5837-30BA variant sensors.

Author: Titon Barua <baruat@email.sc.edu, titon@vimmaniac.com>

"""
from array import array

from ms5837.base import MS5837SensorBase


class MS5837SensorBar30(MS5837SensorBase):
    """Class to communicate with MS5837-30BA variant sensor."""

    _SUPPORTED_SENSOR_IDS = (0x1A,)

    _OSR_ADC_READ_DELAYS_US = array('H', [600, 1170, 2280, 4540, 9040])

    def _calc_pressure_KPa_and_temp_C(
            self,
            raw_p,
            raw_t,
            second_order_compensation):
        """Convert raw pressure and temperature into compensated physical units.

        Args:
        - raw_p: (unsigned int) Raw pressure value from ADC.
        - raw_t: (unsigned int) Raw temperature value from ADC.
        - second_order_compensation: (bool) If `True`, an expensive, second
          order compensation is applied for optimum accuracy.

        Returns a tuple of format:
          (<pressure-KPa>, <temperature-°C>).

        """
        C1 = self._prom_data[1]
        C2 = self._prom_data[2]
        C3 = self._prom_data[3]
        C4 = self._prom_data[4]
        C5 = self._prom_data[5]
        C6 = self._prom_data[6]

        D1 = raw_p
        D2 = raw_t

        # First order compensation.
        dT = D2 - (C5 << 8)
        TEMP = 2000 + ((dT * C6) // 8388608)  # 2^23 = 8388608
        TEMP_C = TEMP * 0.01

        OFF = (C2 << 16) + ((C4 * dT) // 128)
        SENS = (C1 << 15) + ((C3 * dT) // 256)

        if not second_order_compensation:
            P = (((D1 * SENS) // 2097152) - OFF) // 8192  # 2^13 = 8192
            # Units: (KPa, °C)
            return (P * 0.1, TEMP_C)

        # Second order compensation.
        # NOTE: This algorithm is described in 'Figure-10' in datasheet.
        # -----------------------------------------------------------,
        if TEMP_C < 20:
            # Low temperature.
            Ti = (3 * dT * dT) // 8589934592  # 2^33 = 8589934592
            tx = TEMP - 2000
            tx2 = tx * tx
            OFFi = (3 * tx2) // 2
            SENSi = (5 * tx2) // 8

            # Very low temperature.
            if TEMP_C < -15:
                ty = TEMP + 1500
                ty2 = ty * ty
                OFFi = OFFi + 7 * ty2
                SENSi = SENSi + 4 * ty2
        else:
            # High temperature.
            Ti = (2 * dT * dT) // 137438953472  # 2^37 = 137438953472
            tz = TEMP - 2000
            OFFi = (tz * tz) // 16
            SENSi = 0

        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi
        TEMP2 = (TEMP - Ti) * 0.01  # °C
        P2 = (
            (((D1 * SENS2) // 2097152) - OFF2)  # 2^21 = 2097152
            // 8192
        ) * 0.1  # mBar = KPa
        # -----------------------------------------------------------'

        # Units: (KPa, °C)
        return (P2, TEMP2)
