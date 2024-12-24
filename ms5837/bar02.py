"""This module contains class to interface with MS5837-02BA variant sensors.

Author: Titon Barua <baruat@email.sc.edu, titon@vimmaniac.com>

"""
from ms5837.base import MS5837SensorBase


class MS5837SensorBar02(MS5837SensorBase):
    """Class to communicate with MS5837-02BA variant sensor."""

    _SUPPORTED_SENSOR_IDS = (0x00, 0x15)

    _OSR_ADC_READ_DELAYS_US = (560, 1100, 2170, 4320, 8610)

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
          order compensation applied for optimum accuracy.

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

        OFF = (C2 << 17) + ((C4 * dT) // 64)
        SENS = (C1 << 16) + ((C3 * dT) // 128)
        TEMP_C = TEMP * 0.01

        if not second_order_compensation:
            P = (((D1 * SENS) // 2097152) - OFF) // 32768  # 2^15 = 32768
            # Units: (KPa, °C)
            return (P * 0.01, TEMP_C)

        # Second order compensation.
        # -----------------------------------------------------------,
        if TEMP_C < 20:
            # Low temperature.
            Ti = (11 * dT * dT) // 34359738368  # 2^35 = 34359738368
            tx = TEMP - 2000
            tx2 = tx * tx
            OFFi = (31 * tx2) // 8
            SENSi = (63 * tx2) // 32
        else:
            Ti = 0
            OFFi = 0
            SENSi = 0

        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi
        TEMP2 = (TEMP - Ti) * 0.01  # °C
        P2 = (
            (((D1 * SENS2) // 2097152) - OFF2)  # 2^21 = 2097152
            // 32768
        ) * 0.01  # mBar = KPa
        # -----------------------------------------------------------'

        # Units: (KPa, °C)
        return (P2, TEMP2)
