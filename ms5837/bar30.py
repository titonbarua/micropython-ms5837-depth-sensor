"""This module contains a class to interface with MS5837-30BA variant sensors.

Author: Titon Barua <baruat@email.sc.edu, titon@vimmaniac.com>

"""
from array import array

from ms5837.base import MS5837SensorBase


class MS5837SensorBar30(MS5837SensorBase):
    """Class to communicate with MS5837-30BA variant sensor."""

    _SUPPORTED_SENSOR_IDS = (0x1A,)

    _OSR_ADC_READ_DELAYS_US = array('H', (600, 1170, 2280, 4540, 9040))

    def _init_constants(self):
        C = self._prom_data
        self._SENS_T1 = C[1] << 15
        self._OFF_T1 = C[2] << 16
        self._TCS = C[3]
        self._TCO = C[4]
        self._T_REF = C[5] << 8
        self._TEMPSENS = C[6]

    def _calc_pressure_temp(self, raw_p, raw_t):
        D1 = raw_p
        D2 = raw_t

        # First order compensation.
        dT = D2 - self._T_REF
        TEMP = 2000 + ((dT * self._TEMPSENS) >> 23)
        TEMP_C = TEMP * 0.01

        OFF = self._OFF_T1 + ((self._TCO * dT) >> 7)
        SENS = self._SENS_T1 + ((self._TCS * dT) >> 8)

        # Second order compensation.
        if TEMP_C < 20:
            # Low temperature.
            Ti = (3 * dT * dT) >> 33
            x = TEMP - 2000
            x *= x
            OFFi = (3 * x) >> 1
            SENSi = (5 * x) >> 3

            # Very low temperature.
            if TEMP_C < -15:
                x = TEMP + 1500
                x *= x
                OFFi += 7 * x
                SENSi += 4 * x
        else:
            # High temperature.
            Ti = (2 * dT * dT) >> 37
            x = TEMP - 2000
            OFFi = (x * x) >> 4
            SENSi = 0

        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi
        TEMP2 = (TEMP - Ti) * 0.01  # °C
        P2 = (
            (((D1 * SENS2) >> 21) - OFF2) >> 13
        ) * 0.01  # 1 KPa = 10 mBar

        # Units: (KPa, degree-C)
        return P2, TEMP2
