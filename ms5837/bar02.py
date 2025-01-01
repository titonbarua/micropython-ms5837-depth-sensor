"""This module contains class to interface with MS5837-02BA variant sensors.

Author: Titon Barua <baruat@email.sc.edu, titon@vimmaniac.com>

"""
from ms5837.base import MS5837SensorBase
from array import array


class MS5837SensorBar02(MS5837SensorBase):
    """Class to communicate with MS5837-02BA variant sensor."""

    _SUPPORTED_SENSOR_IDS = (0x00, 0x15)

    _OSR_ADC_READ_DELAYS_US = array('H', (560, 1100, 2170, 4320, 8610, 17020))

    def _init_constants(self):
        C = self._prom_data
        self._SENS_T1 = C[1] << 16
        self._OFF_T1 = C[2] << 17
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

        OFF = self._OFF_T1 + ((self._TCO * dT) >> 6)
        SENS = self._SENS_T1 + ((self._TCS * dT) >> 7)
        TEMP_C = TEMP * 0.01

        # Second order compensation.
        if TEMP_C < 20:
            # Low temperature.
            Ti = (11 * dT * dT) >> 35
            x = TEMP - 2000
            x *= x
            OFFi = (31 * x) >> 3
            SENSi = (63 * x) >> 5
        else:
            Ti = 0
            OFFi = 0
            SENSi = 0

        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi
        TEMP2 = (TEMP - Ti) * 0.01  # degree-C
        P2 = (
            (((D1 * SENS2) >> 21) - OFF2) >> 15
        ) * 0.01  # mBar = KPa

        # Units: (KPa, degree-C)
        return P2, TEMP2
