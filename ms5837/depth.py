"""This module contains classes to estimate depth using an MS5837 sensor.

Author: Titon Barua <baruat@email.sc.edu, titon@vimmaniac.com>

"""

_STANDARD_G = 9.80665  # m/s^2
_REF_PRESSURE = 101.325  # KPa = 1000 N/m^2


class NaiveWaterDepthEstimator(object):
    """Class to estimate depth by reading data from MS5837 family sensors.
    """

    def __init__(
            self,
            ms5837_sensor_obj,
            water_density,
            ref_pressure=_REF_PRESSURE,
            g=_STANDARD_G):
        """Instantiate NaiveWaterDepthEstimator class.

        Args:
        - ms5837_sensor_obj: A instance of MS5837Sensor class.
        - water_density: Density of water in units of Kg/m^3.
        - ref_pressure: (optional) Reference pressure in units of KPa.
        - g: (optional) Gravitational constant in m/s^2.
        """
        self._ms5837 = ms5837_sensor_obj
        self._g_x_rho = g * water_density
        self._p_ref = float(ref_pressure)

    def set_ref_pressure(self, n_measurements=10):
        """Set reference pressure in KPa from measurements by the sensor."""
        s = 0.0
        for _ in range(n_measurements):
            p_abs, _ = self._ms5837.read()
            s += p_abs

        ref = s / n_measurements
        self._p_ref = ref

        return ref

    def _calc_depth_m(self, abs_pressure):
        """Calculate depth in meters from given absolute pressure."""
        p_rel = abs_pressure - self._p_ref

        # Liquid pressure, P = h x rho x g
        #               => h = P / (rho x g)
        h = (p_rel * 1000.0) / self._g_x_rho

        # Unit: m
        return h

    def read_depth(self):
        """Read depth in meters from the sensor."""
        p_abs, _ = self._ms5837.read()
        return self._calc_depth_m(p_abs)

    async def async_read_depth(self):
        """Asynchronously read depth in meters from the sensor."""
        p_abs, _ = await self._ms5837.read()
        return self._calc_depth_m(p_abs)
