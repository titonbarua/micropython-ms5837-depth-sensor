"""
Collection of tests to benchmark MS5837 variants on RPi Pico 2040 microcontroller.

Author: Titon Barua <baruat@email.sc.edu, titon@vimmaniac.com>
"""

import gc
import os
import math
import time
import asyncio
from machine import I2C, Pin

from micropython import const
from ms5837.bar30 import MS5837SensorBar30
from ms5837.bar02 import MS5837SensorBar02
from ms5837.depth import NaiveWaterDepthEstimator

if not os.uname().machine == 'Raspberry Pi Pico with RP2040':
    print("These were written for Raspberry Pi Pico RP2040!")

SENSOR_BAR30 = const(0)
SENSOR_BAR02 = const(1)

# Specific hardware configuration.
_I2C_FREQ = const(400000)

_I2C_CHAN_BAR30 = const(1)
_SCL_PIN_BAR30 = "GP19"
_SDA_PIN_BAR30 = "GP18"

_I2C_CHAN_BAR02 = const(0)
_SCL_PIN_BAR02 = "GP17"
_SDA_PIN_BAR02 = "GP16"


def _create_sensor(type_, osr):
    if type_ == SENSOR_BAR30:
        i2c_obj = I2C(
            _I2C_CHAN_BAR30,
            freq=_I2C_FREQ,
            sda=_SDA_PIN_BAR30,
            scl=_SCL_PIN_BAR30)

        sensor = MS5837SensorBar30(
            i2c_obj,
            pressure_osr=osr,
            temperature_osr=osr,
            debug=True)

    elif type_ == SENSOR_BAR02:
        i2c_obj = I2C(
            _I2C_CHAN_BAR02,
            freq=_I2C_FREQ,
            sda=_SDA_PIN_BAR02,
            scl=_SCL_PIN_BAR02)

        sensor = MS5837SensorBar02(
            i2c_obj,
            pressure_osr=osr,
            temperature_osr=osr,
            debug=True)

    else:
        raise ValueError("Unknown sensor type.")

    return (i2c_obj, sensor)


# _OSR_RATES = [256, 512, 1024, 2048, 4096]
_BAR30_OSR_RATES = [256, 512, 1024, 2048, 4096]
_BAR02_OSR_RATES = [256, 512, 1024, 2048, 4096, 8192]
# [256, 512, 1024, 2048, 4096]

# Number of measurements per OSR for timing.
_N = 100

# Setup internal pull up resistors for I2C communication.
Pin(_SDA_PIN_BAR30, pull=Pin.PULL_UP)
Pin(_SCL_PIN_BAR30, pull=Pin.PULL_UP)
Pin(_SDA_PIN_BAR02, pull=Pin.PULL_UP)
Pin(_SCL_PIN_BAR02, pull=Pin.PULL_UP)


def _calc_stats(data):
    data = list(data)

    n = len(data)
    mean = sum(data) / float(n)
    min_ = min(data)
    max_ = max(data)

    var_sum = 0.0
    for dt in data:
        diff = (dt - mean)
        var_sum += (diff * diff)

    variance = var_sum / (n - 1)
    stddev = math.sqrt(variance)

    return {
        "mean": mean,
        "min": min_,
        "max": max_,
        "stddev": stddev,
        "data_rate_hz": 1000000 / mean,
    }


def _time_n_reads(sensor_type, osr, n):
    print(f"Measuring timing of {n} samples with OSR={osr} ...")
    i2c_obj, sensor = _create_sensor(sensor_type, osr)
    try:
        p = 0
        t = 0
        end_t = 0
        start_t = 0
        time_taken = [0] * n

        gc.disable()
        i = 0
        while i < n:
            start_t = time.ticks_us()
            p, t = sensor.read(max_tries=1)
            end_t = time.ticks_us()
            time_taken[i] = time.ticks_diff(end_t, start_t)
            i += 1
            gc.collect()
        gc.enable()

    finally:
        del i2c_obj

    # print(time_taken)
    return _calc_stats(time_taken)


async def _time_n_async_reads(sensor_type, osr, n):
    print(f"Measuring timing of {n} samples with OSR={osr} ...")

    i2c_obj, sensor = _create_sensor(sensor_type, osr)
    try:
        p = 0
        t = 0
        end_t = 0
        start_t = 0
        time_taken = [0] * n

        i = 0
        gc.disable()
        while i < n:
            start_t = time.ticks_us()
            p, t = await sensor.async_read(max_tries=3)
            end_t = time.ticks_us()
            time_taken[i] = time.ticks_diff(end_t, start_t)
            i += 1
            gc.collect()
        gc.enable()

    finally:
        del i2c_obj

    # print(time_taken)
    return _calc_stats(time_taken)


def _print_timing(data, osr_rates):
    dkeys = [
        "mean",
        "stddev",
        "min",
        "max",
        "data_rate_hz"]
    headers = [
        "T_avg[us]",
        "T_stddev[us]",
        "T_min[us]",
        "T_max[us]",
        "Data Rate[Hz]"]

    # Print headers.
    print(
        "|    OSR |" +
        "|".join(["{:>14}".format(h) for h in headers]) +
        "|")

    format_str = "|{:8d}|{:>14.2f}|{:>14.2f}|{:>14.2f}|{:>14.2f}|{:>14.2f}|"
    for osr in osr_rates:
        d = data[osr]
        print(format_str.format(
            *([osr] + [d[k] for k in dkeys])))


def benchmark_blocking_read(sensor_type):
    """Benchmark sensor using regular blocking reads."""
    print("Measuring timing of blocking reads ...")
    osr_rates = (
        _BAR30_OSR_RATES
        if sensor_type == SENSOR_BAR30
        else _BAR02_OSR_RATES)

    data = {}
    for osr in osr_rates:
        data[osr] = _time_n_reads(
            sensor_type, osr=osr, n=_N)

    # print(data)
    _print_timing(data, osr_rates)


async def _benchmark_async_read(sensor_type):
    osr_rates = (
        _BAR30_OSR_RATES
        if sensor_type == SENSOR_BAR30
        else _BAR02_OSR_RATES)

    data = {}
    for osr in osr_rates:
        data[osr] = await _time_n_async_reads(
            sensor_type, osr=osr, n=_N)

    # print(data)
    _print_timing(data, osr_rates)


def benchmark_async_read(sensor_type):
    """Benchmark sensor using asyncio reads."""
    print("Measuring timing of async reads ...")
    task = _benchmark_async_read(sensor_type)
    asyncio.run(task)


def print_data(sensor_type, interval_sec=1.0):
    """Continuously read and print pressure and temperature."""
    i2c_obj, sensor = _create_sensor(sensor_type, 4096)
    try:
        while True:
            try:
                p, t = sensor.read()
                print(f"P: {p:.2f} KPa, T: {t:.2f} C")
                time.sleep(interval_sec)
            except KeyboardInterrupt:
                break
    finally:
        del i2c_obj


def print_depth(sensor_type, water_density=1000, interval_sec=1.0):
    """Continuously estimate depth under water using."""
    i2c_obj, sensor = _create_sensor(sensor_type, 4096)

    depth_estimator = NaiveWaterDepthEstimator(sensor, water_density)
    ref = depth_estimator.set_ref_pressure()
    print(f"Reference pressure set to {ref} KPa")

    try:
        while True:
            try:
                d_m = depth_estimator.read_depth()
                print("Depth: {:.2f} mm".format(d_m * 1000))
                time.sleep(interval_sec)
            except KeyboardInterrupt:
                break
    finally:
        del i2c_obj
