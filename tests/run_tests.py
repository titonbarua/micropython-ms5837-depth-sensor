import gc
import os
import sys
import math
import time
import asyncio
from machine import I2C

from ms5837.bar30 import MS5837SensorBar30
from ms5837.bar02 import MS5837SensorBar02


if not os.uname().machine == 'Raspberry Pi Pico with RP2040':
    print("This tests were written for Raspberry Pi Pico RP2040!")


# Specific hardware configuration.
_I2C_CHAN = 1
_I2C_FREQ = 400000
_SCL_PIN = "GP19"
_SDA_PIN = "GP18"

# _OSR_RATES = [256, 512, 1024, 2048, 4096]
_OSR_RATES = [256, 512, 1024, 2048, 4096]
# [256, 512, 1024, 2048, 4096]

# Number of measurements per OSR for timing.
_N = 100


def calc_stats(data):
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


def time_n_reads(i2c, osr, n):
    print(f"Measuring timing of {n} samples with OSR={osr} ...")
    sensor = MS5837SensorBar02(
        i2c,
        pressure_osr=osr,
        temperature_osr=osr)

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

    # print(time_taken)
    return calc_stats(time_taken)


async def time_n_async_reads(i2c, osr, n):
    print(f"Measuring timing of {n} samples with OSR={osr} ...")
    sensor = MS5837SensorBar02(
        i2c,
        pressure_osr=osr,
        temperature_osr=osr)

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

    # print(time_taken)
    return calc_stats(time_taken)


def print_timing(data):
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
    for osr in _OSR_RATES:
        d = data[osr]
        print(format_str.format(
            *([osr] + [d[k] for k in dkeys])))


def benchmark_blocking_read():
    print("Measuring timing of blocking reads ...")

    i2c_obj = I2C(
        _I2C_CHAN,
        freq=_I2C_FREQ,
        sda=_SDA_PIN,
        scl=_SCL_PIN)

    data = {}
    for osr in _OSR_RATES:
        data[osr] = time_n_reads(
            i2c_obj, osr=osr, n=_N)

    # print(data)
    print_timing(data)


async def benchmark_async_read():
    print("Measuring timing of async reads ...")

    i2c_obj = I2C(
        _I2C_CHAN,
        freq=_I2C_FREQ,
        sda=_SDA_PIN,
        scl=_SCL_PIN)

    data = {}
    for osr in _OSR_RATES:
        data[osr] = await time_n_async_reads(i2c_obj, osr=osr, n=_N)

    # print(data)
    print_timing(data)


def run_benchmark_async_read():
    task = benchmark_async_read()
    asyncio.run(task)


def print_pressure_temperature():
    i2c_obj = I2C(
        _I2C_CHAN,
        freq=_I2C_FREQ,
        sda=_SDA_PIN,
        scl=_SCL_PIN)

    sensor = MS5837SensorBar02(
        i2c_obj,
        pressure_osr=4096,
        temperature_osr=4096)

    while True:
        try:
            p, t = sensor.read(second_order_compensation=True)
            print(f"Pressure: {p:.2f} KPa, Temp: {t:.2f} °C")
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
