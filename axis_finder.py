"""
Axis Finder & Live Tilt Monitor for GY-521 (MPU-6051M / MPU-6050)
MicroPython (Raspberry Pi Pico: SDA -> GP0, SCL -> GP1)

Uses +/-2000 dps gyro range (to prevent clipping during fast shakes) plus
the MPU-6051M's built-in 3-axis accelerometer in a complementary filter so X and Y
tilt angles never lose their zero point after shaking.
"""

import sys
import time
import math
import struct
from machine import Pin, I2C

PIN_SDA, PIN_SCL = 0, 1
PIN_IN1, PIN_IN2 = 14, 15

SAMPLE_HZ = 250
DT = 1.0 / SAMPLE_HZ
COMP_ALPHA = 0.985       # 98.5% gyro (fast response) + 1.5% accel (locks zero point to gravity)
RAD_TO_DEG = 180.0 / math.pi

_MPU_SMPLRT_DIV = 0x19
_MPU_CONFIG = 0x1A
_MPU_GYRO_CONFIG = 0x1B
_MPU_ACCEL_CONFIG = 0x1C
_MPU_ACCEL_XOUT_H = 0x3B
_MPU_PWR_MGMT_1 = 0x6B
_MPU_WHO_AM_I = 0x75
_GYRO_DPS_PER_LSB = 1.0 / 16.4   # +/-2000 dps full-scale (prevents clipping on hard shakes)


class MPU6050SixAxis:
    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = None
        scanned = i2c.scan()
        for a in (0x68, 0x69):
            if a in scanned:
                try:
                    i2c.readfrom_mem(a, _MPU_WHO_AM_I, 1)
                    self.addr = a
                    break
                except OSError:
                    pass
        if self.addr is None:
            raise RuntimeError("MPU-6051M / GY-521 not found at 0x68 or 0x69.")

        self._buf = bytearray(14)
        i2c.writeto_mem(self.addr, _MPU_PWR_MGMT_1, b'\x80')
        time.sleep_ms(50)
        i2c.writeto_mem(self.addr, _MPU_PWR_MGMT_1, b'\x01')
        time.sleep_ms(20)
        i2c.writeto_mem(self.addr, _MPU_CONFIG, b'\x01')
        i2c.writeto_mem(self.addr, _MPU_SMPLRT_DIV, b'\x00')
        i2c.writeto_mem(self.addr, _MPU_GYRO_CONFIG, b'\x18')   # +/-2000 dps
        i2c.writeto_mem(self.addr, _MPU_ACCEL_CONFIG, b'\x08')  # +/-4g
        time.sleep_ms(100)

    def read_accel_gyro(self):
        """Read 14 bytes (accel X,Y,Z + temp + gyro X,Y,Z) in a single I2C burst."""
        self.i2c.readfrom_mem_into(self.addr, _MPU_ACCEL_XOUT_H, self._buf)
        ax, ay, az, _, gx, gy, gz = struct.unpack('>hhhhhhh', self._buf)
        return (
            ax, ay, az,
            gx * _GYRO_DPS_PER_LSB,
            gy * _GYRO_DPS_PER_LSB,
            gz * _GYRO_DPS_PER_LSB,
        )


def accel_tilt_xy(ax, ay, az):
    """Compute X and Y tilt angles (deg) from gravity vector."""
    tilt_x = math.atan2(ay, math.sqrt(ax * ax + az * az)) * RAD_TO_DEG
    tilt_y = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * RAD_TO_DEG
    return tilt_x, tilt_y


def calibrate_all_axes(imu, n=500):
    print("Calibrating all 3 axes (start flat & hold still for 2s)...")
    bx = by = bz = 0.0
    ax0 = ay0 = 0.0
    for _ in range(n):
        ax, ay, az, gx, gy, gz = imu.read_accel_gyro()
        tx, ty = accel_tilt_xy(ax, ay, az)
        ax0 += tx
        ay0 += ty
        bx += gx
        by += gy
        bz += gz
        time.sleep_ms(4)
    return bx / n, by / n, bz / n, ax0 / n, ay0 / n


def main():
    # Ensure DRV8871 motor driver stays off while testing axes
    Pin(PIN_IN1, Pin.OUT, value=0)
    Pin(PIN_IN2, Pin.OUT, value=0)

    i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=400000)
    imu = MPU6050SixAxis(i2c)
    bx, by, bz, ref_x, ref_y = calibrate_all_axes(imu)
    print("Calibration done! Tilt the board to see angles (Ctrl-C to stop):")

    deg_x = 0.0
    deg_y = 0.0
    deg_z = 0.0

    step_us = int(DT * 1000000)
    next_tick = time.ticks_us()
    tick = 0

    try:
        while True:
            ax, ay, az, gx, gy, gz = imu.read_accel_gyro()
            gx -= bx
            gy -= by
            gz -= bz

            # Accelerometer gravity reference (relative to starting flat pose)
            acc_x, acc_y = accel_tilt_xy(ax, ay, az)
            acc_x -= ref_x
            acc_y -= ref_y

            # Complementary filter locks X and Y zero point to gravity
            deg_x = COMP_ALPHA * (deg_x + gx * DT) + (1.0 - COMP_ALPHA) * acc_x
            deg_y = COMP_ALPHA * (deg_y + gy * DT) + (1.0 - COMP_ALPHA) * acc_y
            if abs(gz) > 0.5:
                deg_z += gz * DT

            tick += 1
            if tick >= 10:  # update display at 25 Hz on a single line
                tick = 0
                sys.stdout.write(
                    "\rX (Axis 0): %+6.1f deg | Y (Axis 1): %+6.1f deg | Z (Axis 2): %+6.1f deg   "
                    % (deg_x, deg_y, deg_z)
                )

            next_tick = time.ticks_add(next_tick, step_us)
            if time.ticks_diff(next_tick, time.ticks_us()) < 0:
                next_tick = time.ticks_us()
            else:
                while time.ticks_diff(next_tick, time.ticks_us()) > 0:
                    pass
    except KeyboardInterrupt:
        print("\nStopped.")


main()
