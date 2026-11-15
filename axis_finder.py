"""
Axis Finder & Live Tilt Monitor for GY-521 (MPU-6051M / MPU-6050)
MicroPython (Raspberry Pi Pico: SDA -> GP0, SCL -> GP1)

Calibrates all 3 gyro axes at rest (assuming the board starts flat at 0 deg)
and updates a single line in the terminal showing the integrated tilt angle (deg)
for X (GYRO_AXIS = 0), Y (GYRO_AXIS = 1), and Z (GYRO_AXIS = 2).
"""

import sys
import time
import struct
from machine import Pin, I2C

PIN_SDA, PIN_SCL = 0, 1
PIN_IN1, PIN_IN2 = 14, 15

SAMPLE_HZ = 250
DT = 1.0 / SAMPLE_HZ

_MPU_SMPLRT_DIV = 0x19
_MPU_CONFIG = 0x1A
_MPU_GYRO_CONFIG = 0x1B
_MPU_GYRO_XOUT_H = 0x43
_MPU_PWR_MGMT_1 = 0x6B
_MPU_WHO_AM_I = 0x75
_GYRO_DPS_PER_LSB = 1.0 / 65.5   # +/-500 dps full-scale


class MPU6050AllAxes:
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

        self._buf = bytearray(6)
        i2c.writeto_mem(self.addr, _MPU_PWR_MGMT_1, b'\x80')
        time.sleep_ms(50)
        i2c.writeto_mem(self.addr, _MPU_PWR_MGMT_1, b'\x01')
        time.sleep_ms(20)
        i2c.writeto_mem(self.addr, _MPU_CONFIG, b'\x01')
        i2c.writeto_mem(self.addr, _MPU_SMPLRT_DIV, b'\x00')
        i2c.writeto_mem(self.addr, _MPU_GYRO_CONFIG, b'\x08')
        time.sleep_ms(100)

    def read_xyz_dps(self):
        """Return (gx, gy, gz) angular velocity in deg/s."""
        self.i2c.readfrom_mem_into(self.addr, _MPU_GYRO_XOUT_H, self._buf)
        rx, ry, rz = struct.unpack('>hhh', self._buf)
        return (
            rx * _GYRO_DPS_PER_LSB,
            ry * _GYRO_DPS_PER_LSB,
            rz * _GYRO_DPS_PER_LSB,
        )


def calibrate_all_axes(imu, n=500):
    print("Calibrating all 3 axes (start flat & hold still for 2s)...")
    bx = by = bz = 0.0
    for _ in range(n):
        gx, gy, gz = imu.read_xyz_dps()
        bx += gx
        by += gy
        bz += gz
        time.sleep_ms(4)
    return bx / n, by / n, bz / n


def main():
    # Ensure DRV8871 motor driver stays off while testing axes
    Pin(PIN_IN1, Pin.OUT, value=0)
    Pin(PIN_IN2, Pin.OUT, value=0)

    i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=400000)
    imu = MPU6050AllAxes(i2c)
    bx, by, bz = calibrate_all_axes(imu)
    print("Calibration done! Tilt the board to see angles (Ctrl-C to stop):")

    deg_x = 0.0
    deg_y = 0.0
    deg_z = 0.0

    step_us = int(DT * 1000000)
    next_tick = time.ticks_us()
    tick = 0

    try:
        while True:
            gx, gy, gz = imu.read_xyz_dps()
            gx -= bx
            gy -= by
            gz -= bz

            # Ignore tiny resting noise (< 0.5 deg/s) to minimize stationary drift
            if abs(gx) > 0.5:
                deg_x += gx * DT
            if abs(gy) > 0.5:
                deg_y += gy * DT
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
