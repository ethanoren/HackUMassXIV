"""
Tremor canceller: LSM6DSOX gyro  ->  band-pass + PD  ->  SparkFun DRV8835  ->  8 ohm speaker
MicroPython (written for Raspberry Pi Pico / RP2040; adjust pins for other boards)

WIRING
  LSM6DSOX (Qwiic/I2C):  SDA -> GP0, SCL -> GP1, 3V3, GND   (address 0x6A or 0x6B, auto-detected)

  DRV8835 (used in PHASE/ENABLE mode, channel A only):
      MODE            -> GP13  (driven HIGH by this script; or tie to VCC)
      APHASE (AIN1)   -> GP14  (direction)
      AENABLE (AIN2)  -> GP15  (PWM = magnitude)
      VCC             -> 3V3 (logic)
      VM              -> motor supply, 3.0 - 5 V recommended (set VM_VOLTS below to match!)
      AOUT1 / AOUT2   -> speaker terminals
      GND             -> common ground with the Pico

  In PHASE/ENABLE mode, ENABLE low = brake (slow decay), which gives clean, crisp reversals.

SPEAKER SAFETY
  8 ohm / 500 mW  ->  absolute limit ~2.0 V DC-equivalent.  This script:
    * caps the PWM duty so the output never exceeds SPEAKER_MAX_V
    * runs a running-power limiter that scales output down if average power gets too high
    * cuts output to zero on any sensor error or program exit
"""

import time
import math
import gc
import struct
from machine import Pin, I2C, PWM

# ==============================================================================
# 1. CONFIGURATION
# ==============================================================================
GYRO_AXIS = 0            # 0 = X, 1 = Y, 2 = Z  (axis the tremor rotates about)
OUTPUT_SIGN = 1          # flip to -1 if the speaker pushes WITH the tremor instead of against it
ENABLE_OUTPUT = True     # False = telemetry only (motor driver never driven) for bring-up

# Loop timing
SAMPLE_HZ = 250
DT = 1.0 / SAMPLE_HZ

# Tremor band (physiological tremor is typically ~4-12 Hz; essential tremor ~4-8 Hz)
BAND_LOW_HZ = 3.0        # high-pass corner: rejects slow voluntary movement
BAND_HIGH_HZ = 12.0      # low-pass corner: rejects noise / vibration

# Control gains
KP = 0.5                 # cancellation strength. START LOW (0.3 - 0.5), raise slowly
KD = 0.004               # phase lead (seconds) to offset filter + mechanical lag
DEADZONE_DPS = 3.0       # soft deadzone, deg/s (stops idle buzz)
FULL_SCALE_DPS = 100.0   # (KP*tremor + KD*accel) of this size = 100% command

# Power / driver limits
VM_VOLTS = 5.0           # actual voltage on the DRV8835 VM pin (MEASURE IT)
SPEAKER_OHMS = 8.0
SPEAKER_MAX_V = 2.0      # sqrt(0.5 W * 8 ohm) = 2.0 V peak DC-equivalent
SPEAKER_CONT_W = 0.25    # continuous average power budget (half of rated, for margin)
POWER_WINDOW_S = 2.0     # averaging window for the power limiter
PWM_HZ = 20000           # above hearing range

MAX_DUTY = min(1.0, SPEAKER_MAX_V / VM_VOLTS)
MAX_MEAN_SQ_V = SPEAKER_CONT_W * SPEAKER_OHMS    # allowed mean(V^2) in volts^2

# Pins
PIN_SDA, PIN_SCL = 0, 1
PIN_MODE, PIN_PHASE, PIN_ENABLE = 13, 14, 15

# Safety
MAX_CONSECUTIVE_I2C_FAILS = 10
USE_WATCHDOG = False     # True = reset board if loop hangs (note: Ctrl-C in Thonny will
                         # then reboot the board ~1 s later, since a WDT cannot be stopped)

# ==============================================================================
# 2. LSM6DSOX DRIVER (gyro only)
# ==============================================================================
_WHO_AM_I = 0x0F
_CTRL1_XL = 0x10
_CTRL2_G = 0x11
_CTRL3_C = 0x12
_OUTX_L_G = 0x22
_LSM6DSOX_ID = 0x6C
_GYRO_DPS_PER_LSB = 0.0175   # 17.50 mdps/LSB at +/-500 dps


class LSM6DSOX:
    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = None
        for a in (0x6A, 0x6B):
            if a in i2c.scan():
                try:
                    if i2c.readfrom_mem(a, _WHO_AM_I, 1)[0] == _LSM6DSOX_ID:
                        self.addr = a
                        break
                except OSError:
                    pass
        if self.addr is None:
            raise RuntimeError("LSM6DSOX not found. Check wiring / I2C address.")

        self._buf = bytearray(6)
        # Software reset, then configure
        i2c.writeto_mem(self.addr, _CTRL3_C, b'\x01')
        time.sleep_ms(20)
        i2c.writeto_mem(self.addr, _CTRL3_C, b'\x44')   # BDU=1, IF_INC=1
        i2c.writeto_mem(self.addr, _CTRL1_XL, b'\x00')  # accelerometer off (not used)
        # ODR_G = 833 Hz (0111), FS_G = 500 dps (01) -> 0x74
        i2c.writeto_mem(self.addr, _CTRL2_G, b'\x74')
        time.sleep_ms(100)  # let gyro settle after power-up

    def read_dps(self):
        """Angular velocity (deg/s) on GYRO_AXIS."""
        self.i2c.readfrom_mem_into(self.addr, _OUTX_L_G, self._buf)
        raw = struct.unpack_from('<h', self._buf, GYRO_AXIS * 2)[0]
        return raw * _GYRO_DPS_PER_LSB


# ==============================================================================
# 3. DRV8835 SPEAKER DRIVER (PHASE/ENABLE mode)
# ==============================================================================
class Speaker:
    def __init__(self):
        self.mode = Pin(PIN_MODE, Pin.OUT, value=1)    # HIGH = PHASE/ENABLE mode
        self.phase = Pin(PIN_PHASE, Pin.OUT, value=0)
        self.en = PWM(Pin(PIN_ENABLE))
        self.en.freq(PWM_HZ)
        self.en.duty_u16(0)

    def drive(self, cmd):
        """cmd in [-1, 1]. Returns the voltage actually applied (signed, volts)."""
        if cmd > 1.0:
            cmd = 1.0
        elif cmd < -1.0:
            cmd = -1.0
        duty = abs(cmd) * MAX_DUTY
        self.phase.value(1 if cmd >= 0 else 0)
        self.en.duty_u16(int(duty * 65535))
        return duty * VM_VOLTS * (1 if cmd >= 0 else -1)

    def stop(self):
        self.en.duty_u16(0)


# ==============================================================================
# 4. SECOND-ORDER BUTTERWORTH FILTERS (RBJ biquads, Direct Form II Transposed)
# ==============================================================================
class Biquad:
    def __init__(self, kind, fc, fs, q=0.70710678):
        w0 = 2.0 * math.pi * fc / fs
        cw, sw = math.cos(w0), math.sin(w0)
        alpha = sw / (2.0 * q)
        if kind == 'lp':
            b0 = (1.0 - cw) / 2.0
            b1 = 1.0 - cw
            b2 = b0
        else:  # 'hp'
            b0 = (1.0 + cw) / 2.0
            b1 = -(1.0 + cw)
            b2 = b0
        a0 = 1.0 + alpha
        self.b0, self.b1, self.b2 = b0 / a0, b1 / a0, b2 / a0
        self.a1, self.a2 = (-2.0 * cw) / a0, (1.0 - alpha) / a0
        self.z1 = 0.0
        self.z2 = 0.0

    def step(self, x):
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y


# ==============================================================================
# 5. MAIN
# ==============================================================================
def calibrate_bias(imu, n=500):
    """Average gyro at rest to remove zero-rate offset. Keep the device STILL."""
    print("Calibrating gyro bias - hold still...")
    total = 0.0
    lo, hi = 1e9, -1e9
    for _ in range(n):
        v = imu.read_dps()
        total += v
        lo = min(lo, v)
        hi = max(hi, v)
        time.sleep_ms(4)
    bias = total / n
    if (hi - lo) > 6.0:
        print("WARNING: device moved during calibration (spread %.1f dps). Restart for best results." % (hi - lo))
    print("Gyro bias: %.2f dps" % bias)
    return bias


def main():
    spk = Speaker()          # outputs forced to zero immediately
    wdt = None
    try:
        i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=400000)
        imu = LSM6DSOX(i2c)
        print("LSM6DSOX found at", hex(imu.addr))
        print("Max duty %.0f%%  (%.2f V peak at VM=%.1f V)" % (MAX_DUTY * 100, MAX_DUTY * VM_VOLTS, VM_VOLTS))

        bias = calibrate_bias(imu)

        hp = Biquad('hp', BAND_LOW_HZ, SAMPLE_HZ)
        lp = Biquad('lp', BAND_HIGH_HZ, SAMPLE_HZ)

        # Settle the filters with output disabled (~0.5 s)
        for _ in range(SAMPLE_HZ // 2):
            lp.step(hp.step(imu.read_dps() - bias))
            time.sleep_ms(4)

        if USE_WATCHDOG:
            from machine import WDT
            wdt = WDT(timeout=1000)

        last_tremor = 0.0
        accel_s = 0.0
        mean_sq_v = 0.0
        pw_alpha = DT / POWER_WINDOW_S
        fails = 0
        overruns = 0
        tick = 0
        out_scale = 1.0

        gc.collect()
        print("Running. Ctrl-C to stop.  KP=%.2f KD=%.4f sign=%+d output=%s" % (KP, KD, OUTPUT_SIGN, ENABLE_OUTPUT))
        step_us = int(DT * 1000000)
        next_tick = time.ticks_us()

        while True:
            # ---- 1. sense -----------------------------------------------------
            try:
                raw = imu.read_dps() - bias
                fails = 0
            except OSError:
                fails += 1
                spk.stop()
                if fails >= MAX_CONSECUTIVE_I2C_FAILS:
                    raise RuntimeError("IMU communication lost")
                raw = None

            if raw is not None:
                # ---- 2. band-pass (3-12 Hz) ----------------------------------
                tremor = lp.step(hp.step(raw))

                # ---- 3. derivative for phase lead (lightly smoothed) ---------
                accel = (tremor - last_tremor) / DT
                last_tremor = tremor
                accel_s += 0.3 * (accel - accel_s)

                # ---- 4. soft deadzone + inverted PD --------------------------
                mag = abs(tremor) - DEADZONE_DPS
                if mag <= 0.0:
                    cmd = 0.0
                else:
                    s = 1.0 if tremor > 0 else -1.0
                    cmd = -OUTPUT_SIGN * (KP * s * mag + KD * accel_s) / FULL_SCALE_DPS

                # ---- 5. running power limiter --------------------------------
                cmd *= out_scale
                if ENABLE_OUTPUT:
                    v = spk.drive(cmd)
                else:
                    spk.stop()
                    v = 0.0
                mean_sq_v += pw_alpha * (v * v - mean_sq_v)
                if mean_sq_v > MAX_MEAN_SQ_V:
                    out_scale = max(0.1, out_scale * 0.98)   # back off
                elif out_scale < 1.0:
                    out_scale = min(1.0, out_scale * 1.002)  # recover slowly

                # ---- 6. telemetry (~25 Hz) for Thonny Plotter ----------------
                tick += 1
                if tick >= 10:
                    tick = 0
                    print("Gyro:%.1f,Tremor:%.1f,Out:%.1f,Scale:%.2f" %
                          (raw, tremor, cmd * 100.0, out_scale))

            if wdt:
                wdt.feed()

            # ---- 7. pace the loop; collect garbage in spare time ---------------
            next_tick = time.ticks_add(next_tick, step_us)
            if time.ticks_diff(next_tick, time.ticks_us()) < 0:
                overruns += 1
                next_tick = time.ticks_us()          # resync, don't spiral
                if overruns % 50 == 1:
                    print("WARNING: loop overrun x%d" % overruns)
            else:
                if tick == 5:
                    gc.collect()
                while time.ticks_diff(next_tick, time.ticks_us()) > 0:
                    pass

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        spk.stop()   # ALWAYS de-energize the speaker, whatever happened


main()
