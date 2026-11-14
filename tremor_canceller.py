"""
Tremor canceller: LSM6DSOX gyro -> band-pass (1st-order HP + 2nd-order LP) + PD -> DRV8835 -> DAEX25FHE-4 Exciter
MicroPython (written for Raspberry Pi Pico / RP2040; adjust pins for other boards)

WIRING
  LSM6DSOX (Qwiic/I2C):  SDA -> GP0, SCL -> GP1, 3V3, GND   (address 0x6A or 0x6B, auto-detected)

  DRV8835 Motor Driver (auto-detects Qwiic I2C or direct GPIO PWM):
    Option A - SparkFun Qwiic Motor Driver (ROB-15451, I2C):
      Qwiic cable       -> Daisy-chained on GP0 (SDA) / GP1 (SCL), 3V3, GND (default addr 0x5D)
      VIN / GND         -> Motor supply (3.0 - 5.0 V recommended) and common ground
      A1 / A2           -> Dayton Audio DAEX25FHE-4 exciter terminals

    Option B - Direct DRV8835 GPIO (PHASE/ENABLE mode, channel A):
      MODE              -> GP13  (driven HIGH by this script; or tie to VCC)
      APHASE (AIN1)     -> GP14  (direction)
      AENABLE (AIN2)    -> GP15  (PWM = magnitude)
      VCC / VM / GND    -> 3V3 logic, 3.0 - 5.0 V motor supply, common ground
      AOUT1 / AOUT2     -> Dayton Audio DAEX25FHE-4 exciter terminals

EXCITER & DRIVER SAFETY
  Dayton Audio DAEX25FHE-4 (4 ohm, 24 W RMS) driven by TI DRV8835 (1.2 A cont / 1.5 A peak per channel):
    * caps output voltage to SPEAKER_MAX_V (4.8 V -> <= 1.2 A peak into 4 ohm)
    * runs a running-power limiter (SPEAKER_CONT_W = 2.5 W) to protect the DRV8835 thermally
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
OUTPUT_SIGN = 1          # flip to -1 if the exciter pushes WITH the tremor instead of against it
ENABLE_OUTPUT = True     # False = telemetry only (motor driver never driven) for bring-up

# Loop timing
SAMPLE_HZ = 250
DT = 1.0 / SAMPLE_HZ

# Tremor band (physiological tremor is typically ~4-12 Hz; essential tremor ~4-8 Hz)
BAND_LOW_HZ = 3.0        # 1st-order high-pass corner: rejects slow voluntary movement with minimal phase lead
BAND_HIGH_HZ = 12.0      # 2nd-order low-pass corner: rejects high-frequency noise / mechanical ringing

# Control gains (tau_d = KD / KP = 0.008 s matches the LP filter lag across 4-10 Hz)
KP = 1.0                 # cancellation strength. Tune upward/downward as needed
KD = 0.008               # phase lead (KP * 0.008 s) to offset low-pass + mechanical lag
DEADZONE_DPS = 3.0       # soft deadzone, deg/s (stops idle buzz)
FULL_SCALE_DPS = 60.0    # (KP*tremor + KD*accel) of this size = 100% command

# Power / driver limits (Dayton Audio DAEX25FHE-4: 4 ohm, 24 W RMS; DRV8835: 1.2 A continuous)
VM_VOLTS = 5.0           # actual voltage on the DRV8835 VM/VIN pin (MEASURE IT)
SPEAKER_OHMS = 4.0       # DAEX25FHE-4 nominal impedance (Re = 4.3 ohm)
SPEAKER_MAX_V = 4.8      # 4.8 V / 4.0 ohm = 1.2 A peak (DRV8835 continuous current rating)
SPEAKER_CONT_W = 2.5     # continuous average power budget (protects DRV8835 IC thermally)
POWER_WINDOW_S = 2.0     # averaging window for the power limiter
PWM_HZ = 20000           # above hearing range (used in direct GPIO PWM mode)

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
# 3. DRV8835 EXCITER DRIVER (Auto-detects SparkFun Qwiic I2C or Direct GPIO PWM)
# ==============================================================================
_SCMD_ID_REG = 0x01
_SCMD_ID_VAL = 0xA9
_SCMD_MA_DRIVE = 0x20
_SCMD_MB_DRIVE = 0x21
_SCMD_DRIVER_ENABLE = 0x70
_SCMD_STATUS_1 = 0x77
_SCMD_ADDRS = (0x5D, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63)


class Speaker:
    def __init__(self):
        # Always initialize GPIO pins to safe zero state immediately
        self.mode = Pin(PIN_MODE, Pin.OUT, value=1)    # HIGH = PHASE/ENABLE mode
        self.phase = Pin(PIN_PHASE, Pin.OUT, value=0)
        self.en = PWM(Pin(PIN_ENABLE))
        self.en.freq(PWM_HZ)
        self.en.duty_u16(0)

        self.i2c = None
        self.scmd_addr = None
        self._tx = bytearray(1)

    def attach_i2c(self, i2c):
        """Detect SparkFun Qwiic Motor Driver (SCMD / DRV8835) on I2C if connected."""
        scanned = i2c.scan()
        for a in _SCMD_ADDRS:
            if a in scanned:
                try:
                    if i2c.readfrom_mem(a, _SCMD_ID_REG, 1)[0] == _SCMD_ID_VAL:
                        self.i2c = i2c
                        self.scmd_addr = a
                        break
                except OSError:
                    pass

        if self.scmd_addr is not None:
            # Wait for PSoC enumeration & busy flags to clear (up to 500 ms)
            for _ in range(100):
                status = self.i2c.readfrom_mem(self.scmd_addr, _SCMD_STATUS_1, 1)[0]
                if status != 0xFF and (status & 0x7F) == 0:
                    break
                time.sleep_ms(5)
            # Zero both motor channels (128 = stop) before enabling H-bridges
            self._tx[0] = 128
            self.i2c.writeto_mem(self.scmd_addr, _SCMD_MA_DRIVE, self._tx)
            self.i2c.writeto_mem(self.scmd_addr, _SCMD_MB_DRIVE, self._tx)
            self._tx[0] = 1
            self.i2c.writeto_mem(self.scmd_addr, _SCMD_DRIVER_ENABLE, self._tx)
            print("SparkFun Qwiic DRV8835 found at", hex(self.scmd_addr), "(I2C mode)")
        else:
            print("No Qwiic motor driver on I2C; using direct GPIO PWM on GP%d/GP%d/GP%d" %
                  (PIN_MODE, PIN_PHASE, PIN_ENABLE))

    def drive(self, cmd):
        """cmd in [-1, 1]. Returns the voltage actually applied (signed, volts)."""
        if cmd > 1.0:
            cmd = 1.0
        elif cmd < -1.0:
            cmd = -1.0

        if self.scmd_addr is not None:
            # SCMD 7-bit signed drive: 128 = stop, 1..127 = reverse, 129..255 = forward
            signed_duty = cmd * MAX_DUTY
            level = int(round(signed_duty * 127.0))
            if level > 127:
                level = 127
            elif level < -127:
                level = -127
            self._tx[0] = 128 + level
            self.i2c.writeto_mem(self.scmd_addr, _SCMD_MA_DRIVE, self._tx)
            return (level / 127.0) * VM_VOLTS

        duty = abs(cmd) * MAX_DUTY
        self.phase.value(1 if cmd >= 0 else 0)
        self.en.duty_u16(int(duty * 65535))
        return duty * VM_VOLTS * (1 if cmd >= 0 else -1)

    def stop(self):
        self.en.duty_u16(0)
        if self.scmd_addr is not None:
            try:
                self._tx[0] = 128
                self.i2c.writeto_mem(self.scmd_addr, _SCMD_MA_DRIVE, self._tx)
                self._tx[0] = 0
                self.i2c.writeto_mem(self.scmd_addr, _SCMD_DRIVER_ENABLE, self._tx)
            except OSError:
                pass


# ==============================================================================
# 4. BUTTERWORTH FILTERS (1st-order HP & 2nd-order RBJ Biquads, DF-II Transposed)
# ==============================================================================
class Biquad:
    def __init__(self, kind, fc, fs, q=0.70710678):
        if kind == 'hp1':
            # 1st-order Butterworth high-pass (bilinear transform with frequency pre-warping)
            # Cuts phase lead in half (+36.9 deg vs +67.0 deg at 4 Hz for fc=3 Hz)
            k = math.tan(math.pi * fc / fs)
            self.b0 = 1.0 / (1.0 + k)
            self.b1 = -self.b0
            self.b2 = 0.0
            self.a1 = (k - 1.0) / (k + 1.0)
            self.a2 = 0.0
        else:
            w0 = 2.0 * math.pi * fc / fs
            cw, sw = math.cos(w0), math.sin(w0)
            alpha = sw / (2.0 * q)
            if kind == 'lp':
                b0 = (1.0 - cw) / 2.0
                b1 = 1.0 - cw
                b2 = b0
            else:  # 'hp' (2nd-order)
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
        spk.attach_i2c(i2c)
        print("Max duty %.0f%%  (%.2f V peak at VM=%.1f V)" % (MAX_DUTY * 100, MAX_DUTY * VM_VOLTS, VM_VOLTS))

        bias = calibrate_bias(imu)

        # 1st-order high-pass (3 Hz) + 2nd-order low-pass (12 Hz)
        hp = Biquad('hp1', BAND_LOW_HZ, SAMPLE_HZ)
        lp = Biquad('lp', BAND_HIGH_HZ, SAMPLE_HZ)

        # Settle the filters at exact SAMPLE_HZ pacing with output disabled (~0.5 s)
        step_us = int(DT * 1000000)
        next_tick = time.ticks_us()
        for _ in range(SAMPLE_HZ // 2):
            lp.step(hp.step(imu.read_dps() - bias))
            next_tick = time.ticks_add(next_tick, step_us)
            while time.ticks_diff(next_tick, time.ticks_us()) > 0:
                pass

        if USE_WATCHDOG:
            from machine import WDT
            wdt = WDT(timeout=1000)

        last_tremor = 0.0
        accel_s = 0.0
        mean_sq_v = 0.0
        pw_alpha = DT / POWER_WINDOW_S
        deadzone_pd = KP * DEADZONE_DPS
        fails = 0
        overruns = 0
        tick = 0
        out_scale = 1.0

        gc.collect()
        print("Running. Ctrl-C to stop.  KP=%.2f KD=%.4f sign=%+d output=%s" % (KP, KD, OUTPUT_SIGN, ENABLE_OUTPUT))
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

                # ---- 4. inverted PD + continuous soft deadzone ---------------
                # Combine proportional and phase-lead terms BEFORE the deadzone
                # so the output transitions smoothly through zero without step jumps.
                pd_dps = KP * tremor + KD * accel_s
                mag = abs(pd_dps) - deadzone_pd
                if mag <= 0.0:
                    cmd = 0.0
                else:
                    s = 1.0 if pd_dps > 0.0 else -1.0
                    cmd = -OUTPUT_SIGN * (s * mag) / FULL_SCALE_DPS

                # ---- 5. running power limiter --------------------------------
                cmd *= out_scale
                try:
                    if ENABLE_OUTPUT:
                        v = spk.drive(cmd)
                    else:
                        spk.stop()
                        v = 0.0
                except OSError:
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
        spk.stop()   # ALWAYS de-energize the exciter, whatever happened


main()
