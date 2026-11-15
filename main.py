"""
Tremor canceller: MPU-6051M / MPU-6050 (GY-521) gyro -> band-pass (1st-order HP + 2nd-order LP) + PD -> Adafruit DRV8871 -> Speaker/Exciter
MicroPython (written for Raspberry Pi Pico / RP2040; adjust pins for other boards)

WIRING
  MPU-6051M / MPU-6050 on GY-521 breakout (I2C0):
      VCC             -> 3V3(OUT) (Physical Pin 36)
      GND             -> GND      (Physical Pin 3)
      SCL             -> GP1      (Physical Pin 2)
      SDA             -> GP0      (Physical Pin 1)
      AD0             -> GND or leave floating for 0x68 (or 3V3 for 0x69; auto-detected)

  Adafruit DRV8871 Motor Driver (dual-PWM slow-decay/brake mode):
      IN1             -> GP14     (Physical Pin 19, PWM7 A)
      IN2             -> GP15     (Physical Pin 20, PWM7 B)
      VM  (or + term) -> Boost converter output (6.5 V - 45 V; set VM_VOLTS below to match!)
      GND (or - term) -> Common ground with Pico and boost converter
      MOTOR OUT1/OUT2 -> Speaker / exciter terminals

SPEAKER & DRIVER SAFETY
  Adafruit DRV8871 has built-in 2.0 A current limiting and thermal shutdown.
  In addition, this script:
    * scales PWM duty by VM_VOLTS so peak voltage never exceeds SPEAKER_MAX_V
    * runs a rolling thermal power limiter (SPEAKER_CONT_W) over POWER_WINDOW_S
    * cuts both IN1 and IN2 to 0 (coast / sleep) on any sensor error or program exit
"""

import time
import math
import gc
import struct
from machine import Pin, I2C, PWM

time.sleep(1)

# ==============================================================================
# 1. CONFIGURATION
# ==============================================================================
USE_BOTH_XY_AXES = True  # True = smooth energy-weighted X+Y blending; False = single GYRO_AXIS
GYRO_AXIS = 0            # Fallback axis (0 = X, 1 = Y, 2 = Z) if USE_BOTH_XY_AXES is False
OUTPUT_SIGN = 1          # flip to -1 if the actuator pushes WITH the tremor instead of against it
ENABLE_OUTPUT = True     # False = telemetry only (motor driver never driven) for bring-up

# Loop timing
SAMPLE_HZ = 250
DT = 1.0 / SAMPLE_HZ

# Tremor band (physiological tremor is typically ~4-12 Hz; essential tremor ~4-8 Hz)
BAND_LOW_HZ = 3.0        # 1st-order high-pass corner: rejects slow voluntary movement with minimal phase lead
BAND_HIGH_HZ = 12.0      # 2nd-order low-pass corner: rejects high-frequency noise / mechanical ringing

# Control gains (tau_d = KD / KP = 0.004 s for phase lead without exciting speaker rattle)
KP = 1.0                 # cancellation strength. Tune upward/downward as needed
KD = 0.004               # phase lead (seconds) to offset filter + mechanical lag
DEADZONE_DPS = 4.0       # soft deadzone, deg/s (stops idle buzz and mechanical rattle feedback)
FULL_SCALE_DPS = 60.0    # (KP*tremor + KD*accel) of this size = 100% command

# Feedback-loop mitigation (detects sustained self-oscillation and throttles gain)
FB_TRIGGER_CMD = 0.25    # sustained command envelope threshold indicating feedback oscillation
FB_MIN_SCALE = 0.25      # minimum backoff gain during feedback suppression to break the loop

# Power / driver limits (4 ohm, 5W speaker powered by 12V supply through DRV8871)
VM_VOLTS = 12.0          # actual voltage on DRV8871 VM pin (12V supply)
SPEAKER_OHMS = 4.0       # 4 ohm, 5W speaker nominal impedance
SPEAKER_MAX_V = 6.0      # 6.0 V peak -> 1.5 A peak & 4.5 W RMS sine burst into 4 ohm (50% duty at 12V)
SPEAKER_CONT_W = 2.5     # 2.5 W continuous average thermal budget (50% of 5W speaker rating)
POWER_WINDOW_S = 1.0     # 1.0 s window before thermal limiter forces burst power down
PWM_HZ = 20000           # 20 kHz (ultrasonic; DRV8871 supports up to 200 kHz)

MAX_DUTY = min(1.0, SPEAKER_MAX_V / VM_VOLTS)
MAX_MEAN_SQ_V = SPEAKER_CONT_W * SPEAKER_OHMS    # allowed mean(V^2) in volts^2

# Pins
PIN_SDA, PIN_SCL = 0, 1
PIN_IN1, PIN_IN2 = 14, 15

# Safety
MAX_CONSECUTIVE_I2C_FAILS = 10
USE_WATCHDOG = False     # True = reset board if loop hangs (note: Ctrl-C in Thonny will
                         # then reboot the board ~1 s later, since a WDT cannot be stopped)

# ==============================================================================
# 2. MPU-6051M / MPU-6050 DRIVER (GY-521 breakout, gyro only)
# ==============================================================================
_MPU_SMPLRT_DIV = 0x19
_MPU_CONFIG = 0x1A
_MPU_GYRO_CONFIG = 0x1B
_MPU_GYRO_XOUT_H = 0x43
_MPU_PWR_MGMT_1 = 0x6B
_MPU_WHO_AM_I = 0x75
_GYRO_DPS_PER_LSB = 1.0 / 16.4   # 16.4 LSB/(deg/s) at +/-2000 dps (~0.060976 dps/LSB)


class MPU6050:
    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = None
        self.who_am_i = None
        # Retry for up to 3 seconds on cold VSYS power-up while GY-521 LDO powers up
        for _ in range(30):
            scanned = i2c.scan()
            for a in (0x68, 0x69):
                if a in scanned:
                    try:
                        self.who_am_i = i2c.readfrom_mem(a, _MPU_WHO_AM_I, 1)[0]
                        self.addr = a
                        break
                    except OSError:
                        pass
            if self.addr is not None:
                break
            time.sleep_ms(100)
        if self.addr is None:
            raise RuntimeError("MPU-6051M / GY-521 not found at 0x68 or 0x69. Check wiring.")

        self._buf = bytearray(6)
        # Device reset, then wake from sleep and select X-axis gyro PLL clock (0x01)
        i2c.writeto_mem(self.addr, _MPU_PWR_MGMT_1, b'\x80')
        time.sleep_ms(50)
        i2c.writeto_mem(self.addr, _MPU_PWR_MGMT_1, b'\x01')
        time.sleep_ms(20)
        # DLPF_CFG = 1 (188 Hz bandwidth, 1 kHz internal rate), SMPLRT_DIV = 0 (1 kHz)
        i2c.writeto_mem(self.addr, _MPU_CONFIG, b'\x01')
        i2c.writeto_mem(self.addr, _MPU_SMPLRT_DIV, b'\x00')
        # FS_SEL = 3 (+/-2000 dps -> 0x18) so vigorous shaking never clips/saturates
        i2c.writeto_mem(self.addr, _MPU_GYRO_CONFIG, b'\x18')
        time.sleep_ms(100)  # let gyro PLL and DLPF settle

    def read_dps(self):
        """Angular velocity (deg/s) on GYRO_AXIS. MPU-605x registers are big-endian (>h)."""
        self.i2c.readfrom_mem_into(self.addr, _MPU_GYRO_XOUT_H, self._buf)
        raw = struct.unpack_from('>h', self._buf, GYRO_AXIS * 2)[0]
        return raw * _GYRO_DPS_PER_LSB

    def read_xy_dps(self):
        """Angular velocity (gx, gy) in deg/s on both X and Y axes in a single I2C read."""
        self.i2c.readfrom_mem_into(self.addr, _MPU_GYRO_XOUT_H, self._buf)
        rx, ry = struct.unpack_from('>hh', self._buf, 0)
        return rx * _GYRO_DPS_PER_LSB, ry * _GYRO_DPS_PER_LSB


# ==============================================================================
# 3. ADAFRUIT DRV8871 DRIVER (Dual-PWM with Slow-Decay / Brake Mode)
# ==============================================================================
class Speaker:
    def __init__(self):
        # GP14 (PWM7 A) and GP15 (PWM7 B) share hardware slice 7 for phase-locked PWM
        self.in1 = PWM(Pin(PIN_IN1))
        self.in2 = PWM(Pin(PIN_IN2))
        self.in1.freq(PWM_HZ)
        self.in2.freq(PWM_HZ)
        self.in1.duty_u16(0)
        self.in2.duty_u16(0)

    def drive(self, cmd):
        """
        cmd in [-1, 1]. Returns the voltage actually applied (signed, volts).
        Uses drive + slow-decay brake (IN1=1, IN2=1 during PWM off-time) for
        crisp, linear bidirectional current reversals across the voice coil.
        """
        if cmd > 1.0:
            cmd = 1.0
        elif cmd < -1.0:
            cmd = -1.0
        duty = abs(cmd) * MAX_DUTY
        off_u16 = int((1.0 - duty) * 65535)
        if cmd >= 0.0:
            self.in1.duty_u16(65535)
            self.in2.duty_u16(off_u16)
            return duty * VM_VOLTS
        else:
            self.in1.duty_u16(off_u16)
            self.in2.duty_u16(65535)
            return -duty * VM_VOLTS

    def stop(self):
        """Both inputs LOW = coast / low-power sleep (zero current through speaker)."""
        self.in1.duty_u16(0)
        self.in2.duty_u16(0)


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
    """Average gyro X and Y at rest to remove zero-rate offset. Keep the device STILL."""
    print("Calibrating gyro bias (X & Y) - hold still...")
    total_x = 0.0
    total_y = 0.0
    lo_x, hi_x = 1e9, -1e9
    lo_y, hi_y = 1e9, -1e9
    for _ in range(n):
        vx, vy = imu.read_xy_dps()
        total_x += vx
        total_y += vy
        lo_x = min(lo_x, vx)
        hi_x = max(hi_x, vx)
        lo_y = min(lo_y, vy)
        hi_y = max(hi_y, vy)
        time.sleep_ms(4)
    bias_x = total_x / n
    bias_y = total_y / n
    spread = max(hi_x - lo_x, hi_y - lo_y)
    if spread > 6.0:
        print("WARNING: device moved during calibration (spread %.1f dps). Restart for best results." % spread)
    print("Gyro bias: X=%.2f dps, Y=%.2f dps" % (bias_x, bias_y))
    return bias_x, bias_y


def main():
    spk = Speaker()          # outputs forced to zero immediately
    wdt = None
    try:
        led = Pin("LED", Pin.OUT)
    except Exception:
        led = Pin(25, Pin.OUT)
    led.value(1)             # solid ON during boot & gyro calibration
    try:
        i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=400000)
        imu = MPU6050(i2c)
        print("GY-521 / MPU-6051M found at %s (WHO_AM_I=%s)" % (hex(imu.addr), hex(imu.who_am_i)))
        print("DRV8871 PWM on GP%d/GP%d | Max duty %.0f%% (%.2f V peak at VM=%.1f V)" %
              (PIN_IN1, PIN_IN2, MAX_DUTY * 100, MAX_DUTY * VM_VOLTS, VM_VOLTS))

        bias_x, bias_y = calibrate_bias(imu)

        # 1st-order high-pass (3 Hz) + 2nd-order low-pass (12 Hz) on both X and Y axes
        hp_x = Biquad('hp1', BAND_LOW_HZ, SAMPLE_HZ)
        lp_x = Biquad('lp', BAND_HIGH_HZ, SAMPLE_HZ)
        hp_y = Biquad('hp1', BAND_LOW_HZ, SAMPLE_HZ)
        lp_y = Biquad('lp', BAND_HIGH_HZ, SAMPLE_HZ)

        # Settle the filters at exact SAMPLE_HZ pacing with output disabled (~0.5 s)
        step_us = int(DT * 1000000)
        next_tick = time.ticks_us()
        for _ in range(SAMPLE_HZ // 2):
            vx, vy = imu.read_xy_dps()
            lp_x.step(hp_x.step(vx - bias_x))
            lp_y.step(hp_y.step(vy - bias_y))
            next_tick = time.ticks_add(next_tick, step_us)
            while time.ticks_diff(next_tick, time.ticks_us()) > 0:
                pass

        if USE_WATCHDOG:
            from machine import WDT
            wdt = WDT(timeout=1000)

        last_tremor = 0.0
        accel_s = 0.0
        env_x = 0.0
        env_y = 0.0
        corr_xy = 0.0
        sy_smooth = 1.0
        cmd_s = 0.0
        cmd_env = 0.0
        fb_scale = 1.0
        mean_sq_v = 0.0
        pw_alpha = DT / POWER_WINDOW_S
        deadzone_pd = KP * DEADZONE_DPS
        fails = 0
        overruns = 0
        tick = 0
        out_scale = 1.0

        gc.collect()
        print("Running. Ctrl-C to stop.  KP=%.2f KD=%.4f xy=%s sign=%+d output=%s" %
              (KP, KD, USE_BOTH_XY_AXES, OUTPUT_SIGN, ENABLE_OUTPUT))
        next_tick = time.ticks_us()

        while True:
            # ---- 1. sense (X and Y axes in a single I2C transaction) ----------
            try:
                vx, vy = imu.read_xy_dps()
                raw_x = vx - bias_x
                raw_y = vy - bias_y
                fails = 0
            except OSError:
                fails += 1
                spk.stop()
                if fails >= MAX_CONSECUTIVE_I2C_FAILS:
                    raise RuntimeError("IMU communication lost")
                raw_x = None
                raw_y = None

            if raw_x is not None:
                # ---- 2. band-pass (3-12 Hz) & smooth energy-weighted X+Y ------
                tx = lp_x.step(hp_x.step(raw_x))
                ty = lp_y.step(hp_y.step(raw_y))

                if USE_BOTH_XY_AXES:
                    # Smooth energy-weighted blending prevents sign-flip discontinuities
                    # and avoids sqrt(2) gain inflation when shaking diagonally.
                    env_x += 0.015 * (abs(tx) - env_x)
                    env_y += 0.015 * (abs(ty) - env_y)
                    corr_xy += 0.015 * (tx * ty - corr_xy)
                    target_sy = 1.0 if corr_xy >= 0.0 else -1.0
                    sy_smooth += 0.02 * (target_sy - sy_smooth)
                    env_tot = env_x + env_y + 0.1
                    tremor = (env_x * tx + env_y * sy_smooth * ty) / env_tot
                    raw = (env_x * raw_x + env_y * sy_smooth * raw_y) / env_tot
                else:
                    tremor = tx if GYRO_AXIS == 0 else ty
                    raw = raw_x if GYRO_AXIS == 0 else raw_y

                # ---- 3. derivative for phase lead (heavily smoothed) ----------
                accel = (tremor - last_tremor) / DT
                last_tremor = tremor
                accel_s += 0.15 * (accel - accel_s)

                # ---- 4. inverted PD + soft deadzone + command smoothing -------
                # Combine proportional and phase-lead terms BEFORE the deadzone
                # so the output transitions smoothly through zero without step jumps.
                pd_dps = KP * tremor + KD * accel_s
                mag = abs(pd_dps) - deadzone_pd
                if mag <= 0.0:
                    raw_cmd = 0.0
                else:
                    s = 1.0 if pd_dps > 0.0 else -1.0
                    raw_cmd = -OUTPUT_SIGN * (s * mag) / FULL_SCALE_DPS

                cmd_s += 0.35 * (raw_cmd - cmd_s)

                # ---- 5. active feedback-loop suppressor & power limiter -------
                cmd_env += 0.02 * (abs(cmd_s) - cmd_env)
                if cmd_env > FB_TRIGGER_CMD:
                    fb_scale = max(FB_MIN_SCALE, fb_scale * 0.96)
                elif cmd_env < FB_TRIGGER_CMD * 0.6:
                    fb_scale = min(1.0, fb_scale * 1.005)

                cmd = cmd_s * out_scale * fb_scale
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

                # ---- 6. telemetry (~25 Hz) & heartbeat LED -------------------
                tick += 1
                if tick >= 10:
                    tick = 0
                    led.toggle()
                    print("Gyro:%.1f,Tremor:%.1f,Out:%.1f,Scale:%.2f" %
                          (raw, tremor, cmd * 100.0, out_scale * fb_scale))

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
