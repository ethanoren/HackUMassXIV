# Active Hand Tremor Canceller (HackUMass XIV)

An active, real-time hand tremor cancellation device built with a Raspberry Pi Pico (MicroPython), an InvenSense MPU-6051M / MPU-6050 6-DoF IMU (GY-521 breakout), an Adafruit DRV8871 H-bridge motor driver fed by a DC-DC boost converter, and a weighted speaker driver acting as an inertial reaction-mass actuator.

---

## 1. Parts List

* **Microcontroller:** Raspberry Pi Pico (RP2040) with pre-soldered headers, running MicroPython
* **Motion Sensor (IMU):** **InvenSense MPU-6051M / MPU-6050** 6-DoF Accelerometer & Gyroscope on a **GY-521** breakout board
* **Actuator Driver:** **Adafruit DRV8871** Brushed DC Motor Driver Breakout (6.5V–45V motor supply, 3.3V logic PWM, 2.0A internal current limiting)
* **Inertial Actuator:** Weighted speaker driver / voice-coil actuator (2–4 $\Omega$, 5–8 W) with a counterweight attached to act as an inertial reaction-mass shaker
* **Power & Breadboarding:**
  * 5V USB power brick (2A+ recommended) with a **spliced USB power cable** (`+5V` and `GND`)
  * **DC-DC Step-Up (Boost) Converter** (steps 5V USB up to **9V–12V** for the DRV8871 `VM` input, which requires $\ge 6.5\text{V}$)
  * Solderless breadboard, jumper wires, and bulk decoupling capacitors across the 5V and boosted `VM` rails

---

## 2. General Design Overview

Human hand tremors (such as essential and Parkinsonian tremors) typically oscillate between **4 Hz and 12 Hz**, while intentional everyday hand movements occur at much slower frequencies (**below 3 Hz**).

This device operates as a closed-loop **active inertial damper**:

```
[ MPU-6051M Gyro ] ---> [ 3-12 Hz Band-Pass + Phase Lead ] ---> [ Adafruit DRV8871 ] ---> [ Weighted Speaker ]
   (Senses hand              (Isolates tremor & inverts           (Dual-PWM slow-          (Pushes against
 angular velocity)              the waveform on Pico)              decay H-bridge)           hand motion)
```

1. **Sense:** The **MPU-6051M (GY-521)** gyroscope measures the angular velocity of the hand 250 times per second over I2C (`0x68`).
2. **Filter:** The **Raspberry Pi Pico** passes the sensor readings through a **3–12 Hz band-pass filter** (1st-order high-pass at 3 Hz + 2nd-order low-pass at 12 Hz). This strips out slow, intentional hand movements (<3 Hz) as well as high-frequency vibrations and sensor noise (>12 Hz), leaving only the involuntary tremor wave with minimal phase distortion.
3. **Invert & Phase-Align:** The Pico generates an **inverted counter-wave** (`180°` out of phase with the tremor) and applies proportional-derivative (PD) phase compensation so the actuator responds without lag.
4. **Cancel:** The **Adafruit DRV8871** drives the weighted speaker back and forth with the inverted wave using 20 kHz phase-locked dual PWM (`GP14` and `GP15`). Accelerating the attached weight produces an equal-and-opposite reaction force that damps the tremor to stabilize the hand.

---

## 3. Breadboard & Pin Wiring Guide

### A. Power Distribution (Spliced 5V USB + Boost Converter)

> **CRITICAL VOLTAGE RULES:**  
> 1. The **Adafruit DRV8871** requires **6.5V to 45V** on its `VM` (`POWER +`) input. Feed your 5V USB rail into your **Boost Converter (`VIN+`/`VIN-`)**, adjust the boost output to **9V–12V**, and connect the boost output (`VOUT+`) **ONLY** to the DRV8871 `VM` (`+`) terminal.  
> 2. **Never connect the boosted 9V–12V rail to the Raspberry Pi Pico or the GY-521!**  
> 3. Remember to set `VM_VOLTS` in `tremor_canceller.py` to match the measured voltage of your boost converter output.

| Source / Rail | Connects To | Notes |
| :--- | :--- | :--- |
| **Spliced USB `+5V` (Red)** | **Pico `VSYS` (Physical Pin 39)**<br>**Boost Converter `VIN+`** | Powers the Pico and feeds the input of the boost converter |
| **Spliced USB `GND` (Black)** | **Breadboard Common `GND` Rail** | Common ground reference for the entire circuit |
| **Boost Converter `VOUT+` (9V–12V)** | **Adafruit DRV8871 `VM` / `POWER +`** | **6.5V–45V motor power only.** Place a decoupling capacitor across `VM` and `GND` |
| **Pico `3V3(OUT)` (Physical Pin 36)** | **GY-521 `VCC`** | Supplies regulated 3.3V power to the MPU-6051M board *(GY-521 also has an onboard LDO that accepts 3.3V–5V on `VCC`)* |
| **Breadboard Common `GND` Rail** | **Pico `GND` (Pin 3 & Pin 38)**<br>**GY-521 `GND`**<br>**Boost Converter `VIN-` / `VOUT-`**<br>**Adafruit DRV8871 `GND` / `POWER -`** | **All grounds must be tied together** |

---

### B. MPU-6051M (GY-521 Breakout) I2C Wiring

| Pico Pin Name | Pico Physical Pin # | GY-521 Pin | Function |
| :--- | :---: | :--- | :--- |
| **`3V3(OUT)`** | **Pin 36** *(right, 5th down)* | **`VCC`** | 3.3V Power |
| **`GND`** | **Pin 3** *(left, 3rd down)* | **`GND`** | Common Ground |
| **`GP1` (I2C0 SCL)** | **Pin 2** *(left, 2nd down)* | **`SCL`** | I2C Clock (400 kHz) |
| **`GP0` (I2C0 SDA)** | **Pin 1** *(top-left)* | **`SDA`** | I2C Data (400 kHz) |
| **`GND`** *(or leave unconnected)* | **Pin 3** | **`AD0`** | Sets I2C address to `0x68` (`3.3V` = `0x69`; code auto-detects both) |

*(Note: `XDA`, `XCL`, and `INT` on the GY-521 can be left unconnected.)*

---

### C. Adafruit DRV8871 Control & Speaker Wiring

The Adafruit DRV8871 generates its own internal logic supply from `VM`, so it only needs **`IN1`**, **`IN2`**, and **`GND`** from the Pico:

| Pico / Power Pin | Pico Physical Pin # | Adafruit DRV8871 Pin | Function |
| :--- | :---: | :--- | :--- |
| **`GP14` (PWM7 A)** | **Pin 19** *(left, 2nd from bottom)* | **`IN1`** | 20 kHz PWM Half-Bridge Input 1 (3.3V logic) |
| **`GP15` (PWM7 B)** | **Pin 20** *(bottom-left corner)* | **`IN2`** | 20 kHz PWM Half-Bridge Input 2 (3.3V logic) |
| **Breadboard `GND`** | **Pin 18 / Pin 38** | **`GND` / `POWER -`** | Common Ground |
| **Boost Converter `VOUT+`** | *(9V–12V Boosted Rail)* | **`VM` / `POWER +`** | Motor Power Supply (6.5V–45V) |
| *Speaker Terminal `+`* | — | **`MOTOR OUT1`** | Speaker / Exciter Voice Coil `+` |
| *Speaker Terminal `-`* | — | **`MOTOR OUT2`** | Speaker / Exciter Voice Coil `-` |

---

## 4. Quick-Reference Raspberry Pi Pico Pin Map

```
                   +---------[USB]---------+
  GY-521 SDA -> GP0|  1                 40 | VBUS
  GY-521 SCL -> GP1|  2                 39 | VSYS <--- +5V from Spliced USB
  GY-521 GND -> GND|  3                 38 | GND  <--- Common GND (USB, Boost, DRV8871)
                GP2|  4                 37 | 3V3_EN
                GP3|  5                 36 | 3V3(OUT) -> GY-521 VCC
                ...| ..                 .. | ...
               GP13| 17                 24 | GP18
 DRV8871 GND -> GND| 18                 23 | GND
 DRV8871 IN1 ->GP14| 19                 22 | GP17
 DRV8871 IN2 ->GP15| 20                 21 | GP16
                   +-----------------------+
```

---

## 5. Running & Tuning

1. **Measure Boost Converter Voltage:** Measure the output voltage of your boost converter with a multimeter (must be $\ge 6.5\text{V}$ for the DRV8871) and set `VM_VOLTS` in `tremor_canceller.py` to match.
2. **Hold Still at Startup:** Hold the rig still for **~2 seconds** at power-up while `calibrate_bias()` zeroes the gyroscope.
3. **Verify Polarity & Axis:**
   * Set `GYRO_AXIS` (`0` = X, `1` = Y, `2` = Z) in `tremor_canceller.py` to match the axis the tremor rotates around.
   * If the speaker reinforces the shake instead of damping it, flip `OUTPUT_SIGN` from `1` to `-1` (or swap `MOTOR OUT1`/`OUT2`).
