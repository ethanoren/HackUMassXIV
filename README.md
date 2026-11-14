# Active Hand Tremor Canceller (HackUMass XIV)

An active, real-time hand tremor cancellation device built with a Raspberry Pi Pico (MicroPython), a 6-DoF IMU, a dual H-bridge motor driver, and a weighted audio exciter acting as an inertial reaction-mass actuator.

---

## 1. Parts List

* **Microcontroller:** Raspberry Pi Pico (RP2040) with pre-soldered headers, running MicroPython
* **Motion Sensor (IMU):** SparkFun / Adafruit **LSM6DSOX** 6-DoF Accelerometer & Gyroscope Breakout (Qwiic / STEMMA QT)
* **Actuator Driver:** **SparkFun Qwiic Motor Driver (DRV8835)** *(also supports direct GPIO-driven DRV8835 carrier boards)*
* **Inertial Actuator:** **Dayton Audio DAEX25FHE-4** Framed High-Efficiency 25mm Sound Exciter ($4\,\Omega$, 24 W RMS) with a counterweight glued to the voice coil coupler
* **Power & Breadboarding:**
  * 5V USB power brick (2A+ recommended) with a **spliced USB power cable** (`+5V` and `GND`)
  * Solderless breadboard & jumper wires (plus Qwiic-to-breadboard jumpers or spliced Qwiic cables)
  * Bulk decoupling capacitor (e.g., $100\,\mu\text{F}$–$470\,\mu\text{F}$ electrolytic) across the 5V motor supply rail

---

## 2. General Design Overview

Human hand tremors (such as essential and Parkinsonian tremors) typically oscillate between **4 Hz and 12 Hz**, while intentional everyday hand movements occur at much slower frequencies (**below 3 Hz**).

This device operates as a closed-loop **active inertial damper**:

```
[ LSM6DSOX Gyro ] ---> [ 3-12 Hz Band-Pass + Phase Lead ] ---> [ DRV8835 Driver ] ---> [ Weighted Exciter ]
   (Senses hand              (Isolates tremor & inverts           (Bidirectional          (Pushes against
 angular velocity)              the waveform on Pico)               power stage)           hand motion)
```

1. **Sense:** The **LSM6DSOX** gyroscope measures the angular velocity of the hand 250 times per second over I2C.
2. **Filter:** The **Raspberry Pi Pico** passes the sensor readings through a **3–12 Hz band-pass filter**. This strips out slow, intentional hand movements (<3 Hz) as well as high-frequency vibrations and sensor noise (>12 Hz), leaving only the involuntary tremor wave.
3. **Invert & Phase-Align:** The Pico generates an **inverted counter-wave** (`180°` out of phase with the tremor) and applies proportional-derivative (PD) phase compensation so the actuator responds without lag.
4. **Cancel:** The **DRV8835** drives the weighted **Dayton Audio DAEX25FHE-4** exciter back and forth with the inverted wave. Accelerating the glued weight produces an equal-and-opposite reaction force that pushes against the tremor to stabilize the hand.

---

## 3. Breadboard & Pin Wiring Guide

### A. Power Distribution (Spliced 5V USB Cable)

> **CRITICAL VOLTAGE RULE:**  
> The Raspberry Pi Pico logic, LSM6DSOX IMU, and Qwiic bus are strictly **3.3V logic**.  
> * Send the **5V USB rail** ONLY to the Pico's **`VSYS`** pin and the motor driver's **`VIN` / `VM`** motor power input.  
> * Power the sensor and Qwiic logic from the Pico's regulated **`3V3(OUT)`** pin.

| Source / Rail | Wire Color (Standard) | Connects To | Notes |
| :--- | :--- | :--- | :--- |
| **Spliced USB `+5V`** | Red | **Breadboard `+5V` Rail** | Main 5V supply from USB wall brick |
| **Spliced USB `GND`** | Black | **Breadboard `GND` Rail** | Common ground reference for all boards |
| **Breadboard `+5V` Rail** | Red | **Pico `VSYS` (Physical Pin 39)**<br>**DRV8835 `VIN` / `VM` (Motor Power)** | Place your decoupling capacitor across `+5V` and `GND` near the DRV8835 `VIN` pin |
| **Pico `3V3(OUT)` (Physical Pin 36)** | Orange / Red | **Breadboard `+3.3V` Rail** | Supplies regulated 3.3V logic power to the LSM6DSOX and Qwiic bus |
| **Breadboard `GND` Rail** | Black | **Pico `GND` (Physical Pin 38 & Pin 3)**<br>**LSM6DSOX `GND`**<br>**DRV8835 `GND`** | **All grounds must be tied together** |

*(Note: If you also plug a micro-USB data cable from your laptop into the Pico for Thonny telemetry while the spliced 5V brick is powered, power the Pico via **`VSYS` (Pin 39)**— ideally through a 1N5817 Schottky diode—rather than `VBUS` (Pin 40), so the two 5V supplies do not back-feed each other.)*

---

### B. Sensor & Motor Driver Signal Wiring

`tremor_canceller.py` automatically detects whether you are using the **SparkFun Qwiic Motor Driver over I2C** (Option 1) or **Direct DRV8835 GPIO PWM pins** (Option 2).

#### Option 1: Qwiic I2C Wiring (SparkFun Qwiic Motor Driver ROB-15451 + LSM6DSOX)
Both the **LSM6DSOX** and the **SparkFun Qwiic Motor Driver** share the same 2-wire I2C bus (`I2C0`) on the Pico. You can daisy-chain them with a Qwiic cable or wire both to the same breadboard rows.

*(Standard Qwiic cable wire colors: **Black** = `GND`, **Red** = `3.3V`, **Blue** = `SDA`, **Yellow** = `SCL`)*

| Pico Pin Name | Pico Physical Pin # | LSM6DSOX Pin | SparkFun Qwiic Motor Driver Pin | Function |
| :--- | :---: | :--- | :--- | :--- |
| **`GP0` (I2C0 SDA)** | **Pin 1** *(top-left)* | `SDA` *(Blue)* | `SDA` *(Blue)* | I2C Data (400 kHz) |
| **`GP1` (I2C0 SCL)** | **Pin 2** *(left, 2nd down)* | `SCL` *(Yellow)* | `SCL` *(Yellow)* | I2C Clock (400 kHz) |
| **`GND`** | **Pin 3** *(left, 3rd down)* | `GND` *(Black)* | `GND` *(Black)* | Common Ground |
| **`3V3(OUT)`** | **Pin 36** *(right, 5th down)* | `3V3` / `VIN` *(Red)* | `3.3V` *(Red, Qwiic logic)* | 3.3V Logic Supply |
| **Breadboard `+5V`** | *(connected to Pin 39 `VSYS`)* | *Do not connect* | Screw Terminal **`VIN`** *(Max 11V)* | 5V Motor H-Bridge Power |
| **Breadboard `GND`** | **Pin 38** *(right, 3rd down)* | `GND` | Screw Terminal **`GND`** | Motor Power Return |

---

#### Option 2: Direct DRV8835 GPIO PWM Wiring (If Using Breakout Pins Directly)
If using a DRV8835 board with direct `MODE`, `APHASE`, and `AENABLE` pins instead of Qwiic I2C, wire the LSM6DSOX to `GP0`/`GP1` as above and wire the DRV8835 control pins to the bottom-left of the Pico:

| Pico Pin Name | Pico Physical Pin # | DRV8835 Breakout Pin | Function |
| :--- | :---: | :--- | :--- |
| **`GP13`** | **Pin 17** *(left, 4th from bottom)* | `MODE` | Driven `HIGH` (`3.3V`) for PHASE/ENABLE mode |
| **`GP14`** | **Pin 19** *(left, 2nd from bottom)* | `APHASE` (`AIN1`) | Exciter push/pull direction |
| **`GP15`** | **Pin 20** *(bottom-left corner)* | `AENABLE` (`AIN2`) | 20 kHz PWM magnitude control |
| **`3V3(OUT)`** | **Pin 36** | `VCC` | 3.3V Logic Power |
| **Breadboard `+5V`** | *(from spliced USB +5V)* | `VM` / `VIN` | 5V Motor Supply |
| **Breadboard `GND`** | **Pin 18 / Pin 38** | `GND` | Common Ground |

---

### C. Exciter Output Wiring (DRV8835 to Dayton Audio DAEX25FHE-4)

| DRV8835 Terminal | Connects To | Notes |
| :--- | :--- | :--- |
| **Channel A Output 1 (`A1` / `AOUT1`)** | **DAEX25FHE-4 Terminal `+`** | Either polarity works; if the exciter pushes *with* the tremor instead of against it, either swap `A1`/`A2` wires or set `OUTPUT_SIGN = -1` in `tremor_canceller.py`. |
| **Channel A Output 2 (`A2` / `AOUT2`)** | **DAEX25FHE-4 Terminal `-`** | |

---

## 4. Quick-Reference Raspberry Pi Pico Pin Map

```
                   +---------[USB]---------+
  I2C0 SDA -> GP0  |  1                 40 | VBUS
  I2C0 SCL -> GP1  |  2                 39 | VSYS <--- +5V from Spliced USB
       GND -> GND  |  3                 38 | GND  <--- GND from Spliced USB
              GP2  |  4                 37 | 3V3_EN
              GP3  |  5                 36 | 3V3(OUT) -> +3.3V Logic to IMU & Qwiic
              ...  | ..                 .. | ...
 DRV MODE* -> GP13 | 17                 24 | GP18
       GND -> GND  | 18                 23 | GND
DRV PHASE* -> GP14 | 19                 22 | GP17
   DRV EN* -> GP15 | 20                 21 | GP16
                   +-----------------------+
  (*GP13/GP14/GP15 only needed if wiring DRV8835 via direct GPIO instead of Qwiic I2C)
```

---

## 5. Running & Tuning

1. Hold the device still at power-up for **~2 seconds** while `calibrate_bias()` measures the gyroscope's resting zero offset.
2. Verify axis orientation:
   * Set `GYRO_AXIS` (`0` = X, `1` = Y, `2` = Z) in `tremor_canceller.py` to match the axis the hand tremor rotates around.
   * If the exciter reinforces the shake instead of damping it, flip `OUTPUT_SIGN` from `1` to `-1` (or swap the `A1`/`A2` exciter wires).
3. Adjust `KP` (cancellation strength) and `KD` (phase-lead compensation) in `tremor_canceller.py` to match the weight glued to your exciter.
