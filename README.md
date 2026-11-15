# Active Hand Tremor Canceller (HackUMass XIV)

An active, real-time hand tremor cancellation device built with a Raspberry Pi Pico (MicroPython), an InvenSense MPU-6051M / MPU-6050 6-DoF IMU (GY-521 breakout), an Adafruit DRV8871 H-bridge motor driver powered directly by a 12V battery pack (8 $\times$ AA), and a weighted 4 $\Omega$, 5W speaker driver acting as an inertial reaction-mass actuator.

> [!IMPORTANT]
> **Disclaimer:** This is **not** a medical device. This project is an experimental hackathon proof of concept, and your mileage may vary (YMMV) for actually reducing the intensity of hand tremors.

---

## 1. Parts List

* **Microcontroller:** Raspberry Pi Pico (RP2040) with pre-soldered headers, running MicroPython (`main.py`)
* **Motion Sensor (IMU):** **InvenSense MPU-6051M / MPU-6050** 6-DoF Accelerometer & Gyroscope on a **GY-521** breakout board
* **Actuator Driver:** **Adafruit DRV8871** Brushed DC Motor Driver Breakout (6.5V–45V motor supply, 3.3V logic PWM, 2.0A internal current limiting)
* **Inertial Actuator:** Weighted **4 $\Omega$, 5W** speaker driver with an attached counterweight acting as an inertial reaction-mass shaker
* **Power & Breadboarding:**
  * **5V Micro-USB Power Supply** plugged directly into the Raspberry Pi Pico's Micro-USB port (powers the Pico and the GY-521 IMU via `3V3(OUT)`)
  * **12V Battery Pack (8 $\times$ AA cells in series)** connected directly to the Adafruit DRV8871 `VM` (`POWER +`) motor power input
  * Solderless breadboard, jumper wires, and a bulk decoupling capacitor across the 12V `VM` / `GND` rail

---

## 2. General Design Overview

Human hand tremors (such as essential and Parkinsonian tremors) typically oscillate between **4 Hz and 12 Hz**, while intentional everyday hand movements occur at much slower frequencies (**below 3 Hz**).

This device operates as a closed-loop **active inertial damper**:

```
[ MPU-6051M Gyro ] ---> [ 3-12 Hz Band-Pass + PD Control ] ---> [ Adafruit DRV8871 ] ---> [ Weighted 4Ω 5W Speaker ]
 (Senses X + Y hand       (Isolates tremor, applies 4x->2x        (Dual-PWM slow-          (Pushes against
 angular velocity)        tapered gain & inverts wave)           decay H-bridge)           hand motion)
```

1. **Sense (`250 Hz`, $\pm 2000\text{ dps}$):** The **MPU-6051M (GY-521)** gyroscope reads both **X and Y axes** simultaneously over I2C (`0x68`) at $\pm 2000\text{ deg/s}$ full scale so fast wrist shakes never clip.
2. **Filter (`3–12 Hz`):** The **Raspberry Pi Pico** passes the combined X+Y signal through a **3–12 Hz band-pass filter** (1st-order high-pass at 3 Hz + 2nd-order low-pass at 12 Hz). This removes slow intentional movements (<3 Hz) and high-frequency mechanical/sensor noise (>12 Hz) while keeping phase distortion minimal.
3. **Invert, Phase-Align & Scale:** The Pico generates an **inverted counter-wave** (`180°` out of phase) with PD phase-lead compensation (`KP = 1.0`, `KD = 0.008`), a soft deadzone (`DEADZONE_DPS = 3.0`), and a **tapered gain curve** (`GAIN_LOW = 4.0` for subtle tremors tapering smoothly down to `GAIN_HIGH = 2.0` at full scale).
4. **Cancel & Thermal Limit:** The **Adafruit DRV8871** drives the weighted 4 $\Omega$, 5W speaker at 20 kHz phase-locked dual PWM (`GP14` and `GP15`) from the 12V battery pack. Duty cycle is capped at **50%** (`SPEAKER_MAX_V = 6.0` V at `VM_VOLTS = 12.0` V) for up to a 1-second 4.5W RMS burst (`POWER_WINDOW_S = 1.0`) before the rolling thermal limiter scales average dissipation down to **2.5W continuous** (`SPEAKER_CONT_W = 2.5`).

---

## 3. Breadboard & Pin Wiring Guide

### A. Power Distribution (5V Micro-USB + 12V Battery Pack)

> **CRITICAL VOLTAGE RULES:**    
> 1. **NEVER connect the 12V battery pack `+` wire to the Raspberry Pi Pico or the GY-521!** The 12V `+` rail connects **ONLY** to the Adafruit DRV8871 `VM` (`POWER +`) terminal.  
> 2. **Share a common `GND`:** The 12V battery pack `-`, DRV8871 `GND`, GY-521 `GND`, and Pico `GND` must all be tied to the same breadboard ground rail so the 3.3V PWM signals have a common reference.

| Source / Rail | Connects To | Notes |
| :--- | :--- | :--- |
| **5V Micro-USB Cable and USB Power Supply** | **Raspberry Pi Pico Micro-USB Port** | Powers the Pico logic and onboard 3.3V regulator |
| **Pico `3V3(OUT)` (Physical Pin 36)** | **GY-521 `VCC`** | Supplies regulated 3.3V power to the MPU-6051M board |
| **12V Battery Pack `+` (8 $\times$ AA)** | **Adafruit DRV8871 `VM` / `POWER +`** | **12V motor power only.** Place a decoupling capacitor across `VM` and `GND` |
| **Common `GND` Rail** | **12V Battery Pack `-`**<br>**Adafruit DRV8871 `GND` / `POWER -`**<br>**GY-521 `GND`**<br>**Pico `GND` (Pin 3, Pin 18, Pin 38)** | **All grounds must be tied together** |

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
| **Common `GND` Rail** | **Pin 18 / Pin 38** | **`GND` / `POWER -`** | Common Ground (shared with Pico & 12V pack `-`) |
| **12V Battery Pack `+`** | *(8 $\times$ AA in series)* | **`VM` / `POWER +`** | 12V Motor Power Supply (DRV8871 accepts 6.5V–45V) |
| *Speaker Terminal `+`* | — | **`MOTOR OUT1`** | 4 $\Omega$, 5W Speaker Voice Coil `+` |
| *Speaker Terminal `-`* | — | **`MOTOR OUT2`** | 4 $\Omega$, 5W Speaker Voice Coil `-` |

---

## 4. Quick-Reference Raspberry Pi Pico Pin Map

```
               [5V Micro-USB Power]
                   +-----[USB]-----+
  GY-521 SDA -> GP0| 1          40 | VBUS
  GY-521 SCL -> GP1| 2          39 | VSYS
  GY-521 GND -> GND| 3          38 | GND  <--- Common GND (12V Pack -, DRV8871 GND)
                GP2| 4          37 | 3V3_EN
                GP3| 5          36 | 3V3(OUT) -> GY-521 VCC
                ...| ..         .. | ...
               GP13| 17         24 | GP18
 DRV8871 GND -> GND| 18         23 | GND
 DRV8871 IN1 ->GP14| 19         22 | GP17
 DRV8871 IN2 ->GP15| 20         21 | GP16
                   +---------------+

  [12V Battery Pack (8x AA)] + ---> DRV8871 VM (POWER +)
  [12V Battery Pack (8x AA)] - ---> Common GND Rail (shared with Pico GND)
```

---

## 5. Wearable Mounting & Ergonomics

* **Speaker Apparatus (Inertial Actuator):** Mounted **below the wrist** on the forearm/wrist underside so the heavier reaction-mass assembly is comfortably supported.
* **Gyroscope / IMU (`GY-521` / `MPU-6051M`):** Mounted on the back of the hand **behind the index finger knuckle**, where hand tremor angular velocity (`X` and `Y` axes) is most pronounced.

![Wearable Mounting Setup](mounting.jpg)

---

## 6. Files & Running

* **[main.py](main.py):** Main tremor-cancellation firmware (upload/save to the Pico as `main.py` to auto-run on boot).
* **[axis_finder.py](axis_finder.py):** Diagnostic script that outputs single-line live tilt angles (in degrees) across X, Y, and Z using a complementary gyro+accelerometer filter while keeping the motor driver disabled.

### Startup & Status LED (`main.py`)
1. **Place Still on Power-Up:** Set the device flat and still when plugging in the 5V Micro-USB cable.
2. **Calibration (~2.5 seconds):** The Pico's onboard green LED stays **solid ON** while waiting for the GY-521 to wake, averaging 500 samples for `bias_x`/`bias_y`, and settling the 3–12 Hz filters.
3. **Active Cancellation:** Once calibration finishes, the onboard green LED **blinks steadily** (~12.5 Hz toggle at the 25 Hz telemetry tick), indicating the 250 Hz cancellation loop is live.
4. **Verify Polarity:** If the weighted speaker reinforces the shake instead of damping it, flip `OUTPUT_SIGN` from `1` to `-1` in `main.py` (or swap `MOTOR OUT1`/`OUT2`).

---

## 7. Team & Roles (HackUMass XIV)

* **Henry Resnick** — *Embedded Systems Software Engineer & Hardware Procurement* (Real-time DSP filter & control-loop firmware, hardware sourcing)
* **Shreyas Jatling** — *Hardware Integration Engineer & Ergonomic Test Lead* (Circuit soldering, electronics assembly, wearable hand-modeling & live testing)
* **Ethan Oren** — *Release Engineer & Repository Maintainer* (Version control, codebase integration, and repository management)
* **Robert Tao** — *Mechanical Enclosure & Product Packaging Designer* (Wearable mounting mechanics, ergonomics, and product packaging)
