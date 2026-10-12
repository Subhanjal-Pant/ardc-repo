# Pneumatic Air Spring Control & Telemetry System

An advanced embedded control framework designed for ESP32-based pneumatic air spring test benches. This repository implements multiple control strategies—including **PID with Mass/Pressure Gain Scheduling** **Active Disturbance Rejection Control (ADRC)** and **Reinforcement Learning (PPO)**coupled with ToF and IMU sensor fusion, binary PWM valve pulsing and Python-based real-time telemetry dashboards.

---

## Repository Structure & Programs

The project contains several distinct firmware implementations and diagnostic scripts:

* **PID Control Firmware**: 
  * Implements a deterministic 25 Hz control loop (40 ms).
  * Includes directional and pressure-gradient gain scheduling for fill/drain dynamics.
  * Utilizes an asynchronous 2-state Kalman filter combining VL53L0X ToF range data with MPU6050 accelerometer readings.
* **ADRC Control Firmware**: 
  * Implements a second-order Active Disturbance Rejection Control (ADRC) scheme utilizing an Extended State Observer (ESO) to estimate and cancel system disturbances in real-time.
  * Features a 3-state Kalman filter and dynamic coasting logic around a 4 mm deadzone.
* **Python ADRC Dashboard**: 
  * A Tkinter-based graphical interface for real-time monitoring and command input on `COM12`.
  * Logs high-resolution session telemetry, tracks step responses, and computes real-time pressure and leaky-integrated velocity metrics.
* **PPO Policy Inference Firmware**: 
  * Embeds a trained reinforcement learning policy (`policy_infer.h`) running directly on the ESP32.
  * Structures an 8-dimensional normalized observation vector matching the training environment (`pneumatic_env.py`) for closed-loop height regulation.
* **Sensor Logging / System ID Diagnostic**: 
  * A lightweight open-loop diagnostic script that forces valves closed and streams raw ToF, IMU, and pressure transducer values at 25 Hz for system identification.

---

## Hardware Architecture & Pin Mapping

The system targets an ESP32 microcontroller interfaced with the following components over I2C and GPIO:

* **Sensors**:
  * **VL53L0X**: Time-of-Flight (ToF) distance sensor connected via I2C (`GPIO 21`, `GPIO 22`) for absolute height measurement.
  * **MPU6050**: 6-DoF IMU (`0x68`) providing vertical acceleration and angular velocity data for sensor fusion.
  * **Pressure Transducers**: Connected to ADC Pins with 12-bit resolution (`GPIO 33` for supply/tank pressure, `GPIO 32` for air spring pressure).
* **Actuators (Solenoid Valves via Motor Drivers)**:
  * **Exhaust Valve Driver**: Enable `enA` (`GPIO 25` or `26`), Inputs `in1`, `in2` (`GPIO 16`, `17`).
  * **Inlet Valve Driver**: Enable `enB` (`GPIO 26` or `25`), Inputs `in3`, `in4` (`GPIO 18`/`19`).

---

## Getting Started

### Prerequisites
* **Hardware**: ESP32 Development Board, VL53L0X ToF sensor, MPU6050 IMU, two pressure transducers, and dual solenoid valve drivers.
* **Software**: 
  * Arduino IDE with ESP32 board support.
  * Required Libraries: `Wire`, `VL53L0X`, `MPU6050`.
  * Python 3.x with `pyserial` (for running the dashboard script).

### Installation & Usage
1. **Upload Firmware**: Open your preferred controller file (`.ino`) in the Arduino IDE, select your ESP32 board configuration, and flash the code.
2. **Set Target Height**: Send serial commands formatted as `T<value_in_mm>` (e.g., `T130` or `T185`) through the Serial Monitor or dashboard to adjust the height set-point.
3. **Run Dashboard**: 