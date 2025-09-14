# Interrupt-Driven Odometry with Hall Effect Sensors (STM32 Nucleo-H563ZI)

## 📌 Overview
This project implements a **4-wheel odometry system** using **Hall effect sensors** on an **STM32 Nucleo-H563ZI** board.  
The firmware counts pulses from 4 encoders, applies debouncing, computes wheel distances, and estimates the robot’s **pose (x, y, θ)** and **velocities (v, w)**.  
A **Kalman filter** smooths noisy odometry readings for more realistic behavior.

---

## 🚀 Features
- ✅ 4 Hall effect sensors read simultaneously (1 per wheel)  
- ✅ **Interrupt-driven pulse counting** with debouncing (5 ms lockout)  
- ✅ Handles simultaneous interrupts without losing counts  
- ✅ **Odometry estimation**: position `(x, y, θ)`  
- ✅ **Velocity estimation**: linear `(v)` and angular `(w)`  
- ✅ **Kalman filter smoothing** for realistic trajectory  

---

## 🛠️ Hardware Setup
- **Microcontroller**: STM32 Nucleo-H563ZI  
- **Sensors**: 4x Digital Hall effect sensors  
- **Connections**:
  - Hall Sensor 1 → D2  
  - Hall Sensor 2 → D3  
  - Hall Sensor 3 → D4  
  - Hall Sensor 4 → D5  
  - Vcc → 3.3V / 5V (depending on sensor)  
  - GND → GND  

---

## 📐 Odometry Equations
- Wheel distance:  
dist = counts × (2πR / pulsesPerRev)
- Robot displacement:
d = (dLeft + dRight) / 2
Δθ = (dRight - dLeft) / wheelBase
- Pose update:
x += d × cos(θ)
y += d × sin(θ)
θ += Δθ

## 📊 Serial Output Format
The firmware outputs at **50 Hz**:   Pose: x, y, θ | Vel: v, w


Where:  
- `x, y` → position in meters  
- `θ` → heading in radians  
- `v` → linear velocity (m/s)  
- `w` → angular velocity (rad/s)  

## 👤 Author
Developed by Harsh Jain 
Bachelor of Computer Science & Engineering, KIET Group of Institutions  
