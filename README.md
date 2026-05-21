# 2-DOF Robotic Manipulator using Arduino

![2DOF Robot](<img width="960" height="1280" alt="img1" src="https://github.com/user-attachments/assets/49d47758-cc2c-4348-a010-5e4216d48dcc" />
)

## Overview

This project presents a 2-DOF robotic manipulator capable of performing forward kinematics, inverse kinematics, inverse dynamics, and forward dynamics using Arduino and stepper motors.

The manipulator accepts serial commands from the Arduino Serial Monitor to control the robotic arm either through joint angles or Cartesian coordinates. The calculated values are translated into precise stepper motor motion.

This project demonstrates concepts of robotics, embedded systems, manipulator dynamics, and motion control.

---

# Features

- Forward Kinematics
- Inverse Kinematics
- Inverse Dynamics
- Forward Dynamics
- Stepper Motor Position Control
- Cartesian Coordinate Motion
- Joint Angle Motion
- Workspace Validation
- Real-Time Serial Communication

---

# Hardware Setup

## Components Required

| Component | Quantity |
|---|---|
| Arduino Nano / Uno | 1 |
| NEMA Stepper Motor | 2 |
| Stepper Driver Module | 2 |
| SMPS Power Supply | 1 |
| Aluminum Extrusion Frame | 1 |
| Acrylic Base Plate | 1 |
| Shaft Couplers | 2 |
| Bearings | As required |
| Connecting Wires | Multiple |
| Perfboard / PCB | 1 |
| USB Cable | 1 |
| Mounting Hardware | As required |

---

# System Architecture

The robotic manipulator consists of:

- Two revolute joints
- Two stepper motors for actuation
- Arduino Nano for control
- Stepper motor drivers
- Mechanical frame using aluminum extrusion
- External SMPS power supply

The system calculates the required joint angles using inverse kinematics and actuates the motors accordingly.

---

# Pin Configuration

| Motor | DIR Pin | STEP Pin |
|---|---|---|
| Motor 1 | 2 | 3 |
| Motor 2 | 10 | 11 |

---

# Robot Parameters

| Parameter | Value |
|---|---|
| Link 1 Length (L1) | 0.10 m |
| Link 2 Length (L2) | 0.11 m |
| Mass 1 (M1) | 0.5 kg |
| Mass 2 (M2) | 0.23 kg |
| Gravity (G) | 9.81 m/s² |

---

# Forward Kinematics

The end-effector position is calculated using:

\[
x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2)
\]

\[
y = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2)
\]

---

# Inverse Kinematics

The joint angles are computed from the desired Cartesian coordinates.

\[
\theta_2 = \cos^{-1}\left(\frac{x^2+y^2-L_1^2-L_2^2}{2L_1L_2}\right)
\]

\[
\theta_1 = \phi - \psi
\]

Where:

- \(\phi = \tan^{-1}(y/x)\)
- \(\psi = \tan^{-1}\left(\frac{L_2\sin\theta_2}{L_1+L_2\cos\theta_2}\right)\)

---

# Dynamics

## Inverse Dynamics

The manipulator calculates required joint torques considering gravity.

\[
\tau_1 = M_1G\frac{L_1}{2}\cos\theta_1 + M_2G\left(L_1\cos\theta_1 + \frac{L_2}{2}\cos(\theta_1+\theta_2)\right)
\]

\[
\tau_2 = M_2G\frac{L_2}{2}\cos(\theta_1+\theta_2)
\]

---

# Workspace Validation

The manipulator checks whether the desired position lies inside the reachable workspace.

- Minimum Reach = |L1 − L2|
- Maximum Reach = L1 + L2

---

# Software Requirements

- Arduino IDE
- Arduino Core Libraries
- Serial Monitor

---

# Installation

## Clone Repository

```bash
git clone https://github.com/yourusername/2DOF-Robot-Arm.git
