# 🤖 Cosine Switch Control Algorithm for Nonholonomic Robots

> A full implementation of the **Cosine Switch Control (CSC)** algorithm for steering differential-drive mobile robots — from mathematical simulation to real hardware deployment via ROS.

---

## 🎯 Project Overview

This project implements the **Cosine Switch Control** algorithm, a nonlinear steering strategy for nonholonomic systems in chained form, based on the research paper:

> *"Steering Nonholonomic Systems with Cosine Switch Control"*

The algorithm divides the robot's trajectory into three time intervals — alternating between angular (rotation-only) and linear (translation-only) motion using cosine-shaped velocity profiles. The implementation spans three layers:

1. **Mathematical simulation** — verifying the algorithm against paper results
2. **ROS Turtlesim visualization** — 2D simulation in ROS Noetic
3. **Real hardware deployment** — running on a physical differential-drive robot via ROSSERIAL

---

## 🧮 How the Algorithm Works

The robot moves from a start pose `[x₀, y₀, θ₀]` to a goal pose `[xf, yf, θf]` over a total time `T`, split into 3 equal intervals (n=3):

| Interval | Motion Type | Velocity Profile |
|----------|------------|-----------------|
| `0 → T/3` | Pure Rotation | `u₂ = c₁ · (1 - cos(ωt))` |
| `T/3 → 2T/3` | Pure Translation | `u₁ = c₂ · (1 - cos(ωt))` |
| `2T/3 → T` | Pure Rotation | `u₂ = c₃ · (1 - cos(ωt))` |

Constants `c₁`, `c₂`, `c₃` are computed analytically from the start/goal poses and used to calculate individual **wheel angular velocities** via differential drive kinematics.

---

## 📁 Repository Structure

```
├── src/
│   ├── CSC_Simulation.py      # Pure math simulation + matplotlib plots
│   ├── TurtleSimulation.py    # ROS Turtlesim visualization node
│   ├── main.py                # ROS node for real robot control
│   ├── GetConstants.py        # Analytic constant computation
│   └── CSC_Main.ino           # Arduino firmware (motor execution)
└── Results/
    ├── Turtlesim Simulation.mp4    # ROS Turtlesim demo
    └── Full Implementation.MOV     # Real robot demo
```

---

## ✅ Validation

The input parameters used match exactly those from the original paper — and the resulting trajectories are **identical**, validating the implementation against published results.

---

## 🚀 Getting Started

### Prerequisites
- ROS Noetic (Ubuntu 20.04)
- Python 3.8+
- `rosserial` package
- `sympy`, `numpy`, `matplotlib`

```bash
git clone https://github.com/Baher-Kherbek/Implementing-Cosine-Switch-Control-Algorithm.git
cd Implementing-Cosine-Switch-Control-Algorithm

pip install sympy numpy matplotlib

# Run the math simulation
python src/CSC_Simulation.py

# Run Turtlesim visualization (requires ROS Noetic)
rosrun turtlesim turtlesim_node &
python src/TurtleSimulation.py

# Deploy on real robot (requires roscore + rosserial)
python src/main.py
```

---

## 🎥 Demo

| Turtlesim Simulation | Real Robot |
|---------------------|------------|
| See `Results/Turtlesim Simulation.mp4` | See `Results/Full Implementation.MOV` |

---

## 🛠️ Tech Stack

![Python](https://img.shields.io/badge/Python-3.8+-blue?logo=python)
![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen?logo=ros)
![SymPy](https://img.shields.io/badge/SymPy-Symbolic%20Math-lightblue)
![Arduino](https://img.shields.io/badge/Arduino-Firmware-teal?logo=arduino)

---

## 👤 Author

**Baher Kherbek** — Robotics Engineer & AI Systems Developer  
[github.com/Baher-Kherbek](https://github.com/Baher-Kherbek)
