# Parabolic Trough Temperature Control System

A robust temperature control system for parabolic trough solar collectors, combining model-based feedforward compensation with PI feedback control to maintain stable outlet oil temperature despite solar irradiance and inlet temperature disturbances.

![MATLAB](https://img.shields.io/badge/MATLAB-R2025a-orange?logo=mathworks)
![Simulink](https://img.shields.io/badge/Simulink-Model-blue)
![License](https://img.shields.io/badge/License-MIT-green)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [Project Structure](#-project-structure)
- [System Architecture](#system-architecture)
- [Technical Approach](#-technical-approach)
- [Requirements](#-requirements)
- [Installation and Usage](#-installation-and-usage)
- [Model Structure](#-model-structure)
- [Results](#-results)
- [Parameters](#parameters)
- [Future Work](#-future-work)
- [Documentation](#-documentation)
- [License](#-license)
- [Author](#-author)

---

## 🎯 Overview

Parabolic trough collectors (PTCs) are critical components in concentrated solar power (CSP) plants. They use parabolic mirrors to focus sunlight onto an absorber tube containing thermal oil, which is then used to generate steam for electricity production.

**1. The Challenge:**  
Maintaining stable outlet oil temperature (~400°C) is essential for:
- Efficient thermal-to-electric energy conversion
- Preventing damage to downstream equipment (heat exchangers, turbines)

**2. The Problem:**  
The system faces two major uncontrollable disturbances:
- **Solar irradiance `J(t)`**: Varies with weather and time of day
- **Inlet oil temperature `Tₑ(t)`**: Fluctuates based on upstream conditions

**3. The Solution:**  
This project implements a hybrid control strategy that combines:
- **Feedforward control:** Anticipates disturbances using a physics-based model
- **PI feedback control:** Corrects residual errors and ensures robustness to parameter uncertainty

The complete system is implemented in MATLAB/Simulink with proper actuator saturation and anti-windup protection.

---

## ✨ Key Features

- **Nonlinear Plant Model**: Energy balance formulation for absorber tube dynamics
- **Hybrid Control Architecture**: 
  - Model-based feedforward for disturbance compensation
  - PI feedback for robustness and error elimination
- **Actuator Constraints**: Flow rate saturation `(0 – 10 l/s)` with anti-windup
- **Robustness Validation**: Tested under `±20%` parameter variations
- **Modular Simulink Design**: Clean subsystem structure (Plant, Feedforward, PI, Saturation)
- **IMC-Based Tuning**: Systematic controller gain design using Internal Model Control method

---

## 🗂 Project Structure

```text
parabolic-trough-control/
├── models/
│   └── ParabolicTrough.slx          # Main Simulink model 
├── parameters/
│   └── ParabolicTrough.mat          # Model and controller parameters 
├── docs/
│   └── ParabolicTrough.pdf          # Technical report 
├── README.md                       
├── LICENSE                         
└── .gitignore                       
```

---

## System Architecture

```
                    ┌─────────────────────────────────────┐
                    │                                     │
    J(t) ───────────┤                                     │
 (Irradiance)       │      Parabolic Trough Plant         │
                    │      (Absorber Tube)                │      T(t)
    q(t) ───────────┤                                     ├────────►
 (Flow Rate)        │      Nonlinear Energy Balance       │   (Outlet Temp)
                    │                                     │
    Tₑ(t) ──────────┤                                     │
 (Inlet Temp)       │                                     │
                    └─────────────────────────────────────┘
                                    ▲
                                    │
                         q(t) = q_ff(t) + q_fb(t)
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
            ┌───────┴────────┐            ┌────────┴─────────┐
            │  Feedforward   │            │   PI Feedback    │
            │   Controller   │            │    Controller    │
            │                │            │                  │
            │  q_ff = f(J,   │            │  q_fb = -Kₚe     │
            │      Tₑ, Tsp)  │            │        - Kᵢ∫e dt │
            └────────────────┘            └──────────────────┘
                    ▲                              ▲
                    │                              │
              J, Tₑ, Tsp                      e = Tsp - T
```

**Control Flow:**
1. **Feedforward path** computes nominal flow rate based on current irradiance and inlet temperature
2. **Feedback path** measures outlet temperature error and applies PI correction
3. **Combined signal** is saturated to respect physical actuator limits `(0 – 10 l/s)`
4. **Anti-windup** prevents integrator buildup when saturation is active

---

## 🔬 Technical Approach

### 1. Plant Model

The system dynamics are governed by an energy balance around the absorber tube:

```
C · dTₐ(t)/dt = η₀·S·J(t) - c_cp·q(t)·[Tₐ(t) - Tₑ(t)]
```

Where:
- `C` = Effective heat capacity of oil + tube (2267 kJ/K)
- `η₀·S·J(t)` = Absorbed solar power
- `c_cp·q(t)·(T - Tₑ)` = Convective heat transport (nonlinear bilinear term)

**Key Characteristics:**
- **Relative degree = 1**: Control input appears in first derivative of output
- **Nonlinear**: Flow rate `q(t)` multiplies state deviation `(T - Tₑ)`
- **Input-affine structure**: Suitable for feedback linearization (though not used here due to robustness concerns)

### 2. Feedforward Controller

Derived from steady-state equilibrium analysis:

```
q_ff(t) = (η₀ · S · J(t)) / (c_cp · (Tsp - Tₑ(t)))
```

**Purpose:**  
Anticipates the effect of measurable disturbances `(J, Tₑ)` and computes the required flow rate to maintain `T = Tsp`.

**Limitation:**  
Requires accurate knowledge of system parameters. Any mismatch leads to steady-state error.

### 3. PI Feedback Controller

```
q_fb(t) = -Kₚ·e(t) - Kᵢ·∫e(t)dt
```

Where: `e(t) = Tsp - T(t)`

**Gain Tuning (IMC Method):**
- Linearize plant around operating point → First-order model with gain `Kₚ = -18.18` and time constant `τ = 214.2 s`
- Set integral time `Tᵢ = τ` (cancels plant pole)
- Choose closed-loop time constant `τc = τ/3` for fast response
- Calculate gains: `Kₚ = 0.165`, `Kᵢ = 0.014`

**Purpose:**  
- Eliminates steady-state error (integral action)
- Provides robustness against parameter mismatch
- Faster transient response (proportional action)

### 4. Combined Control Law

```
q(t) = saturate(q_ff(t) + q_fb(t), [0, 10])
```

This two-degrees-of-freedom (2DOF) architecture:
- Feedforward handles predictable disturbances → fast response
- Feedback ensures robustness → eliminates modeling errors
- Saturation enforces physical constraints → prevents actuator damage
- Anti-windup prevents integrator buildup → maintains stability during saturation

---

## 📦 Requirements

### 1. Software
- MATLAB R2025a (or later)
- Simulink
- Control System Toolbox

### 2. Hardware
- Standard desktop/laptop capable of running MATLAB

---

## 🚀 Installation and Usage

### Step 1: Clone the Repository

```bash
git clone https://github.com/charizardmigo/parabolic-trough-control.git
cd parabolic-trough-control
```

### Step 2: Open MATLAB

Navigate to the project directory in MATLAB:

```matlab
cd parabolic-trough-control
```

### Step 3: Load System Parameters

```matlab
load('parameters/ParabolicTrough.mat')
```

This initializes all constants in the workspace:
- Plant parameters: `C`, `S`, `eta0`, `c_cp`
- Operating points: `Tsp`, `Te`, `J`
- Controller gains: `Kp`, `Ki`

### Step 4: Open the Simulink Model

```matlab
open_system('models/ParabolicTrough.slx')
```

### Step 5: Select Test Scenario

The model supports four configurations (connect/disconnect blocks as needed):

1. **Open-loop (baseline)**: Constant flow rate, no control
2. **Feedforward only**: Model-based compensation, no feedback
3. **PI only**: Feedback control without feedforward
4. **Combined (default)**: Feedforward + PI with saturation

### Step 6: Configure Disturbance Profiles

Modify the input blocks:
- **J (Irradiance)**: Step block (default: `0.8 → 0.6 kW/m²` at `t=500s`)
- **Tₑ (Inlet temp)**: Constant (default: `300°C`) or step to `320°C` at `t=1200s`
- **Tsp (Setpoint)**: Constant at `400°C`

### Step 7: Run Simulation

**Simulation Settings:**
- Solver: `ode45` (variable-step)
- Stop time: `2000` seconds

Click Run or execute:

```matlab
sim('models/ParabolicTrough.slx')
```

### Step 8: View Results

- **Scope_T**: Outlet temperature `T(t)` vs. Setpoint `Tsp`  
- **Scope_q**: Flow components (`q_ff`, `q_fb`, `q_total`)

Compare your results to the figures in `docs/ParabolicTrough.pdf` (To be added).

---

## 🧩 Model Structure

The Simulink model is organized into four main subsystems:

### 1. Plant Subsystem
**Inputs:** J (irradiance), Tₑ (inlet temp), q (flow rate)  
**Output:** T (outlet temperature)

**Implementation:**
- Gain block: `η₀·S` applied to J
- Subtraction: `(T - Tₑ)`
- Gain block: `c_cp` applied to flow term
- Subtraction: Solar input minus convective transport
- Gain block: `1/C` (inverse heat capacity)
- Integrator: `1/s` → produces outlet temperature T

### 2. Feedforward Subsystem
**Inputs:** J, Tsp, Tₑ  
**Output:** q_ff (feedforward flow command)

**Implementation:**
- Numerator: `η₀·S·J`
- Denominator: `c_cp·(Tsp - Tₑ)`
- Division: Computes steady-state flow requirement

### 3. PI Controller Subsystem
**Inputs:** Tsp (setpoint), T (measured outlet temp)  
**Output:** q_fb (feedback correction)

**Implementation:**
- Error computation: `e = Tsp - T`
- PI block: Simulink PID controller (set to PI mode)
- Gains: Kₚ = 0.165, Kᵢ = 0.014
- Anti-windup: Enabled (back-calculation method)

### 4. Saturation Block
**Input:** q_total = q_ff + q_fb  
**Output:** q_saturated ∈ [0, 10] l/s

**Purpose:**
- Enforces physical actuator limits
- Feeds back to PI controller for anti-windup

---

## 📊 Results

### Test Case 1: Irradiance Step (0.8 → 0.6 kW/m²)

| Controller | Steady-State Error | Settling Time | Max Deviation |
|------------|-------------------|---------------|---------------|
| Open-loop | ~40°C | N/A (unstable) | N/A |
| Feedforward only | ~2°C | ~300s | ~5°C |
| PI only | 0°C | ~600s | ~15°C |
| **Combined** | **0°C** | **~200s** | **~3°C** |

**Observation:**  
Combined controller achieves fastest response with zero steady-state error.

### Test Case 2: Inlet Temperature Step (300 → 320°C)

| Controller | Steady-State Error | Recovery Time |
|------------|-------------------|---------------|
| Feedforward only | ~8°C | Does not recover |
| **Combined** | **0°C** | **~250s** |

**Observation:**  
Feedforward alone cannot reject inlet temperature disturbances (not included in its model). PI feedback corrects the error.

### Test Case 3: Parameter Mismatch (c_cp reduced by 20%)

| Controller | Steady-State Error | Stability |
|------------|-------------------|-----------|
| Feedforward only | ~15°C | Stable but inaccurate |
| **Combined** | **0°C** | **Stable and accurate** |

**Observation:**  
PI feedback compensates for model uncertainty, demonstrating robustness.

### Key Findings

- **Feedforward** provides fast disturbance rejection but is sensitive to parameter errors  
- **PI feedback** ensures zero steady-state error and robustness  
- **Combined approach** achieves best overall performance: fast + robust  
- **Actuator saturation** with anti-windup prevents instability during large disturbances

---

## Parameters

### Plant Parameters

| Symbol | Description | Value | Unit |
|--------|-------------|-------|------|
| C | Effective heat capacity (oil + tube) | 2267 | kJ/K |
| S | Collector aperture area | 1500 | m² |
| η₀ | Optical efficiency | 0.8813 | — |
| c_cp | Specific heat transport coefficient | 1.924 | kJ/(l·K) |

### Operating Conditions

| Symbol | Description | Value | Unit |
|--------|-------------|-------|------|
| Tsp | Desired outlet temperature (setpoint) | 400 | °C |
| Tₑ | Inlet oil temperature (nominal) | 300 | °C |
| J | Solar irradiance (test range) | 0.6 – 0.9 | kW/m² |
| q | Volume flow rate (actuator limits) | 0 – 10 | l/s |

### Controller Gains

| Symbol | Description | Value | Unit |
|--------|-------------|-------|------|
| Kₚ | Proportional gain | 0.165 | — |
| Kᵢ | Integral gain | 0.014 | 1/s |

**Tuning Method:** Internal Model Control (IMC)  
**Closed-loop time constant:** `τc = 71.4 s` (`τ/3` for fast response)

---

## 🔮 Future Work

Potential extensions to enhance the system:

1. **Advanced Control Strategies**
   - Model Predictive Control (MPC) for constraint handling and optimization
   - Adaptive control for online parameter estimation
   - Sliding-mode control for enhanced robustness

2. **Multi-Variable Control (MIMO)**
   - Extend to multiple collector fields
   - Coordinate temperature and pressure control
   - Optimize overall plant efficiency

3. **Real-World Validation**
   - Test with actual weather data (DNI profiles)
   - Include measurement noise and sensor dynamics
   - Account for actuator delays and pump dynamics

4. **Digital Implementation**
   - Discretize controller for embedded deployment
   - Analyze sampling effects and quantization
   - Implement on industrial PLC or microcontroller

5. **Energy Optimization**
   - Minimize pump power consumption
   - Optimize setpoint trajectory for efficiency
   - Integrate with grid demand forecasting

---

## 📚 Documentation

Detailed technical documentation is available in:

**[`docs/ParabolicTrough.pdf`](docs/ParabolicTrough.pdf)** (To be added)

This technical note covers:
- Complete mathematical derivation of the plant model
- Equilibrium analysis and relative degree calculation
- Detailed controller design methodology (feedforward + PI)
- IMC tuning procedure with calculations
- Comprehensive simulation results with analysis
- Robustness validation under parameter uncertainty

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

You are free to use, modify, and distribute this code for educational and commercial purposes.

---

## 👤 Author

**Priestley Fomeche Njukang**  
Elektro- und Informationstechnik (Electrical and Computer Engineering)  
Hochschule Anhalt (Anhalt University of Applied Sciences)

📧 **Email:** fomechepriestly7@gmail.com  
🔗 **LInkedIn:** [Priestley Fomeche](https://linkedin.com/in/priestley-fomeche)  
💻 **GitHub:** [@charizardmigo](https://github.com/charizardmigo)

---

## 🙏 Acknowledgments

The plant model is based on Example 5.1.3 from:  
[Adamy, J. (2018). Nonlinear Systems and Control. Springer.](https://www.springerprofessional.de/en/nonlinear-systems-and-controls/23780520)

---

## 📌 Project Context

**Course:** Control Systems  
**Institution:** Hochschule Anhalt, Department of Electrical, Mechanical and Industrial Engineering  
**Date:** September 2025  
**Supervisor:** Prof. Dr. Marc Enzmann

**Skills Demonstrated:**
- Nonlinear control system design
- MATLAB/Simulink modeling and simulation
- Feedforward and feedback control integration
- Controller tuning (IMC method)
- Robustness analysis
- Technical documentation

---

**⭐ If you find this project useful, please consider giving it a star!**