# Energy Optimization and Powertrain Simulation for Ultra-Light Electric Vehicles (ULEV)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b%2B-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.XXXXXXX.svg)](https://doi.org/10.5281/zenodo.XXXXXXX)

This repository contains the simulation models, sizing algorithms, and numerical datasets presented in the paper:  
> **"Energy Savings in Ultra-Light Electric Vehicles through Custom Power Electronics and Optimized BLDC Control"**

The provided scripts model the mechanical and electrical dynamics of an ultra-light energy-efficient prototype (Hualkana II) under a closed-circuit track driving cycle, computing vehicle load requirements, BLDC hub motor sizing parameters, and dynamic speed/current tracking performance.

---

## Repository Structure

| File | Description |
| :--- | :--- |
| `vehicle.m` | Vehicle dynamic parameters (aerodynamic drag, rolling resistance, inertia) and track elevation model. |
| `motor_parameters_350W.m` | Electrical and electromechanical baseline parameters for the reference 350 W commercial BLDC motor. |
| `motor_parameters_opt.m` | Parametric specifications for the customized and optimized BLDC hub motor. |
| `performance.m` | Inverse dynamics simulation resolving resistive road loads, required shaft torque, and per-lap energy consumption. |
| `opt_selec_motor.m` | Motor sizing routine determining optimal continuous torque ($T_{\text{RMS}}$), peak limits, operating maps, and $K_v$/$K_t$ constants. |
| `PI_controller.m` | Stiff dynamic simulation (`ode15s`) implementing a closed-loop PI speed controller, back-EMF decoupling, and phase current dynamics. |
| `p_current.mat` | Precomputed numerical dataset containing time vectors, battery current draw ($I_{\text{bat}}$), and PWM duty cycle trajectories. |

---

## System Requirements

- **Environment:** MATLAB R2020b or later.
- **Required Toolboxes:**
  - Control System Toolbox (recommended)
  - Optimization Toolbox (optional, depending on solver version)

---

## Reproducibility & Execution Flow

To reproduce the findings reported in the paper, execute the scripts in the following sequence within the MATLAB environment:

### Step 1: Baseline Track Loads and Inverse Dynamics
Run `performance.m` to calculate the longitudinal forces ($F_{\text{roll}}$, $F_{\text{aero}}$, $F_{\text{grade}}$, $F_{\text{inertia}}$) along the 850-meter circuit and extract the load torque ($T_L$) profile.
```matlab
run('performance.m')
```

### Step 2: Optimal Motor Sizing and Operating Maps
With the inverse load profile loaded in workspace, execute opt_selec_motor.m to evaluate the Safe Operating Area (SOA), determine optimal $K_v$ and $K_t$, and generate motor efficiency histograms:
```matlab
run('opt_selec_motor.m')
```

### Step 3: Closed-Loop Dynamic Response
Run `PI_controller.m` to simulate the electrical and mechanical response during transient acceleration and evaluate current draw under closed-loop control:
```matlab
run('PI_controller.m')
```

### Telemetry & Simulation Data
The file `p_current.mat` contains the exported variables from the averaged quasi-static simulation:
- `t`: Time vector (seconds).
- `I_bateria_consumo`: Battery current profile across the lap (Amperes).
- `Duty`: Effective PWM duty cycle command $[0, 1]$.

### Citation
If you use this code or datasets in your research, please cite my article:

@article{autor2026ulev,
  title={Energy Savings in Ultra-Light Electric Vehicles through Custom Power Electronics and Optimized BLDC Control},
  author={Your Name and Co-authors},
  journal={Journal/Conference Name},
  year={2026},
  doi={10.XXXX/XXXXXX}
}
