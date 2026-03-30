# PID Tuning & System Optimization App (MATLAB App Designer) — Black Hole Algorithm (BHA)

A MATLAB App Designer application for tuning control systems using the Black Hole Optimization Algorithm (BHA).  
The app supports multiple dynamic systems and uses simulation-based optimization to automatically tune controller parameters.

---

## Overview

This project applies the Black Hole Optimization Algorithm (BHA) to tune controller parameters (PI / PD / PID) for different dynamic systems.

Instead of manual tuning or classical methods, this approach:
- Uses performance-based cost functions
- Works with multiple systems using a unified framework

---

## Supported Systems

The application currently supports:

- DC Motor Speed Control  
- Ball and Beam System  
- Cruise Control System  

Each system includes:
- Simulink model  
- Initialization file (`init_*`)  
- Cost function (`cost_*`)  

---

## Core Features

### Optimization Engine
- Black Hole Algorithm (BHA)
- Supports:
  - PI, PD, PID controllers
- Configurable:
  - Number of stars (population size)
  - Number of iterations
- Tracks best solution per iteration

---

### Cost Function

The optimizer evaluates controller performance using real metrics:

- Overshoot (OS)
- Steady-state error (ESS)
- Peak time / rise time
- Stability checks

Example:

```matlab
J = 0.8*OS + 0.1*ess + 0.1*PeakTime;
```

Unstable or invalid simulations are penalized:

```matlab
J = 1e6;
```

---

### Visualization & Results

- Step response plots
- Performance metrics using `stepinfo`
- Results table:
  - Kp, Ki, Kd
  - Overshoot
  - Settling time
  - Peak time
- Iteration tracking (Best Cost per iteration)

---

### App Interface (MATLAB App Designer)

The GUI allows the user to:

- Select system (Motor / Ball & Beam / Cruise)
- Select controller type (PI / PD / PID)
- Set parameter bounds
- Configure optimization settings
- Run optimization directly
- View results in plots and tables

---

## Project Structure

```text
project/

├── app/
│   └── OptimizerApp.mlapp

├── systems/
│   ├── motor/
│   │   ├── CLMSBD.slx
│   │   ├── init_motor.m
│   │   └── cost_motor.m
│
│   ├── ball_and_beam/
│   │   ├── CLBandBBD.slx
│   │   ├── init_ballandbeam.m
│   │   └── cost_ballandbeam.m
│
│   ├── cruise_control/
│   │   ├── CLCCBD.slx
│   │   ├── init_cruise.m
│   │   └── cost_cruise.m

├── optimization/
│   └── blackHole.m

├── main.m
└── README.md
```

---

## How It Works

### 1. Initialization
Each system defines:
- Physical parameters
- Transfer function
- Reference signal

---

### 2. Black Hole Optimization

- Generate random candidate solutions (stars)
- Evaluate cost using simulation
- Select best solution → Black Hole
- Move stars toward Black Hole
- Replace swallowed stars randomly

---

### 3. Simulation-Based Evaluation

Each candidate solution:
- Applies controller gains to Simulink model
- Runs simulation
- Extracts output response
- Computes performance metrics

---

### 4. Constraint Handling

The system enforces constraints such as:
- Maximum overshoot
- Desired rise time
- Stability requirements

Violations result in large penalties in the cost function.

---

## Requirements

### Required
- MATLAB
- Simulink

### Recommended
- Control System Toolbox

---

## How to Run

### Using main.m
```matlab
run('main.m')
```

### Using the App
- Open the `.mlapp` file
- Click Run
- Configure parameters
- Start optimization

---

## Key Improvements

- Added multi-system support
- Integrated Simulink-based evaluation
- Implemented realistic cost functions with constraints
- Built full MATLAB App Designer interface
- Added results table and performance metrics
- Improved BHA stability and tracking

---

## Known Issues

- Simulink path conflicts (shadowed models)
- Missing base workspace variables (e.g., `ref`, `step_amp`)
- Requires initialization before simulation

---

## Future Work

- Real-time hardware integration (Arduino)
- Advanced BHA visualization
- Automatic system identification
- Multi-objective optimization
- Improved constraint handling in GUI

---

## Project Context

This project is developed as part of:

EN8914 — Final Year Project

It demonstrates:
- Application of optimization algorithms in control systems
- Integration between MATLAB, Simulink, and GUI design
- Practical engineering problem solving

---

## Notes

- Update the parameters of the system in initialization files before simulation
- Use realistic parameter bounds to avoid instability
- Cost function design strongly affects optimization results
