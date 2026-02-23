# PID Tuning App (MATLAB App Designer) — Black Hole Algorithm (BHA)

A MATLAB **App Designer** application that tunes a **PID controller** (`Kp`, `Ki`, `Kd`) for a **DC motor** using the **Black Hole Algorithm (BHA)**.

The app supports:
- PID gain tuning with BHA
- Response plots (setpoint vs output), error, and control signal
- Detailed performance metrics (overshoot, rise time, settling time, IAE/ITAE/ISE, effort, smoothness)
- Optional serial connection to Arduino/Raspberry Pi for reading motor status and streaming telemetry
- Optional BHA animation (stars + black hole + event horizon) during optimization

---

## Features

### Core
- **Black Hole Algorithm (BHA)** optimizer for `[Kp, Ki, Kd]`
- Configurable:
  - PID bounds
  - BHA population size and iterations
  - Fitness weights and penalties
  - Sample time and test duration
- **Offline simulation mode** using a motor model `Gc` (transfer function or state-space)

### Visualization
- Motor response graphs:
  - Setpoint vs Output
  - Error vs Time
  - Control Signal vs Time
- Performance metrics table
- Optional BHA animation:
  - Stars (candidate solutions)
  - Black hole (best solution)
  - Event horizon (swallowing boundary)
  - “Swallowed” stars re-spawned

### Hardware (Optional)
- Serial streaming of:
  - `time, setpoint, speed/position, control (PWM)`
- Send updated PID gains from MATLAB to Arduino

---

## Project Structure

Recommended structure:

```text
PID_BHA_App/
  app/
    PID_BHA_App.mlapp            # App Designer file (main GUI)
  matlab/
    bha_pid_tune.m               # Black Hole Algorithm implementation
    fitness_pid.m                # Fitness function (calls simulation + metrics)
    simulate_pid_discrete.m      # Discrete closed-loop simulation (PID + plant)
    compute_metrics.m            # Overshoot, rise time, settling time, IAE/ITAE/ISE, etc.
    motor_model_example.m        # Example motor model creator (optional)
  arduino/
    motor_pid_stream.ino         # Arduino template (optional)
```

---

## Requirements

### Required
- MATLAB (App Designer included)

### Recommended (offline motor model simulation)
- **Control System Toolbox** (for `tf`, `ss`, `c2d`, etc.)

### Optional (hardware integration)
- For Arduino:
  - **MATLAB Support Package for Arduino Hardware** (only if you want MATLAB `arduino()` API; not required for `serialport`)
- For Raspberry Pi:
  - **MATLAB Support Package for Raspberry Pi Hardware**
- Note: Basic serial communication in MATLAB can be done with `serialport` (no extra toolbox required).

---

## Setup

### 1) Clone the repo
```bash
git clone <your-repo-url>
cd PID_BHA_App
```

### 2) Add project folders to MATLAB path
In MATLAB:
- Home → Set Path → Add Folder…
  - Add: `PID_BHA_App/matlab`

Or via command:
```matlab
addpath(genpath(fullfile(pwd,'matlab')));
savepath;
```

### 3) Open the app
Open:
```text
app/PID_BHA_App.mlapp
```
Then click **Run**.

---

## Offline Simulation Mode (Recommended First)

### 1) Define your motor model `Gc` in base workspace
The app expects a variable named `Gc` in MATLAB base workspace.

Example:
```matlab
s = tf('s');
Gc = 1/(0.2*s + 1);     % placeholder example
```

Or run:
```matlab
run("matlab/motor_model_example.m");
```

### 2) Configure app parameters
In the app:
- Set sample time `Ts` (e.g., `0.01`)
- Set test duration `Ttest` (e.g., `2` to `5` seconds)
- Set setpoint (e.g., `1000`)
- Set PID bounds (`Kp/Ki/Kd min/max`)
- Set BHA settings (`Stars`, `Iterations`)
- Set fitness weights and penalties

### 3) Run optimization
Click **Run BHA Tuning**  
Then use **Simulate Best** to re-plot best result and metrics.

---

## Fitness Function Overview

The optimizer minimizes a scalar fitness score based on:
- Tracking error (ITAE/IAE/ISE)
- Overshoot penalty
- Control effort penalty
- Smoothness penalty (reduces PWM jitter)
- Large penalties for unstable/poor behavior (settling too slow, excessive saturation, etc.)

You can edit `matlab/fitness_pid.m` and adjust:
- weights `w1..w4`
- penalties (`penaltyUnstable`, `penaltySaturation`)
- settling band, saturation threshold, etc.

---

## Hardware Streaming Mode (Optional)

### Serial format (suggested)
Arduino sends one line per sample:
```text
time,setpoint,measurement,control
```

Example:
```text
0.12,1000,932,180
```

### Arduino control architecture (recommended)
- Arduino runs the real-time PID loop (fixed `Ts`)
- Arduino streams telemetry to MATLAB app
- MATLAB app visualizes and optionally sends updated gains:
  - `KPID=Kp,Ki,Kd`

> See `/arduino/motor_pid_stream.ino` for a template (adapt to your encoder + driver).

---

## BHA Animation (Optional)

If enabled in the app, an extra axes shows:
- star positions (candidates)
- current black hole (best)
- event horizon circle (2D projection)
- swallowed stars

Visualization typically maps:
- `x = Kp`
- `y = Ki`
- marker size (or color) = `Kd`

---

## Usage Notes / Tips
- Start with **conservative PID bounds**, then expand gradually.
- Use **penalties** to prevent unstable controllers during tuning.
- For DC motor speed:
  - Use derivative filtering (`alpha`) to reduce noise sensitivity.
  - Use saturation limits (`uMax`) consistent with PWM range.

---

## Troubleshooting

### “Property assignment is not allowed when the object is empty”
This usually happens if `createComponents` was overwritten manually.

If using App Designer:
- Do not manually edit `createComponents`.
- In Design View, add a dummy component → Save → remove it → Save again.

### No motor model found
Define `Gc` in MATLAB base workspace:
```matlab
Gc = tf(...);
```

---

## Roadmap (Suggested Improvements)
- Add `scatter3` 3D BHA visualization (Kp, Ki, Kd)
- Add live “Best J vs Iteration” plot
- Add automatic motor model identification (step test + fitting)
- Add safe online tuning mode with stop conditions (current/overspeed limits, current limits)

---

## License
Add your license here (e.g., MIT).

---

## Acknowledgments
- Black Hole Algorithm (BHA) metaheuristic concept used for PID gain optimization.
