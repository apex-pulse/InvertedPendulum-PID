# PID-Controlled Inverted Pendulum (Cart)

**Author:** Vahid Hassanzadeh

**Field:** Mechanical Engineering — Control Systems

## Project Overview

Classic benchmark: stabilize an inverted pendulum mounted on a cart using a PID controller that computes horizontal force on the cart. This repo contains a MATLAB implementation (numerical sim) and instructions to build a Simulink model.

**Why this project?** Professors expect to see: system modeling, control design, simulation, and documentation. This repository demonstrates all four with clear code and reproducible results.

---

## What's Included

- `model/parameters.m` — physical parameters and initial conditions.
- `control/pid_design.m` — self-contained MATLAB script implementing the nonlinear dynamics, PID controller, and simulation (RK4 fixed-step solver).
- `control/response_plot.m` — plotting and saving of results.
- `docs/system_equations.md` — derivation and linearization for reporting.
- `model/inverted_pendulum.slx` — Simulink model (instructions to create provided).

---

## Quick Start (MATLAB)

1. Clone this repo.
2. Open MATLAB and set the project folder as current directory.
3. Run `model/parameters.m` to load constants.
4. Run `control/pid_design.m` to simulate closed-loop behavior.
5. Run `control/response_plot.m` to generate `results/step_response.png`.

---

## Suggested Experiments

- Sweep PID gains and measure settling time and overshoot.
- Compare PID vs LQR stabilization (provide linearized model from docs/).
- Add state estimation (simple Kalman filter) for noisy sensor tests.

---

## Citation / Acknowledgements

Based on classical inverted-pendulum derivations (standard control textbooks and Simulink examples).
