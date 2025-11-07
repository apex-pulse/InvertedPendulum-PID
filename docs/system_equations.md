# System Equations

This document derives the mathematical model for the inverted pendulum on a cart system.

## System Description

The system consists of:
- A cart of mass `M` that moves horizontally along a frictionless track
- A pendulum of mass `m` and length `L` attached to the cart via a pivot
- A horizontal control force `F` applied to the cart

## State Variables

- `x` — cart position (m)
- `θ` — pendulum angle from vertical (rad, positive = clockwise)
- `ẋ` — cart velocity (m/s)
- `θ̇` — angular velocity (rad/s)

## Equations of Motion

Using Lagrangian mechanics, the nonlinear equations of motion are:

### Cart Acceleration

```
ẍ = [F + m·L·(θ̇² sin θ - θ̈ cos θ)] / (M + m)
```

### Pendulum Angular Acceleration

```
θ̈ = [g sin θ + cos θ · (F + m·L·θ̇² sin θ)/(M + m)] / [L·(4/3 - m·cos²θ/(M + m))]
```

Where:
- `g` = 9.81 m/s² (gravitational acceleration)
- `F` = control force applied to cart (N)

## Linearization

For small angles (θ ≈ 0), we linearize using:
- sin(θ) ≈ θ
- cos(θ) ≈ 1
- θ̇² ≈ 0

This yields the **linearized state-space model**:

```
ẍ ≈ [F - m·L·θ̈] / (M + m)

θ̈ ≈ [g·θ + F/(M + m)] / [L·(4/3 - m/(M + m))]
```

## State-Space Representation

Defining the state vector:

```
X = [x, θ, ẋ, θ̇]ᵀ
```

The linearized system can be written as:

```
Ẋ = A·X + B·F
```

Where `A` and `B` are derived from the linearized equations.

## Control Strategy

A **PID controller** is implemented to stabilize the pendulum:

```
F(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·ė(t)
```

Where:
- `e(t) = θ_ref - θ(t)` (error signal)
- `Kp`, `Ki`, `Kd` are tunable gains

## Numerical Integration

The nonlinear equations are integrated using the **4th-order Runge-Kutta (RK4)** method with adaptive time stepping.

---

## References

1. Ogata, K. (2010). *Modern Control Engineering*. Prentice Hall.
2. Åström, K. J., & Murray, R. M. (2008). *Feedback Systems*. Princeton University Press.
3. Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2015). *Feedback Control of Dynamic Systems*. Pearson.
