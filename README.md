# Two-Layer Adaptive MPC for Autonomous UAV Weed Management

A software-based, two-layer adaptive Model Predictive Control (MPC) framework for autonomous UAV weed management in precision agriculture, implemented entirely in MATLAB.

---

## Overview

Weed infestation causes 10–15% of global crop yield losses annually, and traditional uniform spraying wastes over 70% of herbicide. This project addresses these issues using a **Site-Specific Weed Management (SSWM)** approach: a UAV treats only where weeds exist.

The core contribution is a **two-layer adaptive MPC** that simultaneously:

1. **Estimates unknown plant parameters online** (drag coefficient, wind components) using Recursive Least Squares (RLS).
2. **Adapts MPC cost-function weights in real time** based on mission state (battery, weed density, wind, goal distance).

A fixed-parameter MPC baseline is used for comparison under identical conditions.

---

## Key Features

- **UAV Model:** Unicycle with wind disturbance, aerodynamic drag, and battery state-of-charge (SOC) dynamics.
- **MPC Formulation:** Weighted multi-term cost function over a 6-step prediction horizon, solved as a QP using `fmincon` (SQP) with an analytic gradient.
- **Layer 1 — RLS Estimator:** Online identification of `k_d` (drag), `w_x`, `w_y` (wind components). Converges to true values within ~30 steps (3 s at 10 Hz).
- **Layer 2 — Weight Adaptation:** Four heuristic rules map mission state to cost weights `w_1`–`w_5`.
- **Sensor Filtering:** Velocity-compensated low-pass filter reduces GPS noise (σ = 0.5 m) position RMSE by 38.7%.
- **Field Boundary Handling:** Plant-side hard clamping avoids QP infeasibility.
- **Deterministic Execution:** Fixed random seed (`rng(42)`) guarantees full reproducibility.

---

## Results Summary

| Metric | Fixed MPC | Adaptive MPC |
|---|---|---|
| Mission duration | 41.0 s | 108.0 s |
| Final SOC | 0.42 | 0.20 |
| SOC depletion rate | 0.0144 SOC/s | 0.0074 SOC/s |
| Weed coverage | 7.4% | 8.9% |
| Spray efficiency | 70.0% | 66.4% |
| Trajectory RMSE | 12.7 m | 18.7 m |
| Energy saving | — | 48.6% |

**Key findings:**

- Adaptive MPC extends mission duration by **2.63×** compared to Fixed MPC.
- Adaptive MPC achieves **~20% higher weed coverage** with **48.6% lower SOC depletion rate**.
- RLS converges to **zero-percent estimation error** for all three estimated parameters within a few timesteps.
- Under realistic GPS noise, the velocity-compensated filter reduces position RMSE by **38.7%** (0.62 m → 0.38 m).
- The trade-off is higher trajectory-tracking RMSE (18.7 m vs 12.7 m), reflecting a deliberate **accuracy–coverage–energy trade-off**.

---

## Repository Structure

```
.
├── main.m                  # Entry point; orchestrates pipeline
├── sim_params.m            # All configuration constants
├── build_weed_map.m        # Weed density map W(x, y)
├── run_mission.m           # Closed-loop mission at 10 Hz
├── uav_dynamics.m          # Plant model + linearization
├── mpc_controller.m        # MPC cost and QP solve
├── adaptive_update.m       # Layer 1: RLS parameter estimation
├── adaptive_weights.m      # Layer 2: weight adaptation
├── low_pass_filter.m       # Velocity-compensated sensor filter
├── compute_metrics.m       # Coverage, energy, RMSE
├── plot_results.m          # Generates eight figures
└── figures/                # Output figures
```

---

## Getting Started

### Requirements

- MATLAB R2022b or later
- Optimization Toolbox (for `fmincon`)

### Run the Simulation

```matlab
% In MATLAB:
main
```

This runs the full pipeline: builds the weed map, runs the closed-loop mission for both controllers, computes metrics, and generates all figures.

### Configuration

All parameters live in `sim_params.m`. To change the mission, edit that file — no magic numbers exist elsewhere.

---

## Methodology

### Plant Dynamics

The true UAV dynamics are:

```
ẋ = [v·cos(ψ) + w_x,  v·sin(ψ) + w_y,  ω,  a - k_d·v²,  -(1/E_max)·(k_v·v² + k_h + k_s·s)]ᵀ
```

Euler integration with `dt = 0.1 s` yields the discrete-time model.

### MPC Cost Function

```
J = Σⱼ [w₁·(vⱼ² + ωⱼ²) + w₂·sⱼ·(1 - 2Wⱼ) + w₃·(1 - SOCⱼ)] + J_terminal
```

### RLS Update

```
θ̂(k) = θ̂(k-1) + K(k)·[y(k) - φ(k)ᵀ·θ̂(k-1)]
K(k) = P(k-1)·φ(k) / (λ + φ(k)ᵀ·P(k-1)·φ(k))
P(k) = (1/λ)·[P(k-1) - K(k)·φ(k)ᵀ·P(k-1)]
```

Forgetting factor: `λ = 0.98`. Estimated parameters: `θ = [k_d, w_x, w_y]ᵀ`.

### Weight Adaptation Rules

| Weight | Nominal | Max | Driving State |
|---|---|---|---|
| w₁ (smoothness) | 0.50 | 1.08 | Crosswind magnitude |
| w₂ (treatment) | 1.00 | 2.00 | Local weed density |
| w₃ (energy) | 1.00 | 2.90 | Battery SOC depletion |
| w₄ (terminal) | 5.00 | 5.10 | Waypoint proximity |
| w₅ (safety) | 10.00 | 10.00 | Invariant |

---

## Figures

The repository generates eight MATLAB figures:

1. Trajectory comparison (Fixed vs Adaptive)
2. Battery state of charge over time
3. Control inputs (acceleration, yaw rate, spray)
4. Parameter estimation convergence (k_d, k_v, w_x, w_y)
5. Sensor noise: true vs noisy vs filtered position
6. Adaptive weight evolution over mission
7. Individual trajectories on weed map
8. Performance metrics comparison (bar chart)

---

## Future Directions

1. **Real dataset integration:** Replace the synthetic Gaussian-mixture weed map with the CoFly-WeedDB dataset.
2. **Kalman filter:** Replace the low-pass filter with an Extended Kalman Filter.
3. **Hardware validation:** Port to Gazebo/PX4 for sim-to-real transfer.
4. **Multi-UAV coordination:** Extend to multiple UAVs with task allocation.
5. **Theoretical stability analysis:** Formal Lyapunov-based proofs.
6. **Monte Carlo analysis:** Run across multiple seeds for mean ± std.

---

## References

1. Åström, K. J., & Wittenmark, B. (2013). *Adaptive Control* (2nd ed.). Dover.
2. Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design* (2nd ed.). Nob Hill.
3. Ljung, L. (1999). *System Identification: Theory for the User* (2nd ed.). Prentice Hall.
4. Fahimi, F. (2008). *Autonomous Robots: Modeling, Path Planning, and Control*. Springer.
5. Zhang, C., & Kovacs, J. M. (2012). The application of small unmanned aerial systems for precision agriculture: A review. *Precision Agriculture*, 13(6), 693–712.
6. Oerke, E. C. (2006). Crop losses to pests. *Journal of Agricultural Science*, 144(1), 31–43.
7. Chen, X., et al. (2026). Context-Aware Proactive Self-Adaptation: A Two-Layer MPC Approach. *ACM TAAS*.
8. Wei, Y., et al. (2026). Hierarchical adaptive multi-mode fault-tolerant control for UAVs. *Acta Aeronautica et Astronautica Sinica*.

---

## Course Context

Developed for **22AIE441 — Dynamics and Control of Robotics**  
School of Computing, Amaravati — 522503

**Authors:**

- M. Sri Vara Lakshmi (AV.SC.U4AIE24137)
- M. Ujwala (AV.SC.U4AIE24138)
- S. Soumya (AV.SC.U4AIE155)
