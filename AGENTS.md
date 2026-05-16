# AGENTS.md — Quanser IP02 + SEESAW-E

## Quick Start

```matlab
>> startup        % adds folders to path, sets SEESAW_ROOT, runs seesaw_params
>> seesaw_params  % loads hardware params, builds state-space matrices
```

Always call `startup` first — it calls `seesaw_params` and sets `SEESAW_ROOT` for all scripts.

## Pipeline (run section-by-section with Ctrl+Enter per `%%` block)

1. **Modeling** — `scripts/modeling/modeling_pipeline.m`
   - Sections 1–3: analytical model, build `IP02_FreqTest.slx` for chirp
   - §4: load `data/data.mat` (recorded from QUARC hardware)
   - §6: `fminsearch` auto-tunes `B_eq` (cart friction)
   - §10: saves `data/tuned_params.mat`
2. **Control Design** — `scripts/control/`:
   - `pole_placement_design.m` — frequency-domain lead+PI cascade
   - `smc_design.m` — super-twisting sliding mode
   - `mpc_design.m` — model predictive control
   - `luenberger_observer_design.m` — 4-state estimator from `[x_c, theta]`
3. **Evaluation** — `scripts/analysis/`:
   - `robust_controller_comparison.m` — ranks all controllers by theta p2p, voltage effort, smoothness
   - `smc_vs_pp_nonlinear.m` — nonlinear Euler sim with deadzone + Coulomb friction + directional gain bias
   - `hardware_verification.m` — time & frequency domain vs logged QUARC data

## Hardware Deployment

- Model: `models/Seesaw_Control.slx` (cascade), or `models/PolePlacementObserver2024.slx` (state feedback + observer)
- Simulink → **External** mode → Build (`Ctrl+B`) → Connect (`Ctrl+T`) → Start
- **Always hold seesaw level before Start, release gently after.**
- If voltage scopes show rail-to-rail (±22 V) **Stop immediately** and reduce gains.
- Motor nominal limit is **6 V** (not 22 V amplifier limit).
- VoltPAQ-X1 gain switch must be **1×**.

## Critical System Knowledge

- **Plant**: Cart on a seesaw — open-loop unstable (RHP pole ≈ +2.15 rad/s). States: `[x_c, x_c_dot, alpha, alpha_dot]`.
- **Hardware defects** (permanent, software-only mitigation):
  1. **Stiction dead-zone**: ~±0.12 V voltage band where cart won't move.
  2. **Directional asymmetry**: ~2:1 gain bias (fast direction vs slow direction).
  3. These cause a **limit cycle** (±5 cm cart, ±2 deg seesaw) under any linear controller.
- **No current limiting** in the signal chain. Motor current is governed by Ohm's law + back-EMF only.
- **V_sat = 6.0 V** (motor rating). VoltPAQ can output 22 V but will burn the motor.
- **All Simulink models are built programmatically** — do not hand-edit `.slx` files. See `scripts/setup/build_simulink_models.m`.
- **S-functions** in `src/` (cart_table_sfun.m, seesaw_plant_sfun.m) are legacy — main pipeline uses standard State-Space blocks.
- **Build artifacts** go to `build/` (configured by `startup.m` via `Simulink.fileGenControl`).
- **Pole placement design** adds 0.370 kg to cart mass (`M_c_added`).
- **Model tuning**: `fminsearch` on `B_eq` using velocity RMS cost. Tuned params saved to `data/tuned_params.mat`.

## Architecture

- `scripts/config/seesaw_params.m` — single source of truth for all parameters and SS matrices
- `data/tuned_params.mat` — tuned B_eq, rebuilt A/B/C/D for both cart-only and full seesaw
- `data/controller_freq.mat` — pole placement gains
- `data/controller_smc.mat` — SMC gains (sliding surface S, K_eq, k1, k2, phi_bl)
- `data/observer.mat` — Luenberger observer SS matrices

Latest robust benchmark (from deployment plan):
| Controller | theta p2p | Lowest slew limit |
|---|---|---:|
| Fuzzy scheduled SF | ~0.34 deg | 35 V/s |
| LMI-screened SF | ~0.76 deg | 50 V/s |
| Hinf / SMC | ~0.27 deg | 100 V/s |
| Pole placement | ~3.4 deg | 10 V/s |

## Repository Structure

- `scripts/` — MATLAB scripts (config/, control/, modeling/, analysis/). All section-based notebooks.
- `models/` — Simulink models (.slx). Include `Seesaw_Template.slx`, `Seesaw_SMC_*.slx`, `PolePlacement*.slx`, `Seesaw_PP.slx`.
- `data/` — .mat files from hardware runs, tuning, and controller designs.
- `docs/` — figures, deployment plans, thesis source (LaTeX).
- `build/` — Simulink cache + codegen artifacts (gitignored).
- `src/` — legacy S-functions (reference only).
