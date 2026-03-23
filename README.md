# Quanser IP02 + SEESAW-E — MATLAB/QUARC Project

This repo models, identifies, and controls the Quanser IP02 cart + SEESAW-E seesaw
system. The workflow has three phases: **modeling → system identification → control**.

---

## Quick Start

Run this every time you open MATLAB before doing anything else:

```matlab
>> startup        % adds all folders to path, sets SEESAW_ROOT
>> seesaw_params  % loads all hardware constants and builds state-space matrices
```

---

## The Pipeline (in order)

```
seesaw_params
     │
     ▼
frequency_setup         ← builds IP02_FreqTest.slx (chirp excitation model)
     │
     │  [collect hardware data → data/data.mat]
     ▼
modeling_pipeline       ← FRF comparison, auto-tunes B_eq, saves tuned_params.mat
     │
     ▼
control_pipeline        ← designs Lead + PI cascade controller, saves controller.mat
     │
     ▼
build_simulink_models   ← builds Seesaw_Control.slx (closed-loop, ready for QUARC)
     │
     │  [deploy to hardware via QUARC External Mode]
     ▼
  done
```

Each pipeline script is written in notebook style — run it **section by section**
with `Ctrl+Enter` in the MATLAB Editor, reading the console output at each step.

---

## Key Technical Facts

These are critical — getting them wrong will break the model silently.

| Item | Value | Notes |
|------|-------|-------|
| **State ordering** | `[x_c, x_c_dot, alpha, alpha_dot]` | Differs from Quanser ref (`[x_c, theta, x_c_dot, theta_dot]`) |
| **Cart encoder gain** | `K_ec = 2.275e-5 m/count` | Already a *linear* conversion (m, not rad) — do not multiply by `r_pp` again |
| **Seesaw encoder gain** | `K_E_SW / K_gs` rad/count | Gear ratio K_gs = 3 between pivot and encoder shaft |
| **Tuned parameter** | `B_eq` only | Cart friction [N·s/m]. `eta_g` is fixed at 0.90 (hardware spec, not tuned) |
| **Motor model** | Reduced (`L_m = 0`) | `L_m/R_m = 69 µs` — negligible vs mechanical time constants |
| **Back-EMF** | Embedded in `F_c` | Not added separately to `B_eq`; `B_total = B_eq + B_emf` |
| **Open-loop instability** | RHP pole ≈ +2.15 rad/s | Gravity drives seesaw unstable; cart also has a free-drift integrator |
| **Control architecture** | Cascade (not LQR) | Inner loop: Lead on `alpha`; Outer loop: PI on `x_c` |
| **Voltage limit** | ±22 V | VoltPAQ-X1 hard saturation; enforced in every Simulink model |

---

## Scripts

### `startup.m` (root)

Run first. Adds `data/`, `docs/`, `models/`, `scripts/`, `src/` to the MATLAB
path recursively and sets `SEESAW_ROOT` in the base workspace (scripts use this
to locate files without hardcoded paths).

---

### `scripts/config/`

**`seesaw_params.m`** — Central parameter hub. Must be run before anything else.

- Loads all hardware constants from Quanser manuals (masses, motor, gearbox, encoder gains, seesaw geometry)
- Computes derived quantities: `J_pivot`, `alpha_f`, `B_emf`, `B_total`
- Builds **Phase 1** state-space matrices (`A_cart`, `B_cart`, `C_cart`, `D_cart`) — 2-state cart-only model
- Builds **Phase 2** state-space matrices (`A_sw`, `B_sw`, `C_sw`, `D_sw`) — 4-state coupled seesaw model
- Prints open-loop eigenvalues; warns if unstable (expected for the seesaw)

---

### `scripts/setup/`

**`frequency_setup.m`** — Builds `IP02_FreqTest.slx` programmatically.

Chirp (swept-sine) excitation model used during system identification. Logs motor
voltage and encoder data to `data/data.mat`. Called automatically by Section 3 of
`modeling_pipeline.m`, but can be run standalone.

**`build_simulink_models.m`** — Builds all three Simulink models from MATLAB code.

Run after `seesaw_params` (and `control_pipeline` for Phase 3):

| Phase | Model | Purpose |
|-------|-------|---------|
| 1 | `IP02_CartOnTable.slx` | Cart-only open-loop (for table-top ID) |
| 2 | `Seesaw_Full.slx` | Full seesaw open-loop (for FRF collection) |
| 3 | `Seesaw_Control.slx` | **Closed-loop cascade controller** (deploy to hardware) |

Phase 3 is skipped automatically if `controller.mat` does not exist yet.
When QUARC is detected, hardware I/O blocks (HIL Initialize, HIL Write Analog,
HIL Read Encoder) are added; otherwise a simulation-only plant-in-loop variant
is built instead.

---

### `scripts/modeling/`

**`modeling_pipeline.m`** — Master system identification notebook. **Run section by section.**

| Section | What it does |
|---------|-------------|
| 1 | Load `seesaw_params` |
| 2 | Analytical Bode plot of untuned model |
| 3 | Build `IP02_FreqTest.slx` via `frequency_setup.m` |
| 4 | Load & sanity-check hardware data from `data/data.mat` |
| 5 | Overlay untuned model FRF vs hardware FRF (Welch H1 estimator) |
| 6 | Auto-tune `B_eq` (single-variable `fminsearch`; `eta_g` locked at 0.90) |
| 7 | Rebuild `A_cart` / `A_sw` with tuned `B_eq` |
| 8 | FRF overlay tuned model vs hardware (should match) |
| 9 | Time-domain `lsim` vs hardware overlay |
| 10 | Print summary, save `data/tuned_params.mat` |

**`cart_on_table_model.m`** — Standalone Phase 1 nonlinear ODE simulation (`ode45`).
Reference for the cart dynamics; not part of the main pipeline.

**`seesaw_nonlinear_model.m`** — Standalone full seesaw nonlinear ODE simulation.
Useful to quantify linearisation error; not part of the main pipeline.

---

### `scripts/analysis/`

**`frequency_analysis.m`** — Standalone B_eq autotuner. Superseded by Sections 6–8
of `modeling_pipeline.m` but kept as a reference tool.

**`validate_against_hardware.m`** — Standalone time-domain overlay tool.
Superseded by Section 9 of `modeling_pipeline.m` but kept as a reference tool.

---

### `scripts/control/`

**`control_pipeline.m`** — Classical frequency-domain controller design notebook.
**Requires `data/tuned_params.mat`** (run `modeling_pipeline.m` first).
**Run section by section.**

| Section | What it does |
|---------|-------------|
| 1 | Load tuned model; override `B_eq` from `tuned_params.mat` |
| 2 | Extract SISO plants: `G_alpha = sys_full(3,1)`, `G_xc = sys_full(1,1)` |
| 3 | Bode + pole-zero plots of both plants |
| 4 | Design inner-loop **Lead compensator** for `alpha` stabilisation |
| 5 | Nyquist + Bode margins (P=1 RHP pole → need 1 CCW encirclement of −1) |
| 6 | Inner closed-loop poles, step response, overshoot/settling check |
| 7 | Derive outer-loop plant; design **PI compensator** for cart centering |
| 8 | Augmented state-space simulation: 4.5° disturbance recovery |
| 9 | Voltage saturation check (must stay within ±22 V) |
| 10 | Print summary; save `data/controller.mat` |

**Controller architecture:**

```
x_c_ref ──►[Σ]──► C_outer(PI) ──►[Σ]──► C_inner(Lead) ──► Sat(±22V) ──► Motor
               ▲(−)                   ▲(−)
               │ x_c                  │ alpha
```

Inner loop crossover ≈ 5.5 × p_unstable rad/s. Outer loop ≈ 10× slower
than inner (cascade separation). Lead zero placed at 0.85 × p_unstable.

---

## Data Files (`data/`)

| File | Created by | Contents |
|------|-----------|----------|
| `data.mat` | QUARC hardware run | Raw encoder + voltage time series from chirp experiment |
| `tuned_params.mat` | `modeling_pipeline.m` §10 | Tuned `B_eq`, rebuilt `A_sw`/`A_cart`, all SS matrices |
| `controller.mat` | `control_pipeline.m` §10 | `C_inner`, `C_outer` TF objects + gain scalars |

---

## Simulink Models (`models/`)

| File | Built by | Purpose |
|------|---------|---------|
| `IP02_FreqTest.slx` | `frequency_setup.m` | Chirp excitation for system ID |
| `IP02_CartOnTable.slx` | `build_simulink_models.m` | Phase 1 open-loop cart |
| `Seesaw_Full.slx` | `build_simulink_models.m` | Phase 2 open-loop seesaw |
| `Seesaw_Control.slx` | `build_simulink_models.m` | **Phase 3 closed-loop (deploy this)** |

All models are built programmatically — do not edit the `.slx` files by hand,
as they will be overwritten next time the build script runs.

---

## S-Functions (`src/`)

`cart_table_sfun.m` and `seesaw_plant_sfun.m` are Level-2 S-Functions that
implement the plant dynamics directly in MATLAB code. These are **legacy/reference
only** — the main pipeline uses standard Simulink State-Space blocks (no TLC files
required, QUARC compatible). The S-functions are kept if you want to experiment
with nonlinear dynamics inside Simulink.

---

## Hardware Startup Checklist (QUARC)

1. `startup` then `seesaw_params` in MATLAB
2. Connect Q2-USB to PC
3. Power on VoltPAQ-X1 — **set Gain switch to 1×**
4. Open the model in Simulink
5. Set Simulation Mode to **External**
6. QUARC → Build (`Ctrl+B`) → Connect (`Ctrl+T`) → Start
7. **For `Seesaw_Control.slx`: hold the seesaw level by hand before starting, release gently after clicking Start**
8. Watch the Voltage scope — if it's rail-to-rail (±22 V), stop and reduce gains

---

## Other Directories

- **`docs/`** — Hardware manuals, lab reports, and Quanser reference documents
- **`slprj/`** — Auto-generated Simulink/QUARC C-code cache; safe to ignore
