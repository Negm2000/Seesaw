# Quanser IP02 + SEESAW-E — MATLAB/QUARC Project

Models, identifies, and controls the Quanser IP02 cart + SEESAW-E seesaw system.
Three phases: **modeling → system identification → control**.

## Quick Start

```matlab
>> startup        % adds all folders to path, sets SEESAW_ROOT
>> seesaw_params  % loads hardware constants, builds state-space matrices
```

---

## Pipeline

![Pipeline flowchart](docs/figures/pipeline.png)

Run each script **section by section** (`Ctrl+Enter` per `%%` block).

---

## Physical System

![Seesaw system schematic](docs/figures/system_schematic.png)

The seesaw is open-loop unstable: tilt → gravity accelerates the cart further in the same direction (RHP pole ≈ +2.15 rad/s). The cart also has a free-drift integrator. Both must be stabilised by the controller.

---

## State-Space Model

![State-space model and SISO extraction](docs/figures/state_space.png)

---

## Control Architecture

![Cascade control block diagram](docs/figures/control_cascade.png)

| Loop | Compensator | Crossover | Constraint |
|------|-------------|-----------|------------|
| Inner | Lead `Kc(s+zc)/(s+pc)` | `5.5 × p_u` | Must cross over before RHP pole destabilises |
| Outer | PI `K(s+ωi)/s` | `ωc / 10` | 10× slower than inner (cascade separation) |

`G_α` has P=1 open-loop RHP pole → `L(s) = C_inner · G_α` needs exactly **1 CCW encirclement of −1**.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| State vector | `[x_c, ẋ_c, α, α̇]` |
| Cart encoder gain | `K_ec = 2.275e-5 m/count` |
| Seesaw encoder gain | `K_E_SW / K_gs` rad/count |
| Identified parameter | `B_eq` — cart viscous friction [N·s/m] |
| Motor model | Reduced order (`L_m = 0`) |
| Amplifier saturation | ±22 V |

---

## Scripts

**`startup.m`** — Run first. Adds all folders to path, sets `SEESAW_ROOT`.

**`scripts/config/seesaw_params.m`** — Central parameter hub. Builds Phase 1 (`A_cart`) and Phase 2 (`A_sw`) state-space matrices. Prints eigenvalues.

**`scripts/setup/frequency_setup.m`** — Builds `IP02_FreqTest.slx` (chirp excitation for system ID). Called by `modeling_pipeline.m §3`.

**`scripts/setup/build_simulink_models.m`** — Builds all Simulink models programmatically. Do not hand-edit the `.slx` files.

| Phase | Model | Purpose |
|-------|-------|---------|
| 1 | `IP02_CartOnTable.slx` | Cart-only open-loop |
| 2 | `Seesaw_Full.slx` | Full seesaw open-loop |
| 3 | `Seesaw_Control.slx` | Closed-loop cascade — deploy this |

**`scripts/modeling/modeling_pipeline.m`** — System ID notebook.

| § | Action |
|---|--------|
| 1–2 | Load params, analytical Bode |
| 3–4 | Build `IP02_FreqTest.slx`, load `data.mat` |
| 5 | FRF overlay (untuned) |
| 6–8 | Auto-tune `B_eq` (`fminsearch`), rebuild, FRF overlay (tuned) |
| 9–10 | `lsim` vs hardware, save `tuned_params.mat` |

**`scripts/control/control_pipeline.m`** — Controller design notebook. Requires `tuned_params.mat`.

| § | Action |
|---|--------|
| 1–3 | Load model, extract SISO plants, Bode/PZ maps |
| 4–6 | Design Lead compensator, Nyquist, inner CL validation |
| 7–8 | Outer PI design, augmented SS simulation |
| 9–10 | Voltage saturation check, save `controller.mat` |

**`scripts/analysis/frequency_analysis.m`** and **`validate_against_hardware.m`** — Standalone tools, superseded by `modeling_pipeline.m §6–9`. Kept as reference.

**`scripts/modeling/cart_on_table_model.m`** and **`seesaw_nonlinear_model.m`** — Standalone `ode45` sims. Reference only.

---

## Data & Models

| File | Created by | Contents |
|------|-----------|----------|
| `data/data.mat` | QUARC hardware | Raw encoder + voltage from chirp test |
| `data/tuned_params.mat` | `modeling_pipeline.m §10` | Tuned `B_eq`, rebuilt SS matrices |
| `data/controller.mat` | `control_pipeline.m §10` | `C_inner`, `C_outer`, gain scalars |
| `models/IP02_FreqTest.slx` | `frequency_setup.m` | Chirp excitation |
| `models/IP02_CartOnTable.slx` | `build_simulink_models.m` | Phase 1 |
| `models/Seesaw_Full.slx` | `build_simulink_models.m` | Phase 2 |
| `models/Seesaw_Control.slx` | `build_simulink_models.m` | Phase 3 — deploy this |

**`src/`** — Level-2 S-Functions (`cart_table_sfun.m`, `seesaw_plant_sfun.m`). Legacy/reference; main pipeline uses standard State-Space blocks.

---

## Hardware Checklist

1. `startup` → `seesaw_params`
2. Connect Q2-USB, power on VoltPAQ-X1 — **Gain switch to 1×**
3. Simulink → **External** mode → Build (`Ctrl+B`) → Connect (`Ctrl+T`) → Start
4. **`Seesaw_Control.slx` only:** hold seesaw level → Start → release gently
5. Watch Voltage scope — if rail-to-rail (±22 V) **Stop immediately** and reduce gains
