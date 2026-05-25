# Hardware Verification Testing Plan

One monolithic script (`hardware_verification.m`) plus one importer (`import_hardware_validation_log.m`). New per-controller HW models in `validation/models/`. Naming is `theta` everywhere.

## Layout

```
validation/
├── lib/seesaw_hw_lib.slx        shared HIL_IO + DeadzoneInverse + DisturbanceBank + TelemetryBus
├── models/
│   ├── HW_PID.slx                  cascade inner cart-PID + outer theta-PID (has dead-zone inverse)
│   ├── HW_StateFeedback.slx        PP (4-state) or LQR/LQI (5-state augmented)
│   ├── HW_StateFeedback_Observer.slx   LQG (LQR + Kalman) / Luenberger variant
│   ├── HW_SMC.slx                  classical SMC (K_eq + eta*tanh(s/phi))
│   └── legacy/                      old HardwareValidation_HWTest*.slx
├── scripts/
│   ├── hardware_verification.m      monolith: Phase 1 builds signals, Phase 2 analyses
│   ├── import_hardware_validation_log.m  converts To-Host-File raw log to hw_<exp>.mat
│   ├── build_seesaw_hw_lib.m        regenerate the library
│   ├── build_hw_models.m            regenerate the four HW models
│   └── legacy/                      old load_hardware_validation_config.m
└── data/
    ├── _signals/                    From-File-format excitation files (Phase 1 output)
    ├── hw_test_signals.mat          legacy combined excitation file (Phase 1 output)
    ├── hw_free_run.mat              after import_hardware_validation_log(..., 'free')
    ├── hw_step_response.mat         after import (..., 'step')
    ├── hw_prbs_response.mat         after import (..., 'prbs')
    ├── hw_chirp_response.mat        after import (..., 'chirp')
    ├── hw_obs_free.mat              after import (..., 'obs')
    └── verification_results.mat     Phase 2 output (struct `results`)
```

## Pre-test (every session, QUARC PC)

1. `run startup.m`
2. `hardware_verification`  -- Phase 1 unconditionally rebuilds:
   - `data/hw_test_signals.mat`  (the d_free / d_step / d_prbs / d_chirp / d_stepped_sine matrices)
   - `data/_signals/{step,prbs,chirp,stepped_sine}.mat`  (row-pair format read by the HW models' From File blocks)
3. Verify VoltPAQ-X1 gain switch is at **1x**

## Run an experiment

For each controller:

1. Load the relevant gains into the base workspace, e.g.
   - PID:        `load data/controllers/controller_inner_pid; load data/controllers/controller_outer_pid`
   - PP:         `load data/controllers/controller_freq; K_aug_hw = [Kf 0];`
   - LQR/LQI:    `load data/controllers/controller_lqr;  K_aug_hw = K_lqr;`
   - LQG:        `load data/controllers/controller_lqr;  K_aug_hw = K_lqr; load data/params/observer_kalman; A_obs_hw=A_obs; B_obs_hw=B_obs; C_obs_hw=C_obs; D_obs_hw=D_obs;`
   - SMC:        `load data/controllers/controller_smc;  K_eq_smc_hw=classical.K_eq; S_smc_hw=classical.S; eta_smc_hw=classical.eta; phi_bl_smc_hw=classical.phi_bl;`
2. Open the matching model from `validation/models/HW_*.slx`.
3. Pick the disturbance source by setting the `d_select` Constant block:

   | `d_select` | Source        | Use case |
   |------------|---------------|----------|
   | 1          | Zero          | free-run / observer experiment |
   | 2          | Step          | disturbance rejection |
   | 3          | PRBS          | broadband FRF (preferred) |
   | 4          | Chirp         | broadband FRF (sweep alternative) |
   | 5          | Stepped sine  | per-frequency FRF with best SNR |

4. Hold the seesaw level. Build (Ctrl+B) -> Connect (Ctrl+T) -> Start. Release gently.
5. Save the To-Host-File output as `data/hw_raw_<exp>.mat` (a single N x 14 matrix).
6. Import:
   ```matlab
   import_hardware_validation_log('data/hw_raw_free.mat', 'free')   % -> hw_free_run.mat
   import_hardware_validation_log('data/hw_raw_step.mat', 'step')   % -> hw_step_response.mat
   import_hardware_validation_log('data/hw_raw_prbs.mat', 'prbs')   % -> hw_prbs_response.mat
   import_hardware_validation_log('data/hw_raw_obs.mat',  'obs')    % -> hw_obs_free.mat
   ```
7. Re-run `hardware_verification` -- Phase 2 picks up whichever data files exist and writes figures to `validation/docs/figures/Verification-*.png` plus `data/verification_results.mat`.

## To-Host-File log format (all HW models)

Single N x 14 numeric matrix:

```
[ t, x_c, theta, V_m, d, x_fb(1..5), x_obs(1..4) ]
```

`x_obs(1..4)` is `zeros(4,1)` for non-observer models. Older 10/13-column logs from the legacy `HardwareValidation_HWTest.slx` are auto-detected by the importer.

## Safety checks (throughout)

- Voltage scope must not pin rail-to-rail (`+/- 22 V`). If it does, **STOP**.
- `V_sat = 10.4 V`; nominal motor limit **6 V**. The `DeadzoneInverse` block (inside the model) and the `Motor Voltage Saturation` block (in `HIL_IO_Out`) both clamp to `+/- V_sat`.
- Always hold the seesaw level before starting; release gently.
- If `verify` reports `vm_max` above `V_sat * 0.9`, reduce the controller gain or disturbance amplitude before another run.
