# Hardware Verification Testing Plan

## Pre-Test Setup
- [ ] Run `startup.m` on QUARC PC
- [ ] Run `hardware_verification.m` (Phase 1) to generate `data/hw_test_signals.mat`
- [ ] Load signals: `load data/hw_test_signals.mat`
- [ ] Configure controller/observer slot: `load_hardware_validation_config('pole_placement','none')`
- [ ] Open `models/hardware_validation/HardwareValidation_HWTest.slx`
- [ ] Verify VoltPAQ-X1 gain switch is at **1x**

## Experiment A — Free-Run Bounded Oscillation
- [ ] Set `d_inj = d_free;`
- [ ] Build (Ctrl+B) → Connect (Ctrl+T) → Start
- [ ] Hold seesaw level before Start, release gently
- [ ] Let run for full duration (~90 s)
- [ ] Stop and import log: `import_hardware_validation_log('data/hw_raw_free.mat', 'free')`
- [ ] Verify `data/hw_free_run.mat` exists with `hw_t, hw_xc, hw_alpha, hw_vm`

## Experiment B — Disturbance Rejection Step
- [ ] Set `d_inj = d_step;`
- [ ] Build → Connect → Start (hold level, release gently)
- [ ] Let run for full duration
- [ ] Stop and import log: `import_hardware_validation_log('data/hw_raw_step.mat', 'step')`
- [ ] Verify `data/hw_step_response.mat` exists

## Experiment C — Broadband Frequency Response (PRBS)
- [ ] Set `d_inj = d_prbs;` (or `d_chirp` as fallback)
- [ ] Build → Connect → Start (hold level, release gently)
- [ ] Let run for full 90 s
- [ ] Stop and import log: `import_hardware_validation_log('data/hw_raw_prbs.mat', 'prbs')`
- [ ] Verify `data/hw_prbs_response.mat` exists

## Experiment F — Observer Verification
- [ ] Reconfigure with observer: `load_hardware_validation_config('pole_placement','leuenberger')`
- [ ] Set `d_inj = d_free;`
- [ ] Build → Connect → Start (hold level, release gently)
- [ ] Let run for full duration
- [ ] Stop and import log: `import_hardware_validation_log('data/hw_raw_obs.mat', 'obs')`
- [ ] Verify `data/hw_obs_free.mat` exists with `hw_t, hw_xc, hw_alpha, hw_vm, hw_xc_hat, hw_xcdot_hat, hw_alpha_hat, hw_alphadot_hat`

## Post-Test Analysis (Phase 2)
- [ ] Re-run `hardware_verification.m` — all sections should execute
- [ ] Check pass/fail criteria:
  - [ ] RMS angle < 2 deg
  - [ ] Peak voltage < 5 V
  - [ ] Sensitivity bandwidth > 1 Hz
- [ ] Review generated figures in `docs/figures/Verification-*.png`
- [ ] Confirm `data/verification_results.mat` saved

## Optional: Repeat with Alternate Controllers
- [ ] LQR: `load_hardware_validation_config('lqr','leuenberger')`
- [ ] PID: `load_hardware_validation_config('pid','kalman')`
- [ ] Re-run experiments A-C for each and compare results

## Safety Checks (Throughout)
- [ ] Monitor voltage scopes — if rail-to-rail (+/-22 V), **STOP immediately**
- [ ] Motor nominal limit is **6 V** — never exceed
- [ ] Always hold seesaw level before starting
