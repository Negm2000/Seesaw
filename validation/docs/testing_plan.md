# Hardware Verification Testing Plan — Single-Run Protocol

## Pre-Test Setup
- [ ] Run `startup.m` on QUARC PC
- [ ] Run `validation/scripts/hardware_verification.m` (Phase 1) to generate `validation/data/hw_test_signals.mat`
- [ ] Load signals: `load validation/data/hw_test_signals.mat`
- [ ] Configure controller: `load_hardware_validation_config('pole_placement')`
- [ ] Open `models/Seesaw_Validation.slx`
- [ ] Verify VoltPAQ-X1 gain switch is at **1x**
- [ ] Set QUARC stop time to match `total_duration_s` (displayed by Phase 1)

## Single-Run Experiment

### Signal Chain
```
d_inj (FromWorkspace) → Add Disturbance (after controller) → Voltage Saturation (±V_sat) → ON/OFF Switch → Motor
```

### Logged Columns (simplified format)
```
[time | seg_id | xc | alpha | xc_dot | alpha_dot | u_ctrl | u_presat | Vm | d_inj]
```

### Protocol Segments (~5 min total)
| Seg ID | Duration | Description |
|--------|----------|-------------|
| 0 | 5 s | Offset reset + mini-liftoff (excluded) |
| 1 | 10 s | Baseline free-run (d = 0) |
| 2 | 15 s | +1 V disturbance step |
| 3 | 15 s | Recovery (d = 0) |
| 4 | 15 s | -1 V disturbance step |
| 5 | 15 s | Recovery (d = 0) |
| 6 | 0.3 s | +1 V pulse (impulse surrogate) |
| 7 | 10 s | Recovery (d = 0) |
| 10-21 | ~210 s | 12 stepped sine frequencies (0.1–10 Hz) |
| 99 | 10 s | Final zero recovery |

### Sine Frequencies (1 V amplitude, zero-mean)
```
0.10, 0.16, 0.25, 0.40, 0.63, 1.00, 1.60, 2.50, 4.00, 6.30, 8.00, 10.00 Hz
```

### Sine Rule (per frequency)
- 7 cycles total
- Cycle 1: half-cosine ramp in
- Cycles 2-3: discard/settle
- Cycles 4-6: analyze (sine fitting)
- Cycle 7: half-cosine ramp out

### Procedure
- [ ] Build (Ctrl+B) → Connect (Ctrl+T) → Start
- [ ] **Hold seesaw level before Start, release gently**
- [ ] Monitor voltage scope — if rail-to-rail (±22 V), **STOP immediately**
- [ ] Let run for full protocol duration (single continuous run)
- [ ] Stop and save the To Host File log

### Post-Run Import
```matlab
>> import_hardware_validation_log('validation/data/hw_raw_single.mat', 'single')
```
This creates `validation/data/hw_single_run.mat` with all named variables.

## Post-Test Analysis (Phase 2)
- [ ] Re-run `validation/scripts/hardware_verification.m`
- [ ] Script auto-detects `hw_single_run.mat` and runs single-run analysis
- [ ] Review generated figures in `validation/docs/figures/Validation-*.png`
- [ ] Confirm `validation/data/validation_results_single_run.mat` saved

## Data Validity Rules
- **Hard invalid**: actuator saturation (|u_presat| > 6V) during analyzed window
- **Hard abort**: cart approaches rail or operator sees unsafe behavior
- **Reported diagnostics**: max |x_c|, max |alpha|, sine-fit residual, baseline floor ratio

## Report Deliverables
- [ ] Time validation: measured vs ideal model for +1V and -1V step (baseline-subtracted)
- [ ] Impulse surrogate: measured vs model for +1V pulse
- [ ] Bode plot: measured sine-fit points over ideal model curve for d -> alpha
- [ ] Supporting Bode: d -> x_c
- [ ] Full-run overview plot with segment labels
- [ ] Data validity diagnostic plot

## Optional: Repeat with Alternate Controllers
- [ ] LQR: `load_hardware_validation_config('lqr')`
- [ ] Re-run single-run protocol for each controller
