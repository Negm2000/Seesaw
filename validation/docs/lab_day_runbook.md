# Lab Day Runbook

## Non-Negotiable Rule

Do not debug the generated validation harness in the lab. If preflight fails, switch to the fallback hardware model and collect simpler data.

The generated `ControllerValidationHarness.slx` is a simulation/report harness. It proves controller logic and reference protocol behavior in simulation. It is not the lowest-risk QUARC deployment path.

## Tonight / Before Leaving

Prebuild the controller artifacts required by the course brief:

```matlab
startup
prebuild_required_controllers
```

This stages the controller/observer `.mat` files required by the Seesaw objectives from `docs/references/Requirements and Timeline.pdf` and compiles the generated harness cases.

Run this in MATLAB from the repository root:

```matlab
startup
preflight_lab_readiness('level','step')
```

This default uses the `lab_minimum` case profile: PP and SMC only. Those are the only generated cases that should be considered candidates for lab use right now.

Pass condition:

```text
PREFLIGHT PASS
```

If it fails, do not plan to use the generated harness tomorrow. Fix tonight or use fallback.

For a deeper check, run the full protocol. This can take longer:

```matlab
preflight_lab_readiness('level','full','case_profile','lab_minimum')
```

Do not use `case_profile='quick'` for lab readiness unless it passes. The quick profile includes LQI, LQG, and PID, which have already shown unsafe step-simulation behavior in the current generated harness.

## What The Preflight Proves

- MATLAB path and startup work.
- Required parameter/controller files exist.
- Reference protocol contains free-run, positive/negative theta steps, pulse, and sine segments.
- Generated harness compiles.
- `From`/`Goto` tags are local.
- PP and SMC lab-minimum cases simulate without NaN/Inf through the first step segment.
- Simulated `V_m` stays within the configured `+/-6 V` saturation.

## What The Preflight Does Not Prove

- QUARC driver availability.
- Encoder polarity on the lab rig.
- VoltPAQ wiring or gain switch position.
- Hardware build/download success.
- That PP/SMC are safe on the physical rig without first checking voltage behavior.

## Lab PC Preflight

On the lab PC, run:

```matlab
startup
preflight_lab_readiness('level','step','require_quarc',true)
```

If `quarc_callback` fails, do not waste time trying to use hardware models until QUARC is installed/configured.

## Primary Data Plan

Use the generated harness for simulation evidence and report plots. Use hardware only after the lab PC preflight passes and the hardware model builds cleanly.

Start with the safest controller:

```matlab
tune_validation_case
build_controller_validation_harness('controller_id','PP','feedback_source_id','measured')
sim('ControllerValidationHarness')
```

## Tuning In One Place

Use one file for all generated-harness tuning:

```text
validation/scripts/tune_validation_case.m
```

Typical emergency gain reduction:

```matlab
startup
tune_validation_case
validation_tuning.global.controller_gain_scale = 0.8;
build_controller_validation_harness('controller_id','PP','feedback_source_id','measured')
preflight_lab_readiness('level','step')
```

For SMC-only softening:

```matlab
tune_validation_case
validation_tuning.smc.k1_scale = 0.8;
validation_tuning.smc.k2_scale = 0.8;
validation_tuning.smc.phi_scale = 1.5;
build_controller_validation_harness('controller_id','SMC','feedback_source_id','measured')
preflight_lab_readiness('level','step')
```

Do not tune scattered block parameters manually. If a gain needs to change, change `validation_tuning` or edit `tune_validation_case.m`, rebuild, and rerun preflight.

Then run the quick simulation suite only if there is time:

```matlab
run_controller_validation_suite('quick',true)
analyze_controller_validation_suite
```

If LQI, LQG, or PID fail preflight, do not use them for hardware. Use PP first, then SMC only if PP data is already secured.

## Hardware Fallback Plan

If the generated validation path feels risky or fails on the lab PC, use the existing PP hardware model instead:

```text
models/controllers/pole_placement/PolePlacementObserver2024.slx
```

Collect at minimum:

- Free-run at `r_theta = 0`.
- One small positive theta/manual disturbance response if available.
- One small negative theta/manual disturbance response if available.
- Voltage command trace.
- Cart position and theta trace.

This is lower ambition but much easier to defend than collecting nothing.

## Hard Stop Criteria

Stop immediately if any of these happen:

- Voltage rails near `+/-22 V`.
- Sustained motor command above `6 V`.
- Cart approaches rail limits.
- Seesaw angle approaches physical stops.
- Oscillation grows instead of staying bounded.
- Encoder sign looks reversed.

## Minimum Reportable Dataset

If time is collapsing, prioritize this order:

1. PP free-run bounded behavior.
2. PP positive theta response.
3. PP negative theta response.
4. PP sine or chirp/frequency response.
5. Observer comparison.
6. Other controllers.

## If Something Fails

Do not investigate the whole generated model. Check only these in order:

1. Did `startup` complete?
2. Did `preflight_lab_readiness('level','smoke')` pass?
3. Is VoltPAQ at `1x` gain?
4. Is the model using `+/-6 V` saturation?
5. Are encoder signs correct?
6. If still failing, switch to `PolePlacementObserver2024.slx` fallback.
