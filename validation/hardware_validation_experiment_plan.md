# Hardware Validation And Controller Test Suite Plan

## Purpose

Validate the closed-loop Quanser IP02 + SEESAW-E controllers using the professor's standard requirement: inject test signals into the seesaw-angle reference command `r_theta(t)`, not into the motor-voltage disturbance path.

The validation target is closed-loop seesaw-angle tracking performance:

```text
r_theta(t) -> theta(t)
```

This corresponds to the complementary sensitivity response `T(s)`. The required report evidence is therefore:

- Free-run validation: hold `r_theta = 0`, record the bounded closed-loop behavior, and quantify the natural limit-cycle envelope for each controller.
- Time validation: command seesaw-angle reference steps and pulse/impulse-like references, record `theta(t)` and `u(t)`, and compare against the model step/pulse response.
- Frequency validation: command sinusoidal seesaw-angle references, record sinusoidal output responses, and compare measured gain/phase against the model Bode response from `r_theta` to `theta`.
- Observer validation: compare dirty derivative, Luenberger, and Kalman estimates on the same trajectories.

Voltage-disturbance injection is not the primary validation method. It can remain as an optional diagnostic, but it should not be the main report experiment.

## Core Decisions

- Validate reference tracking, not voltage-disturbance rejection.
- The primary validation input is `r_theta(t)`.
- The primary measured output is `theta(t)`.
- Every controller test includes a free-run segment at `r_theta = 0` before forced reference steps and sines.
- Positive and negative theta step responses are mandatory time-domain validation outputs.
- Frequency validation uses a theta-reference stepped sine sweep to estimate `r_theta -> theta` gain and phase.
- The controller remains closed-loop and active throughout the test.
- Keep dead-zone compensation enabled, because this is part of the deployable controller behavior.
- Keep the final motor-voltage safety saturation active at `+/-6 V`.
- Use the same reference protocol, logging schema, and analysis code for every controller.
- Use one canonical validation harness instead of creating separate test models for PP, PID, SMC, LQR, etc.
- Existing controller models are reference sources for logic and parameters, not the long-term testing architecture.

## Reference Channel

The primary validation channel is fixed to seesaw-angle tracking:

```text
r_theta(t) -> theta(t)
```

Cart position `x_c` is still logged and used as a safety/performance output, but it is not a commanded validation reference. The cart moves as much as needed to track and balance the seesaw angle.

There is no external cart-position reference in the validation protocol. If a controller internally computes a cart target as part of its cascade logic, that target is an internal controller signal, not a report-level command.

This removes ambiguity: the report's `T(s)` channel is:

```text
T_theta(s) = Theta(s) / R_theta(s)
```

Supporting plots may include `r_theta -> x_c` and `r_theta -> u_motor` to explain how much cart motion and actuator effort were required to track the seesaw angle.

## Reference Amplitudes

The previous `1 V` disturbance amplitude is obsolete for the primary validation. Reference amplitudes must be in output units.

Initial simulation amplitudes:

```text
theta_ref_step_amp = deg2rad(1.0)
theta_ref_sine_amp = deg2rad(1.0)
```

These are starting values for simulation. Before hardware, verify that each controller remains unsaturated and does not drive the cart toward rail limits.

If a reference amplitude produces saturation, reduce the amplitude or mark the segment invalid for ideal-model comparison.

## Canonical Harness

Create one canonical validation model:

```text
validation/models/ControllerValidationHarness.slx
```

Build or maintain it from scripts so the structure stays repeatable:

```text
validation/scripts/build_controller_validation_harness.m
```

The harness should have this top-level structure:

```text
ProtocolSource
PlantInterface
StateEstimation
ReferenceBuilder
ControllerBank
ControllerSelector
ActuatorInterface
SafetyAndManualGate
Logger
```

Do not create controller-specific harnesses such as:

```text
PP_HWTest.slx
PID_HWTest.slx
SMC_HWTest.slx
LQI_HWTest.slx
```

That model sprawl is the failure mode we are explicitly avoiding.

## ProtocolSource

Replace the old `d_free`, `d_step`, `d_prbs`, and `d_chirp` workflow with one canonical reference-tracking protocol.

Generate it from:

```text
validation/scripts/generate_validation_protocol.m
```

The generated MAT file should contain:

```text
t_protocol
r_theta_ts
segment_id_ts
segment_frequency_hz
analyze_window
```

Suggested output file:

```text
validation/data/protocol/reference_tracking_protocol.mat
```

The harness reads `r_theta_ts` and `segment_id_ts` from the workspace or MAT file.

If a subsystem still expects `r_x`, the harness supplies `r_x = 0` and lets that subsystem compute any internal cart target from `r_theta`.

## Reference Tracking Protocol

The single-run reference protocol mirrors the earlier validation structure but moves the excitation from voltage disturbance to reference command.

Recommended sequence:

```text
prep/reset        offset reset + mini-liftoff, excluded from analysis
free-run          r_theta = 0, measured baseline limit-cycle behavior
+step             r_theta = +theta_ref_step_amp, finite duration
recovery          r = 0
-step             r_theta = -theta_ref_step_amp, finite duration
recovery          r = 0
pulse             short +theta_ref_step_amp pulse, impulse surrogate
recovery          r = 0
stepped sine sweep  zero-mean theta-reference sine segments
final recovery    r = 0 for 10 s
```

Recommended time block before sines:

```text
free-run          10 s
+theta step       15 s
recovery          15 s
-theta step       15 s
recovery          15 s
+theta pulse      0.3 s
recovery          10 s
```

Theta-reference stepped sine sweep frequencies:

```text
0.10, 0.16, 0.25, 0.40, 0.63, 1.00,
1.60, 2.50, 4.00, 6.30, 8.00, 10.00 Hz
```

Each sine-sweep segment uses a consistent 7-cycle structure:

```text
cycle 1      ramp in
cycles 2-3   discard / settle
cycles 4-6   analyze
cycle 7      ramp out
```

Use a smooth half-cosine ramp to avoid hard-switching transients.

Do not use PRBS as the primary frequency-validation input. PRBS can be a secondary diagnostic, but the report explicitly needs sine-wave tracking response. The primary frequency experiment is therefore a stepped sine sweep, not PRBS and not a broadband voltage disturbance.

## Segment IDs

Log `segment_id` directly. Do not rely only on timestamps.

Suggested labels:

```text
0   prep/reset, excluded
1   free-run baseline
2   +theta reference step
3   recovery
4   -theta reference step
5   recovery
6   +theta reference pulse
7   recovery
10  sine 0.10 Hz
11  sine 0.16 Hz
12  sine 0.25 Hz
13  sine 0.40 Hz
14  sine 0.63 Hz
15  sine 1.00 Hz
16  sine 1.60 Hz
17  sine 2.50 Hz
18  sine 4.00 Hz
19  sine 6.30 Hz
20  sine 8.00 Hz
21  sine 10.00 Hz
30  final recovery
```

## ControllerBank

The current generic validation model is too state-feedback-specific. It can support PP/LQR/LQI gain slots, but it does not truly support PID or SMC.

The clean solution is a real controller bank.

Each controller subsystem exposes the same external interface:

```text
inputs:
  x_feedback        [x_c; x_c_dot; theta; theta_dot]
  r_theta
  r_state
  integral_error
  controller_enable

output:
  u_controller
```

Controllers in the bank:

```text
Controller_PP
Controller_PP_Integral
Controller_LQR
Controller_LQI
Controller_LQG
Controller_PID_Cascade
Controller_SMC_STA
```

The `ControllerSelector` selects exactly one `u_controller` for a test case.

Do not force PID or SMC into a fake `K_hw` gain-slot interface.

## Controller Implementations

### PP

Basic state feedback:

```text
u = u_ss(r) - K_pp * (x_feedback - x_ss(r))
```

For tracking without integral action, compute the steady-state feedforward from the tuned linear model for the seesaw-angle output channel.

For `y = theta`, solve:

```text
[A  B] [x_ss] = [0]
[C  0] [u_ss]   [r]
```

Then apply the feedback to `x_feedback - x_ss`.

### PP With Integral

Integrate the selected tracking error:

```text
e_theta = r_theta - theta
z_dot = e_theta
u = -K_aug * [x_feedback; z]
```

This should be tested separately from plain PP because it answers the unresolved integral-action question.

### LQR / LQI / LQG

Use the same tracking wrapper as PP for plain LQR.

For LQI, use the augmented integral state:

```text
e_theta = r_theta - theta
z_dot = e_theta
u = -K_lqi * [x_feedback; z]
```

For LQG, use the Kalman estimate as the feedback state:

```text
x_feedback = x_hat_kalman
```

LQG should be treated as a controller/observer pairing, not just another gain.

### PID

Use a real cascade PID subsystem copied or reconstructed from `models/pid/PID_seesaw.slx`.

External interface stays common:

```text
r_theta, theta, x_c -> u_controller
```

The PID subsystem may internally compute a cart-position target from angle error and then motor voltage from cart-position error.

Test at least one PID variant with integral action. If time allows, also test the no-integral or PD-like variant.

### SMC

Use the deployed super-twisting SMC logic from `models/controllers/smc/SMC_STA_HW_2_r2024b.slx`.

The SMC subsystem must be a real MATLAB Function / subsystem implementation, not a gain-slot approximation.

For tracking, operate on tracking error coordinates where possible:

```text
x_error = x_feedback - x_ref
s = S * x_error
```

The reference builder must provide a consistent `x_ref` for sine tracking:

```text
x_ref = [x_c_ref; x_c_ref_dot; theta_ref; theta_ref_dot]
```

For step tracking, use the reference value and set unavailable derivatives to zero or use a smoothed reference step if derivative discontinuity causes unrealistic SMC spikes.

## ReferenceBuilder

The `ReferenceBuilder` converts `r_theta` into the reference information each controller needs.

Outputs:

```text
y_ref
x_ref
u_ss
x_ss
tracking_error
integral_error
```

For pure output tracking, `y_ref = r_theta` is enough.

For state-feedback tracking, compute `x_ss` and `u_ss` from the tuned linear model.

For sine tracking, also provide reference derivatives when useful:

```text
r_dot
r_ddot
```

This keeps reference handling out of individual controller blocks.

## StateEstimation

Always compute all feedback estimates in parallel:

```text
x_dirty       dirty derivative + low-pass filter
x_luenberger  Luenberger observer
x_kalman      Kalman observer
```

One selector chooses the active feedback state:

```text
feedback_source_id:
  1 dirty derivative + LPF
  2 Luenberger
  3 Kalman
```

All observers are logged even when they are not the active feedback source.

This lets the observer comparison use the exact same trajectory.

## ActuatorInterface

The actuator path should be shared by every controller:

```text
u_controller
  -> dead-zone compensation / nonlinear motor command block
  -> final +/-6 V saturation
  -> manual ON/OFF motor gate
  -> motor command / plant input
```

The non-linear dead-zone block from `SS_tracking.slx` belongs here as a standard harness block, not copied randomly into every controller model.

Log every actuator stage:

```text
u_controller
u_compensated
u_motor
saturation_active
manual_on
```

## PlantInterface

The same harness should eventually support three plant modes:

```text
plant_mode = linear_sim
plant_mode = nonlinear_sim
plant_mode = hardware
```

For the Simulink testing suite, start with simulation modes.

Hardware mode should reuse the same controller bank and logger later.

## Test Matrix

Do not test every possible combination first. Start with a defensible matrix.

Full matrix candidate:

```text
PP_dirty
PP_luenberger
PP_kalman

PP_integral_dirty
PP_integral_luenberger
PP_integral_kalman

LQR_dirty
LQI_dirty
LQG_kalman

PID_dirty
PID_luenberger
PID_kalman

SMC_dirty
SMC_luenberger
SMC_kalman
```

Reduced first-pass matrix:

```text
PP_dirty
PP_luenberger
PP_kalman
LQI_dirty
LQG_kalman
PID_dirty
PID_kalman
SMC_dirty
SMC_kalman
```

This covers:

- PP baseline.
- PP observer sensitivity.
- LQI/LQG optimal-control family.
- PID family.
- SMC family.
- Luenberger and Kalman observer evaluation.

## Simulation Runner

Create one runner:

```text
validation/scripts/run_controller_validation_suite.m
```

Each test case should be a config row:

```text
controller_id
controller_variant
feedback_source_id
plant_mode
deadzone_comp_enabled
protocol_file
result_file
```

Use `Simulink.SimulationInput` for each run and save one result file per test case:

```text
validation/data/sim_results/PP_dirty_theta.mat
validation/data/sim_results/LQG_kalman_theta.mat
validation/data/sim_results/PID_dirty_theta.mat
validation/data/sim_results/SMC_kalman_theta.mat
```

## Logging Schema

Every controller must produce the same logged signal schema.

Required signals:

```text
time
segment_id
r_theta
theta_ref
theta_measured
theta_tracking_error
x_c
theta
x_dirty
x_luenberger
x_kalman
x_feedback
u_controller
u_compensated
u_motor
saturation_active
manual_on
controller_id
feedback_source_id
plant_mode
```

Optional but useful:

```text
x_ref
x_ss
u_ss
integral_error
SMC_sliding_variable
PID_outer_output
PID_inner_error
observer_innovation
```

## Analysis

Create one analyzer:

```text
validation/scripts/analyze_controller_validation_suite.m
```

For every test case, compute the same metrics.

Time-domain tracking metrics:

- Rise time.
- Peak response.
- Overshoot.
- Settling or recovery time.
- Steady tracking error.
- RMS tracking error.
- Maximum cart travel.
- Maximum and RMS voltage.
- Saturation count.

Frequency-domain tracking metrics:

- Fit `r_theta -> theta` gain and phase at every sine frequency.
- Plot measured Bode points over the ideal tuned linear model `T(s)`.
- Fit `r_theta -> u_motor` gain if actuator demand needs explanation.
- Report sine-fit residuals.

Observer metrics:

- Position estimate RMS error.
- Velocity estimate RMS difference against filtered derivative reference.
- Bias.
- Phase lag during sine segments.
- Effect on controller voltage and tracking error when used as active feedback.

Ranking metrics:

- Time tracking score.
- Frequency tracking bandwidth.
- Voltage effort.
- Saturation robustness.
- Observer sensitivity.
- Cart travel safety margin.

## Model Comparison

The report comparison is:

```text
ideal tuned linear closed-loop tracking model vs real/simulated controller response
```

For Bode plots, use the tracking transfer function:

```text
T_theta(s) = Theta(s) / R_theta(s)
```

For state-feedback controllers with explicit reference feedforward, derive `T(s)` from the closed-loop model.

For nonlinear controllers such as SMC, use sine-fit simulation results as the model-side comparison if an analytical `T(s)` is not meaningful.

## Required Report Figures And Tables

Time validation figures:

- Full-run overview: `r_theta(t)`, `theta(t)`, `x_c(t)`, and `u_motor(t)` with segment labels.
- Free-run response at `r_theta = 0`, including theta limit-cycle envelope and voltage effort.
- `+theta` step response measured/simulated vs ideal tracking model.
- `-theta` step response measured/simulated vs ideal tracking model.
- Short theta-reference pulse response as impulse surrogate.
- Controller comparison overlay for the same reference step.

Frequency validation figures:

- Bode plot of measured/simulated sine-fit points for `r_theta -> theta` over ideal `T_theta(s)`.
- Optional Bode or gain plot of `r_theta -> u_motor` to show actuator demand.
- Controller comparison Bode plot for PP, LQI/LQG, PID, and SMC.

Observer validation figures:

- Measured `x_c` and `theta` vs observer estimates.
- Dirty derivative vs Luenberger vs Kalman velocity estimates.
- Observer error table.

Controller comparison table:

- Rise time.
- Overshoot.
- Settling/recovery time.
- RMS tracking error.
- Frequency bandwidth.
- Peak voltage.
- Voltage RMS.
- Saturation count.
- Maximum cart travel.

## Organization

Keep the validation work organized like this:

```text
validation/
  models/
    ControllerValidationHarness.slx

  scripts/
    build_controller_validation_harness.m
    generate_validation_protocol.m
    load_controller_validation_case.m
    run_controller_validation_suite.m
    analyze_controller_validation_suite.m

  data/
    protocol/
      reference_tracking_protocol.mat
    sim_results/
      PP_dirty_theta.mat
      LQG_kalman_theta.mat
      PID_dirty_theta.mat
      SMC_kalman_theta.mat

  docs/
    controller_validation_suite_plan.md
```

## What To Reuse

Reuse existing work deliberately:

- `HardwareValidation_HWTest_2024_Proper.slx`: generic harness concepts, ON/OFF gate, observer selector, logging, saturation.
- `SS_tracking.slx`: non-linear dead-zone / motor command block.
- `PID_seesaw.slx`: actual cascade PID logic.
- `SMC_STA_HW_2_r2024b.slx`: actual super-twisting SMC MATLAB Function and ON/OFF pattern.
- `PolePlacementObserver2024.slx`: PP and observer wiring reference.
- `lqr_design.m`: LQR/LQI/Kalman gains.
- `luenberger_observer_design.m`: Luenberger matrices.

Do not keep editing all of those models independently.

## Implementation Order

1. Generate the reference-tracking protocol.
2. Build the simulation-only harness with PP and LQI first.
3. Add the shared actuator path: dead-zone compensation, final saturation, manual ON/OFF.
4. Add the observer bank: dirty derivative, Luenberger, Kalman.
5. Add PID cascade as a real subsystem.
6. Add SMC STA as a real subsystem.
7. Add controller selector and config registry.
8. Add batch simulation runner.
9. Add analysis and ranking scripts.
10. Only after simulation results are clean, prepare hardware mode.

## Report Language

Use this framing:

```text
The controllers were validated in closed loop by injecting the required test signals into the seesaw-angle reference command r_theta(t). This evaluates closed-loop tracking performance, represented by the complementary sensitivity function T_theta(s)=Theta(s)/R_theta(s). Each controller was first measured in free-run at r_theta=0 to quantify the bounded limit-cycle envelope. Time-domain validation used finite positive and negative theta-reference steps and a short theta-reference pulse. Frequency-domain validation used zero-mean sinusoidal theta-reference commands over the selected frequency range. The measured/simulated output response theta(t) and control effort u(t) were compared against the ideal tuned linear closed-loop tracking model, while deviations were interpreted in terms of actuator saturation, dead-zone compensation, friction, directional asymmetry, and observer limitations.
```

For observers:

```text
Dirty derivative, Luenberger, and Kalman observers were evaluated on the same reference-tracking trajectories. Only one feedback source was active for a given controller run, but all observer estimates were logged in parallel for fair comparison.
```
