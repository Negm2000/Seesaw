# Seesaw Hardware Validation

This context defines the experiment language for validating the Quanser IP02 + SEESAW-E closed-loop system against the simulation model.

## Language

**Closed-Loop Validation**:
An experiment that keeps the stabilizing controller active while comparing measured response against the closed-loop simulation model.
_Avoid_: Open-loop validation, raw plant validation

**Validation Excitation**:
A small commanded signal injected into the stabilized closed-loop system for validation.
_Avoid_: Open-loop input, raw motor command

**Reference Excitation**:
A validation excitation applied as a setpoint command rather than as an added voltage disturbance.
_Avoid_: Plant input

**Disturbance Injection**:
A validation excitation added inside the closed loop while the stabilizing reference remains fixed.
_Avoid_: Open-loop voltage step

**Control Effort**:
The motor voltage command produced by the controller during a closed-loop experiment.
_Avoid_: Input signal

**Measured Response**:
The logged hardware output used for comparison against the model response.
_Avoid_: Plant output when the controller is included

**Final Safety Saturation**:
The last voltage limit before the hardware actuator command.
_Avoid_: Optional limiter

**Single-Run Protocol**:
A concatenated validation experiment that collects baseline, time-response, and frequency-response data in one hardware run.
_Avoid_: Separate ad-hoc runs

**Zero-Mean Sine Segment**:
A sine validation window with no DC bias, so positive and negative actuator directions are exercised within each segment.
_Avoid_: Biased sine sweep

**Offset Reset**:
A pre-segment routine that recenters the cart and re-establishes the same zeroed initial condition before validation windows.
_Avoid_: Uncontrolled initial condition

**Mini-Liftoff**:
A small pre-segment motion used to break stiction before returning to the zeroed initial condition.
_Avoid_: Test excitation

**Observer Validation**:
A comparison of all available state estimates against measured encoder states and derived velocity references during the same hardware run.
_Avoid_: Validating only the feedback source

**Feedback Source**:
The single state signal path used by the controller to close the hardware loop during validation.
_Avoid_: All observers controlling at once

**Dirty-Derivative Feedback**:
The baseline feedback source that derives velocities from encoder position signals using a low-pass-filtered differentiator.
_Avoid_: Raw unfiltered derivative

**Validation Model**:
The ideal tuned linear closed-loop model used as the primary comparison target for hardware validation.
_Avoid_: Hardware-defect model

**Explanation Model**:
A secondary nonlinear model used to explain mismatch caused by dead-zone, Coulomb friction, and directional bias.
_Avoid_: Official Bode target

## Relationships

- **Closed-Loop Validation** applies a **Validation Excitation** and records the **Measured Response** and **Control Effort**.
- **Validation Excitation** may be a **Reference Excitation** or **Disturbance Injection**.
- **Control Effort** is not the excitation in closed-loop validation; it is a measured consequence of the controller.
- **Final Safety Saturation** bounds the sum of controller command and any **Disturbance Injection** before the actuator.
- **Single-Run Protocol** uses time windows to separate free-run baseline, step or pulse response, and steady sine segments.
- **Single-Run Protocol** uses **Zero-Mean Sine Segment** windows for frequency validation.
- **Offset Reset** and **Mini-Liftoff** prepare the initial condition but are excluded from validation analysis windows.
- **Observer Validation** may run all observers in parallel, but **Feedback Source** is singular for a given hardware run.
- **Dirty-Derivative Feedback** is the baseline **Feedback Source** for the validation run.
- **Validation Model** represents the nominal design expectation, while hardware data represents the deployed reality.
- **Explanation Model** accounts for hardware nonlinearities that are not part of the linear frequency-response comparison.

## Example dialogue

> **Dev:** "Do we step the motor voltage directly for validation?"
> **Domain expert:** "No - the stabilizing loop stays active; we inject a small disturbance inside the loop, keep the final safety saturation active, and log the measured response plus control effort."

## Flagged ambiguities

- "input" can mean reference command, injected disturbance, or motor voltage; resolved: use **Validation Excitation** for the commanded test signal and **Control Effort** for the resulting motor voltage.
