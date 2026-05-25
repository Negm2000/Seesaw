# Required Controller Prebuild

Source: `docs/references/Requirements and Timeline.pdf`, Seesaw slides 41-42.

## Extracted Seesaw Requirements

- Cart position control: frequency-based control (`FB`).
- Body horizontal stabilization: frequency-based and state-space control (`FB & SS`).
- Periodic body trajectory tracking and lift-up: state-space and advanced control (`SS & AC`).

## One Command

```matlab
startup
prebuild_required_controllers
```

Outputs:

- `validation/data/prebuilt/required_controllers.mat`
- `validation/data/prebuilt/prebuild_report_*.mat`

## Artifact Mapping

- Cart FB: `data/controllers/controller_inner_pid.mat`
- Body FB PID/lead: `data/controllers/controller_outer_pid.mat`
- Body FB pole placement: `data/controllers/controller_freq.mat`
- Body SS LQR/LQI: `data/controllers/controller_lqr.mat`
- Body SS Luenberger observer: `data/params/observer.mat`
- Body SS Kalman observer: `data/params/observer_kalman.mat`
- Body AC SMC: `data/controllers/controller_smc.mat`
- Optional MPC: `data/controllers/controller_mpc.mat`
- Optional lift-up: `data/params/liftoff_params.mat`

## Lab Safety Status

Prebuild means the required artifacts exist and generated harness cases compile.

It does not mean every controller is lab-safe. Current lab-minimum preflight only trusts:

- `PP_measured`
- `SMC_measured`

Run before lab use:

```matlab
preflight_lab_readiness('level','step')
```
