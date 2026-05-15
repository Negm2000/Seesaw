# Fuzzy Scheduled State Feedback Simulink Deployment Plan

## Goal

Deploy the fuzzy scheduled state-feedback controller on the Quanser seesaw hardware without requiring Fuzzy Logic Toolbox on the lab machine.

The deployed law is:

```text
u = -K(x) x
```

with:

```text
x = [x_c, xdot_c, theta, theta_dot]^T
K(x) = w_small K_small + w_mid K_mid + w_large K_large
```

The scheduler keeps the controller gentle near balance and uses stronger feedback only when the seesaw is escaping.

## Why This Controller

The robust-controller benchmark showed:

| Controller | theta p2p | Lowest OK slew limit |
|---|---:|---:|
| Fuzzy scheduled SF | about 0.34 deg | 35 V/s |
| LMI-screened SF | about 0.76 deg | 50 V/s |
| Hinf / SMC | about 0.27 deg | 100 V/s |
| Pole placement | about 3.4 deg | 10 V/s |

The empirical pinion limit from sine tests was approximately:

| Test | Peak slew if 4 V is peak amplitude |
|---|---:|
| 4 V at 14 rad/s | 56 V/s |
| 4 V at 18 rad/s | 72 V/s |

Fuzzy scheduled SF is therefore the best candidate that keeps theta peak-to-peak low while staying close to the safe slew-rate region.

## Simulink Architecture

Use the existing observer-based seesaw model.

Signal path:

1. Encoder measurements: cart position `x_c` and seesaw angle `theta`.
2. Luenberger observer estimates `xdot_c` and `theta_dot`.
3. Assemble `x = [x_c; xdot_c_hat; theta; theta_dot_hat]`.
4. Fuzzy scheduler computes the blended gain `K(x)`.
5. State feedback computes `u_raw = -K(x) * x`.
6. Apply safety filters: voltage saturation, slew-rate limit, rail/angle guard.
7. Send final voltage command to QUARC motor output.

## Fuzzy Scheduler Logic

No Fuzzy Logic Toolbox is required for deployment if the scheduler is implemented as equations or lookup tables.

Inputs:

| Signal | Meaning |
|---|---|
| `abs(theta)` | tilt magnitude |
| `abs(theta_dot)` | rocking speed |
| `theta * theta_dot > 0` | true when moving away from level |

Rules:

| Condition | Behavior |
|---|---|
| Small `theta` and slow motion | use gentle gain |
| Medium `theta` | blend toward stronger gain |
| Large `theta` | use strong gain |
| Moving away from zero | bias toward stronger gain |

## MATLAB Function Block Skeleton

Use a MATLAB Function block with constants loaded from `data/robust_controller_comparison.mat` or copied into the model workspace.

```matlab
function u = fuzzy_scheduled_sf(x)
%#codegen
% x = [xc; xcdot; theta; thetadot]

th = x(3);
thd = x(4);

abs_th = abs(th);
abs_thd = abs(thd);
moving_away = th * thd > 0;

% Replace these with tuned values from robust_controller_comparison.mat.
theta_soft = deg2rad(0.35);
theta_mid  = deg2rad(1.0);
theta_full = deg2rad(2.5);
rate_break = deg2rad(25);

% K_small, K_mid, and K_large must be defined in the model workspace.
w_small = max(0, min(1, (theta_mid - abs_th) / (theta_mid - theta_soft)));
w_large = max(0, min(1, (abs_th - theta_mid) / (theta_full - theta_mid)));
w_mid = max(0, 1 - w_small - w_large);

if moving_away
    boost = max(0, min(1, abs_thd / rate_break));
    w_large = w_large + 0.35 * boost;
    w_small = w_small * (1 - 0.25 * boost);
end

s = max(w_small + w_mid + w_large, eps);
w_small = w_small / s;
w_mid = w_mid / s;
w_large = w_large / s;

K = w_small * K_small + w_mid * K_mid + w_large * K_large;
u = -K * x;
u = max(min(u, 6), -6);
end
```

## Safety Settings For First Hardware Test

Start conservative:

| Parameter | Initial value |
|---|---:|
| Voltage saturation | +/- 6 V |
| Slew-rate limit | 50 V/s, or 35 V/s if the pinion sounds stressed |
| Initial angle release | less than 1 deg |
| Stop test if `abs(theta)` | greater than 6 deg |
| Stop test if `abs(x_c)` | greater than 0.25 m |

Recommended first test sequence:

1. Start with the cart centered and the seesaw held nearly level.
2. Enable controller with voltage output disabled or limited, confirm signals are sane.
3. Enable voltage command with `50 V/s` slew limit.
4. Release from a very small angle, less than 1 deg.
5. Log `theta`, `x_c`, `V_m`, and estimated velocities.
6. Inspect theta p2p, voltage slew, and pinion sound/temperature.
7. Only then test a release up to 2 deg.

## Required Lab Toolboxes

Required for deployment:

| Dependency | Needed? |
|---|---:|
| MATLAB | yes |
| Simulink | yes |
| QUARC / Quanser real-time support | yes |

Not required if using the MATLAB Function implementation:

| Toolbox | Needed for deployment? |
|---|---:|
| Fuzzy Logic Toolbox | no |
| Robust Control Toolbox | no |
| Optimization Toolbox | no |

Those toolboxes are useful for design and tuning on the development PC, but the deployed scheduler can be ordinary arithmetic.

## Fallback Controllers

If fuzzy scheduled SF is unstable or mechanically harsh:

1. `Hinf actuator-aware`: smoother command behavior, about 0.38 deg p2p in simulation, but survived only around 100 V/s.
2. `LMI-screened SF`: lower slew than aggressive Hinf/SMC, about 0.76 deg p2p.
3. `Pole placement`: safest at low slew, but returns to the large about 3.4 deg limit cycle.

## Pre-Deployment Checklist

- Confirm observer initialization uses measured `x_c` and `theta` with zero initial velocities.
- Confirm sign convention: positive theta and positive motor voltage must match the simulation model.
- Confirm saturation is +/- 6 V, not VoltPAQ maximum voltage.
- Confirm slew limiter is after state feedback and before motor output.
- Confirm emergency stop is available and tested.
- Save every hardware run with timestamped logs for comparison against the simulation benchmark.
