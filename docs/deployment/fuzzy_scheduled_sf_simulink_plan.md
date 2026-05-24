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

## Crash Course: Fuzzy Logic For This Controller

Fuzzy logic is a smooth `if/then` scheduler. Instead of switching abruptly between controllers, it assigns partial truth values to conditions such as "theta is small" or "theta is large" and blends the corresponding gains.

For this seesaw, fuzzy logic is used only to choose the feedback gain. The final controller is still ordinary state feedback:

```text
u = -K(x) x
```

The fuzzy part only computes `K(x)`.

### Step 1: Define The Inputs

Use the smallest set of inputs that explains when the controller should be gentle or aggressive.

Recommended inputs:

| Input | Why it matters |
|---|---|
| `abs(theta)` | tells how far from level the seesaw is |
| `abs(theta_dot)` | tells how fast the rocking motion is |
| `moving_away = theta * theta_dot > 0` | tells whether the angle is getting worse |

Do not start with too many fuzzy inputs. More inputs make the rule base harder to tune and harder to debug on hardware.

### Step 2: Define The Output

The output is not voltage directly. The output is a blend between three gains:

```text
K_small: gain used near balance
K_mid:   gain used for normal correction
K_large: gain used for recovery when escaping
```

The fuzzy scheduler returns weights:

```text
w_small, w_mid, w_large
```

Then:

```text
K = w_small*K_small + w_mid*K_mid + w_large*K_large
```

This is safer than fuzzy-direct-voltage control because each individual gain can be inspected and tested as a normal controller.

### Step 3: Understand Membership Functions

A membership function maps a number to a value between `0` and `1`.

Example for `abs(theta)`:

| Region | Meaning |
|---|---|
| `small` | close to balance; avoid pinion abuse |
| `medium` | normal correction |
| `large` | falling or close to leaving safe zone |

Selected simulation breakpoints were:

```text
theta_breaks_deg = [0.25, 0.80, 2.00]
rate_break_deg_s = 10
```

Interpretation:

| Breakpoint | Interpretation |
|---|---|
| `0.25 deg` | below this, treat the seesaw as nearly balanced |
| `0.80 deg` | transition region where stronger control starts to matter |
| `2.00 deg` | large error; recovery action is justified |
| `10 deg/s` | angular velocity where moving-away boost becomes important |

These are not final hardware constants. They are simulation-selected starting points.

### Step 4: Write The Rule Base

Start with simple rules:

| Rule | Meaning |
|---|---|
| If `theta` is small and not moving away, use mostly `K_small` |
| If `theta` is medium, use mostly `K_mid` |
| If `theta` is large, use mostly `K_large` |
| If `theta` is moving away quickly, boost toward `K_large` |

The key rule is the moving-away rule. It prevents the controller from being too gentle while the seesaw is actually falling.

### Step 5: Start With Safe Gains

The automatically selected gains were:

```text
K_small = [ 888.5, 70.0, -1307.6, -456.0 ]
K_mid   = [ 201.9, 16.0,  -312.9, -104.4 ]
K_large = [ 249.1, 20.8,  -217.7,  -69.5 ]
```

Warning: `K_small` is numerically aggressive. It worked in simulation because it broke through the modeled deadzone near zero angle. On hardware, this is the first gain to reduce if the pinion sounds stressed.

Recommended first hardware scaling:

```text
K_small_hw = 0.4 to 0.6 * K_small
K_mid_hw   = 0.7 to 0.9 * K_mid
K_large_hw = 1.0 * K_large
```

Why keep `K_large` strong? Because it is only used when the seesaw is escaping. Reducing it too much can make recovery fail.

### Step 6: Tune In Simulation First

Use `scripts/analysis/robust_controller_comparison.m` and change only one thing at a time.

Recommended simulation tuning order:

1. Tune `K_small` scale.
2. Tune `theta_breaks_deg(1:2)`.
3. Tune `rate_break_deg_s`.
4. Tune `K_mid` scale.
5. Tune `K_large` only if recovery is too weak or too violent.

Metrics to watch:

| Metric | Target |
|---|---|
| `theta_p2p_deg` | as low as possible |
| `u_slew_peak` | below empirical pinion-safe region |
| `u_rms` | avoid heating |
| `cart_p2p_mm` | avoid rail/pinion stress |
| saturation percent | near zero if possible |

### Step 7: Hardware Tuning Procedure

Use this exact order on the real setup.

#### Tutorial A: Sign And Sensor Check

Goal: verify that the controller would push in the correct direction before enabling strong voltage.

1. Disable motor output or set saturation to `+/- 0.5 V`.
2. Tilt the seesaw slightly positive.
3. Confirm measured `theta` is positive in Simulink.
4. Confirm estimated `theta_dot` has the expected sign when moving.
5. Confirm computed `u_raw` would drive the cart in the recovery direction.
6. If the sign is wrong, fix sign conventions before any balancing test.

Do not continue until this is correct.

#### Tutorial B: Gentle Balance Test

Goal: see if the low-angle fuzzy region is mechanically safe.

1. Use `K_small_hw = 0.4*K_small`.
2. Use `K_mid_hw = 0.7*K_mid`.
3. Use `K_large_hw = K_large`.
4. Set slew limit to `35 V/s`.
5. Set voltage saturation to `+/- 3 V` for the first attempt.
6. Release from less than `0.5 deg`.
7. If stable, increase saturation to `+/- 6 V`.

If it chatters or sounds harsh near zero, reduce `K_small_hw` or increase the `small` region width.

#### Tutorial C: Rocking Reduction Test

Goal: reduce peak-to-peak theta without increasing pinion stress too much.

1. Log at least 20 seconds of `theta`, `x_c`, `u`, and estimated velocities.
2. Compute steady-state theta p2p after the first 10-15 seconds.
3. If theta p2p is too large and the pinion sounds healthy, increase `K_small_hw` in steps of `0.1*K_small`.
4. If theta p2p is acceptable but voltage sounds sharp, lower slew limit first.
5. If the cart motion is large, reduce gains on `x_c` and `xdot_c` before reducing theta recovery gains.

Do not tune from a single release. Repeat each setting at least three times.

#### Tutorial D: Escape Recovery Test

Goal: check that the controller can recover from a larger but still safe angle.

1. Start from the best gentle-balance setting.
2. Release from `1 deg`.
3. If safe, try `1.5 deg`.
4. Do not jump directly to `2 deg` on hardware.
5. If it fails to recover, increase moving-away boost or `K_large` slightly.
6. If recovery is violent, increase `theta_full` so `K_large` engages later.

#### Tutorial E: Slew-Limit Sweep

Goal: find the lowest mechanically safe slew limit that still gives acceptable rocking.

Suggested test values:

```text
75 V/s -> 50 V/s -> 35 V/s -> 25 V/s
```

Stop reducing the limit when either:

- theta p2p increases sharply,
- the controller can no longer recover from a small release,
- cart travel grows too much.

Based on simulation, `35-50 V/s` is the expected useful region.

### Step 8: What To Tune For Specific Symptoms

| Symptom | Likely cause | First adjustment |
|---|---|---|
| Pinion clicks/chatter near zero | `K_small` too aggressive | reduce `K_small` scale |
| Large slow rocking | too gentle near zero | increase `K_small` or reduce deadzone region |
| Falls when released from larger angle | recovery too weak | increase moving-away boost or `K_large` |
| Violent recovery | `K_large` engages too early | increase `theta_full` or reduce `K_large` |
| Noisy switching between gains | velocity estimate noisy | increase `rate_break` or filter `theta_dot` |
| Cart travels too far | cart-position gains too weak/strong balance tradeoff | tune `x_c` and `xdot_c` entries in all gains |
| Voltage saturates often | gains too high or slew limit too loose | reduce gain scales, lower slew limit |

### Step 9: Do Not Trust These Until Verified

The simulation is useful for ranking controllers, but the following are uncertain on hardware:

- actual deadzone voltage,
- sign and size of actuator asymmetry,
- real Coulomb/static friction,
- backlash and pinion wear,
- observer velocity noise,
- delay from QUARC and Simulink blocks.

For that reason, treat the fuzzy design as a structured tuning starting point, not as a final guaranteed controller.

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

% Tuned starting values from robust_controller_comparison.mat.
deg = pi / 180;
theta_soft = 0.25 * deg;
theta_mid  = 0.80 * deg;
theta_full = 2.00 * deg;
rate_break = 10.0 * deg;

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
