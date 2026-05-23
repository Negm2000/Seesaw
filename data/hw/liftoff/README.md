# Lift-off Hardware Data (19 May 2026 session)

A single afternoon of hardware testing on the IP02 + SEESAW-E rig, focused
on the lift-off supervisor that catches the seesaw from a resting stop and
hands off to the pole-placement balance controller.

Each `.mat` file stores a single 4 x N array `data = [t; x_c; alpha; V_m]`
sampled at `Ts = 2 ms`:

| Row | Signal | Unit |
|-----|--------|------|
| 1 | `t` (time) | s |
| 2 | `x_c` (cart position) | m |
| 3 | `alpha` (seesaw angle) | rad |
| 4 | `V_m` (motor voltage, after saturation) | V |

## successful/

Runs where the controller caught the seesaw upright and held it for at
least 5 s.

| File | Run | What's interesting |
|------|-----|--------------------|
| `14-52-39_lyap_supervisor_manual_cal.mat` | manual cal + Lyapunov supervisor, beta = 0.30 | **The run written up in Experience 3 of the thesis.** Four caught episodes; only the first ($t \approx 28\text{-}60$ s) actually exercises $\mathbf{K}_{lift}$ because the catch latch is one-way. |
| `15-02-32_manual_cal_balance.mat` | manual cal, balance only | Started with $\alpha \approx 0$ (held by hand), not a cold lift-off from the stop. Useful as a "balance baseline" but not a supervisor test. |
| `15-26-25_offset_zero_bias.mat` | direct start + offset, integrator-enabled variant | Steady $\bar\alpha = +0.10^\circ$ -- zero bias. Long pre-catch interval (~272 s) suggests several failed attempts before one stuck. |
| `15-37-22_offset_zero_bias.mat` | direct start + offset, integrator-enabled variant | Same family as 15-26-25. $\bar\alpha = +0.10^\circ$. |
| `16-38-41_quick_catch.mat` | direct start + offset, aggressive initial command | $V_m(0) = -6.22$ V -- the controller committed immediately. Tightest LC (1.5$^\circ$ peak), lowest $V_{rms}$ (1.63 V). |
| `16-40-54_quick_catch.mat` | direct start + offset, aggressive initial command | Same family as 16-38-41. |

## failed/

Runs that did not produce a sustained catch, grouped by what went wrong.

### `failed/over_swung/`

Lift-off authority was too high. $V_m$ saturated at $-7$ V for an extended
period, the seesaw blew straight through upright, and landed on the opposite
stop. Symptom of "aggressive gain without a catch handoff" -- the source of
the pinion-slip problem on the rack.

### `failed/never_lifted/`

The controller commanded voltage but the seesaw stayed on the stop. Static
friction at the pivot + Coulomb friction in the cart drive exceeded what
the commanded voltage could deliver. The "weak gain" failure mode.

### `failed/lifted_fell_back/`

The seesaw rotated off the stop but the controller could not arrest it
near upright; it returned to the same stop. Several zero crossings, eventually
parks back on the original side.

### `failed/transient_cut/`

Run was aborted by the operator while still in transit -- in both cases the
seesaw was approaching upright but the test was cut before any sustained
catch could be evaluated.

## Deleted

Four files were removed from the original session because they were empty
or shorter than three seconds (operator aborted before anything useful was
recorded):

- `14-47-22.mat` (empty)
- `15-06-09.mat` (2.8 s)
- `15-57-29.mat` (5.7 s, never lifted)
- `16-40-22.mat` (3.4 s, never lifted)

## Calibration procedures used in the session

Two procedures were used to set the encoder zero of $\alpha$ before each
run, and they show up clearly in the data:

1. **Manual calibration** (files `14-52-39`, `15-02-32`): model starts with
   $V_m = 0$, operator holds the seesaw at $\alpha \approx 0$ by hand to
   zero the encoder, then releases it onto the stop while the controller
   stays disabled. Logged $\alpha(0) \approx 0$.

2. **Direct start + software offset** (files `15-26-25` and later): the
   seesaw is already on the stop when logging begins, and a software offset
   is added in the controller path to compensate. Logged $\alpha(0) = -11.66^\circ$.

See thesis Experience 3 (Section "Lift-off supervisor and catch from rest")
for the analysis of `14-52-39_lyap_supervisor_manual_cal.mat`.
