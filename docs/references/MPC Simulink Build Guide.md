# Model Predictive Controller -- Manual Simulink Build Guide

This guide describes how to build the MPC controller in Simulink using a **MATLAB Function block** (no MPC Toolbox required). The observer from Ch.\ 2 is reused -- the MPC replaces only the state-feedback gain block.

## 1. Prerequisites

Run these scripts first to populate the workspace with data:

```matlab
run('scripts/config/seesaw_params.m')
load('data/tuned_params.mat')
load('data/observer.mat')
load('data/controller_mpc.mat')    % produced by scripts/control/mpc_design.m
```

Verify the following variables exist:

| Variable    | Size     | Description                        |
|-------------|----------|------------------------------------|
| `A_obs`     | 4 x 4    | Observer dynamics (A - LC)         |
| `B_obs`     | 4 x 3    | Observer input [B_sw, L]           |
| `C_obs`     | 4 x 4    | Observer output (identity)         |
| `D_obs`     | 4 x 3    | Observer feedthrough (zeros)       |
| `H`         | N x N    | QP Hessian (N = prediction horizon)|
| `f`         | 4 x N    | Linear part: f*x0                  |
| `u_max`     | scalar   | Voltage limit = 6.0 V              |
| `u_min`     | scalar   | Voltage limit = -6.0 V             |
| `V_sat`     | scalar   | Voltage saturation = 6.0 V         |
| `K_a`       | scalar   | Amplifier gain = 1                 |
| `K_E_SW`    | scalar   | Seesaw encoder resolution          |
| `K_gs`      | scalar   | Seesaw gear ratio                  |
| `K_ec`      | scalar   | Cart encoder resolution            |

## 2. Block Diagram Overview

```
[QUARC HIL Cards]                     [Observer]                    [MPC QP Solver]
   encoders    ----> [x_c, alpha] ----> State-Space ----> xhat ----> MATLAB Function ----> u ----> [Saturation] ----> [DAC]
                                         (A_obs,B_obs,                                    Block (quadprog)          (+/- 6 V)
                                          C_obs,D_obs)
```

The same architecture as the pole-placement model (`models/PolePlacementObserver2024.slx`), but the `K*u` gain block is replaced with a **MATLAB Function block** that solves the constrained QP at each time step.

## 3. Step-by-Step Build

### 3.1 Start from the working model

Copy `models/PolePlacementObserver2024.slx` to `models/MPC_Observer2024.slx`.

### 3.2 Remove the pole-placement gain

Delete the **Gain** block that multiplies `-Kf` with the observer output `xhat`. Keep the **Mux** that combines the four state estimates into a vector.

### 3.3 Add the MPC MATLAB Function block

1. Drag a **MATLAB Function** block from `User-Defined Functions` library into the model.
2. Double-click it and replace the default code with the MPC solver (see Section 4).
3. Rename the block to `MPC QP Solver`.
4. Connect the `xhat` signal (4-element vector) to the block input.
5. Connect the block output (scalar `u`) to the **Saturation** block that enforces `+/-V_sat`.

### 3.4 Configure the block

Open the MATLAB Function block editor. Under the **Editor** tab:

- Click **Edit Data**.
- Set input `x`: Size `4`, Type `Inherit: Same as Simulink`.
- Set output `u`: Size `1`, Type `Inherit: Same as Simulink`.
- Ensure the block's **Sample time** is set to `-1` (inherited).

### 3.5 Solver and code generation settings

The block calls `quadprog` internally. This requires:

1. **Solver**: Set to `Fixed-step`, step size = `0.001` (1 kHz). This matches the controller sample frequency.
2. **Code generation**: This block **cannot generate code** for QUARC because `quadprog` is not supported by the MATLAB Coder. For simulation only, set **Simulation range** appropriately.
3. For **hardware deployment**, a custom online QP solver or explicit MPC look-up table must replace `quadprog` (see Section 6).

### 3.6 Saturation block

The MPC block has internal constraint handling, but a hardware safety saturation block **must** remain between the MPC output and the DAC to guard against solver numerical errors. Set limits to `+/- V_sat`.

## 4. MATLAB Function Block Code

Replace the default code inside the MATLAB Function block with:

```matlab
function u = mpc_solve(x)
% MPC_SOLVE  Compute the first control move of a condensed-form
%            constrained MPC at a single time step.
%
%   x : current 4-state estimate [x_c; x_c_dot; alpha; alpha_dot]
%   u : optimal motor voltage (clipped to +/- V_sat as backup)

%#codegen

persistent H f N_ u_max u_min
if isempty(H)
    % Load precomputed MPC data from workspace
    data   = load('controller_mpc.mat');
    H      = data.H;
    f      = data.f;
    N_     = data.N;
    u_max  = data.u_max;
    u_min  = data.u_min;
end

% Build the linear part of the cost gradient:  g = f * x
g = (x' * f')';

% Solve the constrained QP:
%   min_U  0.5*U'*H*U + g'*U
%   s.t.   u_min <= U_i <= u_max
opts = optimoptions('quadprog', 'Display', 'off', ...
    'Algorithm', 'interior-point-convex');
[U_opt, ~, exitflag] = quadprog(H, g, [], [], [], [], ...
    u_min*ones(N_,1), u_max*ones(N_,1), [], opts);

% Receding horizon: apply first move only
if exitflag >= 0 && ~isempty(U_opt)
    u = U_opt(1);
else
    % Fallback: zero voltage (safe for seesaw -- it returns to rest)
    u = 0;
end
end
```

**Important note**: The `load('controller_mpc.mat')` call inside a persistent block works in normal simulation but **will not work** with code generation (QUARC). For code generation, precompute `H`, `f`, etc., as constant parameters in the base workspace and pass them as block **parameters** (tunable), not workspace loads. See Section 5.

## 5. QUARC / Hardware Deployment (Explicit MPC)

Since `quadprog` is not compatible with MATLAB Coder / QUARC, the hardware implementation requires **explicit MPC**: pre-solve the QP offline for a grid of states, then use a look-up table at runtime.

### 5.1 Generate explicit MPC controller

Run this in MATLAB (uses the same `controller_mpc.mat` data):

```matlab
load('data/controller_mpc.mat');

% Define a grid over the 4-D state space
% Focus on the operating region: small cart displacements, small angles
x_c_lim      = 0.1;               % +/- 10 cm
x_c_dot_lim  = 0.2;               % +/- 0.2 m/s
alpha_lim    = deg2rad(5);        % +/- 5 deg
alpha_dot_lim = 0.5;              % +/- 0.5 rad/s

n_grid = 15;   % points per dimension (15^4 = 50625 -- manageable)

[xc, xcd, al, ald] = ndgrid( ...
    linspace(-x_c_lim, x_c_lim, n_grid), ...
    linspace(-x_c_dot_lim, x_c_dot_lim, n_grid), ...
    linspace(-alpha_lim, alpha_lim, n_grid), ...
    linspace(-alpha_dot_lim, alpha_dot_lim, n_grid));

N_total = numel(xc);
U_table = zeros(n_grid, n_grid, n_grid, n_grid);

opts = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
lb_vec = u_min * ones(N, 1);
ub_vec = u_max * ones(N, 1);

wb = waitbar(0, 'Generating explicit MPC...');
for i = 1:N_total
    xi = [xc(i); xcd(i); al(i); ald(i)];
    g = (xi' * f')';
    [U_opt, ~] = quadprog(H, g, [], [], [], [], lb_vec, ub_vec, [], opts);
    if ~isempty(U_opt)
        U_table(i) = U_opt(1);
    end
    if mod(i, 1000) == 0
        waitbar(i/N_total, wb);
    end
end
close(wb);

% Save the lookup table and grid vectors
x_c_grid      = linspace(-x_c_lim, x_c_lim, n_grid);
x_c_dot_grid  = linspace(-x_c_dot_lim, x_c_dot_lim, n_grid);
alpha_grid    = linspace(-alpha_lim, alpha_lim, n_grid);
alpha_dot_grid = linspace(-alpha_dot_lim, alpha_dot_lim, n_grid);

save('data/controller_mpc_explicit.mat', ...
     'U_table', 'x_c_grid', 'x_c_dot_grid', 'alpha_grid', 'alpha_dot_grid');
```

### 5.2 Explicit MPC Simulink block

Replace the MATLAB Function block code with an **n-D Lookup Table** block:

1. Drag an **n-D Lookup Table** from `Lookup Tables` library.
2. Set **Number of table dimensions** to `4`.
3. Configure the table data: `U_table`
4. Configure breakpoints:
   - Breakpoints 1: `x_c_grid`
   - Breakpoints 2: `x_c_dot_grid`
   - Breakpoints 3: `alpha_grid`
   - Breakpoints 4: `alpha_dot_grid`
5. Set **Interpolation method** to `Linear` and **Extrapolation method** to `Clip`.
6. Connect the four state components (demuxed from `xhat`) to the four inputs.
7. The table output is the MPC control voltage `u`.

> **Caution**: Extrapolation uses the nearest boundary value ("Clip"). Outside the grid the controller degrades to the boundary solution, which may not stabilise. Ensure the grid covers all expected operating points.

### 5.3 Warnings

- **Grid explosion**: 15^4 = 50,625 points is the practical limit for a 4-D table in Simulink. Higher grid resolutions cause memory issues.
- **Coverage**: The explicit MPC controller is undefined outside the lookup grid. The seesaw can exceed these bounds during disturbances -- the fallback must handle this (saturation block is the last line of defence).
- **Integral action**: If using the augmented 5-state model, the explicit MPC becomes 5-D (15^5 = 759,375 points -- too large). Consider a separate integral controller connected in parallel, or reduce the grid resolution.
- **Limit cycle**: The explicit controller inherits the same bounded oscillation as the pole-placement controller. The lookup table does not eliminate Coulomb friction effects.

## 6. Solver Configuration (Simulink)

| Parameter                      | Value                          |
|--------------------------------|--------------------------------|
| **Solver type**                | Fixed-step                     |
| **Solver**                     | `ode4` (Runge-Kutta) or `ode1` (Euler) |
| **Fixed-step size**            | `0.001` (1 kHz)                |
| **Stop time**                  | `inf` (hardware) / `5` (sim)   |
| **Simulation mode**            | `Normal` for quadprog / `External` for QUARC explicit |

## 7. Verification Checklist

Before deploying to hardware:

- [ ] Run `mpc_design.m` to confirm MPC gains and matrices.
- [ ] Simulate the model for 5 s with `theta0 = 2 deg` IC. Peak voltage must be < 6 V.
- [ ] Simulate with `theta0 = 5 deg` IC. CMPC must show lower peak voltage than PP (constraint-awareness benefit).
- [ ] Test bias load (0.123 Nm) with integral action. Mean `alpha` must converge to ~0.
- [ ] Verify the observer initialisation: `xhat0 = [x_c_meas; 0; alpha_meas; 0]`.
- [ ] For explicit MPC: verify the lookup table covers the full simulation trajectory (no clipping at grid boundaries).

## 8. Limitations (for thesis discussion)

1. **Computational cost**: `quadprog` per sample is ~1-5 ms on a typical PC. At 1 kHz (1 ms sample), this is borderline for soft real-time. The explicit LUT approach trades memory for speed.

2. **Curse of dimensionality**: A 4-D lookup table at 15 points/dim requires 50,625 entries. Adding integral action (5-D) is infeasible.

3. **Unmodelled dynamics**: Coulomb friction, encoder quantisation, and backlash are not represented in the prediction model. The MPC solution is optimal for the *model*, not the *plant*. Hardware performance is bounded by these unmodelled effects.

4. **No asymptotic stability**: As with pole placement, the seesaw never reaches `alpha = 0` asymptotically due to Coulomb friction dead-zone and encoder resolution. MPC reduces the limit-cycle amplitude through constraint-aware control, but cannot eliminate it.

5. **Tuning**: The Q and R weights (state and input penalties) trade off regulation aggressiveness against voltage usage. The weights shown here (`Q(3,3) = 2000`, `R = 1`) were tuned to match the pole-placement bandwidth; different weights produce different trade-off surfaces.

6. **Comparison to pole placement**: For small disturbances, CMPC and PP behave similarly (the unconstrained MPC reduces to finite-horizon LQR, which reduces to the discrete algebraic Riccati solution -- close to pole placement if poles are placed near the LQR solution). The advantage of CMPC appears only when constraints are active (large ICs, bias torque near saturation).
