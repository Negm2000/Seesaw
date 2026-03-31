%% CART PID PIPELINE — Quanser IP02 Cart on Table (No Seesaw)
%  =====================================================================
%  PID position control for the IP02 cart mounted flat on the table.
%  Ignores seesaw entirely — single-loop SISO position controller.
%
%  Plant:  G(s) = alpha_f * eta_m / (M_c * s^2 + B_total * s)
%          States: [x_c; x_c_dot]    Input: V_m    Output: x_c
%
%  Controller:  PID with derivative filter
%    u = Kp * e + Ki * integral(e) + Kd * N / (1 + N/s) * e
%    e = x_c_ref - x_c
%
%  Design method:
%    Interactive tuning via MATLAB's sisotool / pidTuner.
%    The pipeline opens sisotool with the plant pre-loaded and a PID
%    compensator structure.  The user tunes graphically (Bode shaping,
%    root locus, step response) and closes sisotool when satisfied.
%    The script then extracts the tuned gains automatically.
%
%  The pipeline:
%    §1  Load parameters & build plant TF
%    §2  PID design via sisotool (interactive)
%    §2b Voltage feasibility check (linear, post-tuning)
%    §3  Loop analysis (Bode, Nyquist, margins)
%    §4  Step response simulation (with saturation + anti-windup)
%    §5  Voltage & travel feasibility check
%    §6  Sensitivity analysis
%    §7  Build Simulink model: IP02_CartPID.slx (PID with anti-windup)
%    §8  Hardware deployment (QUARC)
%
%  Prerequisites: Run startup.m (loads seesaw_params.m)
%  =====================================================================

%% 1. LOAD PARAMETERS & BUILD PLANT
fprintf('\n============================================================\n');
fprintf('  CART PID PIPELINE — IP02 on Table (No Seesaw)\n');
fprintf('============================================================\n');

if ~exist('SEESAW_ROOT', 'var')
    SEESAW_ROOT = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end

% Load tuned B_eq if available, otherwise use default from seesaw_params
seesaw_params;

tp = fullfile(SEESAW_ROOT, 'data', 'tuned_params.mat');
if exist(tp, 'file')
    tuned = load(tp);
    B_eq = tuned.B_eq;
    fprintf('  Using tuned B_eq = %.3f N*s/m (from tuned_params.mat)\n', B_eq);
else
    fprintf('  Using default B_eq = %.3f N*s/m (from seesaw_params.m)\n', B_eq);
    fprintf('  NOTE: Run modeling_pipeline.m first for a tuned value.\n');
end

% Recompute derived constants with (possibly tuned) B_eq
alpha_f = (eta_g * K_g * k_t) / (R_m * r_mp);
B_emf   = alpha_f * K_g * k_m / r_mp;
B_total = B_eq + B_emf;

fprintf('\n  Plant parameters:\n');
fprintf('    M_c     = %.3f kg\n', M_c);
fprintf('    B_total = %.2f N*s/m  (B_eq=%.2f + B_emf=%.2f)\n', B_total, B_eq, B_emf);
fprintf('    alpha_f = %.4f\n', alpha_f);
fprintf('    eta_m   = %.2f\n', eta_m);
fprintf('    V_sat   = %.1f V\n', V_sat);
fprintf('    x_c_max = %.3f m (±%.0f mm)\n', x_c_max, x_c_max*1000);

% Cart state-space (same as Phase 1 in seesaw_params.m)
A_cart = [0,  1;
          0, -B_total/M_c];
B_cart_ss = [0;  alpha_f*eta_m/M_c];
C_cart = eye(2);
D_cart = zeros(2, 1);

% SISO transfer function: V_m → x_c
s = tf('s');
G_xc = alpha_f * eta_m / (M_c * s^2 + B_total * s);

fprintf('\n  Open-loop poles:  ');
p_ol = pole(G_xc);
for k = 1:length(p_ol)
    fprintf('%.4f  ', p_ol(k));
end
fprintf('\n');
fprintf('  Open-loop zeros:  ');
z_ol = zero(G_xc);
if isempty(z_ol), fprintf('(none)'); else fprintf('%.4f  ', z_ol); end
fprintf('\n');
fprintf('  DC gain of G_xcdot = V_m → x_c_dot: %.4f m/s per V\n', ...
    alpha_f * eta_m / B_total);

%% 2. PID DESIGN VIA SISOTOOL (INTERACTIVE)
%  -----------------------------------------------------------------------
%  Uses MATLAB's sisotool for interactive graphical PID tuning.
%
%  Workflow:
%    1. pidtune() computes a reasonable starting-point PID
%    2. sisotool opens with the plant + initial compensator
%    3. YOU tune graphically: drag poles/zeros, shape Bode, watch step
%    4. Close sisotool when satisfied → gains are extracted automatically
%
%  sisotool views you should use:
%    - Open-Loop Bode    → shape gain crossover & phase margin
%    - Root Locus        → drag closed-loop poles
%    - Step Response     → see overshoot / settling in real time
%
%  Alternative: If you prefer a simpler interface, set USE_PIDTUNER = true
%  below to use pidTuner (app) instead of sisotool.
%  -----------------------------------------------------------------------

fprintf('\n--- §2: PID Design (Interactive — sisotool) ---\n');

USE_PIDTUNER = false;   % Set true to use pidTuner app instead of sisotool

% Derivative filter coefficient (used by PID block in Simulink)
N_filt = 100;   % 100 rad/s cutoff (tau_d = 0.01 s)

% --- Step 2a: Compute a starting-point PID with pidtune ---
%  pidtune picks gains that give ~60 deg phase margin by default.
%  We then check if the resulting controller would saturate for a 5 cm step.
%  If it does, we reduce the crossover frequency until peak voltage fits
%  within V_sat.  This gives sisotool a feasible starting point.

fprintf('  Computing initial PID gains with pidtune()...\n');

x_c_ref_design = 0.05;  % design step size [m] — match §4

% Helper: compute linear peak |V_m| for a given PID and step reference.
%   V_m = C(s) / (1 + C(s)*G(s)) * ref   (sensitivity × C × ref)
ref_to_Vm = @(C_pid) tf(C_pid) * feedback(1, tf(C_pid) * G_xc);
check_Vpeak = @(C_pid) max(abs(step(ref_to_Vm(C_pid) * x_c_ref_design, 0:0.001:5)));

% First try: unconstrained pidtune
C_pid0 = pidtune(G_xc, 'PIDF');
[~, ~, ~, wc0] = margin(C_pid0 * G_xc);
Vpeak0 = check_Vpeak(C_pid0);

fprintf('    Unconstrained pidtune:\n');
fprintf('      Crossover freq: %.2f rad/s\n', wc0);
fprintf('      Peak voltage (5cm step): %.2f V  (limit %.1f V)\n', Vpeak0, V_sat);

if Vpeak0 > V_sat
    % --- Voltage-aware tuning: binary search for max feasible wc ---
    fprintf('\n    Peak voltage exceeds V_sat! Reducing bandwidth...\n');

    wc_lo = 0.5;     % conservative lower bound [rad/s]
    wc_hi = wc0;     % unconstrained upper bound
    wc_best = wc_lo;

    for iter = 1:20
        wc_try = (wc_lo + wc_hi) / 2;
        C_try = pidtune(G_xc, 'PIDF', wc_try);
        Vp = check_Vpeak(C_try);

        if Vp <= V_sat * 0.95   % 5% margin from saturation
            wc_best = wc_try;
            wc_lo = wc_try;     % can go faster
        else
            wc_hi = wc_try;     % too aggressive
        end

        if (wc_hi - wc_lo) < 0.1
            break;
        end
    end

    C_pid0 = pidtune(G_xc, 'PIDF', wc_best);
    Vpeak_final = check_Vpeak(C_pid0);
    [~, ~, ~, wc_final] = margin(C_pid0 * G_xc);

    fprintf('    Voltage-constrained pidtune:\n');
    fprintf('      Max feasible wc: %.2f rad/s  (was %.2f)\n', wc_final, wc0);
    fprintf('      Peak voltage:    %.2f V  (limit %.1f V)\n', Vpeak_final, V_sat);
    fprintf('      Bandwidth reduced by %.0f%% to respect V_sat.\n', ...
        (1 - wc_final/wc0) * 100);
else
    fprintf('    OK: Within voltage limits.\n');
end

Kp0 = C_pid0.Kp;
Ki0 = C_pid0.Ki;
Kd0 = C_pid0.Kd;
Tf0 = C_pid0.Tf;    % filter time constant (= 1/N)
N0  = 1 / Tf0;

fprintf('\n    Starting-point PID gains:\n');
fprintf('      Kp = %.4f\n', Kp0);
fprintf('      Ki = %.4f\n', Ki0);
fprintf('      Kd = %.4f\n', Kd0);
fprintf('      N  = %.1f rad/s  (Tf = %.4f s)\n', N0, Tf0);

[~, pm0] = margin(C_pid0 * G_xc);
fprintf('      Phase margin: %.1f deg\n', pm0);

% Build the compensator as a transfer function for sisotool
%   C(s) = Kp + Ki/s + Kd*s/(1 + s/N)
%         = Kp + Ki/s + Kd*N*s/(s + N)
%   Combined:  C(s) = [Kd*N*s^2 + (Kp*N + Kd*N^2 + Ki)*s + Ki*N]
%                      / [s*(s + N)]
%
% Alternatively, we pass the PID object directly to sisotool.

% Convert PID object to tf for sisotool compatibility
C_tf0 = tf(C_pid0);

% --- Step 2b: Interactive tuning ---
if USE_PIDTUNER
    % ---- pidTuner (simpler app) ----
    fprintf('\n  Opening pidTuner app...\n');
    fprintf('  Tune the PID interactively. Close the app when done.\n');
    fprintf('  The tuned controller will be extracted automatically.\n\n');

    pidTuner(G_xc, C_pid0);

    fprintf('  >>> Waiting for you to close pidTuner... <<<\n');
    uiwait(gcf);

    % After pidTuner closes, the tuned controller is in the base workspace
    % as 'C' if the user clicked "Export". Otherwise we keep C_pid0.
    if evalin('base', 'exist(''C'', ''var'')')
        C_tuned = evalin('base', 'C');
        fprintf('  Tuned PID exported from pidTuner.\n');
    else
        C_tuned = C_pid0;
        fprintf('  NOTE: No export detected — using pidtune defaults.\n');
        fprintf('  Tip: Click "Export" in pidTuner before closing.\n');
    end
else
    % ---- sisotool (full SISO Design Tool) ----
    fprintf('\n  Opening sisotool with plant G(s) and PID compensator...\n');
    fprintf('  ┌─────────────────────────────────────────────────┐\n');
    fprintf('  │  SISOTOOL USAGE:                                │\n');
    fprintf('  │                                                 │\n');
    fprintf('  │  The compensator C(s) is pre-loaded as a PID.   │\n');
    fprintf('  │                                                 │\n');
    fprintf('  │  Recommended views (View → Design Plots):       │\n');
    fprintf('  │    • Open-Loop Bode  — shape crossover freq     │\n');
    fprintf('  │    • Root Locus      — drag CL poles            │\n');
    fprintf('  │    • Nichols         — see gain + phase together │\n');
    fprintf('  │                                                 │\n');
    fprintf('  │  Analysis plots (View → Analysis Plots):        │\n');
    fprintf('  │    • Closed-Loop Step — see overshoot/settling  │\n');
    fprintf('  │    • Closed-Loop Bode — bandwidth               │\n');
    fprintf('  │                                                 │\n');
    fprintf('  │  Tips:                                          │\n');
    fprintf('  │    • Right-click → Add Pole/Zero to change      │\n');
    fprintf('  │      compensator structure                       │\n');
    fprintf('  │    • Drag poles/zeros or type exact values       │\n');
    fprintf('  │    • Aim for PM ≈ 45-65 deg, low overshoot      │\n');
    fprintf('  │    • Keep bandwidth < 50 rad/s (noise + V_sat)  │\n');
    fprintf('  │                                                 │\n');
    fprintf('  │  When satisfied: simply CLOSE the sisotool      │\n');
    fprintf('  │  window. The gains will be extracted.            │\n');
    fprintf('  └─────────────────────────────────────────────────┘\n\n');

    % Open sisotool: negative feedback, plant = G_xc, compensator = C_tf0
    sisotool(G_xc, C_tf0);

    fprintf('  >>> Waiting for you to tune and close sisotool... <<<\n');
    uiwait(gcf);

    % Extract the tuned compensator from the SISO Design Task
    % sisotool stores it in the DesignTask object in the base workspace.
    % The compensator is accessible via the sisotool session data.
    try
        % Get the SISO Tool session data
        % After closing sisotool, the compensator is stored in the session
        ST = get(gcf, 'UserData');  % May not work after close

        % More robust: sisotool exports to base workspace on close
        if evalin('base', 'exist(''DesignTask'', ''var'')')
            DT = evalin('base', 'DesignTask');
            C_tuned_tf = tf(DT.Compensator);
            C_tuned = pid(C_tuned_tf);
            fprintf('  Tuned compensator extracted from DesignTask.\n');
        elseif evalin('base', 'exist(''C'', ''var'')')
            C_tuned = evalin('base', 'C');
            fprintf('  Tuned compensator extracted from workspace variable C.\n');
        else
            % Last resort: use the Control System Toolbox store
            fprintf('  Could not auto-extract. Checking ltiview store...\n');
            C_tuned = C_pid0;
            fprintf('  WARNING: Using pidtune defaults. To override:\n');
            fprintf('    1. Re-run sisotool(G_xc) and export compensator\n');
            fprintf('    2. Or set Kp_cart, Ki_cart, Kd_cart manually below\n');
        end
    catch
        C_tuned = C_pid0;
        fprintf('  Auto-extraction failed — using pidtune defaults.\n');
        fprintf('  To manually set gains, edit Kp_cart/Ki_cart/Kd_cart below.\n');
    end
end

% --- Step 2c: Extract PID gains from tuned controller ---
try
    % Try to interpret as a PID object
    if ~isa(C_tuned, 'pid') && ~isa(C_tuned, 'pidstd')
        C_tuned = pid(C_tuned);  % convert tf → pid
    end
    Kp_cart = C_tuned.Kp;
    Ki_cart = C_tuned.Ki;
    Kd_cart = C_tuned.Kd;
    if isprop(C_tuned, 'Tf') && C_tuned.Tf > 0
        N_filt = round(1 / C_tuned.Tf);
    end
catch
    % If conversion fails, parse from transfer function
    fprintf('  Direct PID extraction failed. Parsing from TF...\n');
    C_tuned_tf = tf(C_tuned);
    [num, den] = tfdata(C_tuned_tf, 'v');
    % For PID+filter: C(s) = (a*s^2 + b*s + c) / (s*(s + N))
    %   Kd = a/1, Kp = (b - a*N)/1 ... (complex, just warn user)
    fprintf('  WARNING: Could not auto-parse PID gains from TF.\n');
    fprintf('  Compensator TF:\n');
    C_tuned_tf
    fprintf('  Falling back to pidtune defaults.\n');
    Kp_cart = Kp0;
    Ki_cart = Ki0;
    Kd_cart = Kd0;
end

fprintf('\n  ═══════════════════════════════════════\n');
fprintf('  FINAL PID GAINS (from sisotool):\n');
fprintf('    Kp = %.4f\n', Kp_cart);
fprintf('    Ki = %.4f\n', Ki_cart);
fprintf('    Kd = %.4f\n', Kd_cart);
fprintf('    N  = %d rad/s (derivative filter)\n', N_filt);
fprintf('  ═══════════════════════════════════════\n');

% Build augmented system for later analysis (§3, §6)
A_aug = [A_cart,       zeros(2,1);
         -1, 0,        0];          % xi_dot = -x_c (when ref=0)
B_aug = [B_cart_ss; 0];
K_aug = [Kp_cart, Kd_cart, -Ki_cart];  % state feedback equivalent

A_aug_cl = A_aug - B_aug * K_aug;
ev_cl = eig(A_aug_cl);
fprintf('\n  Closed-loop poles:\n');
for k = 1:length(ev_cl)
    if abs(imag(ev_cl(k))) > 1e-6
        fprintf('    %.4f %+.4fi\n', real(ev_cl(k)), imag(ev_cl(k)));
    else
        fprintf('    %.4f\n', real(ev_cl(k)));
    end
end

if any(real(ev_cl) > 1e-3)
    warning('Closed-loop is UNSTABLE! Re-tune in sisotool.');
    fprintf('  Re-run this section or manually adjust gains.\n');
end

%% 2b. VOLTAGE FEASIBILITY CHECK (post-tuning)
%  -----------------------------------------------------------------------
%  Quick linear step simulation to check if the tuned PID will saturate.
%  This runs BEFORE the full nonlinear sim in §4 — it's a fast sanity
%  check so you know immediately whether to re-tune.
%  -----------------------------------------------------------------------

fprintf('\n--- §2b: Voltage Feasibility (linear, pre-saturation) ---\n');

% Closed-loop from reference to [x_c; x_c_dot; V_m]
%   V_m = -K_aug * x_aug + Kp_cart * ref
% For a unit step ref, the peak V_m tells us if we'll saturate.

% Build CL system: ref → V_m
%   x_dot = (A_aug - B_aug*K_aug)*x + B_aug*Kp_cart*ref
%   V_m   = -K_aug*x + Kp_cart*ref
B_ref = B_aug * Kp_cart;         % reference input matrix
C_Vm  = -K_aug;                   % voltage output
D_Vm  = Kp_cart;                  % direct feedthrough from ref
sys_ref2Vm = ss(A_aug_cl, B_ref, C_Vm, D_Vm);

% Simulate 5 cm step (same as §4)
x_c_ref_check = 0.05;  % [m]
t_check = linspace(0, 5, 2000)';
[Vm_linear, ~] = step(sys_ref2Vm * x_c_ref_check, t_check);

V_peak_linear = max(abs(Vm_linear));

fprintf('  5 cm step — LINEAR peak voltage: %.2f V\n', V_peak_linear);
fprintf('  Saturation limit:                %.1f V\n', V_sat);

if V_peak_linear > V_sat
    V_ratio = V_peak_linear / V_sat;
    fprintf('\n  *** WARNING: Peak voltage is %.1fx the saturation limit! ***\n', V_ratio);
    fprintf('  The controller WILL saturate. Consequences:\n');
    fprintf('    - Slower response than the linear analysis predicts\n');
    fprintf('    - Integrator windup → overshoot\n');
    fprintf('    - Phase margin from §3 is NOT reliable during transients\n');
    fprintf('\n  Options:\n');
    fprintf('    1. Accept it (anti-windup in Simulink will help) — mild sat is OK\n');
    fprintf('    2. Re-tune in sisotool: reduce bandwidth (lower crossover freq)\n');
    fprintf('    3. Reduce step size (e.g., 2 cm instead of 5 cm)\n');
    if V_ratio > 2.0
        fprintf('\n  STRONG RECOMMENDATION: Re-tune. %.1fx saturation is excessive.\n', V_ratio);
        fprintf('  In sisotool, drag the Bode gain curve down to reduce crossover.\n');
    end
else
    margin_V = V_sat - V_peak_linear;
    fprintf('  OK: %.2f V headroom — no saturation expected for 5 cm step.\n', margin_V);
end

%% 3. LOOP TRANSFER FUNCTION ANALYSIS
%  -----------------------------------------------------------------------
%  L(s) = K_aug * (sI - A_aug)^{-1} * B_aug
%  Standard SISO analysis: Bode, Nyquist, margins.
%  -----------------------------------------------------------------------

fprintf('\n--- §3: Loop Analysis ---\n');

sys_loop = ss(A_aug, B_aug, K_aug, 0);
L = tf(sys_loop);

fprintf('  Loop TF poles:  '); fprintf('%.4f  ', pole(L)); fprintf('\n');
fprintf('  Loop TF zeros:  '); fprintf('%.4f  ', zero(L)); fprintf('\n');

[Gm, Pm, wpc, wgc] = margin(L);
fprintf('\n  Stability margins:\n');
fprintf('    Phase margin:  PM = %.1f deg at omega = %.2f rad/s\n', Pm, wgc);
fprintf('    Gain margin:   GM = %.1f dB  at omega = %.2f rad/s\n', 20*log10(Gm), wpc);

if Pm >= 30 && Pm <= 70
    fprintf('    PM is in the acceptable range [30, 70] deg.\n');
elseif Pm < 30
    fprintf('    WARNING: PM < 30 deg — system may be too aggressive.\n');
elseif Pm > 70
    fprintf('    NOTE: PM > 70 deg — conservative, may be sluggish.\n');
end

% PID transfer function for documentation
fprintf('\n  PID transfer function:\n');
fprintf('    C(s) = (%.4f s^2 + %.4f s + %.4f) / s\n', Kd_cart, Kp_cart, Ki_cart);
fprintf('    with derivative filter: N = %d\n', N_filt);

% --- Plots ---
omega = logspace(-1, 2.5, 500);

figure('Name', 'Cart PID — Loop Analysis', 'Position', [50 50 1200 800]);

subplot(2,2,1);
margin(L);
title(sprintf('Bode: L(s) — PM = %.1f deg, GM = %.1f dB', Pm, 20*log10(Gm)));
grid on;

subplot(2,2,2);
nyquist(L, omega);
hold on;
plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2);
title('Nyquist: L(s)');
grid on;

subplot(2,2,3);
rlocus(L);
title('Root Locus');
xline(0, 'k--');
sgrid(0.5, []);
grid on;

subplot(2,2,4);
C_out = [eye(2), zeros(2,1)];  % output [x_c; x_c_dot] from augmented
sys_cl_pz = ss(A_aug_cl, B_aug, C_out, zeros(2,1));
pzmap(sys_cl_pz);
title('Closed-Loop Pole-Zero Map');
sgrid;
grid on;

sgtitle('Cart PID — Frequency-Domain Loop Analysis', 'FontWeight', 'bold');

if ~exist(fullfile(SEESAW_ROOT, 'docs', 'figures'), 'dir')
    mkdir(fullfile(SEESAW_ROOT, 'docs', 'figures'));
end
exportgraphics(gcf, fullfile(SEESAW_ROOT, 'docs', 'figures', 'cart_pid_loop_analysis.png'), 'Resolution', 300);

%% 4. STEP RESPONSE SIMULATION (WITH SATURATION)
%  -----------------------------------------------------------------------
%  5 cm step at t=1 s.  Saturated ODE to match hardware behaviour.
%  -----------------------------------------------------------------------

fprintf('\n--- §4: Step Response Simulation ---\n');

T_sim = 8;
dt    = 0.001;
t_sim = (0:dt:T_sim)';

x_c_ref_val = 0.05;   % 5 cm step [m]
t_step = 1.0;          % step time [s]

% Reference signal
ref = zeros(length(t_sim), 1);
ref(t_sim >= t_step) = x_c_ref_val;

% Augmented state: [x_c; x_c_dot; xi]
% xi_dot = ref - x_c  (tracking integrator)
% u = -K_aug * [x_c - ref; x_c_dot; xi]  (error-based formulation)
%   Wait — let's be precise. The state feedback is:
%     u = -K_aug(1)*(x_c) - K_aug(2)*(x_c_dot) - K_aug(3)*(xi)
%   But the reference enters via the integrator: xi_dot = ref - x_c
%   and as a feedforward:  u = K_aug(1)*ref - K_aug(1)*x_c - K_aug(2)*x_c_dot - K_aug(3)*xi
%   i.e., u = K_aug(1)*(ref - x_c) + ..., which is exactly a PID on e = ref - x_c
%   because Ki drives xi, Kp drives e, Kd drives -x_c_dot.

sat_v = @(v) sign(v) .* min(abs(v), V_sat);

% Augmented dynamics with reference input
%   x_aug = [x_c; x_c_dot; xi]
%   u = Kp*(ref - x_c) + Kd*(-x_c_dot) + Ki*(xi)
%     = -Kp*x_c - Kd*x_c_dot + Ki*xi + Kp*ref
%     = -K_aug * x_aug + Kp*ref

ode_cart_pid = @(t, x, r) ...
    A_aug * x + B_aug * sat_v( ...
        Kp_cart*(r - x(1)) + Ki_cart*x(3) + Kd_cart*(0 - x(2)) );

% Manual xi_dot override: xi_dot = ref - x_c (not -x_c)
% So we integrate manually with proper reference tracking.
x_aug0 = [0; 0; 0];

% Use ode45 with events-style approach
% Actually, for clean code with saturation + time-varying ref, let's just
% do a simple Euler integration at dt = 0.001 s (plenty accurate for this).

n_steps = length(t_sim);
x_hist = zeros(n_steps, 3);
u_hist = zeros(n_steps, 1);
x_aug = x_aug0;

for k = 1:n_steps
    t_k = t_sim(k);
    r_k = 0;
    if t_k >= t_step
        r_k = x_c_ref_val;
    end

    x_hist(k, :) = x_aug';

    % PID control law: u = Kp*(ref - x_c) + Ki*xi + Kd*(0 - x_c_dot)
    e_k = r_k - x_aug(1);
    u_unsaturated = Kp_cart * e_k + Ki_cart * x_aug(3) + Kd_cart * (0 - x_aug(2));
    u_k = sat_v(u_unsaturated);
    u_hist(k) = u_k;

    % State derivatives
    x_c_ddot = (-B_total * x_aug(2) + alpha_f * eta_m * u_k) / M_c;

    % Anti-windup (clamping): freeze integrator when output is saturated
    % AND the integrator would push it further into saturation.
    is_saturated = (abs(u_unsaturated) > V_sat);
    windup_direction = (sign(u_unsaturated) == sign(e_k));
    if is_saturated && windup_direction
        xi_dot = 0;   % clamp: don't accumulate more error
    else
        xi_dot = e_k;  % normal integration
    end

    % Euler step
    x_aug(1) = x_aug(1) + dt * x_aug(2);
    x_aug(2) = x_aug(2) + dt * x_c_ddot;
    x_aug(3) = x_aug(3) + dt * xi_dot;
end

xc_sim   = x_hist(:, 1) * 100;    % [cm]
xdot_sim = x_hist(:, 2) * 100;    % [cm/s]
Vm_sim   = u_hist;                 % [V]

figure('Name', 'Cart PID — Step Response', 'Position', [100 100 900 700]);

subplot(3,1,1);
plot(t_sim, ref*100, 'k--', 'LineWidth', 1); hold on;
plot(t_sim, xc_sim, 'b-', 'LineWidth', 1.5);
ylabel('x_c [cm]'); title('Cart Position');
legend('Reference', 'Response', 'Location', 'SouthEast');
grid on;

subplot(3,1,2);
plot(t_sim, xdot_sim, 'b-', 'LineWidth', 1.5);
ylabel('dx_c/dt [cm/s]'); title('Cart Velocity');
yline(0, 'k--');
grid on;

subplot(3,1,3);
plot(t_sim, Vm_sim, 'r-', 'LineWidth', 1.5);
ylabel('V_m [V]'); xlabel('Time [s]');
title('Motor Voltage');
yline(V_sat, 'k--', '+V_{sat}');
yline(-V_sat, 'k--', '-V_{sat}');
grid on;

sgtitle(sprintf('Cart PID Step Response (%.0f cm step)', x_c_ref_val*100), 'FontWeight', 'bold');

exportgraphics(gcf, fullfile(SEESAW_ROOT, 'docs', 'figures', 'cart_pid_step_response.png'), 'Resolution', 300);

% Compute step response metrics
idx_step = find(t_sim >= t_step, 1);
xc_after = xc_sim(idx_step:end);
t_after  = t_sim(idx_step:end) - t_step;
ref_cm   = x_c_ref_val * 100;

% Rise time (10% to 90%)
idx_10 = find(xc_after >= 0.1*ref_cm, 1);
idx_90 = find(xc_after >= 0.9*ref_cm, 1);
if ~isempty(idx_10) && ~isempty(idx_90)
    t_rise = t_after(idx_90) - t_after(idx_10);
else
    t_rise = NaN;
end

% Overshoot
os_pct = (max(xc_after) - ref_cm) / ref_cm * 100;

% Settling time (2% band)
band_2pct = 0.02 * ref_cm;
idx_settle = find(abs(xc_after - ref_cm) > band_2pct, 1, 'last');
if ~isempty(idx_settle) && idx_settle < length(t_after)
    t_settle = t_after(idx_settle);
else
    t_settle = 0;
end

% Steady-state error
ss_err = ref_cm - mean(xc_after(end-100:end));

fprintf('\n  Step response metrics (%.0f cm step):\n', ref_cm);
fprintf('    Rise time (10-90%%):   %.3f s\n', t_rise);
fprintf('    Overshoot:            %.1f %%\n', os_pct);
fprintf('    Settling time (2%%):   %.2f s\n', t_settle);
fprintf('    Steady-state error:   %.3f cm\n', ss_err);

%% 5. VOLTAGE & TRAVEL FEASIBILITY CHECK

fprintf('\n--- §5: Hardware Feasibility ---\n');

V_peak = max(abs(Vm_sim));
xc_peak = max(abs(xc_sim));

fprintf('  V_peak (sim):   %.2f V (limit %.1f V)\n', V_peak, V_sat);
if V_peak >= V_sat - 0.01
    fprintf('  *** SATURATION occurred — consider reducing wn or step size.\n');
else
    fprintf('  OK: Within voltage limits (margin %.2f V).\n', V_sat - V_peak);
end

fprintf('  Peak cart travel:  %.1f mm (limit +/-%.0f mm)\n', xc_peak*10, x_c_max*1000);
if xc_peak*10 > x_c_max * 1000 * 0.8
    fprintf('  WARNING: Cart approaching end-stops.\n');
else
    fprintf('  OK: Cart well within track.\n');
end

%% 6. SENSITIVITY ANALYSIS
%  Robustness check: what happens if B_eq is off by ±30%?

fprintf('\n--- §6: Sensitivity to B_eq Uncertainty ---\n');
fprintf('  B_eq variation    PM [deg]    GM [dB]    Max Re(eig)\n');
B_eq_test = B_eq * [0.7, 0.85, 1.0, 1.15, 1.3];
for i = 1:length(B_eq_test)
    B_t_i = B_eq_test(i) + B_emf;
    A_cart_i = [0, 1; 0, -B_t_i/M_c];
    A_aug_i = [A_cart_i, zeros(2,1); -1, 0, 0];
    B_aug_i = [0; alpha_f*eta_m/M_c; 0];
    A_cl_i = A_aug_i - B_aug_i * K_aug;
    ev_i = eig(A_cl_i);
    max_re = max(real(ev_i));
    L_i = tf(ss(A_aug_i, B_aug_i, K_aug, 0));
    [Gm_i, Pm_i] = margin(L_i);
    stab_str = '';
    if max_re > 0.01, stab_str = '<-- UNSTABLE'; end
    fprintf('  B_eq = %5.2f (%+3.0f%%)   %5.1f       %5.1f       %+.4f  %s\n', ...
        B_eq_test(i), (B_eq_test(i)/B_eq - 1)*100, Pm_i, 20*log10(Gm_i), ...
        max_re, stab_str);
end

%% 7. BUILD SIMULINK MODEL: IP02_CartPID
%  -----------------------------------------------------------------------
%  Programmatically creates a closed-loop cart PID model.
%
%  Architecture:
%    x_c_ref ──►[Sum]──► PID ──►[V_Sat]──► Plant ──► x_c
%                  ▲(-)                                │
%                  └───────────────────────────────────┘
%
%  Two modes (auto-detected):
%    Simulation: A_cart/B_cart State-Space plant in the loop
%    Hardware:   QUARC I/O (encoder → position → PID → DAC → motor)
%
%  Following the pattern from build_simulink_models.m and
%  control_pipeline_pid.m §11.
%  -----------------------------------------------------------------------

fprintf('\n============================================================\n');
fprintf('  §7: BUILDING SIMULINK MODEL: IP02_CartPID\n');
fprintf('============================================================\n');

mdl = 'IP02_CartPID';

% Push gains to base workspace
assignin('base', 'Kp_cart',  Kp_cart);
assignin('base', 'Ki_cart',  Ki_cart);
assignin('base', 'Kd_cart',  Kd_cart);
assignin('base', 'N_filt',   N_filt);
assignin('base', 'V_sat',    V_sat);
assignin('base', 'A_cart',   A_cart);
assignin('base', 'B_cart_ss', B_cart_ss);
assignin('base', 'C_cart',   C_cart);
assignin('base', 'D_cart',   D_cart);

% Detect QUARC
quarc_available = exist('quarc_library', 'file') == 4 || ...
                  ~isempty(which('hil_initialize_block'));
if ~quarc_available
    try
        quarc_available = ~isempty(ver('quarc'));
    catch
        quarc_available = false;
    end
end

if quarc_available
    fprintf('  QUARC detected — model will include hardware I/O blocks.\n');
else
    fprintf('  QUARC not detected — building simulation-only model.\n');
end

% Close old model if loaded
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

% --- Solver settings ---
if quarc_available
    set_param(mdl, 'SolverType', 'Fixed-step', ...
        'Solver',    'ode1', ...
        'FixedStep', '0.002', ...
        'StopTime',  'inf');
else
    set_param(mdl, 'Solver', 'ode45', ...
        'StopTime', '10', ...
        'MaxStep',  '1e-3', ...
        'RelTol',   '1e-4', ...
        'AbsTol',   '1e-6');
end

% ==================================================================
%  REFERENCE INPUT
% ==================================================================
% Step reference: 5 cm step at t = 2 s (default).
% Change 'After' to desired position in meters.
add_block('simulink/Sources/Step', [mdl '/x_c_ref'], ...
    'Time',   '2', ...
    'Before', '0', ...
    'After',  '0.05', ...
    'Position', [50 95 90 135]);

% ==================================================================
%  PID CONTROLLER
% ==================================================================

% Sum: e = x_c_ref - x_c
add_block('simulink/Math Operations/Sum', [mdl '/Sum_err'], ...
    'Inputs', '+-', ...
    'Position', [170 100 190 120]);

% PID block (Kp + Ki/s + Kd*N/(1+N/s)) with anti-windup
add_block('simulink/Continuous/PID Controller', [mdl '/PID_Cart'], ...
    'P', 'Kp_cart', ...
    'I', 'Ki_cart', ...
    'D', 'Kd_cart', ...
    'N', 'N_filt', ...
    'AntiWindupMode',    'clamping', ...
    'UpperSaturationLimit', 'V_sat', ...
    'LowerSaturationLimit', '-V_sat', ...
    'Position', [260 87 350 133]);

% ==================================================================
%  VOLTAGE SATURATION
% ==================================================================
add_block('simulink/Discontinuities/Saturation', [mdl '/V_Sat'], ...
    'UpperLimit', 'V_sat', ...
    'LowerLimit', '-V_sat', ...
    'Position', [410 98 460 132]);

% ==================================================================
%  WIRING: FORWARD PATH
% ==================================================================
add_line(mdl, 'x_c_ref/1', 'Sum_err/1',  'autorouting', 'smart');
h = add_line(mdl, 'Sum_err/1', 'PID_Cart/1', 'autorouting', 'smart');
set_param(h, 'Name', 'e_{xc}');
add_line(mdl, 'PID_Cart/1', 'V_Sat/1',   'autorouting', 'smart');

% ==================================================================
%  DISPLAY: SCOPES + TO-WORKSPACE
% ==================================================================
add_block('simulink/Math Operations/Gain', [mdl '/ref_m_to_cm'], ...
    'Gain', '100', 'Position', [800 65 840 85]);

add_block('simulink/Math Operations/Gain', [mdl '/xc_m_to_cm'], ...
    'Gain', '100', 'Position', [800 115 840 135]);

% Scopes
add_block('simulink/Sinks/Scope', [mdl '/Scope_Cart'], ...
    'NumInputPorts', '2', 'Position', [920 80 960 130]);
set_param([mdl '/Scope_Cart'], 'Name', 'Cart [cm]');

add_block('simulink/Sinks/Scope', [mdl '/Scope_Vel'], ...
    'NumInputPorts', '1', 'Position', [920 160 960 190]);
set_param([mdl '/Scope_Vel'], 'Name', 'Velocity [m/s]');

add_block('simulink/Sinks/Scope', [mdl '/Scope_Vm'], ...
    'NumInputPorts', '1', 'Position', [920 240 960 270]);
set_param([mdl '/Scope_Vm'], 'Name', 'Voltage [V]');

% To Workspace
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_xc'], ...
    'VariableName', 'cart_xc', 'SaveFormat', 'Timeseries', ...
    'Position', [920 310 990 330]);

add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_xdot'], ...
    'VariableName', 'cart_xdot', 'SaveFormat', 'Timeseries', ...
    'Position', [920 345 990 365]);

add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_Vm'], ...
    'VariableName', 'cart_Vm', 'SaveFormat', 'Timeseries', ...
    'Position', [920 380 990 400]);

% Voltage → scope + workspace
add_line(mdl, 'V_Sat/1', 'Voltage [V]/1', 'autorouting', 'smart');
add_line(mdl, 'V_Sat/1', 'ToWS_Vm/1',     'autorouting', 'smart');

% Reference → Cart scope port 1
add_line(mdl, 'x_c_ref/1',     'ref_m_to_cm/1', 'autorouting', 'smart');
add_line(mdl, 'ref_m_to_cm/1', 'Cart [cm]/1',   'autorouting', 'smart');

% ==================================================================
%  FEEDBACK PATH — hardware vs simulation
% ==================================================================
if quarc_available
    fprintf('  Adding QUARC hardware I/O blocks...\n');

    try
        % HIL Initialize
        add_block('quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize', ...
            [mdl '/HIL Initialize'], 'Position', [50 350 134 425]);

        % Motor output: V_Sat → DAC ch0
        add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Write Analog', ...
            [mdl '/Motor Command'], 'Position', [520 98 605 132]);

        % Encoder input: ch0 = cart (only one encoder used)
        add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Read Encoder', ...
            [mdl '/Encoder'], 'Position', [50 450 135 510]);

        % Encoder → position [m]
        add_block('simulink/Math Operations/Gain', [mdl '/Enc_to_m'], ...
            'Gain', 'K_ec', 'Position', [200 465 260 495]);

        % Dirty derivative for velocity display (not used by PID — PID has its own D)
        tau_d = 0.01;
        assignin('base', 'tau_d', tau_d);
        add_block('simulink/Continuous/Transfer Fcn', [mdl '/Deriv_xc'], ...
            'Numerator',   '[1, 0]', ...
            'Denominator', '[tau_d, 1]', ...
            'Position', [320 465 420 495]);

        % Wire motor output
        add_line(mdl, 'V_Sat/1', 'Motor Command/1', 'autorouting', 'smart');

        % Wire encoder → conversion
        add_line(mdl, 'Encoder/1', 'Enc_to_m/1',  'autorouting', 'smart');

        % Position → dirty derivative (for velocity scope)
        add_line(mdl, 'Enc_to_m/1', 'Deriv_xc/1', 'autorouting', 'smart');

        % Feedback: x_c → error sum
        add_line(mdl, 'Enc_to_m/1', 'Sum_err/2', 'autorouting', 'smart');

        % Feedback → display scopes
        add_line(mdl, 'Enc_to_m/1',  'xc_m_to_cm/1',   'autorouting', 'smart');
        add_line(mdl, 'xc_m_to_cm/1', 'Cart [cm]/2',   'autorouting', 'smart');
        add_line(mdl, 'Deriv_xc/1',  'Velocity [m/s]/1', 'autorouting', 'smart');

        % Feedback → To Workspace
        add_line(mdl, 'Enc_to_m/1',  'ToWS_xc/1',   'autorouting', 'smart');
        add_line(mdl, 'Deriv_xc/1',  'ToWS_xdot/1', 'autorouting', 'smart');

        fprintf('  QUARC blocks wired successfully.\n');
        fprintf('  PID block handles derivative filtering internally.\n');
        fprintf('  Separate dirty derivative for velocity display only.\n');
    catch ME
        fprintf('  WARNING: Could not add QUARC blocks: %s\n', ME.message);
        fprintf('  Model will NOT work on hardware.\n');
    end
else
    % ============ SIMULATION ONLY: plant in the loop ============
    fprintf('  Adding State-Space plant for closed-loop simulation...\n');

    % State-Space plant: V_m → [x_c; x_c_dot]
    add_block('simulink/Continuous/State-Space', [mdl '/Cart_SS'], ...
        'A', 'A_cart', 'B', 'B_cart_ss', 'C', 'C_cart', 'D', 'D_cart', ...
        'X0', '[0; 0]', ...
        'Position', [540 80 650 160]);

    % Demux: split [x_c; x_c_dot]
    add_block('simulink/Signal Routing/Demux', [mdl '/Demux_Plant'], ...
        'Outputs', '2', 'Position', [700 85 705 155]);

    % V_Sat → Plant → Demux
    h = add_line(mdl, 'V_Sat/1',     'Cart_SS/1',     'autorouting', 'smart');
    set_param(h, 'Name', 'V_m');
    add_line(mdl, 'Cart_SS/1',  'Demux_Plant/1', 'autorouting', 'smart');

    % Feedback: x_c → error sum
    h = add_line(mdl, 'Demux_Plant/1', 'Sum_err/2', 'autorouting', 'smart');
    set_param(h, 'Name', 'x_c');

    % Feedback → display scopes
    add_line(mdl, 'Demux_Plant/1', 'xc_m_to_cm/1',     'autorouting', 'smart');
    add_line(mdl, 'xc_m_to_cm/1',  'Cart [cm]/2',      'autorouting', 'smart');
    add_line(mdl, 'Demux_Plant/2', 'Velocity [m/s]/1',  'autorouting', 'smart');

    % Feedback → To Workspace
    add_line(mdl, 'Demux_Plant/1', 'ToWS_xc/1',   'autorouting', 'smart');
    add_line(mdl, 'Demux_Plant/2', 'ToWS_xdot/1', 'autorouting', 'smart');

    fprintf('  Simulation plant wired. IC = [0; 0].\n');
end

% Save model
mdl_path = fullfile(SEESAW_ROOT, 'models', [mdl '.slx']);
save_system(mdl, mdl_path);
fprintf('  Model saved: models/%s.slx\n', mdl);

% --- Run simulation if no QUARC ---
if ~quarc_available
    fprintf('  Running 10 s closed-loop simulation...\n');
    simout = sim(mdl, 'StopTime', '10');
    fprintf('  Simulation complete.\n');

    figure('Name', 'Simulink Cart PID Simulation', 'Position', [200 100 900 600]);

    if evalin('base', 'exist(''cart_xc'', ''var'')')
        cart_xc   = evalin('base', 'cart_xc');
        cart_xdot = evalin('base', 'cart_xdot');
        cart_Vm   = evalin('base', 'cart_Vm');

        subplot(3,1,1);
        plot(cart_xc.Time, cart_xc.Data * 100, 'b-', 'LineWidth', 1.5);
        hold on;
        % Plot reference (step at t=2 from 0 to 5 cm)
        t_ref = cart_xc.Time;
        r_ref = zeros(size(t_ref));
        r_ref(t_ref >= 2) = 5;
        plot(t_ref, r_ref, 'k--', 'LineWidth', 1);
        ylabel('x_c [cm]'); title('Cart Position');
        legend('Response', 'Reference', 'Location', 'SouthEast');
        grid on;

        subplot(3,1,2);
        plot(cart_xdot.Time, cart_xdot.Data, 'b-', 'LineWidth', 1.5);
        ylabel('v [m/s]'); title('Cart Velocity');
        yline(0, 'k--'); grid on;

        subplot(3,1,3);
        plot(cart_Vm.Time, cart_Vm.Data, 'r-', 'LineWidth', 1.5);
        ylabel('V_m [V]'); xlabel('Time [s]');
        title('Motor Voltage');
        yline(V_sat, 'k--', '+V_{sat}');
        yline(-V_sat, 'k--', '-V_{sat}');
        grid on;

        sgtitle('Simulink Cart PID: 5 cm Step', 'FontWeight', 'bold');
    else
        fprintf('  Note: Workspace variables not found — check To Workspace blocks.\n');
    end
end

%% 8. SUMMARY & SAVE

fprintf('\n');
fprintf('============================================================\n');
fprintf('  CART PID PIPELINE SUMMARY\n');
fprintf('============================================================\n');
fprintf('\n  DESIGN\n');
fprintf('  ------\n');
fprintf('  Plant:           IP02 cart on table (no seesaw)\n');
fprintf('  Method:          Interactive tuning via sisotool / pidtune\n');
fprintf('  Architecture:    Single-loop PID on cart position\n');
fprintf('  PID gains:       Kp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp_cart, Ki_cart, Kd_cart);
fprintf('  Deriv filter:    N = %d rad/s\n', N_filt);
fprintf('\n  CLOSED-LOOP POLES\n');
fprintf('  -----------------\n');
for k = 1:length(ev_cl)
    if abs(imag(ev_cl(k))) > 1e-6
        fprintf('  p = %.4f %+.4fi\n', real(ev_cl(k)), imag(ev_cl(k)));
    else
        fprintf('  p = %.4f\n', real(ev_cl(k)));
    end
end
fprintf('\n  MARGINS\n');
fprintf('  -------\n');
fprintf('  Phase margin:    %.1f deg at %.2f rad/s\n', Pm, wgc);
fprintf('  Gain margin:     %.1f dB at %.2f rad/s\n', 20*log10(Gm), wpc);
fprintf('\n  STEP RESPONSE (5 cm step)\n');
fprintf('  -------------------------\n');
fprintf('  Rise time:       %.3f s\n', t_rise);
fprintf('  Overshoot:       %.1f %%\n', os_pct);
fprintf('  Settling time:   %.2f s (2%% band)\n', t_settle);
fprintf('  SS error:        %.3f cm\n', ss_err);
fprintf('  Peak voltage:    %.2f V (limit %.1f V)\n', max(abs(Vm_sim)), V_sat);
fprintf('\n  SIMULINK MODEL\n');
fprintf('  --------------\n');
fprintf('  File:            models/IP02_CartPID.slx\n');
if quarc_available
    fprintf('  Mode:            QUARC hardware (500 Hz)\n');
else
    fprintf('  Mode:            Simulation only\n');
end
fprintf('============================================================\n');

% Save controller
save_path = fullfile(SEESAW_ROOT, 'data', 'controller_cart_pid.mat');
save(save_path, 'Kp_cart', 'Ki_cart', 'Kd_cart', 'N_filt', ...
    'K_aug', 'ev_cl', ...
    'Pm', 'Gm', 'wgc', 'wpc', 'A_cart', 'B_cart_ss', 'C_cart', 'D_cart', ...
    'A_aug', 'A_aug_cl', 't_rise', 'os_pct', 't_settle', 'ss_err');
fprintf('\n  Controller saved to: data/controller_cart_pid.mat\n');

%% 9. DEPLOY TO HARDWARE (QUARC)
%  -----------------------------------------------------------------------
%  Hardware deployment — much simpler than seesaw because there's no
%  instability. The cart won't run away if something goes wrong.
%  -----------------------------------------------------------------------

fprintf('\n============================================================\n');
fprintf('  §9: HARDWARE DEPLOYMENT\n');
fprintf('============================================================\n');

if ~quarc_available
    fprintf('\n  QUARC not available — skipping hardware deployment.\n');
    fprintf('  To deploy:\n');
    fprintf('    1. Install QUARC and add to MATLAB path\n');
    fprintf('    2. Re-run this pipeline\n');
    fprintf('============================================================\n');
else
    fprintf('\n  QUARC detected. Ready for hardware deployment.\n');
    fprintf('  Model: %s\n', mdl);
    fprintf('  PID:   Kp=%.3f  Ki=%.3f  Kd=%.3f\n', Kp_cart, Ki_cart, Kd_cart);
    fprintf('  Voltage limit: +/-%.1f V\n\n', V_sat);

    fprintf('  SAFETY CHECKLIST:\n');
    fprintf('    [ ] Q2-USB connected and powered\n');
    fprintf('    [ ] VoltPAQ-X1 powered, switch set to 1x\n');
    fprintf('    [ ] Seesaw module REMOVED or track locked flat\n');
    fprintf('    [ ] Cart near center of track\n');
    fprintf('    [ ] Emergency stop: QUARC | Stop or Ctrl+Break\n\n');

    % Step 1: Build
    fprintf('  STEP 1/4: Building QUARC target...\n');
    try
        rtwbuild(mdl);
        fprintf('  Build SUCCESSFUL.\n\n');
    catch ME
        fprintf('  BUILD FAILED: %s\n', ME.message);
        fprintf('============================================================\n');
        return;
    end

    % Step 2: Connect
    fprintf('  STEP 2/4: Connecting to Q2-USB...\n');
    reply = input('  Press ENTER to connect (or "q" to abort): ', 's');
    if strcmpi(reply, 'q')
        fprintf('  Aborted.\n');
        fprintf('============================================================\n');
        return;
    end
    try
        set_param(mdl, 'SimulationCommand', 'connect');
        fprintf('  Connected.\n\n');
    catch ME
        fprintf('  CONNECT FAILED: %s\n', ME.message);
        fprintf('============================================================\n');
        return;
    end

    % Step 3: Start
    fprintf('  STEP 3/4: Starting controller...\n');
    fprintf('  (Cart on table is stable — safe to start directly.)\n');
    reply = input('  Type "go" to START (or "q" to abort): ', 's');
    if ~strcmpi(reply, 'go')
        fprintf('  Aborted. Disconnecting...\n');
        try set_param(mdl, 'SimulationCommand', 'disconnect'); catch, end
        fprintf('============================================================\n');
        return;
    end

    try
        set_param(mdl, 'SimulationCommand', 'start');
        fprintf('  CONTROLLER RUNNING.\n');
        fprintf('  The cart should track the 5 cm step at t=2 s.\n');
        fprintf('  Watch the scopes for response quality.\n\n');
    catch ME
        fprintf('  START FAILED: %s\n', ME.message);
        fprintf('============================================================\n');
        return;
    end

    % Step 4: Stop
    fprintf('  STEP 4/4: Press ENTER to stop...\n');
    input('', 's');
    try
        set_param(mdl, 'SimulationCommand', 'stop');
        fprintf('  Controller STOPPED.\n');
    catch ME
        fprintf('  Stop failed: %s\n', ME.message);
    end

    % Data retrieval
    fprintf('\n  Logged data in workspace:\n');
    fprintf('    cart_xc   — position [m]\n');
    fprintf('    cart_xdot — velocity [m/s]\n');
    fprintf('    cart_Vm   — voltage [V]\n');
    fprintf('  Plot: plot(cart_xc.Time, cart_xc.Data*100)\n');
    fprintf('============================================================\n');
end

%% TUNING GUIDE
%  -----------------------------------------------------------------------
%  This pipeline uses sisotool for interactive PID tuning.
%
%  QUICK START:
%    1. Run §1 to load the plant
%    2. §2 opens sisotool with a pidtune starting point
%    3. Tune graphically — drag poles/zeros, shape Bode
%    4. Close sisotool → gains are extracted
%    5. §3-§8 run analysis, simulation, Simulink build, and deploy
%
%  SISOTOOL TIPS:
%    • Open-Loop Bode:  Drag the gain curve to set crossover frequency.
%                       Aim for phase margin 45-65 deg.
%    • Root Locus:      Drag closed-loop poles to desired locations.
%                       Keep all poles in the left half-plane.
%    • Step Response:   Watch overshoot and settling time live.
%    • Right-click on the Bode/RL to add/remove poles/zeros.
%
%  KEY CONSTRAINTS:
%    V_sat = 6 V   — keep bandwidth reasonable to avoid saturation
%    N_filt = 100   — derivative filter, reduce to 50 if noisy on hardware
%    x_c_max        — cart track is finite, don't overshoot too far
%
%  ALTERNATIVE TOOLS:
%    USE_PIDTUNER = true;   % in §2, to use the simpler pidTuner app
%    controlSystemDesigner(G_xc);  % another option (newer MATLAB)
%
%  RE-TUNING:
%    To re-tune without re-running the whole pipeline:
%      sisotool(G_xc)    % opens fresh session
%    Or load saved gains and modify:
%      load('data/controller_cart_pid.mat');
%      Kp_cart = <new_value>;  % etc.
%  -----------------------------------------------------------------------
