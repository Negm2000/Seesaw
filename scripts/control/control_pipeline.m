%% CONTROL PIPELINE — Quanser IP02 + SEESAW-E
%  =====================================================================
%  Classical frequency-domain design for seesaw balance control.
%
%  Architecture (cascade):
%    Inner loop: Lead compensator  — seesaw angle   alpha  → 0
%    Outer loop: Integral + prop   — cart position  x_c    → 0
%
%  State convention (ours):  x = [x_c,  x_c_dot,  alpha,  alpha_dot]
%  Good ref convention:       x = [x_c,  theta,    x_c_dot, theta_dot]
%  These differ — all output indices below use OUR ordering.
%
%  Prerequisites: run modeling_pipeline.m first to produce tuned_params.mat
%  =====================================================================

%% 1. LOAD TUNED MODEL
%  Pull in tuned B_eq, both state-space models, and derived scalars.

if ~exist('SEESAW_ROOT', 'var')
    SEESAW_ROOT = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
tp = fullfile(SEESAW_ROOT, 'data', 'tuned_params.mat');
if ~exist(tp, 'file')
    error('tuned_params.mat not found — run modeling_pipeline.m first.');
end
load(tp);           % loads A_sw, B_sw, C_sw, D_sw, B_eq, B_total, alpha_f, eta_g, ...
seesaw_params;      % re-loads hardware constants (K_a, V_sat, M_c, etc.)
B_eq = load(tp).B_eq;   % override seesaw_params nominal with tuned value

fprintf('\n=== State-Space Model (tuned, B_eq = %.3f N*s/m) ===\n', B_eq);
fprintf('State ordering: [x_c,  x_c_dot,  alpha,  alpha_dot]\n');
fprintf('Output indices:   1       2          3         4\n\n');

% Open-loop eigenvalues
ev   = eig(A_sw);
p_u  = ev(real(ev) > 0.01);        % RHP pole(s) — expect one near +2.15 rad/s
p_int = ev(abs(ev) < 0.01);        % near-zero pole (cart integrator)
ev_stable = ev(real(ev) < -0.01);  % stable poles

fprintf('Open-loop poles:\n');
for k = 1:length(ev)
    if real(ev(k)) > 0.01
        fprintf('  lambda = %+.4f   ← UNSTABLE (gravity)\n', real(ev(k)));
    elseif abs(ev(k)) < 0.01
        fprintf('  lambda = %+.4f   ← integrator (cart free drift)\n', real(ev(k)));
    else
        fprintf('  lambda = %+.4f\n', real(ev(k)));
    end
end

%% 2. EXTRACT SISO PLANTS
%  Build full ss system and pull out the two SISO plants we need.
%  C_sw = eye(4): outputs are exactly the states in order.
%
%  G_alpha: V_m  → alpha    (the seesaw angle plant — UNSTABLE)
%  G_xc:   V_m  → x_c      (cart position plant — stable but with integrator)

sys_full = ss(A_sw, B_sw, C_sw, D_sw);

G_alpha = sys_full(3, 1);   % output 3 = alpha,   input 1 = V_m
G_xc    = sys_full(1, 1);   % output 1 = x_c,     input 1 = V_m

% Reduce: cancel any near pole-zero pairs (cart integrator may cancel in G_alpha)
G_alpha_r = minreal(G_alpha, 0.1);
G_xc_r    = minreal(G_xc,    0.1);

fprintf('\n--- G_alpha: V_m → alpha ---\n');
fprintf('Poles: '); fprintf('%.4f  ', pole(G_alpha_r)); fprintf('\n');
fprintf('Zeros: '); fprintf('%.4f  ', zero(G_alpha_r)); fprintf('\n');
fprintf('--- G_xc: V_m → x_c ---\n');
fprintf('Poles: '); fprintf('%.4f  ', pole(G_xc_r)); fprintf('\n');
fprintf('Zeros: '); fprintf('%.4f  ', zero(G_xc_r)); fprintf('\n');

%% 3. OPEN-LOOP ANALYSIS
%  Bode plots of both plants so we understand what we are designing for.

omega = logspace(-1, 2, 500);   % 0.1 to 100 rad/s

figure('Name', 'Open-Loop Plant Analysis', 'Position', [50 50 1100 700]);

subplot(2,2,1);
bode(G_alpha_r, omega); grid on;
title('G_\alpha(s): V_m \rightarrow \alpha   [UNSTABLE plant]');

subplot(2,2,2);
bode(G_xc_r, omega); grid on;
title('G_{xc}(s): V_m \rightarrow x_c');

subplot(2,2,3);
pzmap(G_alpha_r); grid on; zplane([], []);
title('Pole-Zero: G_\alpha');
sgrid;

subplot(2,2,4);
pzmap(G_xc_r); grid on;
title('Pole-Zero: G_{xc}');
sgrid;

sgtitle('Open-Loop Plant Analysis');

% --- Key numbers ---
p_unstable = real(p_u(1));   % the RHP pole (e.g. ~2.15 rad/s)
fprintf('\nUnstable pole magnitude: p_u = %.4f rad/s\n', p_unstable);
fprintf('Minimum viable crossover: omega_c > 4*p_u = %.2f rad/s\n', 4*p_unstable);

%% 4. INNER LOOP: LEAD COMPENSATOR FOR SEESAW ANGLE
%  ---------------------------------------------------------------------------
%  DESIGN RATIONALE
%  ---------------------------------------------------------------------------
%  G_alpha has a RHP pole at p_u ≈ 2.15 rad/s (gravity drives instability).
%
%  We use a lead compensator:
%    C_inner(s) = K_c * (s + z_c) / (s + p_c),   with z_c < p_c
%
%  Design choices (tune these three numbers):
%    z_c : zero placed just below p_u. In the root locus, this zero "attracts"
%          the unstable pole and pulls it into the LHP.
%          Rule: z_c ≈ 0.85 * p_u
%
%    p_c : limits high-frequency gain amplification (noise).
%          Choose p_c such that lead ratio α = z_c/p_c gives ~55° maximum
%          phase lead at the geometric mean sqrt(z_c*p_c).
%          α = (1 - sin(φ_max))/(1 + sin(φ_max))
%          For φ_max = 55°: α ≈ 0.099
%          → p_c = z_c / α
%
%    K_c : set so that |C_inner(jω_c) * G_alpha(jω_c)| = 1 at the desired
%          gain crossover frequency ω_c.
%          Rule for unstable plant: ω_c ≥ 5*p_u (conservative).
%  ---------------------------------------------------------------------------

% --- Parameters (tune here) ---
phi_max  = 55 * pi/180;                     % desired max phase lead [rad]
alpha_c  = (1 - sin(phi_max)) / (1 + sin(phi_max));   % lead ratio
z_c      = 0.85 * p_unstable;              % lead zero
p_c      = z_c / alpha_c;                  % lead pole
omega_c  = 5.5 * p_unstable;              % target gain crossover [rad/s]

% Compute K_c to achieve crossover at omega_c
C_shape    = (1j*omega_c + z_c) / (1j*omega_c + p_c);  % compensator shape at omega_c
G_at_wc    = evalfr(G_alpha_r, 1j*omega_c);             % plant at omega_c
K_c        = 1 / abs(C_shape * G_at_wc);                % gain for |C*G|=1 at omega_c

% Build the compensator transfer function
s = tf('s');
C_inner = K_c * (s + z_c) / (s + p_c);

fprintf('\n=== Inner Loop: Lead Compensator ===\n');
fprintf('  z_c     = %.3f rad/s  (zero, ≈ 0.85 × p_u)\n', z_c);
fprintf('  p_c     = %.3f rad/s  (pole, noise rolloff)\n', p_c);
fprintf('  K_c     = %.4f\n', K_c);
fprintf('  α       = %.4f  (lead ratio, max phase = %.1f°)\n', alpha_c, phi_max*180/pi);
fprintf('  ω_c     = %.3f rad/s  (target crossover)\n', omega_c);
fprintf('  ω_c/p_u = %.1f×  (should be ≥ 4-5)\n', omega_c/p_unstable);

% --- Open-loop transfer function ---
L_inner = C_inner * G_alpha_r;

%% 5. STABILITY VERIFICATION — Margins + Nyquist
%  For an unstable plant (P = 1 open-loop RHP pole), the Nyquist criterion
%  requires that the loop gain L(s) = C(s)*G(s) encircle -1+0j exactly
%  P = 1 time counterclockwise (CCW).
%
%  Standard gain/phase margin is still meaningful IF the Nyquist plot
%  crosses the real axis at -1 in a simple, monotonic way.

[Gm_dB, Pm_deg, wpc, wgc] = margin(L_inner);

fprintf('\n=== Stability Margins ===\n');
fprintf('  Phase margin:  PM = %.1f deg  at ω = %.2f rad/s\n', Pm_deg, wgc);
fprintf('  Gain margin:   GM = %.1f dB   at ω = %.2f rad/s\n', 20*log10(Gm_dB), wpc);

if Pm_deg < 30
    fprintf('  WARNING: PM < 30 deg — likely too aggressive for hardware.\n');
elseif Pm_deg > 70
    fprintf('  NOTE: PM > 70 deg — conservative, may be sluggish.\n');
else
    fprintf('  OK: PM in acceptable range [30, 70] deg.\n');
end

if 20*log10(Gm_dB) < 6
    fprintf('  WARNING: GM < 6 dB — may be fragile to parameter variations.\n');
else
    fprintf('  OK: GM >= 6 dB.\n');
end

% --- Plots ---
figure('Name', 'Inner Loop Stability', 'Position', [100 100 1100 500]);

subplot(1,2,1);
margin(L_inner);
title(sprintf('Bode: L_{inner}(s) = C_{inner} \\cdot G_\\alpha\nPM = %.1f° at %.2f rad/s | GM = %.1f dB', ...
    Pm_deg, wgc, 20*log10(Gm_dB)));
grid on;

subplot(1,2,2);
nyquist(L_inner, omega);
title(sprintf('Nyquist: L_{inner}(s)\nNeed 1 CCW encirclement of −1 (P=1 open-loop RHP pole)'));
xline(-1, 'r--', '-1+0j'); grid on;
xlim([-3 1]); ylim([-3 3]);

% --- Root locus (gain perspective) ---
figure('Name', 'Root Locus: Inner Loop', 'Position', [150 150 600 500]);
rlocus(C_inner * G_alpha_r);
title('Root Locus: C_{inner}(s) \cdot G_\alpha(s)');
xline(0, 'k--'); grid on;
sgrid(0.5, []);   % show 50% damping guideline

%% 6. INNER LOOP CLOSED-LOOP PERFORMANCE
%  Close the inner loop and verify:
%    1. All closed-loop poles are in the LHP (stable)
%    2. Step response is well-damped
%    3. Bandwidth is within actuator limits

T_inner = feedback(L_inner, 1);    % CL: alpha/alpha_ref

fprintf('\n=== Inner Closed-Loop Poles ===\n');
p_cl = pole(T_inner);
for k = 1:length(p_cl)
    if real(p_cl(k)) > 0
        fprintf('  *** UNSTABLE *** pole at %.4f + %.4fi\n', real(p_cl(k)), imag(p_cl(k)));
    else
        fprintf('  %.4f + %.4fi\n', real(p_cl(k)), imag(p_cl(k)));
    end
end

if any(real(p_cl) > 0)
    error('Inner loop closed-loop is UNSTABLE. Increase K_c or adjust z_c/p_c.');
end

% Step response (alpha reference step — useful for checking damping)
t_step = 0:0.01:5;
[y_step, t_step] = step(T_inner, t_step);
[~, idx_pk] = max(y_step);
overshoot = (max(y_step) - y_step(end)) / y_step(end) * 100;
idx_ss = find(abs(y_step - y_step(end)) < 0.02 * abs(y_step(end)), 1, 'first');
t_settle = t_step(idx_ss);

fprintf('\n  Step response (inner loop, 2%% criterion):\n');
fprintf('  Overshoot:    %.1f%%\n', overshoot);
fprintf('  Settling time: %.2f s\n', t_settle);

figure('Name', 'Inner Loop Step Response', 'Position', [200 200 800 400]);
plot(t_step, y_step * 180/pi, 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\alpha [deg]');
title('Inner Closed-Loop: Step Response (alpha\_ref = 1 rad ≈ 57°)');
yline(y_step(end)*180/pi, 'k--');
yline(y_step(end)*180/pi * 1.02, 'g:');
yline(y_step(end)*180/pi * 0.98, 'g:');
grid on;

%% 7. OUTER LOOP: CART CENTERING
%  ---------------------------------------------------------------------------
%  Once the inner loop is closed, the seesaw is balanced, but the cart drifts.
%  The outer loop feeds back cart position x_c through the inner closed loop
%  to slowly center the cart.
%
%  Architecture:
%    alpha_ref(s) = -C_outer(s) * x_c(s)
%    V_m(s)       = C_inner(s) * (alpha_ref - alpha)
%
%  Outer loop plant: x_c / alpha_ref with inner loop closed.
%    G_outer(s) = (x_c / V_m) / (alpha / V_m) * T_inner(s)
%    ... more precisely: use the closed-loop input-output map from alpha_ref to x_c.
%  ---------------------------------------------------------------------------

% Build SISO system: alpha_ref → x_c (with inner loop closed)
% Inner loop: V_m = C_inner*(alpha_ref - alpha)
% So x_c/alpha_ref = G_xc * C_inner / (1 + C_inner*G_alpha)
G_outer_plant = G_xc_r * C_inner / (1 + C_inner * G_alpha_r);
G_outer_plant = minreal(G_outer_plant, 0.1);

fprintf('\n=== Outer Loop Plant (alpha_ref → x_c) ===\n');
fprintf('Poles: '); fprintf('%.4f  ', pole(G_outer_plant)); fprintf('\n');
fprintf('Zeros: '); fprintf('%.4f  ', zero(G_outer_plant)); fprintf('\n');

% --- Outer compensator: PI for zero steady-state cart error ---
% The plant alpha_ref → x_c has a double integrator (position integrates velocity,
% velocity integrates alpha through the inner loop). A simple proportional gain
% can work, but a slow PI helps eliminate DC drift from imperfect balance.
%
% Keep outer bandwidth << inner bandwidth to maintain cascade separation.
omega_outer = omega_c / 10;   % outer crossover ~10x slower than inner

% Start with proportional only and compute gain
G_outer_at_wo = evalfr(G_outer_plant, 1j*omega_outer);
K_outer        = 1 / abs(G_outer_at_wo);   % P gain for crossover at omega_outer

% Add slow integral for DC: C_outer = K_outer * (s + omega_outer/3) / s
% (zero at omega_outer/3 ensures PM is not wrecked by the integrator)
omega_i_zero = omega_outer / 3;
C_outer = K_outer * (s + omega_i_zero) / s;

L_outer = C_outer * G_outer_plant;
[Gm_o_dB, Pm_o_deg, ~, wgc_o] = margin(L_outer);

fprintf('\n=== Outer Loop Compensator (PI) ===\n');
fprintf('  K_outer    = %.4f\n', K_outer);
fprintf('  omega_i    = %.4f rad/s  (integral zero)\n', omega_i_zero);
fprintf('  omega_c    = %.4f rad/s  (outer crossover)\n', wgc_o);
fprintf('  PM_outer   = %.1f deg\n', Pm_o_deg);
fprintf('  GM_outer   = %.1f dB\n', 20*log10(Gm_o_dB));

figure('Name', 'Outer Loop', 'Position', [250 50 1000 450]);
subplot(1,2,1);
margin(L_outer); grid on;
title(sprintf('Outer Loop Bode: PM = %.1f°, GM = %.1f dB', Pm_o_deg, 20*log10(Gm_o_dB)));
subplot(1,2,2);
T_outer_xc = feedback(L_outer, 1);
step(T_outer_xc, 0:0.01:20); grid on;
title('Outer CL: x_c step response');
ylabel('x_c [m]'); xlabel('Time [s]');
sgtitle('Outer Loop (Cart Centering)');

%% 8. FULL CLOSED-LOOP SIMULATION
%  Simulate the cascade-controlled system from initial conditions
%  to replicate Good ref Fig 3.3: 4.5° seesaw disturbance, system recovers.
%
%  Full closed-loop (both loops) state-space:
%  u = C_inner*(alpha_ref - alpha) where alpha_ref = -C_outer*x_c
%  → u = -C_inner*alpha - C_inner*C_outer*x_c
%
%  We use the augmented state-space approach for the simulation.

fprintf('\n=== Building Augmented Closed-Loop System ===\n');

% Convert compensators to state-space for augmentation
sys_Ci = ss(C_inner);
sys_Co = ss(C_outer);

% The full system has states: [plant states | Ci states | Co states]
% Plant:  dx = A_sw*x + B_sw*u
% Ci:     dc_i = A_ci*c_i + B_ci*(alpha_ref - alpha)
%         u    = C_ci*c_i + D_ci*(alpha_ref - alpha)
% Co:     dc_o = A_co*c_o + B_co*x_c
%         alpha_ref = -(C_co*c_o + D_co*x_c)
%
% Substituting: alpha_ref = -(C_co*c_o + D_co*x_c)
%               u = C_ci*c_i + D_ci*(alpha_ref - alpha)
%                 = C_ci*c_i - D_ci*(C_co*c_o + D_co*x_c) - D_ci*alpha

n_p  = size(A_sw, 1);           % 4 plant states
n_ci = size(sys_Ci.A, 1);       % Ci states
n_co = size(sys_Co.A, 1);       % Co states
n    = n_p + n_ci + n_co;

% Extract SS matrices
Ap = A_sw;  Bp = B_sw;
Aci = sys_Ci.A; Bci = sys_Ci.B; Cci = sys_Ci.C; Dci = sys_Ci.D;
Aco = sys_Co.A; Bco = sys_Co.B; Cco = sys_Co.C; Dco = sys_Co.D;

% Output indices from plant
i_xc    = 1;   % x_c
i_alpha = 3;   % alpha

Cp_xc    = C_sw(i_xc,    :);   % [1 0 0 0]
Cp_alpha = C_sw(i_alpha, :);   % [0 0 1 0]

% Control law: u = Cci*ci - Dci*(Dco*Cp_xc*x + Cco*co) - Dci*Cp_alpha*x
%                = -[Dci*Dco*Cp_xc + Dci*Cp_alpha]*x - Dci*Cco*co + Cci*ci
% Co input: ci_dot = Aci*ci + Bci*(alpha_ref - alpha)
%         = Aci*ci + Bci*(-(Dco*Cp_xc*x + Cco*co) - Cp_alpha*x)
%         = Aci*ci - Bci*(Dco*Cp_xc + Cp_alpha)*x - Bci*Cco*co

% Co state: co_dot = Aco*co + Bco*x_c = Aco*co + Bco*Cp_xc*x

% Build augmented A matrix (n x n)
u_from_x  = -(Dci * Dco * Cp_xc + Dci * Cp_alpha);   % 1 x n_p
u_from_co = -Dci * Cco;                                % 1 x n_co
u_from_ci =  Cci;                                       % 1 x n_ci

A_aug = zeros(n, n);
% Plant rows (1:n_p)
A_aug(1:n_p, 1:n_p)            = Ap + Bp * u_from_x;
A_aug(1:n_p, n_p+1:n_p+n_ci)  = Bp * u_from_ci;
A_aug(1:n_p, n_p+n_ci+1:end)  = Bp * u_from_co;
% Ci rows (n_p+1:n_p+n_ci)
A_aug(n_p+1:n_p+n_ci, 1:n_p)            = -Bci * (Dco * Cp_xc + Cp_alpha);
A_aug(n_p+1:n_p+n_ci, n_p+1:n_p+n_ci)  = Aci;
A_aug(n_p+1:n_p+n_ci, n_p+n_ci+1:end)  = -Bci * Cco;
% Co rows (n_p+n_ci+1:end)
A_aug(n_p+n_ci+1:end, 1:n_p)            = Bco * Cp_xc;
A_aug(n_p+n_ci+1:end, n_p+n_ci+1:end)  = Aco;

% Output: [x_c; alpha; u] for plotting
C_out = [Cp_xc,    zeros(1,n_ci), zeros(1,n_co);   % x_c
         Cp_alpha, zeros(1,n_ci), zeros(1,n_co);   % alpha
         u_from_x, u_from_ci,    u_from_co    ];   % V_m

% Disturbance input (added directly to alpha_dot = state 4)
B_dist_aug = zeros(n, 1);
B_dist_aug(4) = 1;   % impulse-like kick to alpha_dot

sys_cl = ss(A_aug, B_dist_aug, C_out, zeros(3,1));

% Check closed-loop poles
p_cl_full = eig(A_aug);
n_rhp = sum(real(p_cl_full) > 1e-3);
fprintf('Augmented CL poles: %d total, %d RHP\n', length(p_cl_full), n_rhp);
if n_rhp > 0
    warning('Closed-loop has %d RHP pole(s) — controller is not stabilising!', n_rhp);
end

% --- Simulate disturbance: 4.5° tap at t = 1 s ---
% Good ref: pulse of amplitude 0.08 rad applied for ~0.2 s (Rate Limiter 0.5 rad/s,
% 4.5° = 0.0785 rad). We model this as a short force pulse on alpha_dot.
T_sim = 10;
dt    = 0.001;
t_sim = (0:dt:T_sim)';
u_dist = zeros(length(t_sim), 1);
% Short velocity kick: 0.08 rad/s for 0.1 s starting at t=1 s
u_dist(t_sim >= 1.0 & t_sim < 1.1) = 0.08 / 0.1;   % rad/s^2 impulse → 0.08 rad/s total

[y_cl, ~] = lsim(sys_cl, u_dist, t_sim);
xc_cl    = y_cl(:,1);   % [m]
alpha_cl = y_cl(:,2);   % [rad]
Vm_cl    = y_cl(:,3);   % [V]

figure('Name', 'Closed-Loop Disturbance Response', 'Position', [100 100 900 700]);

subplot(3,1,1);
plot(t_sim, xc_cl*1000, 'b-', 'LineWidth', 1.5);
ylabel('x_c [mm]');
title('Cart Position');
yline(0, 'k--');
grid on;

subplot(3,1,2);
plot(t_sim, alpha_cl*180/pi, 'g-', 'LineWidth', 1.5);
ylabel('\alpha [deg]');
title('Seesaw Angle');
yline(0, 'k--');
yline(11.5, 'r:', 'Hard stop +11.5°');
yline(-11.5, 'r:', 'Hard stop -11.5°');
grid on;

subplot(3,1,3);
plot(t_sim, Vm_cl, 'r-', 'LineWidth', 1.5);
ylabel('V_m [V]');
xlabel('Time [s]');
title('Motor Voltage');
yline(V_sat, 'k--', '+V_{sat}');
yline(-V_sat, 'k--', '-V_{sat}');
grid on;

sgtitle('Closed-Loop Response to 4.5° Disturbance', 'FontWeight', 'bold');

%% 9. VOLTAGE SATURATION CHECK
%  The VoltPAQ hard-clips at ±V_sat = ±22 V. If the simulated voltage exceeds
%  this, the linear model is optimistic — real hardware will show nonlinear
%  behaviour and likely fail to recover.

V_peak   = max(abs(Vm_cl));
V_margin = V_sat - V_peak;

fprintf('\n=== Voltage Saturation Check ===\n');
fprintf('  V_sat (hardware):  %.1f V\n', V_sat);
fprintf('  V_peak (simulated): %.2f V\n', V_peak);
fprintf('  Margin:            %.2f V\n', V_margin);

if V_peak > V_sat
    fprintf('  *** SATURATION! Reduce K_c or reduce disturbance amplitude.\n');
elseif V_margin < 3
    fprintf('  WARNING: Margin < 3 V — hardware may saturate on large disturbances.\n');
else
    fprintf('  OK: Well within saturation limits.\n');
end

% Also check cart travel
xc_peak_mm = max(abs(xc_cl)) * 1000;
track_half  = x_c_max * 1000;
fprintf('\n  Peak cart travel:  %.1f mm (limit = ±%.0f mm)\n', xc_peak_mm, track_half);
if xc_peak_mm > track_half * 0.8
    fprintf('  WARNING: Cart approaching end-stops.\n');
else
    fprintf('  OK: Cart well within track limits.\n');
end

%% 10. PERFORMANCE SUMMARY & SAVE

% Find settling time for alpha (2% criterion, post-disturbance)
post_dist = t_sim > 1.5;   % start looking after disturbance ends
alpha_post = alpha_cl(post_dist);
t_post     = t_sim(post_dist);
ss_alpha   = alpha_post(end);
idx_set    = find(abs(alpha_post - ss_alpha) < 0.02 * 11.5*pi/180, 1, 'first');
t_settle_alpha = t_post(idx_set);

% Max angle deviation
max_alpha_deg = max(abs(alpha_cl(t_sim > 1.0))) * 180/pi;

fprintf('\n');
fprintf('============================================================\n');
fprintf('  CONTROL PIPELINE SUMMARY\n');
fprintf('============================================================\n');
fprintf('\n  INNER LOOP (angle stabilisation)\n');
fprintf('  ─────────────────────────────────\n');
fprintf('  Compensator:     Lead  K_c*(s+%.3f)/(s+%.3f)\n', z_c, p_c);
fprintf('  K_c:             %.4f\n', K_c);
fprintf('  Crossover ω_c:   %.2f rad/s  (%.2f × p_u)\n', wgc, wgc/p_unstable);
fprintf('  Phase margin:    %.1f deg\n', Pm_deg);
fprintf('  Gain margin:     %.1f dB\n', 20*log10(Gm_dB));
fprintf('\n  OUTER LOOP (cart centering)\n');
fprintf('  ─────────────────────────────────\n');
fprintf('  Compensator:     PI  K_outer*(s+%.4f)/s\n', omega_i_zero);
fprintf('  K_outer:         %.4f\n', K_outer);
fprintf('  Crossover ω_c:   %.3f rad/s\n', wgc_o);
fprintf('  Phase margin:    %.1f deg\n', Pm_o_deg);
fprintf('\n  SIMULATION (4.5° disturbance)\n');
fprintf('  ─────────────────────────────────\n');
fprintf('  Max angle:       %.2f deg\n', max_alpha_deg);
fprintf('  Settling time:   %.2f s (2%% of ±11.5° band)\n', t_settle_alpha);
fprintf('  Peak voltage:    %.2f V  (limit = %.1f V)\n', V_peak, V_sat);
fprintf('  Peak cart disp:  %.1f mm (limit = ±%.0f mm)\n', xc_peak_mm, track_half);
fprintf('============================================================\n');

% --- Save controller data ---
save_path = fullfile(SEESAW_ROOT, 'data', 'controller.mat');
save(save_path, ...
    'C_inner', 'C_outer', ...
    'z_c', 'p_c', 'K_c', ...
    'omega_c', 'omega_outer', 'K_outer', ...
    'Pm_deg', 'Gm_dB', 'Pm_o_deg', ...
    'G_alpha_r', 'G_xc_r', ...
    'L_inner', 'T_inner');

fprintf('\n  Controller saved to: data/controller.mat\n');
fprintf('  Next: build_simulink_models → deploys Seesaw_Control.slx\n');
fprintf('============================================================\n');
