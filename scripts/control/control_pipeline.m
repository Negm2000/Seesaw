%% CONTROL PIPELINE — Quanser IP02 + SEESAW-E
%  =====================================================================
%  Frequency-domain state-feedback design for seesaw balance control.
%
%  Architecture: Full state feedback  u = -K * x
%    Both encoders (cart position + seesaw angle) are used.
%    Velocities are obtained via differentiating + filtering.
%
%  Design method:
%    1. Pole placement — choose desired CL pole locations
%    2. Analyse the SIMO loop transfer function L(s) = K*(sI-A)^{-1}*B
%       via Bode, Nyquist, root locus, and gain/phase margins
%    3. Iterate on pole locations to satisfy performance specs
%
%  State convention (ours):  x = [x_c,  x_c_dot,  alpha,  alpha_dot]
%  Good ref convention:       x = [x_c,  theta,    x_c_dot, theta_dot]
%  These differ — all indices below use OUR ordering.
%
%  Prerequisites: run modeling_pipeline.m first to produce tuned_params.mat
%  =====================================================================

%% 1. LOAD TUNED MODEL

if ~exist('SEESAW_ROOT', 'var')
    SEESAW_ROOT = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
tp = fullfile(SEESAW_ROOT, 'data', 'tuned_params.mat');
if ~exist(tp, 'file')
    error('tuned_params.mat not found — run modeling_pipeline.m first.');
end
seesaw_params;
tuned = load(tp);
B_eq = tuned.B_eq;

% Recompute derived damping and rebuild state-space with tuned B_eq
B_emf   = alpha_f * K_g * k_m / r_mp;
B_total = B_eq + B_emf;

M_eff = [M_c,          -M_c*D_T;
         -M_c*D_T,      J_pivot + M_c*D_T^2];
M_inv = inv(M_eff);
G_rhs = [0, -B_total,  -g*M_c,                        0;
         -g*M_c, 0,     g*(M_c*D_T + M_SW*D_C),  -B_SW];
A_sw = [0, 1, 0, 0; M_inv(1,:)*G_rhs; 0, 0, 0, 1; M_inv(2,:)*G_rhs];
G_inp = [alpha_f*eta_m; 0];
B_sw = [0; M_inv(1,:)*G_inp; 0; M_inv(2,:)*G_inp];
C_sw = eye(4);
D_sw = zeros(4,1);

fprintf('\n=== Tuned Plant (B_eq = %.3f N*s/m) ===\n', B_eq);
fprintf('State ordering: [x_c, x_c_dot, alpha, alpha_dot]\n\n');

% Open-loop eigenvalues
ev = eig(A_sw);
fprintf('Open-loop poles:\n');
for k = 1:length(ev)
    if real(ev(k)) > 0.01
        fprintf('  lambda_%d = %+.4f   ← UNSTABLE\n', k, real(ev(k)));
    elseif abs(ev(k)) < 0.01
        fprintf('  lambda_%d = %+.4f   ← integrator (cart drift)\n', k, real(ev(k)));
    else
        fprintf('  lambda_%d = %+.4f\n', k, real(ev(k)));
    end
end

p_unstable = max(real(ev(real(ev) > 0.01)));
fprintf('\nUnstable pole: %.4f rad/s (%.2f Hz)\n', p_unstable, p_unstable/(2*pi));

% Controllability check
Co = ctrb(A_sw, B_sw);
fprintf('Controllability rank: %d / %d\n', rank(Co), size(A_sw,1));

%% 2. SISO PLANT TRANSFER FUNCTIONS
%  Examine the individual SISO plants to understand the open-loop behaviour.
%  Both sensors are available — we will use both in the feedback.

sys_full = ss(A_sw, B_sw, C_sw, D_sw);
G_alpha = sys_full(3,1);   % V_m → alpha
G_xc    = sys_full(1,1);   % V_m → x_c

fprintf('\n=== SISO Plant Analysis ===\n');
fprintf('G_alpha (V_m → alpha): poles '); fprintf('%.3f  ', pole(G_alpha)); fprintf('\n');
fprintf('G_alpha zeros:                '); fprintf('%.3f  ', zero(G_alpha)); fprintf('\n');
z_rhp = zero(G_alpha); z_rhp = z_rhp(real(z_rhp) > 0);
if ~isempty(z_rhp)
    fprintf('  RHP zero at %.2f rad/s — blocks SISO-only design on alpha\n', z_rhp(1));
    fprintf('  → Using full state feedback (both encoders) eliminates this zero\n');
end
fprintf('G_xc (V_m → x_c): poles      '); fprintf('%.3f  ', pole(G_xc)); fprintf('\n');
fprintf('G_xc zeros:                   '); fprintf('%.3f  ', zero(G_xc)); fprintf('\n');

%% 3. POLE PLACEMENT DESIGN
%  -----------------------------------------------------------------------
%  Design K = [k_xc, k_xcdot, k_alpha, k_alphadot] via pole placement.
%
%  Desired pole selection rationale:
%    - The unstable pole at +2.61 rad/s must be moved into the LHP.
%    - Dominant poles: a complex pair with ω_n ≈ 4 rad/s, ζ ≈ 0.7
%      → settling time ≈ 4/(ζ*ω_n) ≈ 1.4 s, moderate overshoot.
%    - Two faster real poles to handle the cart dynamics and the fast
%      motor pole (which is already at ~-36.5 rad/s).
%
%  These are starting values — Section 5 iterates based on margins.
%  -----------------------------------------------------------------------

% Desired closed-loop poles
wn_dom   = 3.0;     % dominant natural frequency [rad/s]
zeta_dom = 0.7;     % dominant damping ratio
p_dom = -zeta_dom * wn_dom + 1j * wn_dom * sqrt(1 - zeta_dom^2);

p_desired = [p_dom;              % dominant complex pair
             conj(p_dom);
             -2.5;               % real pole: cart centering
             -20];               % fast pole: moderate (saves voltage)

fprintf('\n=== Pole Placement Design ===\n');
fprintf('Desired CL poles:\n');
for k = 1:length(p_desired)
    if imag(p_desired(k)) ~= 0
        fprintf('  p_%d = %.4f %+.4fi  (ω_n=%.1f, ζ=%.2f)\n', k, ...
            real(p_desired(k)), imag(p_desired(k)), abs(p_desired(k)), ...
            -real(p_desired(k))/abs(p_desired(k)));
    else
        fprintf('  p_%d = %.4f\n', k, real(p_desired(k)));
    end
end

K = place(A_sw, B_sw, p_desired);

fprintf('\nState feedback gains:\n');
fprintf('  K = [%.4f,  %.4f,  %.4f,  %.4f]\n', K);
fprintf('       k_xc     k_xcdot   k_alpha   k_alphadot\n');

% Verify closed-loop poles
A_cl = A_sw - B_sw * K;
ev_cl = eig(A_cl);
fprintf('\nActual CL poles (verify = desired):\n');
for k = 1:length(ev_cl)
    if abs(imag(ev_cl(k))) > 1e-6
        fprintf('  %.4f %+.4fi\n', real(ev_cl(k)), imag(ev_cl(k)));
    else
        fprintf('  %.4f\n', real(ev_cl(k)));
    end
end

if any(real(ev_cl) > 1e-3)
    error('Closed-loop is UNSTABLE. Adjust desired poles.');
end

%% 4. LOOP TRANSFER FUNCTION ANALYSIS
%  -----------------------------------------------------------------------
%  The SIMO loop transfer function (breaking the loop at the plant input):
%    L(s) = K * (sI - A)^{-1} * B
%
%  This is SISO (1 input, 1 output after K combines the 4 states), so we
%  can analyse it with standard Bode, Nyquist, and root locus tools.
%
%  KEY INSIGHT: L(s) uses all 4 states → its zeros differ from the SISO
%  G_alpha zeros. The RHP zero that blocked the cascade design disappears.
%  -----------------------------------------------------------------------

sys_loop = ss(A_sw, B_sw, K, 0);
L = tf(sys_loop);

fprintf('\n=== Loop Transfer Function L(s) = K*(sI-A)^{-1}*B ===\n');
fprintf('Poles: '); fprintf('%.4f  ', pole(L)); fprintf('\n');
fprintf('Zeros: '); fprintf('%.4f  ', zero(L)); fprintf('\n');

z_loop = zero(L);
rhp_z = z_loop(real(z_loop) > 0.01);
if isempty(rhp_z)
    fprintf('No RHP zeros — full state feedback eliminated the transmission zero.\n');
else
    fprintf('WARNING: RHP zeros remain at '); fprintf('%.4f  ', rhp_z); fprintf('\n');
end

% Gain and phase margins
[Gm, Pm, wpc, wgc] = margin(L);
fprintf('\nStability margins:\n');
fprintf('  Phase margin:  PM = %.1f deg  at ω = %.2f rad/s\n', Pm, wgc);
fprintf('  Gain margin:   GM = %.1f dB   at ω = %.2f rad/s\n', 20*log10(Gm), wpc);

if Pm >= 30 && Pm <= 70
    fprintf('  PM is in the acceptable range [30, 70] deg.\n');
elseif Pm < 30
    fprintf('  WARNING: PM < 30 deg — may be too aggressive.\n');
elseif Pm > 70
    fprintf('  NOTE: PM > 70 deg — conservative, may be sluggish.\n');
end

% --- Bode plot ---
omega = logspace(-1, 2.5, 500);

figure('Name', 'Loop TF Analysis', 'Position', [50 50 1200 800]);

subplot(2,2,1);
margin(L);
title(sprintf('Bode: L(s) — PM = %.1f°, GM = %.1f dB', Pm, 20*log10(Gm)));
grid on;

% --- Nyquist plot ---
subplot(2,2,2);
nyquist(L, omega);
hold on;
plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2);
title('Nyquist: L(s)');
grid on;
xlim([-3 1]); ylim([-3 3]);

% --- Root locus (gain sensitivity) ---
subplot(2,2,3);
rlocus(L);
title('Root Locus: L(s) with varying gain');
xline(0, 'k--');
sgrid(0.5, []);
grid on;

% --- Pole-zero map of CL ---
subplot(2,2,4);
sys_cl = ss(A_cl, B_sw, C_sw, D_sw);
pzmap(sys_cl);
title('Closed-Loop Pole-Zero Map');
sgrid;
grid on;

sgtitle('Frequency-Domain Analysis of State Feedback Loop', 'FontWeight', 'bold');

%% 5. PERFORMANCE TUNING
%  -----------------------------------------------------------------------
%  Adjust the desired poles from Section 3 and re-run to iterate.
%  Use these guidelines:
%
%    To increase PM:  reduce ω_n (slower dominant poles)
%    To reduce overshoot: increase ζ
%    To speed up settling: increase ω_n (but watch PM and voltage)
%    To improve cart centering: move the real pole further left
%
%  The fast pole (~-36) should stay near the plant's existing fast pole
%  to avoid requiring excessive control effort.
%  -----------------------------------------------------------------------

fprintf('\n=== Performance Specs ===\n');

% Dominant pole specs
wn_actual = abs(ev_cl(abs(imag(ev_cl)) > 0.01));
if ~isempty(wn_actual)
    wn_actual = wn_actual(1);
    zeta_actual = -real(ev_cl(abs(imag(ev_cl)) > 0.01));
    zeta_actual = zeta_actual(1) / wn_actual;
    t_settle_est = 4 / (zeta_actual * wn_actual);
    fprintf('  Dominant poles: ω_n = %.2f rad/s, ζ = %.2f\n', wn_actual, zeta_actual);
    fprintf('  Estimated settling time: %.2f s (4/(ζ*ω_n))\n', t_settle_est);
end
fprintf('  Crossover frequency: %.2f rad/s (%.2f Hz)\n', wgc, wgc/(2*pi));
fprintf('  Crossover / p_unstable: %.1f×\n', wgc / p_unstable);

%% 6. DISTURBANCE SIMULATION
%  Replicate Good ref Fig 3.3: 4.5° seesaw tap, system recovers.
%  Disturbance modelled as a short angular velocity pulse on alpha_dot.

T_sim = 10;
dt    = 0.001;
t_sim = (0:dt:T_sim)';

% Initial condition: 4.5° seesaw tilt
x0 = [0; 0; 4.5*pi/180; 0];

% Simulate CL: x_dot = (A - B*K)*x
[t_out, x_out] = ode45(@(t,x) A_cl*x, t_sim, x0);
u_out = -K * x_out';   % control signal [V]

xc_cl    = x_out(:,1) * 1000;    % [mm]
alpha_cl = x_out(:,3) * 180/pi;  % [deg]
Vm_cl    = u_out';                % [V]

figure('Name', 'Disturbance Response (Pole Placement)', 'Position', [100 100 900 700]);

subplot(3,1,1);
plot(t_out, xc_cl, 'b-', 'LineWidth', 1.5);
ylabel('x_c [mm]'); title('Cart Position');
yline(0, 'k--'); grid on;

subplot(3,1,2);
plot(t_out, alpha_cl, 'g-', 'LineWidth', 1.5);
ylabel('\alpha [deg]'); title('Seesaw Angle');
yline(0, 'k--');
yline(11.5, 'r:', 'Hard stop +11.5°');
yline(-11.5, 'r:', 'Hard stop -11.5°');
grid on;

subplot(3,1,3);
plot(t_out, Vm_cl, 'r-', 'LineWidth', 1.5);
ylabel('V_m [V]'); xlabel('Time [s]');
title('Motor Voltage');
yline(V_sat, 'k--', '+V_{sat}');
yline(-V_sat, 'k--', '-V_{sat}');
grid on;

sgtitle('Closed-Loop Response: 4.5° Initial Tilt (Pole Placement)', 'FontWeight', 'bold');

%% 7. REPEATED DISTURBANCE TEST
%  Pulse train: 4.5° taps every 4 seconds (matches Quanser guide setup).

T_sim2 = 20;
t_sim2 = (0:dt:T_sim2)';
n2 = length(t_sim2);

% Disturbance: angular velocity pulse on alpha_dot (state 4)
% 0.08 rad pulse, rate-limited to 0.5 rad/s → 0.16 s duration
B_dist = zeros(4,1);
B_dist(4) = 1;   % disturbance enters alpha_dot

u_dist = zeros(n2, 1);
pulse_times = [1, 5, 9, 13, 17];
pulse_dur   = 0.16;  % seconds
pulse_amp   = 0.08 / pulse_dur;  % rad/s^2 → total impulse = 0.08 rad/s
for pt = pulse_times
    u_dist(t_sim2 >= pt & t_sim2 < pt + pulse_dur) = pulse_amp;
end

% Simulate with disturbance input
A_cl_dist = A_cl;
[t_out2, x_out2] = ode45(@(t,x) A_cl_dist*x + B_dist*interp1(t_sim2, u_dist, t, 'previous', 0), ...
    t_sim2, zeros(4,1));
u_out2 = -K * x_out2';

figure('Name', 'Repeated Disturbance Test', 'Position', [150 80 900 700]);

subplot(3,1,1);
plot(t_out2, x_out2(:,1)*1000, 'b-', 'LineWidth', 1.5);
ylabel('x_c [mm]'); title('Cart Position');
yline(0, 'k--'); grid on;

subplot(3,1,2);
plot(t_out2, x_out2(:,3)*180/pi, 'g-', 'LineWidth', 1.5);
ylabel('\alpha [deg]'); title('Seesaw Angle');
yline(0, 'k--'); grid on;

subplot(3,1,3);
plot(t_out2, u_out2, 'r-', 'LineWidth', 1.5);
ylabel('V_m [V]'); xlabel('Time [s]');
title('Motor Voltage');
yline(V_sat, 'k--', '+V_{sat}');
yline(-V_sat, 'k--', '-V_{sat}');
grid on;

sgtitle('Repeated 4.5° Disturbance (every 4s)', 'FontWeight', 'bold');

%% 8. VOLTAGE SATURATION & TRAVEL CHECK
%  Check both the initial-tilt test (Section 6) and the repeated
%  disturbance test (Section 7) for worst-case voltage and travel.

V_peak_ic   = max(abs(Vm_cl));                         % from initial tilt
V_peak_dist = max(abs(u_out2(:)));                     % from repeated disturbance
V_peak      = max(V_peak_ic, V_peak_dist);
V_margin    = V_sat - V_peak;
xc_peak_ic   = max(abs(xc_cl));                        % [mm] from initial tilt
xc_peak_dist = max(abs(x_out2(:,1))) * 1000;           % [mm] from disturbance
xc_peak      = max(xc_peak_ic, xc_peak_dist);

fprintf('\n=== Hardware Feasibility ===\n');
fprintf('  V_sat:          %.1f V\n', V_sat);
fprintf('  V_peak (sim):   %.2f V\n', V_peak);
fprintf('  V margin:       %.2f V\n', V_margin);
if V_peak > V_sat
    fprintf('  *** SATURATION! Reduce gains or relax pole placement.\n');
else
    fprintf('  OK: Within voltage limits.\n');
end

fprintf('\n  Peak cart travel:  %.1f mm (limit ±%.0f mm)\n', xc_peak, x_c_max*1000);
if xc_peak > x_c_max * 1000 * 0.8
    fprintf('  WARNING: Cart approaching end-stops.\n');
else
    fprintf('  OK: Cart well within track.\n');
end

%% 9. SENSITIVITY ANALYSIS
%  Check robustness: how do margins change if B_eq varies ±30%?

fprintf('\n=== Sensitivity to B_eq Uncertainty ===\n');
fprintf('  B_eq variation    PM [deg]    GM [dB]    Max Re(eig)\n');
B_eq_test = B_eq * [0.7, 0.85, 1.0, 1.15, 1.3];
for i = 1:length(B_eq_test)
    B_t_i = B_eq_test(i) + B_emf;
    G_rhs_i = [0, -B_t_i, -g*M_c, 0; -g*M_c, 0, g*(M_c*D_T+M_SW*D_C), -B_SW];
    A_i = [0,1,0,0; M_inv(1,:)*G_rhs_i; 0,0,0,1; M_inv(2,:)*G_rhs_i];
    A_cl_i = A_i - B_sw * K;   % same K, different plant
    ev_i = eig(A_cl_i);
    max_re = max(real(ev_i));
    L_i = tf(ss(A_i, B_sw, K, 0));
    [Gm_i, Pm_i] = margin(L_i);
    stab_str = '';
    if max_re > 0.01, stab_str = '← UNSTABLE'; end
    fprintf('  B_eq = %5.2f (%+3.0f%%)   %5.1f       %5.1f       %+.4f  %s\n', ...
        B_eq_test(i), (B_eq_test(i)/B_eq - 1)*100, Pm_i, 20*log10(Gm_i), ...
        max_re, stab_str);
end

%% 10. SUMMARY & SAVE

% Settling time from simulation (2% of 4.5°)
alpha_sim = x_out(:,3);
band_2pct = 0.02 * 4.5 * pi/180;
idx_settle = find(abs(alpha_sim) > band_2pct, 1, 'last');
if ~isempty(idx_settle) && idx_settle < length(t_out)
    t_settle_sim = t_out(idx_settle);
else
    t_settle_sim = 0;
end

max_alpha = max(abs(alpha_cl));

fprintf('\n');
fprintf('============================================================\n');
fprintf('  FREQUENCY-BASED CONTROL PIPELINE SUMMARY\n');
fprintf('============================================================\n');
fprintf('\n  DESIGN\n');
fprintf('  ──────\n');
fprintf('  Method:          Pole placement + frequency-domain verification\n');
fprintf('  Feedback:        Full state (both encoders, SIMO)\n');
fprintf('  Gains K:         [%.3f, %.3f, %.3f, %.3f]\n', K);
fprintf('                    k_xc    k_xcdot  k_alpha  k_alphadot\n');
fprintf('\n  DESIRED POLES\n');
fprintf('  ─────────────\n');
fprintf('  Dominant pair:   ω_n = %.1f rad/s, ζ = %.2f\n', wn_dom, zeta_dom);
for k = 1:length(p_desired)
    if imag(p_desired(k)) ~= 0 && imag(p_desired(k)) > 0
        fprintf('  p = %.4f ± %.4fi\n', real(p_desired(k)), abs(imag(p_desired(k))));
    elseif imag(p_desired(k)) == 0
        fprintf('  p = %.4f\n', real(p_desired(k)));
    end
end
fprintf('\n  MARGINS\n');
fprintf('  ───────\n');
fprintf('  Phase margin:    %.1f deg at %.2f rad/s\n', Pm, wgc);
fprintf('  Gain margin:     %.1f dB at %.2f rad/s\n', 20*log10(Gm), wpc);
fprintf('\n  SIMULATION (4.5° initial tilt)\n');
fprintf('  ──────────────────────────────\n');
fprintf('  Max angle:       %.2f deg\n', max_alpha);
fprintf('  Settling time:   %.2f s (2%% band)\n', t_settle_sim);
fprintf('  Peak voltage:    %.2f V (limit %.1f V)\n', max(abs(Vm_cl)), V_sat);
fprintf('  Peak cart disp:  %.1f mm\n', max(abs(xc_cl)));
fprintf('============================================================\n');

% Save
save_path = fullfile(SEESAW_ROOT, 'data', 'controller_freq.mat');
save(save_path, 'K', 'p_desired', 'wn_dom', 'zeta_dom', ...
    'Pm', 'Gm', 'wgc', 'wpc', 'A_sw', 'B_sw', 'C_sw', 'D_sw', ...
    'A_cl', 'p_unstable');
fprintf('\n  Controller saved to: data/controller_freq.mat\n');
fprintf('============================================================\n');
