%% LQR CONTROL PIPELINE — Quanser IP02 + SEESAW-E
%  =====================================================================
%  LQR state-feedback design with integral action for seesaw balance.
%
%  Architecture: Full state feedback  u = -K * eta
%    eta = [x_c, x_c_dot, alpha, alpha_dot, integral(alpha)]
%    The integral of alpha eliminates steady-state tilt error.
%
%  Design method:
%    1. Augment the plant with an integrator on alpha
%    2. Design K via LQR (Q, R weight tuning)
%    3. Analyse loop transfer function, margins, robustness
%    4. Simulate disturbance response
%
%  Reference: Official Quanser Seesaw Laboratory Guide, Section 2.2
%    Quanser state ordering:  [x_c, theta, x_c_dot, theta_dot]
%    Our state ordering:      [x_c, x_c_dot, alpha, alpha_dot]
%    Augmented:               [x_c, x_c_dot, alpha, alpha_dot, int(alpha)]
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

ev = eig(A_sw);
fprintf('Open-loop poles:\n');
for k = 1:length(ev)
    if real(ev(k)) > 0.01
        fprintf('  lambda_%d = %+.4f   ← UNSTABLE\n', k, real(ev(k)));
    elseif abs(ev(k)) < 0.01
        fprintf('  lambda_%d = %+.4f   ← integrator\n', k, real(ev(k)));
    else
        fprintf('  lambda_%d = %+.4f\n', k, real(ev(k)));
    end
end
p_unstable = max(real(ev(real(ev) > 0.01)));

%% 2. AUGMENT WITH INTEGRAL OF ALPHA
%  -----------------------------------------------------------------------
%  The 4-state plant has no integrator on alpha, so a constant disturbance
%  (e.g. CG offset, track tilt) causes a steady-state angle error.
%  Adding eta_5 = integral(alpha) and penalising it in the LQR cost
%  drives alpha to zero in steady state.
%
%  Augmented system:
%    eta = [x_c; x_c_dot; alpha; alpha_dot; int(alpha)]
%    eta_dot = A_aug * eta + B_aug * u
%
%  where:
%    d/dt(int(alpha)) = alpha = C_alpha * x = [0 0 1 0] * x
%  -----------------------------------------------------------------------

C_alpha = [0 0 1 0];   % picks out alpha from the state vector

A_aug = [A_sw,         zeros(4,1);
         C_alpha,      0         ];

B_aug = [B_sw;
         0   ];

fprintf('\n=== Augmented System (5 states) ===\n');
fprintf('States: [x_c, x_c_dot, alpha, alpha_dot, int(alpha)]\n');
fprintf('A_aug size: %dx%d,  B_aug size: %dx%d\n', size(A_aug), size(B_aug));

% Verify augmented controllability
Co_aug = ctrb(A_aug, B_aug);
fprintf('Controllability rank: %d / %d\n', rank(Co_aug), size(A_aug,1));

%% 3. LQR DESIGN
%  -----------------------------------------------------------------------
%  Q and R are the main tuning knobs.
%
%  Q = diag([q_xc, q_xcdot, q_alpha, q_alphadot, q_int_alpha])
%    q_xc:        penalise cart displacement (centering)
%    q_xcdot:     penalise cart velocity (smooth motion)
%    q_alpha:     penalise seesaw angle (stabilisation) — MAIN KNOB
%    q_alphadot:  penalise angular velocity (damping)
%    q_int_alpha: penalise accumulated angle error (zero SS error)
%
%  R: penalise control effort. Larger R → gentler, slower control.
%
%  Quanser guide starting point (adapted to our state ordering):
%    Q = diag([1000, 0, 5000, 0, 2000]),  R = 0.5
%  -----------------------------------------------------------------------

% --- Tuning parameters (adjust these) ---
q_xc        = 1000;     % cart position weight
q_xcdot     = 0;        % cart velocity weight
q_alpha     = 5000;     % seesaw angle weight (dominant)
q_alphadot  = 0;        % angular velocity weight
q_int_alpha = 2000;     % integral alpha weight (kills SS error)
R_lqr       = 0.5;      % control effort weight

Q_lqr = diag([q_xc, q_xcdot, q_alpha, q_alphadot, q_int_alpha]);

fprintf('\n=== LQR Weight Selection ===\n');
fprintf('Q = diag([%.0f, %.0f, %.0f, %.0f, %.0f])\n', ...
    q_xc, q_xcdot, q_alpha, q_alphadot, q_int_alpha);
fprintf('R = %.1f\n', R_lqr);

% Compute LQR gain
[K_lqr, S_lqr, ev_lqr] = lqr(A_aug, B_aug, Q_lqr, R_lqr);

fprintf('\nLQR gain K = [%.4f,  %.4f,  %.4f,  %.4f,  %.4f]\n', K_lqr);
fprintf('              k_xc     k_xcdot   k_alpha   k_alphadot  k_int\n');

% Closed-loop poles
A_cl = A_aug - B_aug * K_lqr;
ev_cl = eig(A_cl);
fprintf('\nClosed-loop poles:\n');
for k = 1:length(ev_cl)
    if abs(imag(ev_cl(k))) > 1e-6
        fprintf('  %.4f %+.4fi  (|%.2f| rad/s, ζ=%.2f)\n', ...
            real(ev_cl(k)), imag(ev_cl(k)), abs(ev_cl(k)), ...
            -real(ev_cl(k))/abs(ev_cl(k)));
    else
        fprintf('  %.4f\n', real(ev_cl(k)));
    end
end

if any(real(ev_cl) > 1e-3)
    error('Closed-loop is UNSTABLE. Adjust Q/R.');
end

%% 4. LOOP TRANSFER FUNCTION ANALYSIS
%  SIMO loop TF: L(s) = K * (sI - A_aug)^{-1} * B_aug

sys_loop = ss(A_aug, B_aug, K_lqr, 0);
L = tf(sys_loop);

fprintf('\n=== Loop Transfer Function ===\n');
fprintf('Poles: '); fprintf('%.4f  ', pole(L)); fprintf('\n');
fprintf('Zeros: '); fprintf('%.4f  ', zero(L)); fprintf('\n');

z_loop = zero(L);
rhp_z = z_loop(real(z_loop) > 0.01);
if isempty(rhp_z)
    fprintf('No RHP zeros in loop TF.\n');
else
    fprintf('RHP zeros: '); fprintf('%.4f  ', rhp_z); fprintf('\n');
end

% Margins
[Gm, Pm, wpc, wgc] = margin(L);
fprintf('\nStability margins:\n');
fprintf('  Phase margin:  PM = %.1f deg  at ω = %.2f rad/s\n', Pm, wgc);
fprintf('  Gain margin:   GM = %.1f dB   at ω = %.2f rad/s\n', 20*log10(Gm), wpc);

% LQR guarantees: PM ≥ 60°, GM ∈ [-6 dB, +∞) at plant input
fprintf('\nLQR guaranteed margins (single-input): PM ≥ 60°, GM ≥ -6 dB\n');
if Pm >= 59  % allow small numerical tolerance
    fprintf('  PM = %.1f° — meets guarantee.\n', Pm);
else
    fprintf('  PM = %.1f° — WARNING: below LQR guarantee.\n', Pm);
end

omega = logspace(-1, 2.5, 500);

figure('Name', 'LQR Loop Analysis', 'Position', [50 50 1200 800]);

subplot(2,2,1);
margin(L);
title(sprintf('Bode: L(s) — PM=%.1f°, GM=%.1f dB', Pm, 20*log10(Gm)));
grid on;

subplot(2,2,2);
nyquist(L, omega);
hold on;
plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2);
title('Nyquist');
grid on; xlim([-3 1]); ylim([-3 3]);

subplot(2,2,3);
rlocus(L);
title('Root Locus');
xline(0, 'k--'); sgrid(0.5, []); grid on;

subplot(2,2,4);
sys_cl_full = ss(A_cl, B_aug, [eye(4), zeros(4,1)], zeros(4,1));
pzmap(sys_cl_full);
title('Closed-Loop Pole-Zero Map');
sgrid; grid on;

sgtitle('LQR Loop Transfer Function Analysis', 'FontWeight', 'bold');

%% 5. COMPARE WITH POLE PLACEMENT (Pipeline 1)
%  Load the frequency-based controller for side-by-side comparison.

freq_file = fullfile(SEESAW_ROOT, 'data', 'controller_freq.mat');
if exist(freq_file, 'file')
    freq = load(freq_file);
    fprintf('\n=== Pipeline Comparison ===\n');
    fprintf('                      Pole Placement    LQR\n');
    fprintf('  Phase margin [deg]     %5.1f           %5.1f\n', freq.Pm, Pm);
    fprintf('  Gain margin [dB]       %5.1f           %5.1f\n', ...
        20*log10(freq.Gm), 20*log10(Gm));
    fprintf('  Crossover [rad/s]      %5.2f           %5.2f\n', freq.wgc, wgc);
    fprintf('  Integral action?        No              Yes\n');
    fprintf('  Number of gains:        4               5\n');
else
    fprintf('\n(Pipeline 1 results not found — run control_pipeline.m first)\n');
end

%% 6. DISTURBANCE SIMULATION — INITIAL TILT
%  4.5° initial seesaw tilt, system recovers.

T_sim = 10;
dt    = 0.001;
t_sim = (0:dt:T_sim)';

% Initial condition: 4.5° tilt, integrator starts at zero
x0_aug = [0; 0; 4.5*pi/180; 0; 0];

[t_out, x_out] = ode45(@(t,x) A_cl*x, t_sim, x0_aug);
u_out = -K_lqr * x_out';

xc_cl    = x_out(:,1) * 1000;       % [mm]
alpha_cl = x_out(:,3) * 180/pi;     % [deg]
int_alpha = x_out(:,5) * 180/pi;    % [deg*s]
Vm_cl    = u_out';                   % [V]

figure('Name', 'LQR: Initial Tilt Response', 'Position', [100 100 900 800]);

subplot(4,1,1);
plot(t_out, xc_cl, 'b-', 'LineWidth', 1.5);
ylabel('x_c [mm]'); title('Cart Position');
yline(0, 'k--'); grid on;

subplot(4,1,2);
plot(t_out, alpha_cl, 'g-', 'LineWidth', 1.5);
ylabel('\alpha [deg]'); title('Seesaw Angle');
yline(0, 'k--'); grid on;

subplot(4,1,3);
plot(t_out, int_alpha, 'm-', 'LineWidth', 1.5);
ylabel('\int\alpha dt [deg·s]'); title('Integral of Alpha (drives SS error to zero)');
yline(0, 'k--'); grid on;

subplot(4,1,4);
plot(t_out, Vm_cl, 'r-', 'LineWidth', 1.5);
ylabel('V_m [V]'); xlabel('Time [s]');
title('Motor Voltage');
yline(V_sat, 'k--', '+V_{sat}');
yline(-V_sat, 'k--', '-V_{sat}');
grid on;

sgtitle('LQR: 4.5° Initial Tilt Response', 'FontWeight', 'bold');

%% 7. REPEATED DISTURBANCE TEST
%  Pulse train: 4.5° taps every 4 seconds (Quanser guide procedure).

T_sim2 = 20;
t_sim2 = (0:dt:T_sim2)';

B_dist_aug = zeros(5,1);
B_dist_aug(4) = 1;   % disturbance enters alpha_dot

u_dist = zeros(length(t_sim2), 1);
pulse_times = [1, 5, 9, 13, 17];
pulse_dur   = 0.16;
pulse_amp   = 0.08 / pulse_dur;
for pt = pulse_times
    u_dist(t_sim2 >= pt & t_sim2 < pt + pulse_dur) = pulse_amp;
end

[t_out2, x_out2] = ode45(@(t,x) A_cl*x + B_dist_aug*interp1(t_sim2, u_dist, t, 'previous', 0), ...
    t_sim2, zeros(5,1));
u_out2 = -K_lqr * x_out2';

figure('Name', 'LQR: Repeated Disturbance', 'Position', [150 80 900 700]);

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

sgtitle('LQR: Repeated 4.5° Disturbance (every 4s)', 'FontWeight', 'bold');

%% 8. HARDWARE FEASIBILITY

V_peak_ic   = max(abs(Vm_cl));
V_peak_dist = max(abs(u_out2(:)));
V_peak      = max(V_peak_ic, V_peak_dist);
V_margin    = V_sat - V_peak;
xc_peak     = max(max(abs(xc_cl)), max(abs(x_out2(:,1)))*1000);

fprintf('\n=== Hardware Feasibility ===\n');
fprintf('  V_sat:          %.1f V\n', V_sat);
fprintf('  V_peak (sim):   %.2f V\n', V_peak);
fprintf('  V margin:       %.2f V\n', V_margin);
if V_peak > V_sat
    fprintf('  *** SATURATION! Increase R or reduce Q weights.\n');
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
%  Check robustness to ±30% B_eq variation.

fprintf('\n=== Sensitivity to B_eq Uncertainty ===\n');
fprintf('  B_eq variation    PM [deg]    GM [dB]    Max Re(eig)\n');
B_eq_test = B_eq * [0.7, 0.85, 1.0, 1.15, 1.3];
for i = 1:length(B_eq_test)
    B_t_i = B_eq_test(i) + B_emf;
    G_rhs_i = [0, -B_t_i, -g*M_c, 0; -g*M_c, 0, g*(M_c*D_T+M_SW*D_C), -B_SW];
    A_i = [0,1,0,0; M_inv(1,:)*G_rhs_i; 0,0,0,1; M_inv(2,:)*G_rhs_i];
    A_aug_i = [A_i, zeros(4,1); C_alpha, 0];
    A_cl_i = A_aug_i - B_aug * K_lqr;
    ev_i = eig(A_cl_i);
    max_re = max(real(ev_i));
    L_i = tf(ss(A_aug_i, B_aug, K_lqr, 0));
    [Gm_i, Pm_i] = margin(L_i);
    stab_str = '';
    if max_re > 0.01, stab_str = '← UNSTABLE'; end
    fprintf('  B_eq = %5.2f (%+3.0f%%)   %5.1f       %5.1f       %+.4f  %s\n', ...
        B_eq_test(i), (B_eq_test(i)/B_eq - 1)*100, Pm_i, 20*log10(Gm_i), ...
        max_re, stab_str);
end

%% 10. Q/R TUNING GUIDE
%  -----------------------------------------------------------------------
%  Use this section to interactively tune the LQR weights.
%  Recompute K_lqr by adjusting the diag(Q) values in Section 3.
%
%  Quick reference:
%    Increase q_alpha     → faster angle stabilisation, more voltage
%    Increase q_xc        → tighter cart centering, may fight alpha loop
%    Increase q_int_alpha → eliminates SS error faster, may overshoot
%    Increase R           → gentler control, slower response
%
%  On-the-fly tuning (Quanser guide approach):
%    K = lqr(A_aug, B_aug, diag([q_xc, q_xcdot, q_alpha, q_alphadot, q_int_alpha]), R);
%  -----------------------------------------------------------------------

%% 11. SUMMARY & SAVE

% Settling time from initial tilt sim
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
fprintf('  LQR CONTROL PIPELINE SUMMARY\n');
fprintf('============================================================\n');
fprintf('\n  DESIGN\n');
fprintf('  ──────\n');
fprintf('  Method:          LQR with integral action on alpha\n');
fprintf('  Feedback:        Full state + integral (SIMO, 5 gains)\n');
fprintf('  Q = diag([%.0f, %.0f, %.0f, %.0f, %.0f])\n', ...
    q_xc, q_xcdot, q_alpha, q_alphadot, q_int_alpha);
fprintf('  R = %.1f\n', R_lqr);
fprintf('  K = [%.3f, %.3f, %.3f, %.3f, %.3f]\n', K_lqr);
fprintf('       k_xc    k_xcdot  k_alpha  k_adot   k_int\n');
fprintf('\n  CLOSED-LOOP POLES\n');
fprintf('  ─────────────────\n');
for k = 1:length(ev_cl)
    if abs(imag(ev_cl(k))) > 1e-6 && imag(ev_cl(k)) > 0
        fprintf('  p = %.4f ± %.4fi  (|%.2f|, ζ=%.2f)\n', ...
            real(ev_cl(k)), abs(imag(ev_cl(k))), abs(ev_cl(k)), ...
            -real(ev_cl(k))/abs(ev_cl(k)));
    elseif imag(ev_cl(k)) == 0 || (abs(imag(ev_cl(k))) > 1e-6 && imag(ev_cl(k)) < 0)
        if imag(ev_cl(k)) >= 0
            fprintf('  p = %.4f\n', real(ev_cl(k)));
        end
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
fprintf('  SS angle error:  %.4f deg (integral action)\n', alpha_cl(end));
fprintf('============================================================\n');

% Save
save_path = fullfile(SEESAW_ROOT, 'data', 'controller_lqr.mat');
save(save_path, 'K_lqr', 'Q_lqr', 'R_lqr', ...
    'A_aug', 'B_aug', 'A_cl', ...
    'Pm', 'Gm', 'wgc', 'wpc', ...
    'A_sw', 'B_sw', 'C_sw', 'D_sw', ...
    'p_unstable');
fprintf('\n  Controller saved to: data/controller_lqr.mat\n');
fprintf('============================================================\n');
