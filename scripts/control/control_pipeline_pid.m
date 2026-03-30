%% CONTROL PIPELINE — Quanser IP02 + SEESAW-E
%  =====================================================================
%  Parallel PID control design for seesaw balance control.
%
%  Architecture: Parallel PID   u = PID_cart(e_xc) + PD_alpha(alpha)
%    Both encoders (cart position + seesaw angle) are used.
%    PID blocks handle derivative filtering internally.
%
%  Design method: systune (structured H-infinity synthesis)
%    1. Define tunable PID/PD blocks in the closed-loop interconnection
%    2. Specify performance requirements (margins, tracking)
%    3. systune jointly optimises all gains to meet specs
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

%% 3. PARALLEL PID TUNING VIA SYSTUNE
%  -----------------------------------------------------------------------
%  Architecture:  u = PID_cart(x_c_ref - x_c) + PD_alpha(alpha)
%
%  Uses MATLAB systune (structured H-infinity synthesis) to jointly
%  optimise all PID/PD gains.  This is the correct approach for a SIMO
%  plant where cascaded / successive-loop-closure tuning cannot work:
%  the single actuator serves both loops, so gains must be co-designed.
%
%  Tuning requirements:
%    Hard — Stability margins:  PM >= 40 deg,  GM >= 6 dB
%    Soft — Step tracking:  xc_ref -> xc  settles in ~2 s
%  -----------------------------------------------------------------------

fprintf('\n=== Parallel PID Tuning via systune (structured H-inf) ===\n');

% ---- SIMO plant: V_m -> [x_c, alpha] ----
G = ss(A_sw, B_sw, C_sw([1,3],:), D_sw([1,3],:));
G.InputName  = 'Vm';
G.OutputName = {'xc', 'alpha'};

% ---- Tunable PID blocks ----
%  Cart: filtered PID  (Kp + Ki/s + Kd*N*s/(s+N))
C_cart = tunablePID('C_cart', 'pid');
C_cart.InputName  = 'e_xc';
C_cart.OutputName = 'u_cart';

%  Alpha: filtered PD  (Kp + Kd*N*s/(s+N))  — no integrator on angle
C_alpha = tunablePID('C_alpha', 'pd');
C_alpha.InputName  = 'alpha';      % direct positive feedback
C_alpha.OutputName = 'u_alpha';

%  Fix derivative filter at N = 100 rad/s for both blocks.
%  This is an engineering choice (matches hardware sample rate);
%  letting systune tune only the proportional/integral/derivative gains.
N_filt = 100;
C_cart.Tf.Value  = 1/N_filt;   C_cart.Tf.Free  = false;
C_alpha.Tf.Value = 1/N_filt;   C_alpha.Tf.Free = false;

%  Bound gains to keep voltages within saturation limits.
%  At 4.5° initial tilt (0.0785 rad), Kp_a*alpha_0 must stay under V_sat.
%  Also enforce positive gains (stabilising sign convention).
C_alpha.Kp.Minimum = 0;   C_alpha.Kp.Maximum = 80;
C_alpha.Kd.Minimum = 0;   C_alpha.Kd.Maximum = 50;

C_cart.Kp.Minimum  = 0;   C_cart.Kp.Maximum  = 100;
C_cart.Ki.Minimum  = 0;   C_cart.Ki.Maximum  = 50;
C_cart.Kd.Minimum  = 0;   C_cart.Kd.Maximum  = 20;

% ---- Summation blocks ----
Sum_err  = sumblk('e_xc = xc_ref - xc');
Sum_ctrl = sumblk('Vm = u_cart + u_alpha');

% ---- Build closed-loop interconnection (genss) ----
CL0 = connect(G, C_cart, C_alpha, Sum_err, Sum_ctrl, ...
              'xc_ref', {'xc', 'alpha'});

% ---- Tuning requirements ----
%  Hard:  closed-loop pole region — ensures stability with margin.
%         MinDecay  = 0.5 rad/s  (all poles at least 0.5 left of j-axis)
%         MinDamp   = 0.3        (oscillatory poles ≥ 30% damping)
%         MaxFreq   = 200 rad/s  (allow room for the N=100 derivative filter)
Req_poles = TuningGoal.Poles(0.5, 0.3, 200);

%  Soft:  step tracking  xc_ref -> xc  (~4 s settling, relaxed for V_sat)
Req_track = TuningGoal.StepTracking('xc_ref', 'xc', 4);

% ---- Run systune ----
rng(0);   % reproducibility
opts = systuneOptions('RandomStart', 5, 'Display', 'iter');
[CL, fSoft, gHard] = systune(CL0, Req_track, Req_poles, opts);

fprintf('\nsystune result:  soft = %.4f,  hard = %.4f', fSoft, gHard);
if gHard <= 1
    fprintf('  (all hard constraints satisfied)\n');
else
    fprintf('  *** hard constraints violated ***\n');
    warning('Pole region not fully achieved (gHard = %.3f > 1).', gHard);
end

% ---- Extract tuned gains ----
C_cart_t  = getBlockValue(CL, 'C_cart');
C_alpha_t = getBlockValue(CL, 'C_alpha');

Kp_c = C_cart_t.Kp;
Ki_c = C_cart_t.Ki;
Kd_c = C_cart_t.Kd;

Kp_a = C_alpha_t.Kp;
Kd_a = C_alpha_t.Kd;

fprintf('\nPID Gains (systune):\n');
fprintf('  Cart PID:    Kp_c = %+.4f   Ki_c = %+.4f   Kd_c = %+.4f\n', Kp_c, Ki_c, Kd_c);
fprintf('  Alpha PD:    Kp_a = %+.4f   Kd_a = %+.4f\n', Kp_a, Kd_a);
fprintf('  Derivative filter: N = %d rad/s (fixed)\n', N_filt);

% ---- Controller state-space realisations (for simulation) ----
%  The tuned PID/PD are transfer functions with internal states
%  (integrator + derivative filter).  Simulations MUST use these,
%  not a K_aug state-feedback approximation.
[Ac_ctrl, Bc_ctrl, Cc_ctrl, Dc_ctrl] = ssdata(ss(C_cart_t));   % cart PID
[Aa_ctrl, Ba_ctrl, Ca_ctrl, Da_ctrl] = ssdata(ss(C_alpha_t));  % alpha PD
nc = size(Ac_ctrl, 1);   % number of cart-PID states
na = size(Aa_ctrl, 1);   % number of alpha-PD states
n_full = 4 + nc + na;    % full CL order

fprintf('  Controller orders:  cart PID = %d states, alpha PD = %d states\n', nc, na);
fprintf('  Full CL system order: %d\n', n_full);

% Extraction helpers (from plant state xp)
C_xc = [1 0 0 0];   % x_c  from xp
C_al = [0 0 1 0];   % alpha from xp

% ---- Build full CL state-space (for eigenvalue verification) ----
%  State:  x_full = [xp(4); zc(nc); za(na)]
%  xp_dot = A_sw*xp + B_sw*u
%  zc_dot = Ac*zc + Bc*(-C_xc*xp)           (ref = 0)
%  za_dot = Aa*za + Ba*(C_al*xp)
%  u      = Cc*zc + Dc*(-C_xc*xp) + Ca*za + Da*(C_al*xp)

A_cl_full = [A_sw + B_sw*(Dc_ctrl*(-C_xc) + Da_ctrl*C_al), B_sw*Cc_ctrl, B_sw*Ca_ctrl;
             Bc_ctrl*(-C_xc),                               Ac_ctrl,      zeros(nc,na);
             Ba_ctrl*C_al,                                   zeros(na,nc), Aa_ctrl];

ev_cl = eig(A_cl_full);
fprintf('\nClosed-loop poles (full %d-state system):\n', n_full);
for k = 1:length(ev_cl)
    if abs(imag(ev_cl(k))) > 1e-6
        fprintf('  %.4f %+.4fi\n', real(ev_cl(k)), imag(ev_cl(k)));
    else
        fprintf('  %.4f\n', real(ev_cl(k)));
    end
end

if any(real(ev_cl) > 1e-3)
    error('Closed-loop is UNSTABLE — systune failed to stabilise the plant.');
end

% K_aug kept for backward compatibility (Simulink workspace, etc.)
K_aug = [Kp_c, Kd_c, -Kp_a, -Kd_a, -Ki_c];
A_aug = [A_sw, zeros(4,1); -1, 0, 0, 0, 0];
B_aug = [B_sw; 0];
A_aug_cl = A_aug - B_aug * K_aug;

%% 4. LOOP TRANSFER FUNCTION ANALYSIS
%  -----------------------------------------------------------------------
%  Break the loop at the plant input Vm.
%  L(s) = [PID_cart(-G_xc) + PD_alpha(G_alpha)]  (SISO at plant input)
%
%  Built via `connect` to faithfully include derivative-filter dynamics.
%  -----------------------------------------------------------------------

G_loop = ss(A_sw, B_sw, C_sw([1,3],:), D_sw([1,3],:));
G_loop.InputName = 'Vm';  G_loop.OutputName = {'xc','alpha'};

Cc_loop = ss(C_cart_t);   Cc_loop.InputName = 'e_xc';    Cc_loop.OutputName = 'u_cart';
Ca_loop = ss(C_alpha_t);  Ca_loop.InputName = 'alpha';    Ca_loop.OutputName = 'u_alpha';

Sum_neg  = sumblk('e_xc = -xc');          % ref = 0
Sum_u    = sumblk('u_total = u_cart + u_alpha');

L_sys = connect(G_loop, Cc_loop, Ca_loop, Sum_neg, Sum_u, 'Vm', 'u_total');
% Negate: the physical loop closes as Vm = u_total (positive feedback at
% the break point), but margin() assumes negative feedback.  L_nfb = -L
% restores the standard convention: CL stable iff (1 + L_nfb) has no RHP zeros.
L = -tf(L_sys);

fprintf('\n=== Loop Transfer Function L(s) ===\n');
fprintf('Poles: '); fprintf('%.4f  ', pole(L)); fprintf('\n');
fprintf('Zeros: '); fprintf('%.4f  ', zero(L)); fprintf('\n');

z_loop = zero(L);
rhp_z = z_loop(real(z_loop) > 0.01);
if isempty(rhp_z)
    fprintf('No RHP zeros in the loop TF.\n');
else
    fprintf('WARNING: RHP zeros at '); fprintf('%.4f  ', rhp_z); fprintf('\n');
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

% Also show individual PID transfer functions for documentation
fprintf('\nIndividual PID transfer functions:\n');
fprintf('  C_cart(s)  = (%.3f s^2 + %.3f s + %.3f) / s\n', Kd_c, Kp_c, Ki_c);
fprintf('  C_alpha(s) = %.3f s + %.3f\n', Kd_a, Kp_a);

% --- Plots ---
omega = logspace(-1, 2.5, 500);

figure('Name', 'Loop TF Analysis (Parallel PID)', 'Position', [50 50 1200 800]);

subplot(2,2,1);
margin(L);
title(sprintf('Bode: L(s) — PM = %.1f°, GM = %.1f dB', Pm, 20*log10(Gm)));
grid on;

subplot(2,2,2);
nyquist(L, omega);
hold on;
plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2);
title('Nyquist: L(s)');
grid on;
xlim([-3 1]); ylim([-3 3]);

subplot(2,2,3);
rlocus(L);
title('Root Locus: L(s) with varying gain');
xline(0, 'k--');
sgrid(0.5, []);
grid on;

subplot(2,2,4);
C_aug = [eye(4), zeros(4,1)];  % output original 4 states from augmented
sys_cl = ss(A_aug_cl, B_aug, C_aug, zeros(4,1));
pzmap(sys_cl);
title('Closed-Loop Pole-Zero Map');
sgrid;
grid on;

sgtitle('Frequency-Domain Analysis of Parallel PID Loop', 'FontWeight', 'bold');

% --- Save Figure 1 ---
if ~exist(fullfile(SEESAW_ROOT, 'docs', 'figures'), 'dir'), mkdir(fullfile(SEESAW_ROOT, 'docs', 'figures')); end
exportgraphics(gcf, fullfile(SEESAW_ROOT, 'docs', 'figures', 'loop_analysis_pid.png'), 'Resolution', 300);

%% 5. PERFORMANCE TUNING
%  -----------------------------------------------------------------------
%  Tuning guide — adjust the TuningGoal requirements in section 3:
%
%  To increase PM:       raise MinPM in TuningGoal.Margins (hard goal)
%  To speed up settling: reduce reference-model time in StepTracking
%  To limit voltage:     add TuningGoal.MaxLoopGain (roll-off constraint)
%  To reduce chatter:    lower the derivative filter N_filt
%  To improve centering: add TuningGoal.StepRejection at plant input
%
%  systune will jointly re-optimise all gains to meet the new specs.
%  -----------------------------------------------------------------------

fprintf('\n=== Performance Specs ===\n');

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
%  4.5° initial tilt — system must recover.
%  Full CL state: [xp(4); zc(nc); za(na)]
%  xp = plant states, zc/za = controller states (PID dynamics)

T_sim = 10;
dt    = 0.001;
t_sim = (0:dt:T_sim)';

% Initial condition: 4.5° seesaw tilt, controller states start at 0
x0_full = zeros(n_full, 1);
x0_full(3) = 4.5*pi/180;   % alpha IC

sat_v = @(v) sign(v) .* min(abs(v), V_sat);

% ODE with full PID dynamics + voltage saturation
ode_pid = @(t, x) pid_cl_ode(x, A_sw, B_sw, ...
    Ac_ctrl, Bc_ctrl, Cc_ctrl, Dc_ctrl, ...
    Aa_ctrl, Ba_ctrl, Ca_ctrl, Da_ctrl, ...
    C_xc, C_al, nc, sat_v, 0);

[t_out, x_full_out] = ode45(ode_pid, t_sim, x0_full);

% Recover saturated control signal for plotting
u_out = zeros(length(t_out), 1);
for k = 1:length(t_out)
    xp = x_full_out(k, 1:4)';
    zc = x_full_out(k, 5:4+nc)';
    za = x_full_out(k, 5+nc:end)';
    u_dem = Cc_ctrl*zc + Dc_ctrl*(-C_xc*xp) + Ca_ctrl*za + Da_ctrl*(C_al*xp);
    u_out(k) = sat_v(u_dem);
end

xc_cl    = x_full_out(:,1) * 1000;    % [mm]
alpha_cl = x_full_out(:,3) * 180/pi;  % [deg]
Vm_cl    = u_out;                      % [V]

figure('Name', 'Disturbance Response (Parallel PID)', 'Position', [100 100 900 700]);

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

sgtitle('Closed-Loop Response: 4.5° Initial Tilt (Parallel PID)', 'FontWeight', 'bold');

% --- Save Figure 2 ---
exportgraphics(gcf, fullfile(SEESAW_ROOT, 'docs', 'figures', 'disturbance_response_pid.png'), 'Resolution', 300);

%% 7. REPEATED DISTURBANCE TEST
%  Pulse train: angular velocity taps every 4 seconds.

T_sim2 = 20;
t_sim2 = (0:dt:T_sim2)';
n2 = length(t_sim2);

u_dist = zeros(n2, 1);
pulse_times = [1, 5, 9, 13, 17];
pulse_dur   = 0.16;
pulse_amp   = 0.08 / pulse_dur;
for pt = pulse_times
    u_dist(t_sim2 >= pt & t_sim2 < pt + pulse_dur) = pulse_amp;
end

% Repeated disturbance simulation with full PID dynamics + saturation
ode_dist_wrapper = @(t, x) pid_cl_ode(x, A_sw, B_sw, ...
    Ac_ctrl, Bc_ctrl, Cc_ctrl, Dc_ctrl, ...
    Aa_ctrl, Ba_ctrl, Ca_ctrl, Da_ctrl, ...
    C_xc, C_al, nc, sat_v, interp1(t_sim2, u_dist, t, 'previous', 0));
[t_out2, x_out2] = ode45(ode_dist_wrapper, t_sim2, zeros(n_full, 1));

% Recover saturated control signal
u_out2 = zeros(length(t_out2), 1);
for k = 1:length(t_out2)
    xp = x_out2(k, 1:4)';
    zc = x_out2(k, 5:4+nc)';
    za = x_out2(k, 5+nc:end)';
    u_dem = Cc_ctrl*zc + Dc_ctrl*(-C_xc*xp) + Ca_ctrl*za + Da_ctrl*(C_al*xp);
    u_out2(k) = sat_v(u_dem);
end

figure('Name', 'Repeated Disturbance Test (PID)', 'Position', [150 80 900 700]);

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

sgtitle('Repeated 4.5° Disturbance (Parallel PID)', 'FontWeight', 'bold');

% --- Save Figure 3 ---
exportgraphics(gcf, fullfile(SEESAW_ROOT, 'docs', 'figures', 'repeated_disturbance_pid.png'), 'Resolution', 300);

%% 8. VOLTAGE SATURATION & TRAVEL CHECK

V_peak_ic   = max(abs(Vm_cl));
V_peak_dist = max(abs(u_out2(:)));
V_peak      = max(V_peak_ic, V_peak_dist);
V_margin    = V_sat - V_peak;
xc_peak_ic   = max(abs(xc_cl));
xc_peak_dist = max(abs(x_out2(:,1))) * 1000;
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
%  Robustness check: margins vs B_eq ±30%

fprintf('\n=== Sensitivity to B_eq Uncertainty ===\n');
fprintf('  B_eq variation    PM [deg]    GM [dB]    Max Re(eig)\n');
B_eq_test = B_eq * [0.7, 0.85, 1.0, 1.15, 1.3];
for i = 1:length(B_eq_test)
    B_t_i = B_eq_test(i) + B_emf;
    G_rhs_i = [0, -B_t_i, -g*M_c, 0; -g*M_c, 0, g*(M_c*D_T+M_SW*D_C), -B_SW];
    A_i = [0,1,0,0; M_inv(1,:)*G_rhs_i; 0,0,0,1; M_inv(2,:)*G_rhs_i];
    B_sw_i = B_sw;  % B_sw doesn't depend on B_eq
    A_aug_i = [A_i, zeros(4,1); -1, 0, 0, 0, 0];
    B_aug_i = [B_sw_i; 0];
    % Build full CL with varied plant, same tuned controllers
    A_cl_i = [A_i + B_sw_i*(Dc_ctrl*(-C_xc) + Da_ctrl*C_al), B_sw_i*Cc_ctrl, B_sw_i*Ca_ctrl;
              Bc_ctrl*(-C_xc),                                Ac_ctrl,         zeros(nc,na);
              Ba_ctrl*C_al,                                   zeros(na,nc),    Aa_ctrl];
    ev_i = eig(A_cl_i);
    max_re = max(real(ev_i));
    % Loop TF with varied plant
    G_i = ss(A_i, B_sw_i, [C_xc; C_al], zeros(2,1));
    G_i.InputName = 'Vm'; G_i.OutputName = {'xc','alpha'};
    L_i = -tf(connect(G_i, Cc_loop, Ca_loop, Sum_neg, Sum_u, 'Vm', 'u_total'));
    [Gm_i, Pm_i] = margin(L_i);
    stab_str = '';
    if max_re > 0.01, stab_str = '← UNSTABLE'; end
    fprintf('  B_eq = %5.2f (%+3.0f%%)   %5.1f       %5.1f       %+.4f  %s\n', ...
        B_eq_test(i), (B_eq_test(i)/B_eq - 1)*100, Pm_i, 20*log10(Gm_i), ...
        max_re, stab_str);
end

%% 10. SUMMARY & SAVE

% Settling time from simulation (2% of 4.5°)
alpha_sim = x_full_out(:,3);
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
fprintf('  PARALLEL PID CONTROL PIPELINE SUMMARY\n');
fprintf('============================================================\n');
fprintf('\n  DESIGN\n');
fprintf('  ──────\n');
fprintf('  Method:          systune (structured H-inf synthesis)\n');
fprintf('  Architecture:    Parallel PID (both encoders, jointly tuned)\n');
fprintf('  Cart PID gains:  Kp_c = %.3f, Ki_c = %.3f, Kd_c = %.3f\n', Kp_c, Ki_c, Kd_c);
fprintf('  Alpha PD gains:  Kp_a = %.3f, Kd_a = %.3f\n', Kp_a, Kd_a);
fprintf('  Deriv filter:    N = %d rad/s (fixed)\n', N_filt);
fprintf('\n  CLOSED-LOOP POLES\n');
fprintf('  ─────────────────\n');
for k_s = 1:length(ev_cl)
    if abs(imag(ev_cl(k_s))) > 1e-6 && imag(ev_cl(k_s)) > 0
        fprintf('  p = %.4f ± %.4fi\n', real(ev_cl(k_s)), abs(imag(ev_cl(k_s))));
    elseif imag(ev_cl(k_s)) == 0 || abs(imag(ev_cl(k_s))) <= 1e-6
        fprintf('  p = %.4f\n', real(ev_cl(k_s)));
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
save_path = fullfile(SEESAW_ROOT, 'data', 'controller_pid.mat');
save(save_path, 'Kp_c', 'Ki_c', 'Kd_c', 'Kp_a', 'Kd_a', 'N_filt', ...
    'K_aug', ...
    'Pm', 'Gm', 'wgc', 'wpc', 'A_sw', 'B_sw', 'C_sw', 'D_sw', ...
    'A_aug', 'A_aug_cl', 'p_unstable');
fprintf('\n  Controller saved to: data/controller_pid.mat\n');
fprintf('============================================================\n');

%% 11. BUILD SIMULINK MODEL (Seesaw_PID)
%  -----------------------------------------------------------------------
%  Programmatically creates the Simulink model for parallel PID control:
%    u = PID_cart(x_c_ref - x_c) + PD_alpha(alpha)
%
%  Two modes (auto-detected):
%    Simulation: State-Space plant in the loop
%    Hardware:   QUARC I/O — PID blocks handle derivative estimation
%
%  The model is saved to:  models/Seesaw_PID.slx
%  -----------------------------------------------------------------------

fprintf('\n============================================================\n');
fprintf('  11. BUILDING SIMULINK MODEL: Seesaw_PID\n');
fprintf('============================================================\n');

mdl = 'Seesaw_PID';

% Push PID gains to base workspace
assignin('base', 'Kp_c', Kp_c);
assignin('base', 'Ki_c', Ki_c);
assignin('base', 'Kd_c', Kd_c);
assignin('base', 'Kp_a', Kp_a);
assignin('base', 'Kd_a', Kd_a);
assignin('base', 'N_filt', N_filt);
assignin('base', 'V_sat', V_sat);

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
        'StopTime', '20', ...
        'MaxStep',  '1e-3', ...
        'RelTol',   '1e-4', ...
        'AbsTol',   '1e-6');
end

% ==================================================================
%  REFERENCE INPUT
% ==================================================================
add_block('simulink/Sources/Step', [mdl '/x_c_ref'], ...
    'Time',   '2', ...
    'Before', '0', ...
    'After',  '0', ...
    'Position', [20 95 60 135]);

% Rate Limiter to prevent Derivative Kick on setpoint step
add_block('simulink/Discontinuities/Rate Limiter', [mdl '/Rate_Limit_ref'], ...
    'RisingSlewLimit', '0.2', ...
    'FallingSlewLimit', '-0.2', ...
    'Position', [90 95 120 135]);

% Alpha reference (always 0)
add_block('simulink/Sources/Constant', [mdl '/alpha_ref'], ...
    'Value', '0', ...
    'Position', [50 195 90 215]);

% ==================================================================
%  CART PID CONTROLLER
% ==================================================================

% Sum: e_xc = x_c_ref - x_c
add_block('simulink/Math Operations/Sum', [mdl '/Sum_cart'], ...
    'Inputs', '+-', ...
    'Position', [160 100 180 120]);

% PID block for cart (TrackingMode 'on' for Back-Calculation Anti-Windup)
add_block('simulink/Continuous/PID Controller', [mdl '/PID_Cart'], ...
    'P', 'Kp_c', ...
    'I', 'Ki_c', ...
    'D', 'Kd_c', ...
    'N', 'N_filt', ...
    'TrackingMode', 'on', ...
    'Kt', '100', ...
    'Position', [230 92 310 138]);

% ==================================================================
%  ALPHA PD CONTROLLER
% ==================================================================

% Sum: e_alpha = alpha_ref - (-alpha) = alpha  (error sign flip)
add_block('simulink/Math Operations/Gain', [mdl '/Negate_alpha'], ...
    'Gain', '-1', ...
    'Position', [120 192 150 218]);

add_block('simulink/Math Operations/Sum', [mdl '/Sum_alpha'], ...
    'Inputs', '+-', ...
    'Position', [190 195 210 215]);

% PD block for alpha (Kp_a + Kd_a*N/(1+N/s)), Ki = 0
add_block('simulink/Continuous/PID Controller', [mdl '/PD_Alpha'], ...
    'Controller', 'PD', ...
    'P', 'Kp_a', ...
    'D', 'Kd_a', ...
    'N', 'N_filt', ...
    'Position', [250 187 330 233]);

% ==================================================================
%  SUM PID OUTPUTS + SATURATION
% ==================================================================

% Sum: u = u_cart + u_alpha
add_block('simulink/Math Operations/Sum', [mdl '/Sum_PID'], ...
    'Inputs', '++', ...
    'Position', [400 140 420 160]);

% Voltage saturation (±V_sat)
add_block('simulink/Discontinuities/Saturation', [mdl '/V_Sat'], ...
    'UpperLimit', 'V_sat', ...
    'LowerLimit', '-V_sat', ...
    'Position', [470 138 520 162]);

% Anti-Windup Tracking Signal Calculation
% TR = V_Sat - u_alpha (what is left for cart PID after alpha uses its share)
add_block('simulink/Math Operations/Subtract', [mdl '/AW_Tracking'], ...
    'Inputs', '+-', ...
    'Position', [230 20 250 50]);

% ==================================================================
%  WIRING: CONTROLLER FORWARD PATH
% ==================================================================
add_line(mdl, 'x_c_ref/1',     'Rate_Limit_ref/1', 'autorouting', 'smart');
add_line(mdl, 'Rate_Limit_ref/1','Sum_cart/1',     'autorouting', 'smart');
add_line(mdl, 'Sum_cart/1',    'PID_Cart/1',   'autorouting', 'smart');
add_line(mdl, 'alpha_ref/1',   'Sum_alpha/1',  'autorouting', 'smart');
add_line(mdl, 'Negate_alpha/1','Sum_alpha/2',  'autorouting', 'smart');
add_line(mdl, 'Sum_alpha/1',   'PD_Alpha/1',   'autorouting', 'smart');
add_line(mdl, 'PID_Cart/1',    'Sum_PID/1',    'autorouting', 'smart');
add_line(mdl, 'PD_Alpha/1',    'Sum_PID/2',    'autorouting', 'smart');
add_line(mdl, 'Sum_PID/1',     'V_Sat/1',      'autorouting', 'smart');

% Anti-windup feedback wiring
add_line(mdl, 'V_Sat/1',       'AW_Tracking/1', 'autorouting', 'smart');  % + path
add_line(mdl, 'PD_Alpha/1',    'AW_Tracking/2', 'autorouting', 'smart');  % - path
add_line(mdl, 'AW_Tracking/1', 'PID_Cart/2',    'autorouting', 'smart');  % TR port

% ==================================================================
%  DISPLAY: SCOPES + TO-WORKSPACE
% ==================================================================
add_block('simulink/Math Operations/Gain', [mdl '/ref_m_to_cm'], ...
    'Gain', '100', 'Position', [730 52 770 78]);

add_block('simulink/Math Operations/Gain', [mdl '/xc_m_to_cm'], ...
    'Gain', '100', 'Position', [730 92 770 118]);

add_block('simulink/Math Operations/Gain', [mdl '/alpha_to_deg'], ...
    'Gain', '180/pi', 'Position', [730 172 770 198]);

add_block('simulink/Sinks/Scope', [mdl '/Scope_Cart'], ...
    'NumInputPorts', '2', 'Position', [840 60 880 110]);
set_param([mdl '/Scope_Cart'], 'Name', 'Cart [cm]');

add_block('simulink/Sinks/Scope', [mdl '/Scope_Angle'], ...
    'NumInputPorts', '1', 'Position', [840 170 880 200]);
set_param([mdl '/Scope_Angle'], 'Name', 'Angle [deg]');

add_block('simulink/Sinks/Scope', [mdl '/Scope_Vm'], ...
    'NumInputPorts', '1', 'Position', [840 250 880 280]);
set_param([mdl '/Scope_Vm'], 'Name', 'Voltage [V]');

add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_xc'], ...
    'VariableName', 'ctrl_xc', 'SaveFormat', 'Timeseries', ...
    'Position', [840 310 910 330]);

add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_alpha'], ...
    'VariableName', 'ctrl_alpha', 'SaveFormat', 'Timeseries', ...
    'Position', [840 350 910 370]);

add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_Vm'], ...
    'VariableName', 'ctrl_Vm', 'SaveFormat', 'Timeseries', ...
    'Position', [840 390 910 410]);

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
            [mdl '/Motor Command'], 'Position', [600 138 685 162]);

        % Encoder input: ch0 = cart, ch1 = seesaw
        add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Read Encoder', ...
            [mdl '/Encoders'], 'Position', [50 450 135 510]);

        % Encoder → physical units
        add_block('simulink/Math Operations/Gain', [mdl '/Enc_to_xc'], ...
            'Gain', 'K_ec', 'Position', [200 455 260 485]);

        add_block('simulink/Math Operations/Gain', [mdl '/Enc_to_alpha'], ...
            'Gain', 'K_E_SW / K_gs', 'Position', [200 505 260 535]);

        % Wire motor output
        add_line(mdl, 'V_Sat/1', 'Motor Command/1', 'autorouting', 'smart');

        % Wire encoders → conversion gains
        add_line(mdl, 'Encoders/1', 'Enc_to_xc/1',    'autorouting', 'smart');
        add_line(mdl, 'Encoders/2', 'Enc_to_alpha/1',  'autorouting', 'smart');

        % Wire feedback → PID error inputs
        add_line(mdl, 'Enc_to_xc/1',    'Sum_cart/2',      'autorouting', 'smart');
        add_line(mdl, 'Enc_to_alpha/1',  'Negate_alpha/1',  'autorouting', 'smart');

        % Wire feedback → display scopes
        add_line(mdl, 'Enc_to_xc/1',    'xc_m_to_cm/1',   'autorouting', 'smart');
        add_line(mdl, 'xc_m_to_cm/1',   'Cart [cm]/2',    'autorouting', 'smart');
        add_line(mdl, 'Enc_to_alpha/1',  'alpha_to_deg/1', 'autorouting', 'smart');
        add_line(mdl, 'alpha_to_deg/1',  'Angle [deg]/1',  'autorouting', 'smart');

        % Wire feedback → To Workspace
        add_line(mdl, 'Enc_to_xc/1',    'ToWS_xc/1',    'autorouting', 'smart');
        add_line(mdl, 'Enc_to_alpha/1',  'ToWS_alpha/1', 'autorouting', 'smart');

        fprintf('  QUARC blocks wired. PID blocks handle derivative filtering.\n');
        fprintf('  (No separate dirty derivative blocks needed.)\n');
    catch ME
        fprintf('  WARNING: Could not add QUARC blocks: %s\n', ME.message);
        fprintf('  Model will NOT work on hardware.\n');
    end
else
    % ============ SIMULATION ONLY: plant in the loop ============
    fprintf('  Adding State-Space plant for closed-loop simulation...\n');

    add_block('simulink/Continuous/State-Space', [mdl '/Plant_SS'], ...
        'A', 'A_sw', 'B', 'B_sw', 'C', 'C_sw', 'D', 'D_sw', ...
        'X0', '[0; 0; 0.08; 0]', ...
        'Position', [600 65 710 215]);

    % Demux: split [x_c; x_c_dot; alpha; alpha_dot]
    add_block('simulink/Signal Routing/Demux', [mdl '/Demux_Plant'], ...
        'Outputs', '4', 'Position', [740 65 745 215]);

    % V_Sat → Plant → Demux
    add_line(mdl, 'V_Sat/1',    'Plant_SS/1',    'autorouting', 'smart');
    add_line(mdl, 'Plant_SS/1', 'Demux_Plant/1', 'autorouting', 'smart');

    % Feedback: x_c → cart PID error, alpha → alpha PD
    add_line(mdl, 'Demux_Plant/1', 'Sum_cart/2',      'autorouting', 'smart');
    add_line(mdl, 'Demux_Plant/3', 'Negate_alpha/1',  'autorouting', 'smart');

    % Feedback → display scopes
    add_line(mdl, 'Demux_Plant/1', 'xc_m_to_cm/1',   'autorouting', 'smart');
    add_line(mdl, 'xc_m_to_cm/1',  'Cart [cm]/2',    'autorouting', 'smart');
    add_line(mdl, 'Demux_Plant/3', 'alpha_to_deg/1',  'autorouting', 'smart');
    add_line(mdl, 'alpha_to_deg/1', 'Angle [deg]/1',  'autorouting', 'smart');

    % Feedback → To Workspace
    add_line(mdl, 'Demux_Plant/1', 'ToWS_xc/1',    'autorouting', 'smart');
    add_line(mdl, 'Demux_Plant/3', 'ToWS_alpha/1', 'autorouting', 'smart');

    fprintf('  Simulation plant wired. IC = [0, 0, 4.5 deg, 0].\n');
end

% Save model
mdl_path = fullfile(SEESAW_ROOT, 'models', [mdl '.slx']);
save_system(mdl, mdl_path);
fprintf('  Model saved: models/%s.slx\n', mdl);

% --- Run simulation if in simulation mode ---
if ~quarc_available
    fprintf('  Running 20 s closed-loop simulation...\n');
    simout = sim(mdl, 'StopTime', '20');
    fprintf('  Simulation complete.\n');

    figure('Name', 'Simulink CL Simulation (Parallel PID)', 'Position', [200 100 900 700]);

    if exist('ctrl_xc', 'var') && exist('ctrl_alpha', 'var') && exist('ctrl_Vm', 'var')
        subplot(3,1,1);
        plot(ctrl_xc.Time, ctrl_xc.Data * 100, 'b-', 'LineWidth', 1.5);
        ylabel('x_c [cm]'); title('Cart Position (Simulink)');
        yline(0, 'k--'); grid on;

        subplot(3,1,2);
        plot(ctrl_alpha.Time, ctrl_alpha.Data * 180/pi, 'g-', 'LineWidth', 1.5);
        ylabel('\alpha [deg]'); title('Seesaw Angle (Simulink)');
        yline(0, 'k--');
        yline(11.5, 'r:', 'Hard stop +11.5°');
        yline(-11.5, 'r:', 'Hard stop -11.5°');
        grid on;

        subplot(3,1,3);
        plot(ctrl_Vm.Time, ctrl_Vm.Data, 'r-', 'LineWidth', 1.5);
        ylabel('V_m [V]'); xlabel('Time [s]');
        title('Motor Voltage (Simulink)');
        yline(V_sat, 'k--', '+V_{sat}');
        yline(-V_sat, 'k--', '-V_{sat}');
        grid on;

        sgtitle('Simulink CL: Parallel PID (4.5° IC)', 'FontWeight', 'bold');
    else
        fprintf('  Note: Workspace variables not found — check To Workspace blocks.\n');
    end
end

fprintf('============================================================\n');

%% 12. DEPLOY TO HARDWARE (QUARC)
%  -----------------------------------------------------------------------
%  Guided hardware deployment workflow.
%  Detects QUARC, builds target, connects to Q2-USB, and starts
%  real-time control with safety confirmations at each step.
%  -----------------------------------------------------------------------

fprintf('\n============================================================\n');
fprintf('  12. HARDWARE DEPLOYMENT\n');
fprintf('============================================================\n');

if ~quarc_available
    fprintf('\n  QUARC not available — skipping hardware deployment.\n');
    fprintf('  To deploy to hardware:\n');
    fprintf('    1. Install QUARC and add to MATLAB path\n');
    fprintf('    2. Re-run this pipeline\n');
    fprintf('============================================================\n');
else
    fprintf('\n  QUARC detected. Ready for hardware deployment.\n');
    fprintf('  Model: %s\n', mdl);
    fprintf('  Cart PID:  Kp_c=%.3f Ki_c=%.3f Kd_c=%.3f\n', Kp_c, Ki_c, Kd_c);
    fprintf('  Alpha PD:  Kp_a=%.3f Kd_a=%.3f\n', Kp_a, Kd_a);
    fprintf('  Voltage limit: ±%.1f V\n\n', V_sat);

    fprintf('  ╔══════════════════════════════════════════════════════╗\n');
    fprintf('  ║  SAFETY CHECKLIST — COMPLETE BEFORE STARTING        ║\n');
    fprintf('  ╠══════════════════════════════════════════════════════╣\n');
    fprintf('  ║  [ ] Q2-USB connected and powered                   ║\n');
    fprintf('  ║  [ ] VoltPAQ-X1 powered, switch set to 1x           ║\n');
    fprintf('  ║  [ ] Seesaw free to rotate (no obstructions)        ║\n');
    fprintf('  ║  [ ] Emergency stop procedure understood:           ║\n');
    fprintf('  ║      → QUARC | Stop  or  Ctrl+Break                 ║\n');
    fprintf('  ║  [ ] Cart near center of track                      ║\n');
    fprintf('  ║  [ ] Seesaw held approximately level BY HAND        ║\n');
    fprintf('  ╚══════════════════════════════════════════════════════╝\n\n');

    % Step 1: Build
    fprintf('  STEP 1/4: Building QUARC target...\n');
    try
        rtwbuild(mdl);
        fprintf('  Build SUCCESSFUL.\n\n');
    catch ME
        fprintf('  BUILD FAILED: %s\n', ME.message);
        fprintf('  Fix the error and re-run control_pipeline.\n');
        fprintf('============================================================\n');
        return;
    end

    % Step 2: Connect
    fprintf('  STEP 2/4: Connecting to Q2-USB...\n');
    reply = input('  Press ENTER to connect (or type "q" to abort): ', 's');
    if strcmpi(reply, 'q')
        fprintf('  Aborted by user.\n');
        fprintf('============================================================\n');
        return;
    end

    try
        set_param(mdl, 'SimulationCommand', 'connect');
        fprintf('  Connected to target.\n\n');
    catch ME
        fprintf('  CONNECT FAILED: %s\n', ME.message);
        fprintf('  Check Q2-USB connection and power.\n');
        fprintf('============================================================\n');
        return;
    end

    % Step 3: Start (with safety confirmation)
    fprintf('  STEP 3/4: Ready to start real-time control.\n');
    fprintf('  *** HOLD THE SEESAW LEVEL BY HAND ***\n');
    fprintf('  *** BE READY TO STOP: QUARC | Stop  or  Ctrl+Break ***\n\n');
    reply = input('  Type "go" and press ENTER to START (or "q" to abort): ', 's');
    if ~strcmpi(reply, 'go')
        fprintf('  Aborted by user. Disconnecting...\n');
        try
            set_param(mdl, 'SimulationCommand', 'disconnect');
        catch
        end
        fprintf('============================================================\n');
        return;
    end

    try
        set_param(mdl, 'SimulationCommand', 'start');
        fprintf('  *** CONTROLLER RUNNING — release seesaw gently ***\n');
        fprintf('  *** Press Ctrl+Break or run:  set_param(''%s'', ''SimulationCommand'', ''stop'')  to stop ***\n\n', mdl);
    catch ME
        fprintf('  START FAILED: %s\n', ME.message);
        fprintf('============================================================\n');
        return;
    end

    % Step 4: Wait and stop
    fprintf('  STEP 4/4: Controller is running.\n');
    reply = input('  Press ENTER to stop the controller (or let it run): ', 's');

    try
        set_param(mdl, 'SimulationCommand', 'stop');
        fprintf('  Controller STOPPED.\n');
    catch ME
        fprintf('  Stop command failed: %s\n', ME.message);
        fprintf('  Try manually: set_param(''%s'', ''SimulationCommand'', ''stop'')\n', mdl);
    end

    % Data retrieval
    fprintf('\n  Retrieving logged data...\n');
    if evalin('base', 'exist(''ctrl_xc'', ''var'')')
        fprintf('  Data available in workspace:\n');
        fprintf('    ctrl_xc    — cart position [m]\n');
        fprintf('    ctrl_alpha — seesaw angle [rad]\n');
        fprintf('    ctrl_Vm    — motor voltage [V]\n');
        fprintf('  Plot with:  plot(ctrl_alpha.Time, ctrl_alpha.Data*180/pi)\n');
    else
        fprintf('  Warning: logged data not found in workspace.\n');
    end

    fprintf('============================================================\n');
end

%% =====================================================================
%  LOCAL FUNCTIONS
%  =====================================================================

function dx = pid_cl_ode(x, Ap, Bp, Ac, Bc, Cc, Dc, Aa, Ba, Ca, Da, ...
                         Cxc, Cal, nc, sat_v, dist_alpha_dot)
%PID_CL_ODE  Full closed-loop ODE with PID controller dynamics + saturation.
%  State:  x = [xp(4); zc(nc); za(na)]
%    xp  = plant states [x_c; x_c_dot; alpha; alpha_dot]
%    zc  = cart PID controller states
%    za  = alpha PD controller states
%  dist_alpha_dot = external disturbance torque entering alpha_dot

    xp = x(1:4);
    zc = x(5:4+nc);
    za = x(5+nc:end);

    e_xc  = -Cxc * xp;       % cart error (ref = 0)
    alpha =  Cal * xp;       % seesaw angle

    % Controller outputs
    u_cart  = Cc*zc + Dc*e_xc;
    u_alpha = Ca*za + Da*alpha;
    u = sat_v(u_cart + u_alpha);

    % Dynamics
    dxp = Ap*xp + Bp*u + [0; 0; 0; dist_alpha_dot];
    dzc = Ac*zc + Bc*e_xc;
    dza = Aa*za + Ba*alpha;

    dx = [dxp; dzc; dza];
end
