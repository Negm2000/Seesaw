%% Pole Placement Design -- Limit-Cycle Minimization Objective
%
% Nonlinearities (Coulomb friction, encoder quantization, backlash) 
% produce a steady-state limit cycle. The design goal is to minimize 
% the oscillation amplitude by independent placement of the four 
% closed-loop poles.
%
% State vector: x = [x_c; x_c_dot; theta; theta_dot]
% Requires:     seesaw_params.m (plant), tuned_params.mat (B_eq)
% Outputs:      controller_freq.mat, figures

close all; clc

% Configure global plot settings
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. LOAD SYSTEM PARAMETERS
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = pwd; end 

% Load physical parameters (g, Me, DT, Jsw, Bsw, Msw, DC, etc.)
seesaw_params; 

% Load the voltage non-linearities
nonlinear_file = fullfile(SEESAW_ROOT, 'data', 'params', 'param_nonlinear.mat');
if exist(nonlinear_file, 'file')
    load(nonlinear_file, 'ud_pos', 'ud_neg', 'ud_sym');
else
    error('Non-linearities not found. Run seesaw nonlinear first.');
end

% Load the tuned state-space representation
cart_file = fullfile(SEESAW_ROOT, 'data', 'tuned', 'tuned_cart.mat');
if exist(cart_file, 'file')
    load(cart_file);
else
    error('Tuned cart model not found. Run cart modeling first.');
end

% Load the tuned state-space representation
seesaw_file = fullfile(SEESAW_ROOT, 'data', 'tuned', 'tuned_seesaw.mat');
if exist(seesaw_file, 'file')
    load(seesaw_file);
else
    error('Tuned seesaw model not found. Run seesaw modeling first.');
end

% Prepare folder for fig saving

figdir = fullfile(SEESAW_ROOT, 'docs', 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end
%% Build plant for current configuration

poles_ol   = sort(eig(A_sw));
p_unstable = max(real(poles_ol));     % seesaw fall rate -- design driver

rank_Co4 = rank(ctrb(A_sw, B_sw));
rank_Co5 = rank(ctrb(A5, B5));

fprintf('Open-loop poles: '); fprintf('%.3f  ', poles_ol); fprintf('\n')
fprintf('Unstable mode: p_OL = %+.3f rad/s (seesaw fall rate)\n', p_unstable)
fprintf('Controllability rank: %d/4\n', rank_Co4)
fprintf('Integral Action Controllability rank: %d/5\n\n', rank_Co5)

figure; hold on; grid on
plot(real(poles_ol), imag(poles_ol), 'rx', 'MarkerSize', 12, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag')
title('Open-Loop Poles -- one in RHP')
saveas(gcf, fullfile(figdir, 'OL-Poles.png'))

%% Pole selection
%
% Constraints:
%   p_OL  ~ +2.24 rad/s   (unstable pole)
%   q_th  ~ 5e-4 rad      (encoder resolution)
%   V_sat = 6 V           (saturation)
%   T_c/2 = 0.407 m       (rail limit)
%
% Design rule: The complex conjugate pair governing theta dynamics is the 
% dominant pair. Cart poles are real and placed at higher frequencies to 
% maintain a ~2:1 bandwidth window.

% --- p1, p2: complex pair, dominant theta-loop mode ---
% sigma_th (= -Re of pair) governs error correction rate.
%   Lower bound: > p_OL to outrun divergence; 2*p_OL recommended.
%   zeta_th = 0.8: balances settling time and stability margins.
sigma_th = 5.5;
zeta_th  = 0.8;

% --- p3: real pole, cart-velocity damping (non-dominant) ---
% Placed at |p3| > sigma_th to ensure velocity settles faster than 
% the balancing mode.
p3 = -1.5 * sigma_th;

% --- p4: real pole, cart-position regulation (non-dominant) ---
% Enforced |Re(p4)| > sigma_th to maintain p1,2 dominance. 
% Sets the cart return-to-center time constant.
p4 = -1.2 * sigma_th;

wn_th = sigma_th / zeta_th;
p_dom = -sigma_th + 1j*wn_th*sqrt(1-zeta_th^2);
p_des = make_placeable_poles([p_dom; conj(p_dom); p3; p4]);

theta0_deg = 2.0;
t = 0:0.002:10;
[K4, pcl_f, yf, uf, mf] = sim_regulator(A_sw, B_sw, p_des, ...
    [0; 0; deg2rad(theta0_deg); 0], t);
A_cl = A_sw - B_sw*K4;

% Values obtained after running SS_RoA_analysis with Q_lyap = eye(4);
xc_lim = 0.0476; theta_lim = 0.0576; xcdot_max = 0.5048; thetadot_max = 0.2266;
Q_lyap = diag([1/xc_lim^2, 1/theta_lim^2, 1/xcdot_max^2, 1/thetadot_max^2]);
P4 = lyap(A_cl', Q_lyap);

% Max IC before saturation
theta_max_deg = theta0_deg * V_nom / mf.peak_v;

% Noise check (mapping measurement resolution to actuator voltage)
q_th = K_E_SW / K_gs;
V_noise_th = abs(K4(2)) * q_th;

% Stability margins
open_loop_L = tf(ss(A_sw, B_sw, K4, 0));
[Gm, Pm, ~, wgc] = margin(open_loop_L);
Gm_dB = 20*log10(Gm);

%% Constant bias-torque test -- shifts the oscillation center
% 100g mass off-center. Without integral action, the oscillation 
% band shifts from zero.
[t_bias, x_bias, u_bias, mb] = sim_bias_load( ...
    A_cl, K4, M_inv);

%% Integral action
% Augment with xi_dot = theta to eliminate steady-state error.
p_int   = -0.5;
p_aug   = make_placeable_poles([p_des; p_int]);

[K5, ~, ~, ~, ~] = sim_regulator(A5, B5, p_aug, ...
    [0; deg2rad(4.5); 0; 0; 0], t);

[t_bi, x_bi, u_bi, mbi] = sim_bias_load_aug( ...
    A5 - B5*K5, K5, M_inv);

%% Stability Check and Derivative Filter

if any(real(p_des) >= 0)
    warning('Closed-loop PP is NOT stable!');
elseif any(real(p_aug) >= 0)
    warning('Closed-loop PPi is NOT stable!');
else
    disp('Closed-loop systems are stable.');
end

% derivative filter to get velocities
dom_pole4 = real(min(p_des));

N4 = 25*abs(dom_pole4);

H4 = N4*s / (s + N4);
H4d = c2d(H4, Ts, 'tustin');

% initial conditions
init_cond4 = [0, deg2rad(1), 0, 0];

%% Print summary
fprintf('Pole selection (vs |p_OL| = %.2f rad/s):\n', abs(p_unstable))
fprintf('  p1,2 (theta pair):   sigma_th=%.2f, zeta_th=%.2f -> %.2f +/- %.2fj\n', ...
    sigma_th, zeta_th, real(p_dom), imag(p_dom))
fprintf('  p3 (cart velocity):  %.2f\n', p3)
fprintf('  p4 (cart position):  %.2f\n\n', p4)

fprintf('Sim results (theta0 = %.1f deg):\n', theta0_deg)
fprintf('  Peak voltage:   %.2f V  (limit 6V)\n', mf.peak_v)
fprintf('  Peak theta:     %.2f deg\n', rad2deg(mf.peak_y(3)))
fprintf('  Peak cart:      %.2f cm\n', mf.peak_y(1)*100)
fprintf('  Max IC (linear):%.2f deg\n', theta_max_deg)
fprintf('  V_noise_th:     %.3f V\n\n', V_noise_th)

fprintf('Stability margins:\n')
fprintf('  Gain margin:    %.2f dB\n', Gm_dB)
fprintf('  Phase margin:   %.2f deg\n', Pm)
fprintf('  Crossover:      %.2f rad/s\n', wgc)

%% Figures
figure
subplot(3,1,1); plot(yf.t, yf.x(:,1)*100, 'LineWidth', 1.2); grid on
ylabel('Cart [cm]')
title(sprintf('Regulator IC Response (%.1f deg)', theta0_deg))
subplot(3,1,2); plot(yf.t, rad2deg(yf.x(:,3)), 'LineWidth', 1.2); grid on
ylabel('\theta [deg]')
subplot(3,1,3); plot(yf.t, uf.v, 'LineWidth', 1.2); grid on
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'IC-Response-Final.png'))

figure; hold on; grid on
plot(real(pcl_f), imag(pcl_f), 'bo', 'MarkerSize', 10, 'LineWidth', 2)
plot(real(poles_ol), imag(poles_ol), 'rx', 'MarkerSize', 10, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag')
legend('Closed-loop poles', 'Open-loop poles')
title('Pole Constellation - Dominant Theta Pair')
saveas(gcf, fullfile(figdir, 'CL-Poles-Final.png'))

figure; 
subplot(2,1,1); plot(t_bias, rad2deg(x_bias(:,3)), 'LineWidth', 1.2); grid on
ylabel('\theta [deg]'); title('Bias load response WITHOUT integral action')
subplot(2,1,2); plot(t_bias, u_bias, 'LineWidth', 1.2); grid on
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'repeated_disturbance.png'))

figure; 
subplot(2,1,1); plot(t_bi, rad2deg(x_bi(:,3)), 'LineWidth', 1.2); grid on
ylabel('\theta [deg]'); title('Bias load response WITH integral action (xi\_dot = theta)')
subplot(2,1,2); plot(t_bi, u_bi, 'LineWidth', 1.2); grid on
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'bias_with_integral.png'))

figure
[~] = loop_analysis(A_sw, B_sw, K4, figdir);

%% Save
p_final = p_des;
save(fullfile(SEESAW_ROOT, 'data', 'params', 'pole_placement.mat'), ...
     'K4', 'p_final', 'sigma_th', 'zeta_th', 'p3', 'p4', 'V_noise_th');
fprintf('Saved data/pole_placement.mat\n')

%% DISCRETE TIME FOR THE HARDWARE
sys4d = c2d(sys4, Ts, 'zoh');

Ad = sys4d.A;
Bd = sys4d.B;
Cd = sys4d.C;
Dd = sys4d.D;

A5d = [Ad, zeros(4,1);
       0, Ts, 0, 0, 1];

B5d = [Bd;
       0];

C5d = [Cd, zeros(size(Cd,1),1)];
D5d = Dd;

rank_Co4d = rank(ctrb(Ad,Bd));
rank_Co5d = rank(ctrb(A5d,B5d));

p_desd = exp(p_des * Ts);
[K4d, ~, ~, ~, ~] = sim_regulator(Ad, Bd, p_desd, ...
    [0; deg2rad(theta0_deg); 0; 0], t);
eig_open_4d  = eig(Ad);
eig_close_4d = eig(Ad - Bd*K4d);

p_augd = exp(p_aug * Ts);
[K5d, ~, ~, ~, ~] = sim_regulator(A5d, B5d, p_augd, ...
    [0; deg2rad(4.5); 0; 0; 0], t);
eig_open_5d  = eig(A5d);
eig_close_5d = eig(A5d - B5d*K5d);

%% OBSERVER

k_obs =  5.0;
p_obs = k_obs*real(p_des);

delta = 0.1;
p_obs = [
    p_obs(1) + delta;
    p_obs(2) - delta;
    p_obs(3) + delta;
    p_obs(4) - delta;
];

L      = place(A_sw', C_sw', p_obs)';

Aobs = A_sw - L*C_sw;
Bobs = [B_sw L];
Cobs = eye(4);
Dobs = zeros(4, 3);

A_combined = [A_sw - B_sw*K4,   B_sw*K4;
              zeros(4,4),       A_sw - L*C_sw];
ev_check   = sort(eig(A_combined));
ev_target  = sort([eig(A_sw - B_sw*K4); eig(A_sw - L*C_sw)]);
err_eig    = max(abs(ev_check - ev_target));
fprintf('\nSeparation principle mismatch: %.2e\n', err_eig);

%% DISCRETE OBSERVER

p_obsd = exp(p_obs * Ts);
Ld = place(Ad', Cd', p_obsd)';

Aobs_d = Ad - Ld*Cd;
Bobs_d = [Bd Ld];
Cobs_d = eye(4);
Dobs_d = zeros(4, 3);
%% Simulation
x0     = [0; 0; deg2rad(theta0_deg); 0];
% In hardware, only positions are measured at startup. Initialize the
% observer with measured positions and zero velocities to avoid an
% unrealistic cold-start transient in the linear, unsaturated simulation.
xhat0  = [x0(1); 0; x0(3); 0];
t = (0:Ts:3)';
sys_combined = ss(A_combined, zeros(8,1), eye(8), zeros(8,1));
z = initial(sys_combined, [x0; x0-xhat0], t);
x_hist = z(:, 1:4); e_hist = z(:, 5:8);
u_hist = -(K4 * (x_hist - e_hist)')';

%% Summary
V_noise_xc = abs(K4 * L(:,1)) * K_ec;
V_noise_th = abs(K4 * L(:,2)) * K_E_SW / K_gs;
fprintf('V_noise (observer): xc=%.4f V, th=%.4f V\n', V_noise_xc, V_noise_th);

%% Figures
figure; hold on; grid on
plot(real(p_obs), imag(p_obs), 'bo', 'MarkerSize', 10, 'LineWidth', 2)
plot(real(p_des), imag(p_des), 'rx', 'MarkerSize', 10, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag')
legend('Observer poles', 'Controller poles')
title('Observer vs Controller Poles')
saveas(gcf, fullfile(figdir, 'Observer-Poles.png'))

figure
subplot(3,1,1); plot(t, x_hist(:,1)*100, 'LineWidth', 1.2); grid on
ylabel('Cart [cm]'); title('Observer IC Response (Position-Initialized)')
subplot(3,1,2); plot(t, rad2deg(x_hist(:,3)), 'LineWidth', 1.2); grid on
ylabel('\theta [deg]')
subplot(3,1,3); plot(t, u_hist, 'LineWidth', 1.2); grid on
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'Observer-IC-Response.png'))

%% Save
save(fullfile(SEESAW_ROOT, 'data', 'params', 'observer.mat'), ...
     'L', 'p_obs', 'k_obs', 'A_sw', 'B_sw', 'K4', ...
     'Aobs', 'Bobs', 'Cobs', 'Dobs');
fprintf('Saved data/observer.mat\n')

%% LIFT-UP

% Re-use the tuned position-control PID from freq based technique to move
% the cart up to 0.0568 because we know from the quasi-static test that
% this is the point at which it starts to lift.
% After that, use the flip-flop to switch from lift to stabilization only
% the first time it crosses 5°.

% Load the voltage non-linearities
inner_pid_file = fullfile(SEESAW_ROOT, 'data', 'controllers', 'controller_inner_pid.mat');
if exist(inner_pid_file, 'file')
    load(inner_pid_file, 'Kp_in', 'Ki_in', 'Kd_in', 'N_in', 'antiwindup_in');
else
    error('Inner-loop PID not found. Run pid_cart first.');
end

init_cond_lift = [-0.407, deg2rad(11.66), 0, 0];
init_cond_switch = [0.057, deg2rad(5), 0, -0.1];

%% Helpers
function [K, poles_cl, x, v, m] = sim_regulator(A, B, p, x0, t)
    K = place(A, B, p);
    Acl = A - B*K;
    poles_cl = eig(Acl);
    sys = ss(Acl, zeros(size(A,1), 1), eye(size(A,1)), zeros(size(A,1), 1));
    [y, t] = initial(sys, x0, t);
    voltage = -K * y';
    m.peak_v = max(abs(voltage));
    m.peak_y = max(abs(y));
    x.t = t; x.x = y;
    v.t = t; v.v = voltage';
end

function [t, x, v, m] = sim_bias_load(Acl, K, M_inv)
    t = 0:0.001:5;
    % Bias torque constant 0.123 Nm maps to accelerations
    % [x_ddot; theta_ddot] = M_inv * [0; tau_bias]
    accel_bias = M_inv * [0; 0.123];
    B_bias = [0; 0; accel_bias(1); accel_bias(2)];
    sys = ss(Acl, B_bias, eye(4), zeros(4,1));
    [x, t] = step(sys, t);
    v = -K * x';
    v = v';
    m = 0;
end

function [t, x, v, m] = sim_bias_load_aug(Acl, K, M_inv)
    t = 0:0.001:5;
    accel_bias = M_inv * [0; 0.123];
    B_bias = [0; 0; accel_bias(1); accel_bias(2); 0];
    sys = ss(Acl, B_bias, eye(5), zeros(5,1));
    [x, t] = step(sys, t);
    v = -K * x';
    v = v';
    m = 0;
end

function p_out = make_placeable_poles(p_in)
    p_out = p_in(:);
    tol = 1e-6; delta = 5e-3;
    for i = 2:numel(p_out)
        while any(abs(p_out(i) - p_out(1:i-1)) < tol)
            p_out(i) = p_out(i) - delta;
        end
    end
end

function margins = loop_analysis(A, B, K, figdir)
    L = tf(ss(A, B, K, 0));
    [Gm, Pm, ~, wgc] = margin(L);
    margins.gain_margin_db = 20*log10(Gm);
    margins.phase_margin_deg = Pm;
    margins.crossover_rad_s = wgc;

    margin(L); grid on
    title('Loop Transfer Function Margin Analysis')
    saveas(gcf, fullfile(figdir, 'loop_analysis.png'))
end
