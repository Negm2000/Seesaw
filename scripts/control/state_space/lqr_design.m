clear; close all; clc;

% Configure global plot settings
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. LOAD SYSTEM PARAMETERS
if ~exist('SEESAW_ROOT', 'var')
    curr = pwd;
    while ~exist(fullfile(curr, 'startup.m'), 'file') && ~strcmp(curr, fileparts(curr))
        curr = fileparts(curr);
    end
    SEESAW_ROOT = curr;
end
disp(['SEESAW_ROOT inside script is: ', SEESAW_ROOT]);

% Load physical parameters (g, Me, DT, Jsw, Bsw, Msw, DC, etc.)
seesaw_params; 

% Load the voltage non-linearities
cart_file = fullfile(SEESAW_ROOT, 'data', 'params', 'param_nonlinear.mat');
if exist(cart_file, 'file')
    load(cart_file, 'ud_pos', 'ud_neg', 'ud_sym');
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
cart_file = fullfile(SEESAW_ROOT, 'data', 'tuned', 'tuned_seesaw.mat');
if exist(cart_file, 'file')
    load(cart_file);
else
    error('Tuned seesaw model not found. Run seesaw modeling first.');
end

%% 2. DEFINE LIMITS/WEIGHTS (BRYSON'S RULE)

req_dist     = 0.05;            % maximum displacement [m]
req_angle    = deg2rad(1);      % maximum angle [rad]
req_speed    = 0.1;             % maximum speed [m/s]
%req_angspeed = +inf
req_integral = 1 * deg2rad(2);  % maximum time to clear a given angle
                                % [rad*s]

req_volt  = 3;                  % maximum voltage [V]

Cq4 = zeros(4);
Cq4(1,1) = 1 / req_dist;
Cq4(2,2) = 1 / req_angle;
Cq4(3,3) = 1 / req_speed;

Q4 = Cq4' * Cq4;
R4 = 1/req_volt^2;

Cq5 = [Cq4, zeros(4,1);
       zeros(1,4), 1 / req_integral];
Q5 = Cq5' * Cq5;
R5 = R4;

%% 3. VERIFY THE CONDITIONS

% Controllability
rank_Co4 = rank(ctrb(A_sw, B_sw));
rank_Co5 = rank(ctrb(A5, B5));

% Observability for LQR/LQI
rank_Ob4 = rank(obsv(A_sw, Cq4));
rank_Ob5 = rank(obsv(A5, Cq5));

% Observability for KF
rank_Ob4_kf = rank(obsv(A_sw, C_sw));

% Throw an error if ranks < 4 and ranks < 5.
if rank_Co4 < size(A_sw, 1)
    error('LQR is not controllable!');
elseif rank_Co5 < size(A5,1)
    error('LQI is not controllable!');
elseif rank_Ob4 < size(A_sw, 1)
    error('LQR is not observable!');
elseif rank_Ob5 < size(A5,1)
    error('LQI is not observable!');
elseif rank_Ob4_kf < size(A_sw, 1)
    error('KF is not observable!');
else
    disp(">>> The continuous systems are Controllable and Observable")
end
%% 4. DETERMINE THE GAINS OF CONTROLLER

K4 = lqr(A_sw, B_sw, Q4, R4);
eig_open_4  = eig(A_sw);
eig_close_4 = eig(A_sw - B_sw*K4);

K5 = lqr(A5,B5,Q5,R5);
eig_open_5  = eig(A5);
eig_close_5 = eig(A5 - B5*K5);

% Verify that the open loops are unstable but closed-loop are stable
if any(real(eig_close_4) >= 0)
    warning('Closed-loop LQR is NOT stable!');
elseif any(real(eig_close_5) >= 0)
    warning('Closed-loop LQI is NOT stable!');
else
    disp('Closed-loop systems are stable.');
end

% derivative filter to get velocities
dom_pole4 = real(min(eig_close_4));
dom_pole5 = real(min(eig_close_5));

N4 = 25*abs(dom_pole4);

H4 = N4*s / (s + N4);
H4d = c2d(H4, Ts, 'tustin');

% initial conditions
init_cond4 = [0, 0, deg2rad(1), 0];

%% 5. PERFORM THE LQR/LQI IN DISCRETE TIME FOR THE HARDWARE

sys4d = c2d(sys4, Ts, 'tustin');

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

% Controllability
rank_Co4d = rank(ctrb(Ad,Bd));
rank_Co5d = rank(ctrb(A5d,B5d));

% Observability
rank_Ob4d = rank(obsv(Ad, Cq4));
rank_Ob5d = rank(obsv(A5d, Cq5));

% Controller gains
K4d = dlqr(Ad, Bd, Q4, R4);
eig_open_4d  = eig(Ad);
eig_close_4d = eig(Ad - Bd*K4d);

K5d = dlqr(A5d, B5d, Q5, R5);
eig_open_5d  = eig(A5d);
eig_close_5d = eig(A5d - B5d*K5d);

% TODO : same checks than before on observability, controllability and
% stability of closed loop.

%% 5. Kalman Observer - GAIN DEFINITION

% 4-state linear Kalman observer.
% Asymmetric tuning by purpose, not by sign:
%   Q[3,3] = 3e-3 (large): the cart's stick-slip / Coulomb / asymmetric
%     deadband are not in the linear model, so we down-weight the velocity
%     prediction and let xc measurements carry xcd. KF effectively reduces
%     to a smoothed differentiator on the cart channel — fine, the cart
%     encoder is clean (K_ec = 23 um/count).
%   Q[4,4] = 2e-5 (small): the seesaw dynamics (gravity + bearing drag)
%     are clean, so we trust the model to smooth the coarser angle encoder
%     (K_E_SW = 1.5 mrad/count). This is where the KF actually beats a
%     causal DD — ~32% lower ald RMSE during balancing.
% R sits at the encoder quantization floors (K^2/12, plus small headroom on
% the cart side for track vibration).
%
% If finer modeling is wanted (asymmetric deadband rectification, DC track
% tilt, etc.), the right move is a proper Extended State Observer rather
% than bolting nonlinearities onto the linear KF predictor.
Gn = eye(4);
Qn = diag([1e-10, 1e-9, 3e-3, 2e-5]);
Rn = diag([5e-8, 1.5e-7]);

L = lqe(A_sw, Gn, C_sw, Qn, Rn);
Aobs = A_sw - L*C_sw;
Bobs = [B_sw, L];
Cobs = eye(4);
Dobs = zeros(4, 3);
A_obs = Aobs;  B_obs = Bobs;  C_obs = Cobs;  D_obs = Dobs;

Ld = dlqe(Ad, Gn, Cd, Qn, Rn);
Aobs_d = Ad - Ld*Cd;
Bobs_d = [Bd, Ld];
Cobs_d = eye(4);
Dobs_d = zeros(4, 3);

save(fullfile(SEESAW_ROOT, 'data', 'params', 'observer_kalman.mat'), ...
    'A_obs', 'B_obs', 'C_obs', 'D_obs', 'L', 'Qn', 'Rn', ...
    'Aobs_d', 'Bobs_d', 'Ld');
save(fullfile(SEESAW_ROOT, 'data', 'params', 'kalman_observer.mat'), ...
    'A_obs', 'B_obs', 'C_obs', 'D_obs', 'L', 'Qn', 'Rn', ...
    'Aobs_d', 'Bobs_d', 'Ld');
fprintf('Saved 4-state Kalman observer to data/params/observer_kalman.mat and kalman_observer.mat\n');

%% 6. TRAJECTORY TRACKING (tension feedforward)

% Parameterized trajectory references for Simulink
% Use these variables directly in Sine Wave blocks.
% Example block settings:
%   theta_ref:     Amp = theta_ref_amp,     Bias = theta_ref_bias,
%                  Freq = theta_ref_freq,   Phase = theta_ref_phase
%   thetadot_ref:  Amp = thetadot_ref_amp,  Bias = thetadot_ref_bias,
%                  Freq = thetadot_ref_freq, Phase = thetadot_ref_phase
%   thetaddot_ref: Amp = thetaddot_ref_amp, Bias = thetaddot_ref_bias,
%                  Freq = thetaddot_ref_freq, Phase = thetaddot_ref_phase
%   xc_ref:        Amp = xc_ref_amp,        Bias = xc_ref_bias,
%                  Freq = xc_ref_freq,      Phase = xc_ref_phase
%   xcdot_ref:     Amp = xcdot_ref_amp,     Bias = xcdot_ref_bias,
%                  Freq = xcdot_ref_freq,   Phase = xcdot_ref_phase
%   xcddot_ref:    Amp = xcddot_ref_amp,    Bias = xcddot_ref_bias,
%                  Freq = xcddot_ref_freq,  Phase = xcddot_ref_phase

% Body-angle reference
Aref = 8/180*pi;               % about 3 deg in rad
fref = 0.1;                 % reference frequency in Hz
wref = 2*pi*fref;           % reference frequency in rad/s

theta_ref_amp   = Aref;
theta_ref_bias  = 0;
theta_ref_freq  = wref;
theta_ref_phase = 0;

thetadot_ref_amp   = theta_ref_amp * theta_ref_freq;
thetadot_ref_bias  = 0;
thetadot_ref_freq  = theta_ref_freq;
thetadot_ref_phase = theta_ref_phase + pi/2;

thetaddot_ref_amp   = theta_ref_amp * theta_ref_freq^2;
thetaddot_ref_bias  = 0;
thetaddot_ref_freq  = theta_ref_freq;
thetaddot_ref_phase = theta_ref_phase + pi;

% Dynamically consistent cart reference associated with theta_ref
% For a sinusoidal theta_ref, solve for a harmonic xc_ref that satisfies
% the coupled plant dynamics together with a single-input feedforward.
% This avoids the inconsistent choice xc_ref = 0 for a nonzero theta_ref.

a31 = A_sw(3,1); a32 = A_sw(3,2); a33 = A_sw(3,3); a34 = A_sw(3,4);
a41 = A_sw(4,1); a42 = A_sw(4,2); a43 = A_sw(4,3); a44 = A_sw(4,4);
b3  = B_sw(3,1); b4  = B_sw(4,1);

r34 = b3 / b4;
c1 = a31 - r34*a41;
c2 = a32 - r34*a42;
c3 = a33 - r34*a43;
c4 = a34 - r34*a44;

Theta_ref_phasor = theta_ref_amp * exp(1j * theta_ref_phase);
Hxc_theta = (c2 + 1j*c4*theta_ref_freq - r34*theta_ref_freq^2) ...
          / (-theta_ref_freq^2 - 1j*c3*theta_ref_freq - c1);
Xc_ref_phasor = Hxc_theta * Theta_ref_phasor;

xc_ref_amp   = abs(Xc_ref_phasor);
xc_ref_bias  = 0;
xc_ref_freq  = theta_ref_freq;
xc_ref_phase = angle(Xc_ref_phasor);

xcdot_ref_amp   = xc_ref_amp * xc_ref_freq;
xcdot_ref_bias  = 0;
xcdot_ref_freq  = xc_ref_freq;
xcdot_ref_phase = xc_ref_phase + pi/2;

xcddot_ref_amp   = xc_ref_amp * xc_ref_freq^2;
xcddot_ref_bias  = 0;
xcddot_ref_freq  = xc_ref_freq;
xcddot_ref_phase = xc_ref_phase + pi;

% Full-state feedforward for trajectory tracking
% Use the full dynamically consistent reference state to build:
%
%   u_ff = uff_xc_gain*xc_ref + uff_theta_gain*theta_ref ...
%        + uff_xcdot_gain*xcdot_ref + uff_thetadot_gain*thetadot_ref ...
%        + uff_thetaddot_gain*thetaddot_ref
%
uff_xc_gain        =  g * M_total / D_T;
uff_theta_gain     = -g * M_SW * D_C / D_T;
uff_xcdot_gain     =  B_eq;
uff_thetadot_gain  =  B_SW / D_T;
uff_thetaddot_gain =  J_pivot / D_T;

% Consistency check for the chosen tracking reference and full feedforward
% This check evaluates whether the chosen reference trajectory xref(t) and
% the model-based feedforward uff(t) satisfy the plant dynamics exactly:
%
%   xref_dot = A*xref + B*uff
%
% A small residual indicates that the selected reference is dynamically
% feasible for the single-input plant.

t_check = linspace(0, 2/fref, 1000);
xc_ref_check        = xc_ref_amp * sin(xc_ref_freq * t_check + xc_ref_phase) + xc_ref_bias;
theta_ref_check     = theta_ref_amp * sin(theta_ref_freq * t_check + theta_ref_phase) + theta_ref_bias;
xcdot_ref_check     = xcdot_ref_amp * sin(xcdot_ref_freq * t_check + xcdot_ref_phase) + xcdot_ref_bias;
thetadot_ref_check  = thetadot_ref_amp * sin(thetadot_ref_freq * t_check + thetadot_ref_phase) + thetadot_ref_bias;
xcddot_ref_check    = xcddot_ref_amp * sin(xcddot_ref_freq * t_check + xcddot_ref_phase) + xcddot_ref_bias;
thetaddot_ref_check = thetaddot_ref_amp * sin(thetaddot_ref_freq * t_check + thetaddot_ref_phase) + thetaddot_ref_bias;

uff_nominal_check = uff_xc_gain * xc_ref_check ...
                  + uff_theta_gain * theta_ref_check ...
                  + uff_xcdot_gain * xcdot_ref_check ...
                  + uff_thetadot_gain * thetadot_ref_check ...
                  + uff_thetaddot_gain * thetaddot_ref_check;

uff_check = uff_nominal_check;

xref_check     = [xc_ref_check; theta_ref_check; xcdot_ref_check; thetadot_ref_check];
xrefdot_check  = [xcdot_ref_check; thetadot_ref_check; xcddot_ref_check; thetaddot_ref_check];
residual_check = zeros(4, numel(t_check));

for k = 1:numel(t_check)
    residual_check(:,k) = xrefdot_check(:,k) - (A_sw * xref_check(:,k) + B_sw * uff_nominal_check(k));
end

residual_max_abs = max(abs(residual_check), [], 2);
residual_rms = sqrt(mean(residual_check.^2, 2));

%% 7. LIFT-UP

% Re-use the tuned position-control PID from freq based technique to move
% the cart up to 0.0568 because we know from the quasi-static test that
% this is the point at which it starts to lift.
% After that, use the flip-flop to switch from lift to stabilization only
% the first time it crosses 5°.

% Load the voltage non-linearities
cart_file = fullfile(SEESAW_ROOT, 'data', 'controllers', 'controller_inner_pid.mat');
if exist(cart_file, 'file')
    load(cart_file, 'Kp_in', 'Ki_in', 'Kd_in', 'N_in', 'antiwindup_in');
else
    error('Inner-loop PID not found. Run pid_cart first.');
end

init_cond_lift = [-0.407, deg2rad(11.66), 0, 0];
init_cond_switch = [0.057, deg2rad(5), 0, -0.1];

% Save LQR controller parameters
K_lqr = K5;
save(fullfile(SEESAW_ROOT, 'data', 'controllers', 'controller_lqr.mat'), 'K_lqr', 'Q5', 'R5', 'A5', 'B5');
fprintf('Saved LQR controller parameters to data/controllers/controller_lqr.mat\n');

%% 8. OBSERVER HW DATA VALIDATION & PLOTTING
% Run the 4-state KF on the hardware log and compare against a causal 1-pole
% dirty derivative (tau=10ms) — the straw-man baseline. Useful to confirm
% the KF still beats DD on alpha_dot (~32% in balance) after any retune.
kf_file = fullfile(SEESAW_ROOT, 'data', 'analysis', 'kf.mat');
if exist(kf_file, 'file')
    fprintf('Running Kalman Filter validation on hardware log data (kf.mat)...\n');
    load(kf_file);

    t_hw     = data(1,:)';
    xc_hw    = data(5,:)';
    alpha_hw = data(3,:)';
    vm_hw    = data(6,:)';
    dt_hw    = mean(diff(t_hw));

    % Zero-phase 15 Hz LPF + central gradient reference
    fc_ref = 15; tau_ref = 1/(2*pi*fc_ref); a_ref = tau_ref/(tau_ref+dt_hw);
    bf = [1-a_ref, 0]; af_ = [1, -a_ref];
    zphase = @(x) flipud(filter(bf, af_, flipud(filter(bf, af_, x))));
    xcd_ref_hw = zphase(gradient(zphase(xc_hw),    dt_hw));
    ald_ref_hw = zphase(gradient(zphase(alpha_hw), dt_hw));

    % Causal 1-pole DD baseline (tau = 10ms)
    tau_dd = 0.01;
    xcd_dd = zeros(size(xc_hw)); ald_dd = zeros(size(alpha_hw));
    for k = 2:length(xc_hw)
        xcd_dd(k) = (tau_dd/(tau_dd+dt_hw))*xcd_dd(k-1) + (1/(tau_dd+dt_hw))*(xc_hw(k) - xc_hw(k-1));
        ald_dd(k) = (tau_dd/(tau_dd+dt_hw))*ald_dd(k-1) + (1/(tau_dd+dt_hw))*(alpha_hw(k) - alpha_hw(k-1));
    end

    % 4-state Kalman observer
    xhat = zeros(4, length(t_hw));
    xhat(:,1) = [xc_hw(1); alpha_hw(1); 0; 0];
    for k = 2:length(t_hw)
        xhat(:, k) = Aobs_d * xhat(:, k-1) + Bobs_d * [vm_hw(k); xc_hw(k); alpha_hw(k)];
    end
    xcd_hat = xhat(3, :)'; ald_hat = xhat(4, :)';

    % Report
    m_lift = (t_hw > 3.5) & (t_hw < 5.0);
    m_bal  = (t_hw > 6.0) & (t_hw < 52.0);
    rmse_  = @(a,b,m) sqrt(mean((a(m)-b(m)).^2));
    fprintf('  RMSE vs zero-phase reference:\n');
    fprintf('  filter               | xcd lift | xcd bal | ald lift | ald bal\n');
    fprintf('  Causal DD (tau=10ms) | %.4f   | %.4f  | %.4f   | %.4f\n', ...
        rmse_(xcd_dd, xcd_ref_hw, m_lift), rmse_(xcd_dd, xcd_ref_hw, m_bal), ...
        rmse_(ald_dd, ald_ref_hw, m_lift), rmse_(ald_dd, ald_ref_hw, m_bal));
    fprintf('  4-state Kalman       | %.4f   | %.4f  | %.4f   | %.4f\n', ...
        rmse_(xcd_hat, xcd_ref_hw, m_lift), rmse_(xcd_hat, xcd_ref_hw, m_bal), ...
        rmse_(ald_hat, ald_ref_hw, m_lift), rmse_(ald_hat, ald_ref_hw, m_bal));

    % Comparison figure
    fig = figure('Position', [80, 80, 1300, 900], 'Visible', 'off');

    subplot(3,1,1);
    plot(t_hw, xcd_ref_hw, 'k', 'LineWidth', 1.6); hold on;
    plot(t_hw, xcd_dd,     'g--','LineWidth', 1.0);
    plot(t_hw, xcd_hat,    'b',  'LineWidth', 1.1);
    grid on; xlim([3.5, 52]); ylabel('Cart Velocity [m/s]');
    legend('Zero-phase ref (LPF+grad)','Causal DD (tau=10ms)','4-state Kalman', ...
           'Location','best');
    title('Velocity estimators on hardware log');

    subplot(3,1,2);
    plot(t_hw, ald_ref_hw, 'k', 'LineWidth', 1.6); hold on;
    plot(t_hw, ald_dd,     'g--','LineWidth', 1.0);
    plot(t_hw, ald_hat,    'b',  'LineWidth', 1.1);
    grid on; xlim([3.5, 52]); ylabel('Angular Velocity [rad/s]');

    subplot(3,1,3);
    plot(t_hw, ald_ref_hw, 'k', 'LineWidth', 2.0); hold on;
    plot(t_hw, ald_dd,     'g--','LineWidth', 1.4);
    plot(t_hw, ald_hat,    'b',  'LineWidth', 1.6);
    grid on; xlim([14, 16]); xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
    title('Balance zoom — KF smooths alpha-encoder quantization that DD steps through');

    save_dir = fullfile(SEESAW_ROOT, 'data', 'analysis');
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    save_path = fullfile(save_dir, 'kf_validation.png');
    saveas(fig, save_path);
    fprintf('  Saved validation plot to: data/analysis/kf_validation.png\n');
else
    warning('kf.mat not found. Skipping plotting validation.');
end