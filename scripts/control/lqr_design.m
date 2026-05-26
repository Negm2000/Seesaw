clear; close all; clc;

% Configure global plot settings
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. LOAD SYSTEM PARAMETERS
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = pwd; end 

% Load physical parameters (g, Me, DT, Jsw, Bsw, Msw, DC, etc.)
seesaw_params; 

% Load the voltage non-linearities
nonlinear_file = fullfile(SEESAW_ROOT, 'data', 'param_nonlinear.mat');
if exist(nonlinear_file, 'file')
    load(nonlinear_file, 'ud_pos', 'ud_neg', 'ud_sym');
else
    error('Non-linearities not found. Run seesaw nonlinear first.');
end

% Load the tuned state-space representation
cart_file = fullfile(SEESAW_ROOT, 'data', 'tuned_cart.mat');
if exist(cart_file, 'file')
    load(cart_file);
else
    error('Tuned cart model not found. Run cart modeling first.');
end

% Load the tuned state-space representation
seesaw_file = fullfile(SEESAW_ROOT, 'data', 'tuned_seesaw.mat');
if exist(seesaw_file, 'file')
    load(seesaw_file);
else
    error('Tuned seesaw model not found. Run seesaw modeling first.');
end

%% 2. DEFINE LIMITS/WEIGHTS (BRYSON'S RULE)

req_dist     = 0.05;            % maximum displacement [m]
req_angle    = deg2rad(1);      % maximum angle [rad]
req_speed    = 0.1;             % maximum speed [m/s]
%req_angspeed = +inf
req_integral = 2 * deg2rad(2);  % maximum time to clear a given angle
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

[K4, P4, ~] = lqr(A_sw, B_sw, Q4, R4);
eig_open_4  = eig(A_sw);
eig_close_4 = eig(A_sw - B_sw*K4);

[K5, P5, ~] = lqr(A5,B5,Q5,R5);
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

N4 = 25*abs(dom_pole4);

H4 = N4*s / (s + N4);
H4d = c2d(H4, Ts, 'tustin');

% initial conditions
init_cond4 = [0, deg2rad(1), 0, 0];

%% 5. PERFORM THE LQR/LQI IN DISCRETE TIME FOR THE HARDWARE

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

% Qn: process noise covariance
% Rn: measurement noise covariance
% In practice, Rn is usually estimated from sensor data first,
% while Qn is then tuned to reflect the confidence in the model.
Gn = eye(4);
Qn = diag([1e-7, 7.6154e-7, 1e-6, 1e-6]);
Rn = diag([2.275e-5, 0.00015]);

L = lqe(A_sw, Gn, C_sw, Qn, Rn);

Aobs = A_sw - L*C_sw;
Bobs = [B_sw, L];
Cobs = eye(4);
Dobs = zeros(4, 3);

Ld = dlqe(Ad, Gn, Cd, Qn, Rn);

Aobs_d = Ad - Ld*Cd;
Bobs_d = [Bd, Ld];
Cobs_d = eye(4);
Dobs_d = zeros(4, 3);

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
inner_pid_file = fullfile(SEESAW_ROOT, 'data', 'controller_inner_pid.mat');
if exist(inner_pid_file, 'file')
    load(inner_pid_file, 'Kp_in', 'Ki_in', 'Kd_in', 'N_in', 'antiwindup_in');
else
    error('Inner-loop PID not found. Run pid_cart first.');
end

init_cond_lift = [-0.407, deg2rad(11.66), 0, 0];
init_cond_switch = [0.057, deg2rad(5), 0, -0.1];