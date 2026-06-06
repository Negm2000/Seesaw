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

%% 6. SIMULATIONS AND PLOTS
% =========================================================================
% Setup Time and Conditions
% =========================================================================
t_sim = 0:0.002:5;
theta0_deg = 1.0;
x0_4 = [0; deg2rad(theta0_deg); 0; 0];

% =========================================================================
% TEST 1: 1° Initial Condition Response (LQR)
% =========================================================================
disp('Running Test 1: 1° initial condition (LQR)...');

[t_yf, yf_x, uf_v, mf] = sim_lqr(A_sw, B_sw, K4, x0_4, t_sim);

figure('Name', 'Test 1: LQR IC Response');
subplot(3,1,1); plot(t_yf, rad2deg(yf_x(:,3)), 'LineWidth', 1.2); grid on
ylabel('Angle $\theta$ [$^\circ$]')
title(sprintf('LQR IC Response (%.1f°)', theta0_deg))
subplot(3,1,2); plot(t_yf, yf_x(:,1), 'LineWidth', 1.2); grid on
ylabel('Position $x_c$ [m]')
subplot(3,1,3); plot(t_yf, uf_v, 'LineWidth', 1.2); grid on
ylabel('Voltage $v_{cmd}$ [V]'); xlabel('Time [s]')

% =========================================================================
% TEST 2: 100g Load Bias Comparison (LQR vs LQI)
% =========================================================================
disp('Running Test 2: 100g Load Bias Comparison...');

% Simulating the bias offset. (M_inv is assumed loaded via seesaw_params.m)
[t_bias4, x_bias4, u_bias4, ~] = sim_bias_load(A_sw - B_sw*K4, K4, M_inv);
[t_bias5, x_bias5, u_bias5, ~] = sim_bias_load_aug(A5 - B5*K5, K5, M_inv);

figure('Name', 'Test 2: 100g Load Bias Comparison');
subplot(3,1,1); hold on; grid on;
plot(t_bias4, rad2deg(x_bias4(:,3)), 'b', 'LineWidth', 1.5);
plot(t_bias5, rad2deg(x_bias5(:,3)), 'r--', 'LineWidth', 1.5);
ylabel('Angle $\theta$ [$^\circ$]');
title('100g Load Bias Response Comparison');
legend('LQR (No Integral)', 'LQI (With Integral)', 'Location', 'Best');

subplot(3,1,2); hold on; grid on;
plot(t_bias4, x_bias4(:,1)*100, 'b', 'LineWidth', 1.5);
plot(t_bias5, x_bias5(:,1)*100, 'r--', 'LineWidth', 1.5);
ylabel('Position $x_c$ [cm]');

subplot(3,1,3); hold on; grid on;
plot(t_bias4, u_bias4, 'b', 'LineWidth', 1.5);
plot(t_bias5, u_bias5, 'r--', 'LineWidth', 1.5);
ylabel('Voltage $v_{cmd}$ [V]');
xlabel('Time [s]');

% =========================================================================
% TEST 3: Kalman Observer vs Full-State LQR Response
% =========================================================================
disp('Running Test 3: Kalman Observer vs Controller...');

% Combine the physical plant dynamics with the Kalman filter error dynamics
A_combined = [A_sw - B_sw*K4,   B_sw*K4;
              zeros(4,4),       A_sw - L*C_sw];

% Initialize observer using only measured positions (zero initial velocities)
xhat0  = [x0_4(1); x0_4(2); 0; 0]; 
sys_combined = ss(A_combined, zeros(8,1), eye(8), zeros(8,1));
z = initial(sys_combined, [x0_4; x0_4-xhat0], t_sim);

x_hist = z(:, 1:4);
e_hist = z(:, 5:8);
u_hist = -(K4 * (x_hist - e_hist)')';

figure('Name', 'Test 3: Observer vs Controller IC');
subplot(3,1,1); plot(t_sim, rad2deg(x_hist(:,3)), 'b-', 'LineWidth', 1.2); hold on;
plot(t_yf, rad2deg(yf_x(:,3)), 'k--', 'LineWidth', 1.2); grid on;
ylabel('Angle $\theta$ [$^\circ$]');
legend('Kalman Observer', 'Full-State LQR', 'Location', 'Best')
title(sprintf('Observer vs LQR IC Response (%.1f°)', theta0_deg))

subplot(3,1,2); plot(t_sim, x_hist(:,1), 'b-', 'LineWidth', 1.2); hold on;
plot(t_yf, yf_x(:,1), 'k--', 'LineWidth', 1.2); grid on;
ylabel('Position $x_c$ [m]')

subplot(3,1,3); plot(t_sim, u_hist, 'b-', 'LineWidth', 1.2); hold on;
plot(t_yf, uf_v, 'k--', 'LineWidth', 1.2); grid on;
ylabel('Voltage $v_{cmd}$ [V]'); xlabel('Time [s]')

%% Helpers

function [t_out, y_out, v_out, m] = sim_lqr(A, B, K, x0, t)
    Acl = A - B*K;
    sys = ss(Acl, zeros(size(A,1), 1), eye(size(A,1)), zeros(size(A,1), 1));
    [y_out, t_out] = initial(sys, x0, t);
    voltage = -K * y_out';
    m.peak_v = max(abs(voltage));
    m.peak_y = max(abs(y_out));
    v_out = voltage';
end

function [t, x, v, m] = sim_bias_load(Acl, K, M_inv)
    t = 0:0.001:5;
    % Bias torque constant 0.123 Nm maps to accelerations
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