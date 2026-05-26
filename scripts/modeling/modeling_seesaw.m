%% MODELING PIPELINE — Quanser IP02 + SEESAW-E
%  =====================================================================
%  Master script for system identification and model validation.
%  Run each section (Ctrl+Enter) in order. Sections 1-3 are pre-hardware,
%  Section 4 requires you to go collect data, Sections 5-10 are post-hardware.
%
%  To convert to Live Script: right-click this file → Open as Live Script
%  =====================================================================

%% 1. LOAD SYSTEM PARAMETERS
%  Load all Quanser hardware specs from seesaw_params.m.
%  This populates ~30 variables including B_eq (tuning target).

seesaw_params;
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
data_file = fullfile(SEESAW_ROOT, 'data', 'tuned_cart.mat');

if ~exist(data_file, 'file')
    error('tuned cart data not found.');
end

if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
data_file = fullfile(SEESAW_ROOT, 'data', 'param_nonlinear.mat');

if ~exist(data_file, 'file')
    error('non-linear data not found.');
end

D_C_nominal = D_C;  % save for comparison later
fprintf('\n----- Nominal Model -----\n');
fprintf('  D_C     = %.2f N*s/m (will be tuned)\n', D_C_nominal);
fprintf('-------------------------\n');

%% 4. LOAD & INSPECT HARDWARE DATA
%  Load the frequency sweep data collected from QUARC.
%  Plot raw time traces to sanity-check before analysis.

if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
data_file = fullfile(SEESAW_ROOT, 'data', 'seesawModeling', 'step_seesaw.mat');

if ~exist(data_file, 'file')
    error('data not found. Run quasi-static test on hardware first.');
end

fprintf('Loading %s ...\n', data_file);
loaded = load(data_file);
vars = fieldnames(loaded);

% Handle different QUARC data formats
if ismember('ip02_freq_data', vars)
    raw = loaded.ip02_freq_data;
elseif ismember('data', vars)
    raw = loaded.data;
else
    error('Expected variable "ip02_freq_data" or "data" in data.mat. Found: %s', strjoin(vars, ', '));
end

% Extract columns: [time; x_c; theta]
t_hw      = raw(1, :)';         % [V]
xc_hw     = raw(2, :)';         % [m]
theta_hw  = raw(3, :)' + 23.32; % [°]

theta_rest = mean(theta_hw(1:50));
threshold = 0.00045;        % 3 encoder counts

% expected idx = 11468
lift_idx = find(abs(theta_hw) < (theta_rest - threshold), 1, 'first');

if isempty(lift_idx)
    error('Lift-off not detected. Check if the threshold is too high or if the cart moved far enough.');
end

t_lift  = t_hw(lift_idx);
xc_lift = xc_hw(lift_idx);

fprintf('--- Lift-Off Detected ---\n');
fprintf('Time of lift-off:  %.3f s\n', t_lift);
fprintf('Resting Angle:     %.4f deg (%.2f rad)\n', theta_rest, deg2rad(theta_rest));
fprintf('Initial Pos:       %.4f m\n', xc_hw(1));
fprintf('Cart Position:     %.4f m\n\n', xc_lift);

% Using the static equilibrium equation:
% g * Me * x_lift = g * (Me * DT + Msw * Dc) * theta_rest
theta_rest_rad = deg2rad(theta_rest);
D_C = abs((M_total * xc_lift - M_total * D_T * theta_rest_rad) / (M_SW * theta_rest_rad));

%% 5. Plot the Results for Visual Verification
figure('Name', 'Seesaw Lift-Off Verification', 'Color', 'w', 'Position', [100, 100, 800, 500]);

% Plot Cart Position
yyaxis left;
plot(t_hw, xc_hw, 'b', 'LineWidth', 1.5);
ylabel('Cart Position, $x_c$ (m)');
hold on;

% Plot Seesaw Angle
yyaxis right;
plot(t_hw, theta_hw, 'r', 'LineWidth', 1.5);
ylabel('Seesaw Angle, $\theta$ (rad)');

% Mark the lift-off point
xline(t_lift, '--k', 'Lift-Off Detected', 'LabelOrientation', 'horizontal', 'LabelHorizontalAlignment', 'left');
plot(t_lift, theta_hw(lift_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(t_lift, xc_lift, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

grid on;
title('Quasi-Static Balance Test: Identifying $D_C$');
xlabel('Time (s)');
xlim([0, t_hw(end)]);

%% 7. APPLY TUNED PARAMETERS & REBUILD MODEL
fprintf('\n--- Applying Tuned Parameters ---\n');

% ===== Transfer Function Model 2: Cart on Seesaw (Coupled, Linearised) =====
% Linearised around equilibrium: x_c=0, x_c_dot=0, theta=0, theta_dot=0
%
% Input:  V_m (motor voltage, after amplifier + saturation)
% Output: [x_c; theta]
%
% Equations of motion (Quanser Seesaw Lab Guide, linearised about
%   x_c=0, x_c_dot=0, theta=0, theta_dot=0, L_m=0):
%
%   Seesaw EOM:
%     (J_pivot + M_c*D_T^2)*theta_ddot - M_c*D_T*x_c_ddot
%         + g*M_c*x_c - g*(M_c*D_T + M_SW*D_C)*theta
%         = -B_SW*theta_dot

P21 =  -M_total*D_T*s^2 + g*M_total;
P22 = (J_pivot + M_total*D_T^2)*s^2 + B_SW*s - g*(M_total*D_T + M_SW*D_C);

Gt = minreal(-P21/P22);
num_t = [M_total*D_T, 0, -g*M_total];
den_t = [(J_pivot + M_total*D_T^2), B_SW, -g*(M_total*D_T + M_SW*D_C)]; 

% ===== Linear State-Space Model 2: Cart on Seesaw (Coupled, Linearised) =====
% Linearised around equilibrium: x_c=0, x_c_dot=0, theta=0, theta_dot=0
% QUARC-compatible: uses standard State-Space block (no S-function/TLC needed)
%
% Input:  V_m (motor voltage, after amplifier + saturation)
% States: [x_c; x_c_dot; theta; theta_dot]
% Output: [x_c; x_c_dot; theta; theta_dot]
%
% Equations of motion (Quanser Seesaw Lab Guide, linearised about
%   x_c=0, x_c_dot=0, theta=0, theta_dot=0, L_m=0):
%
%   Cart EOM:
%     M_c*x_c_ddot - M_c*D_T*theta_ddot + B_total*x_c_dot + g*M_c*theta
%         = alpha_f * eta_m * V_m
%
%   Seesaw EOM:
%     (J_pivot + M_c*D_T^2)*theta_ddot - M_c*D_T*x_c_ddot
%         + g*M_c*x_c - g*(M_c*D_T + M_SW*D_C)*theta
%         = -B_SW*theta_dot
%
% Solve for [x_c_ddot; theta_ddot] via effective inertia matrix:
%
%   M_eff = [ M_c,        -M_c*D_T            ]
%           [ -M_c*D_T,    J_pivot+M_c*D_T^2  ]
%
%   G_rhs (state [x_c; x_c_dot; theta; theta_dot]):
%     row 1 (cart):   [0, -B_total, -g*M_c,                   0     ]
%     row 2 (seesaw): [-g*M_c, 0,   g*(M_c*D_T+M_SW*D_C), -B_SW   ]
%
%   [x_c_ddot; theta_ddot] = inv(M_eff) * G_rhs * z + inv(M_eff) * G_inp * V_m

M_eff = [M_e,          -M_total*D_T;
         -M_total*D_T,      J_pivot + M_total*D_T^2];

State_matrix = [ 0,         -g*M_total,                 -B_total,   0;
                -g*M_total, g*(M_total*D_T + M_SW*D_C),     0,    -B_SW];

Input_matrix = [alpha_f*eta_m;
                   0];

State_matrix = M_eff \ State_matrix;
Input_matrix = M_eff \ Input_matrix;

% Augment the matrices to get X = [xc; \theta; \dot{xc}; \dot{\theta}]
A_sw = [ zeros(2), eye(2);
        State_matrix ];
B_sw = [ zeros(2,1);
        Input_matrix];
C_sw = [eye(2), zeros(2)];
D_sw = zeros(2,1);

sys4 = ss(A_sw, B_sw, C_sw, D_sw);

A5 = [A_sw, zeros(4,1);
      0, 1, 0, 0, 0];
B5 = [B_sw;
      0];
C5 = [C_sw, zeros(2,1)];
D5 = D_sw;

sys5 = ss(A5,B5,C5,D5);

%% 10. SUMMARY & SAVE
fprintf('\n');
fprintf('============================================================\n');
fprintf('  MODELING OF SEESAW COMPLETE\n');
fprintf('============================================================\n');
fprintf('\n  Parameter          Nominal    Tuned      Change\n');
fprintf('  -----------------  ---------  ---------  ------\n');
fprintf('  D_C [m]          %8.3f   %8.3f     %+.1f%%\n', ...
    D_C_nominal, D_C, (D_C-D_C_nominal)/D_C_nominal*100);
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
save_file = fullfile(SEESAW_ROOT, 'data', 'tuned', 'tuned_seesaw.mat');
save(save_file, 'D_C', 'Gt', 'num_t', 'den_t', ...
    'A_sw', 'B_sw', 'C_sw', 'D_sw', 'sys4', ...
    'A5', 'B5', 'C5', 'D5', 'sys5');
fprintf('\n  Tuned parameters saved to: data/tuned_seesaw.mat\n');
fprintf('============================================================\n');
