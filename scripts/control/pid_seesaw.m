%% Inner Loop PID Design (Derivative on Feedback Version)
% This script simulates the "Derivative on Output" architecture
% to verify that current limits are respected without a Bessel filter.

clear; close all; clc;

%% 1. PARAMETERS & DESIGN SPECS
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = pwd; end 

seesaw_params; % Loads V_max, I_max, etc.
load(fullfile(pwd, 'data', 'tuned_cart.mat')); % Loads Gx

wc_out = sqrt(2.2441 * 8.8589); % ~4.45 rad/s
wc_in = 4 * wc_out;             % Bandwidth ~17.8 rad/s (Healthy separation)
PM_in = 80;                     % High PM for no overshoot
Ti_ratio = 10;
N_in = 100;
Tf_in = 1/N_in;

%% 2. GAIN CALCULATION (Same Frequency Point Method)
[mag_p, phase_p] = bode(Gx, wc_in);
mag_p = squeeze(mag_p); phase_p = squeeze(phase_p);

phi_c_deg = -180 + PM_in - phase_p;
C_target = (1/mag_p) * exp(1j * deg2rad(phi_c_deg));

Ti = Ti_ratio / wc_in;
w = wc_in;
D_jw = (1j*w) / (1j*w*Tf_in + 1);

A_mat = [1, real(D_jw); -1/(w*Ti), imag(D_jw)];
B_vec = [real(C_target); imag(C_target)];
gains = A_mat \ B_vec;

Kp_in = gains(1); Kd_in = gains(2); Ki_in = Kp_in / Ti;

%% 3. PHYSICAL VERIFICATION (2-DOF Sensitivity)
s = tf('s');
% Loop Transfer Function (L) remains the same for stability
L = (Kp_in + Ki_in/s + (Kd_in*s)/(Tf_in*s + 1)) * Gx;

% NEW: Sensitivity for Derivative on Feedback
% U(s)/R(s) = (Kp + Ki/s) / (1 + L(s)) 
% Notice Kd is gone from the numerator!
S_u_2dof = (Kp_in + Ki_in/s) / (1 + L);

% Simulation: Raw 5cm Step (No Bessel Filter)
t = 0:0.002:1.0;
ref = 0.05 * ones(size(t)); % Pure Step
[v_cmd, ~] = lsim(S_u_2dof, ref, t);
[x_pos, ~] = lsim(feedback(L,1), ref, t); % Closed-loop tracking
v_cart = gradient(x_pos, t);

% Current Calculation
V_emf = (K_g * k_m / r_mp) * v_cart;
I_req = (v_cmd - V_emf) / R_m;

max_V = max(abs(v_cmd));
max_I = max(abs(I_req));

fprintf('--- 2-DOF Safety Check (5cm RAW Step) ---\n');
fprintf('Max Voltage: %.2f V | Max Current: %.2f A\n', max_V, max_I);

if max_I > 1.0
    warning('Current limit (1.0A) exceeded! Reduce wc_in or PM_in.');
else
    disp('Status: SUCCESS. Derivative-on-Feedback is hardware-safe.');
end