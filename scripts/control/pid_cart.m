%% Manual Inner Loop PID Design (Hardware-Safe Version)
% Targeted at Cart Position (x_c)
% Method: Frequency Point Shaping with Phase Budget Awareness

clear; close all; clc;

%% 1. PARAMETERS & MODEL LOADING
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = pwd; end 

% Load system parameters (Ensure seesaw_params.m defines V_max, I_max, etc.)
seesaw_params; 
load(fullfile(SEESAW_ROOT, 'data', 'param_nonlinear.mat')) % loads ud_pos/neg
load(fullfile(SEESAW_ROOT, 'data', 'tuned_cart.mat')); % Loads Gx


%% 2. DESIGN SPECIFICATIONS
% Geometric mean of unstable pole/zero for reference
wc_out = sqrt(2.2441 * 8.8589); % ~4.45 rad/s

% --- TUNING KNOBS ---
wc_in = 2.5 * wc_out; % Lowered to ~8 rad/s (reduces Kp)
PM_in = 85;         % Dropped to 65 deg (standard for no overshoot, reduces Kd)
Ti_ratio = 10;      % Slower integrator (reduces phase lag/high-freq effort)
wf_bes = 3.5;         % Softer Bessel filter (limits the initial current surge)
N_in = 100;         % Smoother derivative (less noise amplification)
Tf_in = 1/N_in;

%% 3. MANUAL PID CALCULATION
[mag_p, phase_p] = bode(Gx, wc_in);
mag_p = squeeze(mag_p); phase_p = squeeze(phase_p);

G_jw = mag_p * exp(1j * deg2rad(phase_p));

% Target Phase for Controller: Phi_c = -180 + PM - Phi_plant
phi_c_deg = -180 + PM_in - phase_p;

mag_c = 1 / mag_p;
C_target = mag_c * exp(1j * deg2rad(phi_c_deg));

% Set Integrator Time Constant
Ti = Ti_ratio / wc_in; 
w = wc_in;
D_jw = (1j*w) / (1j*w*Tf_in + 1); % Filtered derivative FRF

% Solve 2x2 System for Kp and Kd
A_mat = [1, real(D_jw); -1/(w*Ti), imag(D_jw)];
B_vec = [real(C_target); imag(C_target)];
gains = A_mat \ B_vec;

Kp_in = gains(1);
Kd_in = gains(2);
Ki_in = Kp_in / Ti;
antiwindup_in = Ki_in / Kp_in;

C_in = Kp_in + Ki_in/s + (Kd_in * s)/(Tf_in * s + 1);

fprintf('---- Computed Gains ----\n');
fprintf('Kp: %.4f | Ki: %.4f | Kd: %.4f | N: %.1f\n', Kp_in, Ki_in, Kd_in, N_in);
fprintf('--- Final Controller ---\n');
C_in
fprintf('------------------------\n');


% Define Transfer Functions
% Note: L_in and T_in are defined by the total controller C_total
L_in = series(C_in, Gx);    % Open-Loop
T_in = feedback(L_in, 1);   % Closed-Loop
S_u_fb = C_in / (1 + L_in); % Feedback voltage sensitivity

%% 6. SAVE
%save(fullfile(SEESAW_ROOT, 'data', 'controller_inner_pid.mat'), ...
%    'Kp_in', 'Ki_in', 'Kd_in', 'N_in', 'antiwindup_in', ...
%    'C_in', 'L_in', 'T_in');
fprintf('\n  Tuned Inner Controller saved to: data/controller_inner_pid.mat\n');

%% 4. BESSEL PRE-FILTER DESIGN
num_bes = [3 * wf_bes^2];
den_bes = [1, 3 * wf_bes, 3 * wf_bes^2];
F_bessel = tf(num_bes, den_bes);

%% 5. PHYSICAL VERIFICATION FOR (FILTERED) STEP RESPONSE
% (Voltage, Current, Force with Switching Friction Comp)
% This section models the exact non-linear switching logic to verify 
% Peak and RMS limits for Voltage, Current, and Force.

disp('==================================================');
disp('        BUILDING OPEN-LOOP STEP RESPONSE           ');
disp('==================================================');

% 2. Safety Limits (Mechanical, Electronic, Thermal)
G_force = minreal(alpha_f * eta_m * (M_e * s + B_eq) / (M_e * s + B_total));
F_clicking_limit = 4.5 * abs(evalfr(G_force, 14j)); % Based on 4V @ 14 rad/s test

% 3. Simulation: 5cm Filtered Step
step_size = 0.1; %[m]
t = 0:0.002:5.0; % 5s to allow RMS values to stabilize
ref_filtered = step_size * step(F_bessel, t); 

[v_fb, ~] = lsim(S_u_fb, ref_filtered, t); % Safe Control Voltage (u_safe)
[x_pos, ~] = lsim(T_in, ref_filtered, t);  % Predicted cart motion
v_cart = gradient(x_pos, t);               % Cart velocity for Back-EMF

% 4. NON-LINEAR FRICTION CANCELLATION LOGIC
% Implements: If u_s > 0.1: +ud_pos; elseif u_s < -0.1: -ud_neg; else: 0
v_total = zeros(size(v_fb));
v_total(v_fb > 0.1)  = v_fb(v_fb > 0.1) + ud_pos;
v_total(v_fb < -0.1) = v_fb(v_fb < -0.1) - ud_neg;
% All values between -0.1 and 0.1 remain 0 as per your logic.

% 5. Physics Calculations (Using v_total seen by hardware)
V_emf = (K_g * k_m / r_mp) * v_cart;
I_req = (v_total - V_emf) / R_m;
F_req = (eta_g * eta_m * K_g * k_t / r_mp) * I_req;

% 6. Statistical Analysis (Peak vs RMS)
peak_V = max(abs(v_total));  rms_V = sqrt(mean(v_total.^2));
peak_I = max(abs(I_req));    rms_I = sqrt(mean(I_req.^2));
peak_F = max(abs(F_req));    rms_F = sqrt(mean(F_req.^2));

% 7. Final Comprehensive Safety Report
fprintf('\n--- HARDWARE SAFETY: PEAK (Electronic/Mechanical) ---\n');
fprintf('Peak Voltage: %.2f V | Limit (V_sat):   %.2f V\n', peak_V, V_sat);
fprintf('Peak Current: %.2f A | Limit (I_peak):  %.2f A\n', peak_I, 3.0);
fprintf('Peak Force:   %.2f N | Limit (Clicking): %.2f N\n', peak_F, F_clicking_limit);

fprintf('\n--- HARDWARE SAFETY: RMS (Thermal) ---\n');
fprintf('RMS Voltage:  %.2f V | Limit (V_nom):   %.2f V\n', rms_V, V_nom);
fprintf('RMS Current:  %.2f A | Limit (I_max):   %.2f A\n', rms_I, 1.0);


    % Format array for Simulink "From File" block (Row 1: time, Row 2: data)
simulink_step = [t(:)'; ref_filtered(:)'];
    

% Final Safety Logic
if peak_I > 3.0 || peak_V > V_sat
    warning('ELECTRONIC DANGER: Peak current or voltage exceeds protection limits!');
elseif peak_F > F_clicking_limit
    warning('MECHANICAL DANGER: Peak force exceeds the Clicking Limit!');
elseif rms_I > 1.0
    warning('THERMAL DANGER: RMS current exceeds motor continuous rating.');
else
    disp('VERDICT: Controller + Non-linear Friction Comp is hardware-safe for the step response.');
    
    % --- Save Data for Simulink ---
    if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = pwd; end 
    data_dir = fullfile(SEESAW_ROOT, 'data', 'cartControl');
    if ~exist(data_dir, 'dir'), mkdir(data_dir); end
    
    %save(fullfile(data_dir, 'step_simulink.mat'), 'simulink_step', '-v7.3');
    fprintf('>>> Saved to: /data/cartControl/step_simulink.mat\n');
    fprintf('>>> In Simulink, point your "From File" block to this file and run for 5.0 seconds.\n');
end


%% 7. CLOSED-LOOP FREQUENCY VALIDATION (MULTI-SINE GENERATOR)
% Generates a Schroeder-phased multi-sine POSITION reference to validate 
% the inner loop Bode plot without exceeding mechanical force limits.

disp('==================================================');
disp('   BUILDING CLOSED-LOOP MULTI-SINE REFERENCE      ');
disp('==================================================');

% --- 1. Multi-Sine Parameters ---
% Since closed-loop tracks perfectly at low freq, we can safely go low.
w_min = 0.5;                % [rad/s] Low freq to capture DC behavior
w_max = 1.5 * wc_in;        % [rad/s] High enough to see the ~7.3 rad/s roll-off
N_freq = 10;                % Number of frequency points
X_MAX_BUDGET = 0.1;        % [m] Peak position reference (3 cm)

% Duration: Capture at least 3 full cycles of the slowest wave
T_test = max(15, 3 * (2*pi/w_min)); 

% --- 2. Generate Logarithmic Frequencies & Schroeder Phases ---
w_vals = logspace(log10(w_min), log10(w_max), N_freq);
k = 1:N_freq;
phi = -pi * k .* (k - 1) / N_freq; 

% --- 3. AMPLITUDE TAPERING (The Fix for High Force) ---
% Force scales with A * w^2. We reduce high-frequency amplitude to save the rack.
% A taper_exponent of 1.0 means amplitude drops linearly with frequency.
% Increase to 1.5 or 2.0 if the force is still too high.
taper_exponent = 1.0; 
A = (w_min ./ w_vals) .^ taper_exponent;

t_ms = 0:0.002:T_test;
x_ref_raw = zeros(size(t_ms));

for i = 1:N_freq
    % Multiply the sine wave by our explicitly tapered amplitude array
    x_ref_raw = x_ref_raw + A(i) * sin(w_vals(i) * t_ms + phi(i));
end

% Scale to exactly match our 3cm budget
x_ref_safe = (X_MAX_BUDGET / max(abs(x_ref_raw))) * x_ref_raw;

% --- 3.5 FADE-IN WINDOW (Fix for t=0 Shock) ---
% Schroeder phases cause a non-zero start. We apply a half-cosine 
% fade-in over the first 1.5 seconds to ensure x(0) = 0 and v(0) = 0.
fade_time = 1.5; % seconds
fade_indices = find(t_ms <= fade_time);
fade_envelope = sin(linspace(0, pi/2, length(fade_indices)));

% Apply the envelope to the beginning of the signal
x_ref_safe(fade_indices) = x_ref_safe(fade_indices) .* fade_envelope;

% --- 4. Simulate Closed-Loop Physics ---
[v_fb_ms, ~] = lsim(S_u_fb, x_ref_safe, t_ms); 
[x_pos_ms, ~] = lsim(T_in, x_ref_safe, t_ms);  
v_cart_ms = gradient(x_pos_ms, t_ms);               

% --- 5. Apply Non-Linear Friction Cancellation ---
v_total_ms = zeros(size(v_fb_ms));
v_total_ms(v_fb_ms > 0.1)  = v_fb_ms(v_fb_ms > 0.1) + ud_pos;
v_total_ms(v_fb_ms < -0.1) = v_fb_ms(v_fb_ms < -0.1) - ud_neg;

% --- 6. Calculate Physics (V, I, F) ---
V_emf_ms = (K_g * k_m / r_mp) * v_cart_ms;
I_req_ms = (v_total_ms - V_emf_ms) / R_m;
F_req_ms = (eta_g * eta_m * K_g * k_t / r_mp) * I_req_ms;

% --- 7. Hardware Safety Verification ---
peak_V_ms = max(abs(v_total_ms));
peak_I_ms = max(abs(I_req_ms));
rms_I_ms  = sqrt(mean(I_req_ms.^2));
peak_F_ms = max(abs(F_req_ms));
peak_X_ms = max(abs(x_pos_ms));

fprintf('%-20s | %-10s | %-10s\n', 'PARAMETER', 'SIMULATED', 'LIMIT');
disp('--------------------------------------------------');
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Peak Voltage [V]', peak_V_ms, V_sat);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Peak Current [A]', peak_I_ms, 3.0);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'RMS Current [A]', rms_I_ms, 1.0);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Peak Force [N]', peak_F_ms, F_clicking_limit);
fprintf('%-20s | %-10.4f | %-10.4f\n', 'Max Travel [m]', peak_X_ms, T_c/2 * 0.75);
disp('--------------------------------------------------');

checks_passed = (peak_I_ms <= 3.0) && (peak_V_ms <= V_sat) && ...
                (peak_F_ms <= F_clicking_limit) && (rms_I_ms <= 1.0);

simulink_multisine_CL = [t_ms(:)'; x_ref_safe(:)'];

if checks_passed
    disp('>>> SUCCESS: Multi-Sine Reference is SAFE for hardware deployment. <<<');
    
    % --- 8. Save Data for Simulink ---
    if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = pwd; end 
    data_dir = fullfile(SEESAW_ROOT, 'data', 'cartControl');
    if ~exist(data_dir, 'dir'), mkdir(data_dir); end
    
    %save(fullfile(data_dir, 'multisine_simulink.mat'), 'simulink_multisine_CL', '-v7.3');
    %save(fullfile(data_dir, 'multisine_params.mat'), 't_ms', 'x_ref_safe', 'w_vals', 'N_freq');
    
    fprintf('>>> Saved to: /data/cartControl/multisineL_simulink.mat\n');
    fprintf('>>> Run your Simulink model for %.1f seconds.\n', T_test);
else
    warning('DANGER: Multi-Sine violates hardware limits!');
    fprintf('Try increasing the taper_exponent (currently %.1f) to reduce high-frequency force.\n', taper_exponent);
end

fprintf('>>> Run your Simulink model for %.1f seconds.\n', T_test);
% --- 9. Visualization ---
figure('Name', 'Closed-Loop Multi-Sine Profile', 'Position', [150 150 900 600]);
subplot(2,1,1);
plot(t_ms, x_ref_safe, 'b', 'LineWidth', 1.2); hold on;
plot(t_ms, x_pos_ms, 'r--', 'LineWidth', 1);
ylabel('Position [m]'); title('Command vs Predicted Position');
legend('Reference ($x_{ref}$)', 'Predicted Output ($x_{pos}$)'); grid on;

subplot(2,1,2);
% Plot the compensated "real" hardware command (in red)
plot(t_ms, v_total_ms, 'r--', 'LineWidth', 1, 'DisplayName', 'Hardware Command ($u_{real}$)');
hold on;
% Plot the ideal linear command (in blue)
plot(t_ms, v_fb_ms, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Linear Command ($u_{safe}$)');

% Add threshold lines
yline(V_sat, 'k-.', 'Hardware Peak Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(-V_sat, 'k-.', 'HandleVisibility', 'off');
yline(V_nom, 'k--', 'Max Safe Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(-V_nom, 'k--', 'HandleVisibility', 'off');

xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Simulated Voltage [V]', 'Interpreter', 'latex');
title('Schroeder-Phased Multi-Sine Input Vector', 'Interpreter', 'latex');
grid on;
legend('Location', 'northeast', 'Interpreter', 'latex');
ylim([-V_sat*1.2, V_sat*1.2]);