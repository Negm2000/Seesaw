%% Outer Loop Angle Control (Hardware-Safe PI-Lead / PID)
% Designs the pendulum balancing controller using pole placement, 
% incorporating the validated inner-loop cart dynamics.

clear; close all; clc;

% Configure global plot settings
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. LOAD SYSTEM PARAMETERS & INNER LOOP
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = pwd; end 

% Load physical parameters (g, Me, DT, Jsw, Bsw, Msw, DC, etc.)
seesaw_params; 

% Load the voltage non-linearities
cart_file = fullfile(SEESAW_ROOT, 'data', 'params', 'param_nonlinear.mat');
if exist(cart_file, 'file')
    load(cart_file, 'ud_pos', 'ud_neg', 'ud_sym');
else
    error('Non-linearities not found. Run seesaw nonlinear first.');
end

% Load the validated inner plant model (Gx)
cart_file = fullfile(SEESAW_ROOT, 'data', 'tuned', 'tuned_cart.mat');
if exist(cart_file, 'file')
    load(cart_file, 'Gx', 'num_x', 'den_x');
else
    error('Cart model not found. Run cart modeling first.');
end

% Load the validated inner closed-loop tracking model (T_in)
inner_file = fullfile(SEESAW_ROOT, 'data', 'controllers', 'controller_inner_pid.mat');
if exist(inner_file, 'file')
    load(inner_file, 'T_in', 'C_in', ...
        'Kp_in', 'Ki_in', 'Kd_in', 'N_in', 'antiwindup_in');
else
    error('Inner loop model not found. Run inner loop validation first.');
end

% Load the validated inner plant model (Gx)
seesaw_file = fullfile(SEESAW_ROOT, 'data', 'tuned', 'tuned_seesaw.mat');
if exist(seesaw_file, 'file')
    load(seesaw_file, 'Gt', 'num_t', 'den_t');
else
    error('Cart model not found. Run cart modeling first.');
end

%% 2. OUTER PLANT DEFINITION (G_t * T_in)

% The "True" Plant the outer controller sees
P_out = minreal(series(T_in, Gt)); 

fprintf('--- Outer Plant Formed ---\n');
disp('P_out = T_in(s) * Gt(s)');

%% 3. PI-LEAD POLE PLACEMENT
wn_target = 4.3;    % Speed of balance reaction
zeta_target = 0.6;  % Tolerance for overshoot
p_lead = 25;        % Derivative filter pole (N)
z_i = 0.25;          % Integrator zero 

% 1. Target Pole
sd = -zeta_target * wn_target + 1j * wn_target * sqrt(1 - zeta_target^2);

% 2. Calculate Angle Deficit
P_val = evalfr(P_out, sd);
angle_P = angle(P_val); 
angle_sd = angle(sd);           
angle_zi = angle(sd + z_i);     
angle_pl = angle(sd + p_lead);  

angle_z_lead_needed = angle_sd + angle_pl - angle_zi - angle_P;
angle_z_lead_needed = mod(angle_z_lead_needed, 2*pi);

% 3. Calculate Lead Zero & Gain
z_lead = (imag(sd) / tan(angle_z_lead_needed)) - real(sd);
C_theta_unscaled = (s + z_lead) * (s + z_i) / (s * (s + p_lead));
C_val = evalfr(C_theta_unscaled, sd);
K_pos = 1 / abs(C_val * P_val);

% 4. Final Controller (Enforcing Negative Gain)
C_theta = minreal(-K_pos * C_theta_unscaled);

% 5. Build Formal Loop Transfer Functions
L_theta = minreal(series(C_theta, P_out));
T_theta = minreal(feedback(L_theta, 1));

% 6. Extract Standard PID Parameters
N_out = p_lead;
Ki_out = -K_pos * (z_lead * z_i) / N_out;
Kp_out = (-K_pos * (z_lead + z_i) - Ki_out) / N_out;
Kd_out = (-K_pos - Kp_out) / N_out;
antiwindup_out = Ki_out / Kp_out;

fprintf('\n--- Outer Loop PID Gains ---\n');
fprintf('Kp_out = %8.4f m/rad\n', Kp_out);
fprintf('Ki_out = %8.4f m/(rad*s)\n', Ki_out);
fprintf('Kd_out = %8.4f m*s/rad\n', Kd_out);
fprintf('N_out  = %8.1f\n', N_out);

%% 4. ROOT LOCUS VISUALIZATION
figure('Name', 'Outer Loop Root Locus', 'Position', [100 200 700 500]);
rlocus(C_theta_unscaled * -P_out); hold on;
plot(real(sd), imag(sd), 'r*', 'MarkerSize', 10, 'LineWidth', 2);
title('Root Locus of Outer Loop (Negative Gain)'); 
grid on;

%% 5. PHYSICAL SIMULATION & HARDWARE SAFETY ANALYSIS (Filtered Step)
% Simulates tracking a smooth reference angle to evaluate transient stability 
% without triggering artificial t=0 derivative shocks.

disp('--- Running Filtered Angle Tracking Simulation ---');

% 1. Bessel Pre-Filter for the Angle Reference
wf_bes_outer = 1.5; 
F_bessel_out = tf([3 * wf_bes_outer^2], [1, 3 * wf_bes_outer, 3 * wf_bes_outer^2]);

% 2. Define the Target and Time
t = 0:0.005:5.0; 
target_angle_deg = 2.0; 
theta_ref_raw = deg2rad(target_angle_deg) * ones(size(t));
theta_ref = lsim(F_bessel_out, theta_ref_raw, t);

% 3. Define Cascade Transfer Functions
S_u_outer = minreal(C_theta / (1 + L_theta)); 
S_u_fb = minreal(C_in / (1 + C_in * Gx)); 

% 4. Simulate the Chain of Events
theta_rad = lsim(T_theta, theta_ref, t);   
x_ref     = lsim(S_u_outer, theta_ref, t); 
x_pos     = lsim(T_in, x_ref, t);          
v_cmd     = lsim(S_u_fb, x_ref, t);        

% --- 4.5 Physics Calculations (V, I, F) ---
v_cart = gradient(x_pos, t);
V_emf = (K_g * k_m / r_mp) * v_cart;
I_req = (v_cmd - V_emf) / R_m;
F_req = (eta_g * eta_m * K_g * k_t / r_mp) * I_req;

% --- Hardware Limits ---
X_LIMIT = T_c/2 * 0.75;                 % 75% of half-track
V_LIMIT = V_sat - max(ud_pos, ud_neg);  % ~10.39V Peak Voltage
I_PEAK_LIMIT = I_peak;                  % Peak Amplifier Current [A]
I_RMS_LIMIT  = I_max;                   % Continuous Thermal Current [A]
F_CLICK_LIMIT = 1.48;                   % Rack/Pinion mechanical limit [N]
THETA_LIMIT_DEG = 23.32 / 2;            % Hardware angle constraint (~11.66 deg)

% --- Statistical Analysis ---
peak_X = max(abs(x_pos));
peak_V = max(abs(v_cmd));
peak_I = max(abs(I_req));
rms_I  = sqrt(mean(I_req.^2));
peak_F = max(abs(F_req));
peak_theta_deg = max(abs(rad2deg(theta_rad)));

% --- Evaluation & Reporting ---
fprintf('\n--- Safety Verification (2 Deg Filtered Step) ---\n');
fprintf('%-20s | %-10s | %-10s\n', 'PARAMETER', 'SIMULATED', 'LIMIT');
disp('--------------------------------------------------');
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Max Travel [m]', peak_X, X_LIMIT);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Max Angle [deg]', peak_theta_deg, THETA_LIMIT_DEG);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Peak Voltage [V]', peak_V, V_LIMIT);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Peak Current [A]', peak_I, I_PEAK_LIMIT);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'RMS Current [A]', rms_I, I_RMS_LIMIT);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Peak Force [N]', peak_F, F_CLICK_LIMIT);
disp('--------------------------------------------------');

if peak_X > X_LIMIT
    warning('TRACK DANGER: The cart runs out of track trying to maintain this lean!');
elseif peak_V > V_LIMIT || peak_I > I_PEAK_LIMIT
    warning('ELECTRONIC DANGER: Actuator saturation exceeded (Voltage/Current).');
elseif peak_F > F_CLICK_LIMIT
    warning('MECHANICAL DANGER: Peak force exceeds the gear clicking limit!');
elseif rms_I > I_RMS_LIMIT
    warning('THERMAL DANGER: RMS current exceeds motor continuous rating.');
elseif peak_theta_deg > THETA_LIMIT_DEG
    warning('ANGLE DANGER: Pendulum exceeds the physical limit (will hit stops!).');
else
    disp('VERDICT: Outer loop transient tracking is 100% Hardware-Safe.');
    % --- Save Data for Simulink (Step) ---
    if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = pwd; end 
    data_dir_out = fullfile(SEESAW_ROOT, 'data', 'seesawControl');
    if ~exist(data_dir_out, 'dir'), mkdir(data_dir_out); end
    
    % Format array for Simulink "From File" block (Row 1: time, Row 2: data)
    simulink_step = [t(:)'; theta_ref(:)'];
    
    save(fullfile(data_dir_out, 'step_simulink.mat'), 'simulink_step', '-v7.3');
    fprintf('>>> Step Reference saved to: /data/seesawControl/step_simulink.mat\n');
end

%% 6. TIME DOMAIN VISUALIZATION (Tracking)
figure('Name', 'Outer Loop Tracking Analysis', 'Position', [850 200 800 600]);

% 1. Pendulum Angle Tracking
subplot(2,2,1); 
plot(t, rad2deg(theta_rad), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(theta_ref), 'k--', 'LineWidth', 1); grid on;
title('Pendulum Angle ($\theta$)'); ylabel('Degrees');
legend('Simulated Angle', 'Reference ($\theta_{ref}$)');
yline(0, 'k:', 'HandleVisibility', 'off');
yline(THETA_LIMIT_DEG, 'r--', 'HandleVisibility', 'off'); yline(-THETA_LIMIT_DEG, 'r--', 'HandleVisibility', 'off');

% 2. Cart Position 
subplot(2,2,2); 
plot(t, x_pos, 'g', 'LineWidth', 1.5); hold on;
plot(t, x_ref, 'k--', 'LineWidth', 1); grid on;
title('Cart Position ($x_c$)'); ylabel('Meters');
legend('Simulated Position', 'Target ($x_{ref}$)', 'Location', 'northeast');
yline(X_LIMIT, 'r--', 'HandleVisibility', 'off'); yline(-X_LIMIT, 'r--', 'HandleVisibility', 'off');

% 3. Control Effort (Voltage)
subplot(2,2,[3 4]); 
plot(t, v_cmd, 'r', 'LineWidth', 1.5); grid on;
title('Inner Motor Voltage ($V_{cmd}$)'); ylabel('Volts'); xlabel('Time [s]');
ylim([-V_LIMIT*1.2, V_LIMIT*1.2]); 
yline(V_nom, 'k--', 'HandleVisibility', 'off'); yline(-V_nom, 'k--', 'Max Safe Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(V_LIMIT, 'r--', 'HandleVisibility', 'off'); yline(-V_LIMIT, 'r--', 'Hardware Peak Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');

%% 7. SAVE PARAMS
save_file = fullfile(SEESAW_ROOT, 'data', 'controllers', 'controller_outer_pid.mat');
save(save_file, 'Kp_out', 'Ki_out', 'Kd_out', 'N_out', 'antiwindup_out', 'C_theta', 'L_theta', 'T_theta');
fprintf('\n>>> Outer Controller saved to: %s\n', save_file);

%% 8. CLOSED-LOOP FREQUENCY VALIDATION (T_theta Multi-Sine)
% Injects a multi-sine into theta_ref to validate the outer loop FRF.
% WARNING: Low frequencies cause massive cart drift due to double-integration!

disp('==================================================');
disp('   BUILDING OUTER-LOOP MULTI-SINE (T_theta)       ');
disp('==================================================');

% --- 1. Multi-Sine Parameters ---
wb_out = sqrt(sqrt(-den_t(3)/den_t(1))*sqrt(-num_t(3) / num_t(1)));
w_min = wb_out/10 ;
w_max = wb_out*3;     
N_freq = 10;                
THETA_MAX_BUDGET = 0.65;

T_test = max(15, 3 * (2*pi/w_min)); 

% --- 2. Generate Frequencies & Schroeder Phases ---
w_vals = logspace(log10(w_min), log10(w_max), N_freq);
k = 1:N_freq;
phi = -pi * k .* (k - 1) / N_freq; 

% --- 3. CUSTOM AMPLITUDE TAPERING (Force-Safe) ---
% We assign specific amplitudes based on frequency to keep F = m*A*w^2 constant.
A = zeros(1, N_freq);

for i = 1:N_freq
    if w_vals(i) < 2.5
        % Low Freq: 1.5 degrees to beat stiction and quantization
        A(i) = deg2rad(THETA_MAX_BUDGET); 
    else
        % High Freq: Roll off by w^2 to protect the gears from high acceleration
        A(i) = deg2rad(THETA_MAX_BUDGET/10) * (2.5 / w_vals(i))^2; 
    end
end

t_ms = 0:0.005:T_test;
theta_ref_safe = zeros(size(t_ms));

for i = 1:N_freq
    theta_ref_safe = theta_ref_safe + A(i) * sin(w_vals(i) * t_ms + phi(i));
end

% Fade-in window (prevents t=0 shocks)
fade_time = 1.5; 
fade_indices = find(t_ms <= fade_time);
fade_envelope = sin(linspace(0, pi/2, length(fade_indices)));
theta_ref_safe(fade_indices) = theta_ref_safe(fade_indices) .* fade_envelope;

% --- 4. Simulate Cascade Physics ---
theta_rad_ms = lsim(T_theta, theta_ref_safe, t_ms);   
x_ref_ms     = lsim(S_u_outer, theta_ref_safe, t_ms); 
x_pos_ms     = lsim(T_in, x_ref_ms, t_ms);          
v_cmd_ms     = lsim(S_u_fb, x_ref_ms, t_ms);        

% --- 4.5 Physics Calculations (V, I, F) ---
v_cart_ms = gradient(x_pos_ms, t_ms);
V_emf_ms  = (K_g * k_m / r_mp) * v_cart_ms;
I_req_ms  = (v_cmd_ms - V_emf_ms) / R_m;
F_req_ms  = (eta_g * eta_m * K_g * k_t / r_mp) * I_req_ms;

% --- Statistical Analysis ---
peak_X_ms = max(abs(x_pos_ms));
peak_V_ms = max(abs(v_cmd_ms));
peak_I_ms = max(abs(I_req_ms));
rms_I_ms  = sqrt(mean(I_req_ms.^2));
peak_F_ms = max(abs(F_req_ms));
peak_theta_ms = max(abs(rad2deg(theta_rad_ms)));

% --- Evaluation & Reporting ---
fprintf('%-20s | %-10s | %-10s\n', 'PARAMETER', 'SIMULATED', 'LIMIT');
disp('--------------------------------------------------');
fprintf('%-20s | %-10.4f | %-10.4f\n', 'Max Travel [m]', peak_X_ms, X_LIMIT);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Max Angle [deg]', peak_theta_ms, THETA_LIMIT_DEG);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Peak Voltage [V]', peak_V_ms, V_LIMIT);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Peak Current [A]', peak_I_ms, I_PEAK_LIMIT);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'RMS Current [A]', rms_I_ms, I_RMS_LIMIT);
fprintf('%-20s | %-10.2f | %-10.2f\n', 'Peak Force [N]', peak_F_ms, F_CLICK_LIMIT);
disp('--------------------------------------------------');

checks_passed = (peak_X_ms <= X_LIMIT) && (peak_V_ms <= V_LIMIT) && ...
                (peak_I_ms <= I_PEAK_LIMIT) && (rms_I_ms <= I_RMS_LIMIT) && ...
                (peak_F_ms <= F_CLICK_LIMIT) && (peak_theta_ms <= THETA_LIMIT_DEG);

if ~checks_passed
    warning('DANGER: Outer Multi-Sine violates physical limits. Review console output.');
else
    disp('>>> SUCCESS: Outer Multi-Sine Reference is 100% SAFE for hardware. <<<');
    
    % --- Save Data for Simulink (Multi-Sine) ---
    if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = pwd; end 
    data_dir_out = fullfile(SEESAW_ROOT, 'data', 'seesawControl');
    if ~exist(data_dir_out, 'dir'), mkdir(data_dir_out); end
    
    % Format for Simulink "From File" block
    simulink_multisine_CL = [t_ms(:)'; theta_ref_safe(:)'];
    
    save(fullfile(data_dir_out, 'multisine_simulink.mat'), 'simulink_multisine_CL', '-v7.3');
    save(fullfile(data_dir_out, 'multisine_params.mat'), 't_ms', 'theta_ref_safe', 'w_vals', 'N_freq');
    
    fprintf('>>> Multi-Sine Reference saved to: /data/seesawControl/multisine_simulink.mat\n');
    fprintf('>>> Run your Simulink model for %.1f seconds.\n', T_test);
end

% --- 6. Visualization ---
figure('Name', 'Outer Loop Multi-Sine', 'Position', [150 150 900 600]);
subplot(2,1,1);
plot(t_ms, rad2deg(theta_ref_safe), 'b', 'LineWidth', 1.2); hold on;
plot(t_ms, rad2deg(theta_rad_ms), 'r--', 'LineWidth', 1);
ylabel('Angle [deg]'); title('Command vs Predicted Pendulum Angle');
legend('Reference ($\theta_{ref}$)', 'Predicted Output ($\theta$)'); grid on;

subplot(2,1,2);
plot(t_ms, x_pos_ms, 'k'); hold on;
yline(X_LIMIT, 'r--', 'Track Limit', 'LabelHorizontalAlignment', 'left');
yline(-X_LIMIT, 'r--');
ylabel('Cart Position [m]'); xlabel('Time [s]'); title('Predicted Cart Drift');
ylim([-X_LIMIT*1.2, X_LIMIT*1.2]); grid on;