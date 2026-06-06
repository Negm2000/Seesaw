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
cart_file = fullfile(SEESAW_ROOT, 'data', 'param_nonlinear.mat');
if exist(cart_file, 'file')
    load(cart_file, 'ud_pos', 'ud_neg', 'ud_sym');
else
    error('Non-linearities not found. Run seesaw nonlinear first.');
end

% Load the validated inner plant model (Gx)
cart_file = fullfile(SEESAW_ROOT, 'data', 'tuned_cart.mat');
if exist(cart_file, 'file')
    load(cart_file, 'Gx', 'num_x', 'den_x');
else
    error('Cart model not found. Run cart modeling first.');
end

% Load the validated inner closed-loop tracking model (T_in)
inner_file = fullfile(SEESAW_ROOT, 'data', 'controller_inner_pid.mat');
if exist(inner_file, 'file')
    load(inner_file, 'T_in', 'C_in', ...
        'Kp_in', 'Ki_in', 'Kd_in', 'N_in', 'antiwindup_in');
else
    error('Inner loop model not found. Run inner loop validation first.');
end

% Load the validated inner plant model (Gx)
seesaw_file = fullfile(SEESAW_ROOT, 'data', 'tuned_seesaw.mat');
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
z_i = p_lead/10;    % Integrator zero 

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
S_u_fb = feedback(C_in, Gx);
S_u_outer = feedback(C_theta, P_out);

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

%% 5. PHYSICAL SIMULATION & HARDWARE SAFETY ANALYSIS (100g Load Bias)
% Simulates a static disturbance to demonstrate the controller's integral action.
% A 100g mass placed on the seesaw creates a static unbalance.
% This is mathematically modeled as an equivalent step disturbance at the plant input.

disp('--- Running Load Bias (Disturbance) Simulation ---');

% 1. Define Time and Zero Reference
t = 0:0.005:8.0; % Extended time to clearly see the integrator settle
theta_ref = zeros(size(t)); % Target is to stay perfectly flat (0 rad)

% 2. Define the Disturbance (Equivalent to a 100g load)
% Balancing 0.1kg against a ~1kg cart requires roughly 2cm of cart offset.
dist_time = 1.0; 
x_dist = zeros(size(t));
x_dist(t >= dist_time) = 0.02; % 2cm equivalent plant input step disturbance

% 3. Define Closed-Loop Disturbance Transfer Functions
% Angle response to plant disturbance: theta = G_t / (1 + L_theta) * x_dist
feedback_path = series(C_theta, T_in); 
S_dist_theta = feedback(Gt, feedback_path); 

% Controller target response to disturbance: x_ref = -C_theta * theta
S_dist_xref = series(S_dist_theta, -C_theta);

% 4. Simulate the Chain of Events
theta_rad = lsim(S_dist_theta, x_dist, t);   
x_ref     = lsim(S_dist_xref, x_dist, t); 
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
fprintf('\n--- Safety Verification (100g Load Bias Disturbance) ---\n');
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
    warning('TRACK DANGER: The cart runs out of track trying to balance this load!');
elseif peak_V > V_LIMIT || peak_I > I_PEAK_LIMIT
    warning('ELECTRONIC DANGER: Actuator saturation exceeded (Voltage/Current).');
elseif peak_F > F_CLICK_LIMIT
    warning('MECHANICAL DANGER: Peak force exceeds the gear clicking limit!');
elseif rms_I > I_RMS_LIMIT
    warning('THERMAL DANGER: RMS current exceeds motor continuous rating.');
elseif peak_theta_deg > THETA_LIMIT_DEG
    warning('ANGLE DANGER: Pendulum exceeds the physical limit (will hit stops!).');
else
    disp('VERDICT: Outer loop load bias rejection is 100% Hardware-Safe.');
end

%% 6. TIME DOMAIN VISUALIZATION (Disturbance Rejection)
figure('Name', 'Integral Action: Load Bias Rejection', 'Position', [850 200 800 600]);

% 1. Pendulum Angle Rejection
subplot(2,2,1); 
plot(t, rad2deg(theta_rad), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(theta_ref), 'k--', 'LineWidth', 1); grid on;
xline(dist_time, 'r:', 'Load Applied', 'HandleVisibility', 'off');
title('Pendulum Angle ($\theta$)'); ylabel('Angle [$^\circ$]');
legend('Simulated Angle', 'Target ($0^{\circ}$)');
ylim([-max(abs(rad2deg(theta_rad)))*1.5 - 0.5, max(abs(rad2deg(theta_rad)))*1.5 + 0.5]);

% 2. Cart Position (Showing Integral Offset)
subplot(2,2,2); 
plot(t, x_pos, 'g', 'LineWidth', 1.5); hold on;
plot(t, x_ref, 'k--', 'LineWidth', 1); grid on;
xline(dist_time, 'r:', 'HandleVisibility', 'off');
title('Cart Position ($x_c$) with Steady-State Offset'); ylabel('Position [$m$]');
legend('Simulated Position', 'Target ($x_{ref}$)', 'Location', 'southeast');
yline(X_LIMIT, 'k--', 'HandleVisibility', 'off'); yline(-X_LIMIT, 'k--', 'HandleVisibility', 'off');

% 3. Control Effort (Voltage)
subplot(2,2,[3 4]); 
plot(t, v_cmd, 'r', 'LineWidth', 1.5); grid on;
xline(dist_time, 'r:', 'HandleVisibility', 'off');
title('Inner Motor Voltage ($V_{cmd}$)'); ylabel('Voltage [$V$]'); xlabel('Time [s]');
ylim([-V_LIMIT*1.2, V_LIMIT*1.2]); 
yline(V_nom, 'k--', 'HandleVisibility', 'off'); yline(-V_nom, 'k--', 'Max Safe Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(V_LIMIT, 'k--', 'HandleVisibility', 'off'); yline(-V_LIMIT, 'k--', 'Hardware Peak Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');

%% 7. SAVE PARAMS
save_file = fullfile(SEESAW_ROOT, 'data', 'controller_outer_pid.mat');
save(save_file, 'Kp_out', 'Ki_out', 'Kd_out', 'N_out', 'antiwindup_out', 'C_theta', 'L_theta', 'T_theta');
fprintf('\n>>> Outer Controller saved to: %s\n', save_file);
