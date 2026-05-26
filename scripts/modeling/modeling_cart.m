%% MODELING PIPELINE — Quanser IP02 + SEESAW-E
%  =====================================================================
%  Master script for system identification and model validation.
%  Run each section (Ctrl+Enter) in order. Sections 1-3 are pre-hardware,
%  Section 4 requires you to go collect data, Sections 5-10 are post-hardware.
%
%  To convert to Live Script: right-click this file → Open as Live Script
%  =====================================================================
% setting default parameter with LaTeX interpreter
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');
%% 1. LOAD SYSTEM PARAMETERS
%  Load all Quanser hardware specs from seesaw_params.m.
%  This populates ~30 variables including B_eq (tuning target).

seesaw_params;
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
data_file = fullfile(SEESAW_ROOT, 'data', 'param_nonlinear.mat');

if ~exist(data_file, 'file')
    error('non-linear data not found.');
else
    fprintf('Loading %s ...\n', data_file);
    load(data_file)
end

B_eq_nominal = B_eq;  % save for comparison later
eta_g_nominal = eta_g;
fprintf('\n----- Nominal Model -----\n');
fprintf('  B_eq     = %.2f N*s/m (will be tuned)\n', B_eq);
fprintf('  eta_g    = %.2f N*s/m (will be tuned)\n', eta_g);
fprintf('--- Impacted Parameters ---\n');
fprintf('  B_total  = %.2f N*s/m\n', B_total);
fprintf('  alpha_f  = %.4f (motor force constant)\n', alpha_f);
fprintf('-------------------------\n');

%% 2. LOAD & INSPECT HARDWARE DATA
%  Load the frequency sweep data collected from QUARC.
%  Plot raw time traces to sanity-check before analysis.

if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
data_file = fullfile(SEESAW_ROOT, 'data', 'cartModeling', 'step_output.mat');

if ~exist(data_file, 'file')
    error('data not found. Run on hardware first.');
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

% Extract columns: [time; V_cmd; x_c; x_c_dot]
t_hw      = raw(1, :)';
V_cmd_hw  = raw(2, :)';
xc_hw     = raw(3, :)';      % [m] — with corrected encoder gain
dt_hw     = mean(diff(t_hw));
Fs_hw     = 1 / dt_hw;

% Compute the velocity in post-processing to avoid phase-lag
cutoff_freq = B_total/M_e * 2;
[b, a] = butter(2, cutoff_freq / (Fs_hw/2));
xc_hw_clean = filtfilt(b, a, xc_hw);

% Because of the differential we "lose" the last data point
tdot_hw = t_hw(1:end-1);
xcdot_hw = diff(xc_hw_clean)/dt_hw;

fprintf('  Duration: %.1f s | Fs: %.0f Hz | Samples: %d\n', t_hw(end), Fs_hw, length(t_hw));
fprintf('  x_c range: [%.1f, %.1f] cm\n', min(xc_hw)*100, max(xc_hw)*100);

% Sanity check: did the cart hit end-stops?
% Use range of motion (max - min), not abs(x_c), because the encoder may be
% zeroed at one end of the track rather than the centre.
xc_range = max(xc_hw) - min(xc_hw);
if xc_range > T_c * 0.9
    warning('Cart travelled %.1f cm (track travel = %.1f cm). Data may be clipped!', xc_range*100, T_c*100);
end

figure('Name', 'Raw Hardware Data', 'Position', [100 100 1000 700]);
subplot(3,1,1);
plot(t_hw, V_cmd_hw, 'k-'); ylabel('$V_{cmd}$ [V]'); title('Step Input');
ylim([-V_sat, V_sat]);
grid on;
subplot(3,1,2);
plot(t_hw, xc_hw*100, 'k-'); ylabel('$x_c$ [cm]'); title('Cart Position (Hardware)');
grid on;
subplot(3,1,3);
plot(tdot_hw, xcdot_hw*100, 'k-'); ylabel('$\dot{x}_c$ [cm/s]'); title('Cart Velocity (Hardware)');
xlabel('Time [s]'); grid on;
sgtitle('Raw Hardware Step Response Data');

%% 3. TUNE BASED ON VELOCITY STEADY-STATE AFTER STEP RESPONSE
% Mathematically tuning eta_g and B_eq.
% This does not mean that their real physical values will be these ones.
% It means that we consider that the model behave as it would if the
% hardware had these values.

V_step = 2;              % The step input applied

% keep datasheet limits
etag_min = eta_g_nominal * 0.5; % -10% from datasheet, but we expect even less because of dust in bearings.
etag_max = eta_g_nominal * 1.1; % +10%

% Define the gain and time constant of hardware
pulse_start_time = 0;
pulse_end_time = 1.0;
pulse_indices = tdot_hw >= pulse_start_time & tdot_hw <= pulse_end_time;

ss_indices = tdot_hw >= (pulse_end_time - 0.2) & tdot_hw <= pulse_end_time;
xcdot_ss = mean(xcdot_hw(ss_indices));
V_target = 0.632 * xcdot_ss;

crossing_index = find(xcdot_hw(pulse_indices) >= V_target, 1, 'first');

t_pulse_window = tdot_hw(pulse_indices);
tau_meas = t_pulse_window(crossing_index) - pulse_start_time;
K_meas = xcdot_ss / V_step;

% 3. Define the Simulation Logic using Anonymous Functions
% The algorithm will only tweak these two values:
% p(1) = eta_g (Gearbox Efficiency)
% p(2) = B_eq (Equivalent Viscous Friction)
calc_alpha = @(p) (k_t * K_g * p(1) * eta_m) / (r_mp * R_m);
calc_Btot  = @(p) ((k_t * k_m * K_g^2 * p(1)) / (r_mp^2 * R_m)) + p(2);

% Helper functions for Gain and Time Constant to keep equations clean
calc_K   = @(p) calc_alpha(p) * eta_m / calc_Btot(p);
calc_tau = @(p) M_e / calc_Btot(p);

% Step Responses Calculation
v_sim = @(p) V_step * (calc_alpha(p) * eta_m / calc_Btot(p)) * (1 - exp(-(calc_Btot(p) / M_e) * tdot_hw));
x_sim = @(p) V_step * calc_K(p) * (t_hw - calc_tau(p) * (1 - exp(-t_hw / calc_tau(p))));

% 4. Define the Cost Function (Mean Squared Error)
% We calculate the normal error, and add a massive penalty if it goes out of bounds
costFunction = @(p) mean((xcdot_hw - v_sim(p)).^2) + ...
                    1e6 * (p(1) < etag_min) + ...  % Penalty if too low
                    1e6 * (p(1) > etag_max);       % Penalty if too high

% 5. Initial Guesses for the 2 Unknowns
% Provide realistic starting points.
init_params = [eta_g_nominal, B_eq_nominal];

% 6. Run Optimization (Nelder-Mead)
options = optimset('Display', 'iter', 'TolFun', 1e-6, 'TolX', 1e-6);
disp('Hunting for optimal gearbox efficiency and friction...');
best_params = fminsearch(costFunction, init_params, options);

% 7. Extract and Display Results
eta_g = best_params(1);
B_eq   = best_params(2);

% Calculate the final transfer function metrics using the locked eta_m
alpha_f = calc_alpha(best_params);
B_total  = calc_Btot(best_params);
Kdc_opt   = alpha_f * eta_m / B_total;
tau_opt   = M_e / B_total;

rmse_init  = sqrt(mean((xc_hw - x_sim(init_params)).^2));
rmse_tuned = sqrt(mean((xc_hw - x_sim(best_params)).^2));

fprintf('\n--- Optimized Physical Parameters ---\n');
fprintf('Gearbox Efficiency (eta_g):    %.2f %%\n', eta_g*100);
fprintf('Viscous Friction (Bc):         %.4f N*s/m\n', B_eq);

fprintf('\n--- Resulting Plant Dynamics ---\n');
fprintf('DC Gain (K):      %.4f (m/s)/V\n', Kdc_opt);
fprintf('Time Const (tau): %.4f s\n', tau_opt);
fprintf('Equivalent Pole:  %.4f rad/s\n', B_total / M_e);

fprintf('\n--- Cross-Validation: Position Error ---\n');
fprintf('Initial Model Position MSE:   %.4e m\n', rmse_init);
fprintf('Optimized Model Position MSE: %.4e m\n', rmse_tuned);
fprintf('Improvement Factor:           %.1fx better\n', rmse_init / rmse_tuned);


figure('Name', 'Comparison Data', 'Position', [100 100 1000 700]);

% --- Subplot 1: Voltage ---
subplot(3,1,1);
plot(t_hw, V_cmd_hw, 'k-'); 
ylabel('$V_{cmd}$ [V]');
title('Step Input');
ylim([-V_sat, V_sat]);
grid on;

% --- Subplot 2: Position ---
subplot(3,1,2);
plot(t_hw, xc_hw*100, 'k-', 'LineWidth', 2, 'DisplayName', 'Hardware'); 
title('Cart Position'); 
hold on;
plot(t_hw, x_sim(best_params)*100, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Optimized Model');
plot(t_hw, x_sim(init_params)*100, 'b--', 'LineWidth', 1, 'DisplayName', 'Initial Model');
grid on;

% --- Subplot 3: Velocity ---
subplot(3,1,3);
plot(tdot_hw, xcdot_hw*100, 'k-', 'LineWidth', 2, 'DisplayName', 'Hardware'); 
ylabel('$\dot{x}_c$ [cm/s]');
title('Cart Velocity'); 
hold on;
plot(tdot_hw, v_sim(best_params)*100, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Optimized Model');
plot(tdot_hw, v_sim(init_params)*100, 'b--', 'LineWidth', 1, 'DisplayName', 'Initial Model');
xlabel('Time [s]'); 
grid on; 
legend('Location', 'southeast');

sgtitle('Step Response Tuning');

%% 4. APPLY TUNED PARAMETERS & REBUILD MODEL
%  Overwrite B_eq with the tuned value, recompute all derived quantities,
%  and rebuild both state-space models (cart-only and full seesaw).

fprintf('\n--- Applying Tuned Parameters ---\n');

% Recompute total damping (alpha_f and B_emf are constants -- not affected by B_eq)
B_total = B_eq + B_emf;

fprintf('  B_eq (tuned)  = %.4f N*s/m\n', B_eq);
fprintf('  B_total       = %.4f N*s/m\n', B_total);

% --- Phase 1: Cart on Table ---
A_cart = [0, 1; 0, -B_total/M_e];
B_cart = [0; alpha_f*eta_m/M_e];
C_cart = eye(2);
D_cart = zeros(2,1);

sys_cart = ss(A_cart, B_cart, C_cart, D_cart);

% Updated transfer functions
Gx = minreal(Kdc_opt / (tau_opt*s^2 + s));
Gv = s * Gx;
Ga = s^2 * Gx;

num_x = alpha_f*eta_m;
den_x = [M_e B_total 0];

fprintf(' Transfer Function and State-space model rebuilt (Gx, A_cart, B_cart)\n');

%% 5. SUMMARY & SAVE
%  Print final results and save tuned parameters.

fprintf('\n');
fprintf('============================================================\n');
fprintf('  MODELING OF CART COMPLETE\n');
fprintf('============================================================\n');
fprintf('\n  Parameter          Nominal    Tuned      Change\n');
fprintf('  -----------------  ---------  ---------  ------\n');
fprintf('  B_eq [N*s/m]       %8.3f   %8.3f   %+.1f%%\n', ...
    B_eq_nominal, B_eq, (B_eq-B_eq_nominal)/B_eq_nominal*100);
fprintf('  B_total [N*s/m]    %8.3f   %8.3f\n', ...
    B_eq_nominal + B_emf, B_total);
fprintf('  eta_g [-]          %8.3f   %8.3f   %+.1f%%\n', ...
    eta_g_nominal, eta_g, (eta_g-eta_g_nominal)/eta_g_nominal*100);
fprintf('\n  Validation:\n');
fprintf('    RMS position error = %.3f cm\n', rmse_tuned);
fprintf('    Velocity pole      = %.2f rad/s (%.2f Hz)\n', B_total/M_e, B_total/M_e/(2*pi));

if rmse_tuned < 1.0
    fprintf('\n  MODEL VALIDATED (RMS < 1 cm)\n');
    fprintf('    Ready for controller design.\n');
else
    fprintf('\n  WARNING: RMS error > 1 cm -- consider:\n');
    fprintf('    - Reducing chirp amplitude (currently %.1f V)\n', A_chirp);
    fprintf('    - Checking for end-stop clipping\n');
    fprintf('    - Manual B_eq adjustment\n');
end

% Save tuned parameters
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
save_file = fullfile(SEESAW_ROOT, 'data', 'tuned', 'tuned_cart.mat');
save(save_file, 'B_eq', 'B_eq_nominal', 'B_total', 'alpha_f', 'B_emf', ...
     'eta_g', 'A_cart', 'B_cart', 'C_cart', 'D_cart', 'sys_cart', ...
     'Gx', 'num_x', 'den_x', 'rmse_tuned');
fprintf('\n  Tuned parameters saved to: data/tuned_cart.mat\n');
fprintf('============================================================\n');

%% 6. DETERMINE TESTING FREQUENCIES
%% DETERMINE DYNAMIC PHYSICAL LIMITS

% Hardware Friction Compensation Values
v_friction_max = max(ud_pos, ud_neg); 
V_max_budget = 3.0 - v_friction_max; % Available budget for the linear sine wave

% Reconstruct the Force Transfer Function
G_force = minreal(alpha_f * eta_m * (M_e * s + B_eq) / (M_e * s + B_total));

% --- Calculate w_max (Control Target vs Torque Limit) ---
% 1. Define closed-loop cascade targets
% because of non-minimum phase plant, we define the ideal outer bandwidth
% like the geometric mean of unstable pole and zero.
w_bw_outer = sqrt(2.2144*8.8589);               % [rad/s] NMP bounded target
w_bw_inner_target = 4 * w_bw_outer;             % [rad/s] Timescale separation
w_validation_target = w_bw_inner_target * 1.5;  % [rad/s] Add 1.5x buffer for phase margin

% 2. Extract the physical torque limit
F_limit = 4 * norm(evalfr(G_force, 14j)); % 4V @ 14 rad/s clicking limit
max_theoretical_force = (alpha_f * eta_m) * V_max_budget;

% 3. Determine the final sweep maximum
if max_theoretical_force < F_limit
    disp('>>> SAFETY BONUS: Torque asymptote is below the clicking limit!');
    fprintf('    Capping sweep based on Cascade Target: %.2f rad/s\n', w_validation_target);
    w_max = w_validation_target;
else
    % The force will eventually hit the limit. Find where it breaks.
    force_error = @(w) (norm(evalfr(G_force, w*1j)) * V_max_budget) - F_limit;
    w_break = fzero(force_error, 30); 
    
    if w_break < w_validation_target
        disp('>>> WARNING: Hardware torque limit prevents reaching cascade target.');
        fprintf('    Capping sweep at physical limit: %.2f rad/s\n', w_break);
        w_max = w_break;
    else
        fprintf('    Hardware is safe past cascade target. Capping at: %.2f rad/s\n', w_validation_target);
        w_max = w_validation_target;
    end
end

% --- Calculate w_min (Displacement Limit) ---
% Max displacement is 75% of half the track length
max_displacement = (T_c / 2) * 0.75; 

% Find w_min: The frequency where our V_max_budget generates exactly max_displacement
pos_error = @(w) (norm(evalfr(Gx, w*1j)) * V_max_budget) - max_displacement;
w_min = abs(fzero(pos_error, 0.1)); % Start search at 0.1 rad/s

disp('--- Dynamic Frequency Limits Computed ---');
fprintf('Min Freq (Track Limit):  %.2f rad/s\n', w_min);
fprintf('Max Freq (Torque Limit): %.2f rad/s\n\n', w_max);


%% SCHROEDER-PHASED MULTI-SINE GENERATOR

% --- User Parameters ---
N_freq = 10; % Number of frequencies (Recommend 10-20 for good resolution)
% Duration: Ensure we capture at least 3 full cycles of the lowest frequency
T_test = max(20, 3 * (2*pi/w_min)); 

% --- Generate Logarithmic Frequencies ---
w_vals = logspace(log10(w_min), log10(w_max), N_freq);

% --- Define Amplitudes and Phases ---
A = ones(1, N_freq); 
k = 1:N_freq;
phi = -pi * k .* (k - 1) / N_freq; % Schroeder phase formula

% --- Build the Time Vector and Signal ---
t = 0:(1/Fs_hw):T_test;
u_raw = zeros(size(t));

for i = 1:N_freq
    u_raw = u_raw + A(i) * sin(w_vals(i) * t + phi(i));
end

% --- Safety Scaling ---
peak_raw = max(abs(u_raw));
% Scale the array so the absolute highest peak touches our V_max_budget
u_safe = (V_max_budget / peak_raw) * u_raw;

% --- Apply the Friction Compensator Logic ---
epsilon = 0.05; % Noise deadband threshold
u_real = zeros(size(u_safe));
u_real(u_safe > epsilon)  = u_safe(u_safe > epsilon) + ud_pos;
u_real(u_safe < -epsilon) = u_safe(u_safe < -epsilon) - ud_neg;


%% VISUALIZATION

figure('Name', 'Schroeder Multi-Sine Validation', 'Position', [200 200 800 400]);

% Plot the compensated "real" hardware command (in red)
plot(t, u_real, 'r--', 'LineWidth', 1, 'DisplayName', 'Hardware Command ($u_{real}$)');
hold on;
% Plot the ideal linear command (in blue)
plot(t, u_safe, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Linear Command ($u_{safe}$)');

V_total_peak = V_max_budget + v_friction_max;

% Add threshold lines
yline(V_total_peak, 'k-.', 'Hardware Peak Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(-V_total_peak, 'k-.', 'HandleVisibility', 'off');
yline(V_max_budget, 'k--', 'Max Safe Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(-V_max_budget, 'k--', 'HandleVisibility', 'off');

xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Voltage [V]', 'Interpreter', 'latex');
title('Schroeder-Phased Multi-Sine Input Vector', 'Interpreter', 'latex');
grid on;
legend('Location', 'northeast', 'Interpreter', 'latex');
ylim([-V_total_peak*1.2, V_total_peak*1.2]);

disp('--- Multi-Sine Profile Generated ---');
fprintf('Frequencies:      %d (from %.2f to %.2f rad/s)\n', N_freq, w_min, w_max);
fprintf('Raw Peak:         %.2f V (Without scaling)\n', peak_raw);
fprintf('Linear Peak:      %.2f V (u_safe)\n', max(abs(u_safe)));
fprintf('Hardware Peak:    %.2f V (u_real)\n\n', max(abs(u_real)));


%% PRE-FLIGHT HARDWARE SAFETY CHECKLIST

disp('==================================================');
disp('      PRE-FLIGHT HARDWARE SAFETY CHECKLIST        ');
disp('==================================================');

% --- Define Hardware Absolute Limits ---
LIMIT_V_PEAK    = 6.0;           % [V] Nominal maximum motor voltage
LIMIT_I_PEAK    = 3.0;           % [A] Absolute peak current before coil damage
LIMIT_I_RMS     = 1.0;           % [A] Continuous RMS current limit (thermal)
LIMIT_X_MAX     = T_c/2 * 0.75;  % [m] Safe track boundary
LIMIT_F_PEAK    = F_limit;       % [N] Gear clicking limit

% --- Simulate the Physical Dynamics ---
u_sim = u_safe(:);
t_sim = t(:);

[x_sim_resp, ~, ~] = lsim(Gx, u_sim, t_sim);
[v_sim_resp, ~, ~] = lsim(Gv, u_sim, t_sim);
[F_sim_resp, ~, ~] = lsim(G_force, u_sim, t_sim);

% --- Calculate Electrical Current ---
v_emf_sim = k_m * (v_sim_resp * K_g / r_mp);
i_sim_resp = (u_sim - v_emf_sim) / R_m;

% --- Extract Peak and RMS Values ---
peak_V = max(abs(u_sim)) + v_friction_max; 
peak_I = max(abs(i_sim_resp));
rms_I  = sqrt(mean(i_sim_resp.^2));

% Calculate max dynamic force PLUS friction compensator's physical force
peak_F_dynamic = max(abs(F_sim_resp));
peak_F_total = peak_F_dynamic + (alpha_f * eta_m * v_friction_max); 
peak_X = max(abs(x_sim_resp));

% --- Evaluate and Report ---
checks_passed = true;
fprintf('%-20s | %-12s | %-12s | %-10s\n', 'PARAMETER', 'SIMULATED', 'LIMIT', 'STATUS');
disp('------------------------------------------------------------------');

% Voltage Check
if peak_V <= LIMIT_V_PEAK
    fprintf('%-20s | %-10.2f V | %-10.2f V | [ PASS ]\n', 'Peak Voltage', peak_V, LIMIT_V_PEAK);
else
    fprintf('%-20s | %-10.2f V | %-10.2f V | [ FAIL ]\n', 'Peak Voltage', peak_V, LIMIT_V_PEAK);
    checks_passed = false;
end

% Peak Current Check
if peak_I <= LIMIT_I_PEAK
    fprintf('%-20s | %-10.2f A | %-10.2f A | [ PASS ]\n', 'Peak Current', peak_I, LIMIT_I_PEAK);
else
    fprintf('%-20s | %-10.2f A | %-10.2f A | [ FAIL ]\n', 'Peak Current', peak_I, LIMIT_I_PEAK);
    checks_passed = false;
end

% RMS Current Check
if rms_I <= LIMIT_I_RMS
    fprintf('%-20s | %-10.2f A | %-10.2f A | [ PASS ]\n', 'RMS Current', rms_I, LIMIT_I_RMS);
else
    fprintf('%-20s | %-10.2f A | %-10.2f A | [ FAIL ]\n', 'RMS Current', rms_I, LIMIT_I_RMS);
    checks_passed = false;
end

% Rack Force Check
if peak_F_total <= LIMIT_F_PEAK
    fprintf('%-20s | %-10.2f N | %-10.2f N | [ PASS ]\n', 'Peak Force', peak_F_total, LIMIT_F_PEAK);
else
    fprintf('%-20s | %-10.2f N | %-10.2f N | [ FAIL ]\n', 'Peak Force', peak_F_total, LIMIT_F_PEAK);
    checks_passed = false;
end

% Position Drift Check
if peak_X <= LIMIT_X_MAX
    fprintf('%-20s | %-10.2f m | %-10.2f m | [ PASS ]\n', 'Max Displacement', peak_X, LIMIT_X_MAX);
else
    fprintf('%-20s | %-10.2f m | %-10.2f m | [ FAIL ]\n', 'Max Displacement', peak_X, LIMIT_X_MAX);
    checks_passed = false;
end

disp('==================================================');
if checks_passed
    fprintf('>>> SUCCESS: Signal is SAFE to deploy on hardware. Run for %.2fs. <<<\n', T_test);
    disp('    (Ensure the cart is physically centered before starting!)');
    
    % --- Save Data for Simulink and Validation ---
    % 1. Dynamically find the project root directory
    if ~exist('SEESAW_ROOT', 'var')
        % Assumes this script is run from a subfolder, climbing up twice
        SEESAW_ROOT = fileparts(mfilename('fullpath')); 
        SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); 
    end
    
    % 2. Ensure the /data directory exists
    data_dir = fullfile(SEESAW_ROOT, 'data', 'cartModeling');
    if ~exist(data_dir, 'dir')
        mkdir(data_dir);
    end
    
    % 3. Format data specifically for Simulink "From File" block
    simulink_multisine = [t(:)'; u_safe(:)'];
    
    % 4. Save the files separately to prevent Simulink confusion
    simulink_file = fullfile(data_dir, 'multisine_simulink.mat');
    params_file = fullfile(data_dir, 'multisine_params.mat');
    
    % Save ONLY the matrix for Simulink (100% safe)
    save(simulink_file, 'simulink_multisine', '-v7.3');
    
    % Save all the metadata for your post-processing analysis script
    save(params_file, 't', 'u_safe', 'w_vals', 'N_freq');
    
    fprintf('\n>>> Validation data successfully saved to the /data/cartModeling directory.\n');
    disp('>>> In Simulink, point your "From File" block exactly to: multisine_simulink.mat');
    fprintf('>>> Run the simulation for %.2fs\n\n', T_test);

else
    disp('>>> WARNING: DANGER LIMITS EXCEEDED! <<<');
    disp('    Do not deploy. The safety limits were violated.');
end
disp(' ');