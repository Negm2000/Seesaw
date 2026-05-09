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

B_eq_nominal = B_eq;  % save for comparison later
eta_g_nominal = eta_g;
fprintf('\n----- Nominal Model -----\n');
fprintf('  B_eq     = %.2f N*s/m (will be tuned)\n', B_eq);
fprintf('  eta_g    = %.2f N*s/m (will be tuned)\n', eta_g);
fprintf('--- Impacted Parameters ---\n');
fprintf('  B_total  = %.2f N*s/m\n', B_total);
fprintf('  alpha_f  = %.4f (motor force constant)\n', alpha_f);
fprintf('-------------------------\n');

%% 4. LOAD & INSPECT HARDWARE DATA
%  Load the frequency sweep data collected from QUARC.
%  Plot raw time traces to sanity-check before analysis.

if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
data_file = fullfile(SEESAW_ROOT, 'data', 'cartModeling', 'step_3V.mat');

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

%% TUNE BASED ON VELOCITY STEADY-STATE AFTER STEP RESPONSE
% Mathematically tuning eta_g and B_eq.
% This does not mean that their real physical values will be these ones.
% It means that we consider that the model behave as it would if the
% hardware had these values.

V_step = 3 - 2*0.6;              % The step input applied

% keep datasheet limits
etag_min = eta_g_nominal * 0.75; % -10%
etag_max = eta_g_nominal * 1.1; % +10%

% Define the gain and time constant of hardware
pulse_start_time = 0;
pulse_end_time = 1.0;
pulse_indices = tdot_hw >= pulse_start_time & tdot_hw <= pulse_end_time;

ss_indices = tdot_hw >= (pulse_end_time - 0.2) & tdot_hw <= pulse_end_time;
V_ss = mean(xcdot_hw(ss_indices));
V_target = 0.632 * V_ss;

crossing_index = find(xcdot_hw(pulse_indices) >= V_target, 1, 'first');

t_pulse_window = tdot_hw(pulse_indices);
tau_meas = t_pulse_window(crossing_index) - pulse_start_time;
K_meas = V_ss / V_step;

% 3. Define the Simulation Logic using Anonymous Functions
% The algorithm will only tweak these two values:
% p(1) = eta_g (Gearbox Efficiency)
% p(2) = B_eq (Equivalent Viscous Friction)
calc_alpha = @(p) (k_t * K_g * p(1) * eta_m) / (r_mp * R_m);
calc_Btot  = @(p) ((k_t * k_m * K_g^2 * p(1)) / (r_mp^2 * R_m)) + p(2);

% Helper functions for Gain and Time Constant to keep equations clean
calc_K   = @(p) calc_alpha(p) / calc_Btot(p);
calc_tau = @(p) M_e / calc_Btot(p);

% Step Responses Calculation
v_sim = @(p) V_step * (calc_alpha(p) / calc_Btot(p)) * (1 - exp(-(calc_Btot(p) / M_e) * tdot_hw));
x_sim = @(p) V_step * calc_K(p) * (t_hw - calc_tau(p) * (1 - exp(-t_hw / calc_tau(p))));

% 4. Define the Cost Function (Mean Squared Error)
% We calculate the normal error, and add a massive penalty if it goes out of bounds
costFunction = @(p) mean((xcdot_hw - v_sim(p)).^2) + ...
                    1e6 * (p(1) < etag_min) + ...  % Penalty if too low
                    1e6 * (p(1) > etag_max);       % Penalty if too high

% 5. Initial Guesses for the 2 Unknowns
% Provide realistic starting points.
initial_params = [eta_g_nominal, 6.7217];
model_params = [eta_g_nominal, B_eq_nominal];

% 6. Run Optimization (Nelder-Mead)
options = optimset('Display', 'iter', 'TolFun', 1e-6, 'TolX', 1e-6);
disp('Hunting for optimal gearbox efficiency and friction...');
best_params = fminsearch(costFunction, initial_params, options);

% 7. Extract and Display Results
eta_g = best_params(1);
B_eq   = best_params(2);

% Calculate the final transfer function metrics using the locked eta_m
alpha_f = calc_alpha(best_params);
B_total  = calc_Btot(best_params);
Kdc_opt   = alpha_f / B_total;
tau_opt   = M_e / B_total;

rmse_init = sqrt(mean((xc_hw - x_sim(model_params)).^2));
rmse_tuned     = sqrt(mean((xc_hw - x_sim(best_params)).^2));

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
plot(t_hw, x_sim(model_params)*100, 'b--', 'LineWidth', 1, 'DisplayName', 'Initial Model');
grid on;

% --- Subplot 3: Velocity ---
subplot(3,1,3);
plot(tdot_hw, xcdot_hw*100, 'k-', 'LineWidth', 2, 'DisplayName', 'Hardware'); 
ylabel('$\dot{x}_c$ [cm/s]');
title('Cart Velocity'); 
hold on;
plot(tdot_hw, v_sim(best_params)*100, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Optimized Model');
plot(tdot_hw, v_sim(model_params)*100, 'b--', 'LineWidth', 1, 'DisplayName', 'Initial Model');
xlabel('Time [s]'); 
grid on; 
legend('Location', 'southeast');

sgtitle('Step Response Tuning');

%% 7. APPLY TUNED PARAMETERS & REBUILD MODEL
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

% Updated transfer functions
Gx = minreal(Kdc_opt / (tau_opt*s^2 + s));
Gv = s * Gx;
Ga = s^2 * Gx;

num_x = alpha_f;
den_x = [M_e B_total 0];

fprintf(' Transfer Function and State-space model rebuilt (Gx, A_cart, B_cart)\n');

%% 10. SUMMARY & SAVE
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
save_file = fullfile(SEESAW_ROOT, 'data', 'tuned_cart.mat');
save(save_file, 'B_eq', 'B_eq_nominal', 'B_total', 'alpha_f', 'B_emf', ...
     'eta_g', 'A_cart', 'B_cart', 'C_cart', 'D_cart', ...
     'Gx', 'num_x', 'den_x', 'rmse_tuned');
fprintf('\n  Tuned parameters saved to: data/tuned_cart.mat\n');
fprintf('============================================================\n');

%% DETERMINE TESTING FREQUENCIES

% Define the target limits
min_displacement = 0.01;                     % [m]
max_dB = 20*log10(min_displacement/V_nom);   % [dB]

max_displacement = T_c * 0.75;
V_min = 2;
min_dB = 20*log10(max_displacement/V_min);

% Create anonymous functions for error calculation
mag_error_max = @(w) 20*log10(squeeze(bode(Gx, w))) - max_dB;
mag_error_min = @(w) 20*log10(squeeze(bode(Gx, w))) - min_dB;

% Find crossover frequencies
initial_guess_max = 30; % rad/s
w_cross_max = fzero(mag_error_max, initial_guess_max);

initial_guess_min = 0.1; % rad/s
w_cross_min = abs(fzero(mag_error_min, initial_guess_min));

disp('Numerical frequency range (rad/s):');
fprintf('Min: %.2f, Max: %.2f\n', w_cross_min, w_cross_max);

% Generating the testing frequencies
suite = [0, 1, 2, 4, 8];
w_vals = [];

% Decimal scaling for values < 1
w_vals = [w_vals, suite * 0.1];

% Blocks of 10 for values >= 1
max_tens = floor(w_cross_max / 10) * 10;
for k = 0:10:max_tens
    w_vals = [w_vals, k + suite];
end

% Clean up and filter by boundaries
w_vals = unique(w_vals);
w_out = w_vals(w_vals >= w_cross_min & w_vals <= w_cross_max);

% Initial voltage scaling (will be clamped by safety rules below)
v_raw = V_min + (w_out - w_cross_min) * (V_nom - V_min) / (w_cross_max - w_cross_min);
v_out = max(V_min, min(V_nom, round(v_raw)));

% --- Mechanical Force / Torque Safety Limits ---

% 1. Reconstruct the Force Transfer Function
G_force = minreal(alpha_f * (M_e * s + B_eq) / (M_e * s + B_total));

% 2. Safety thresholds based on bench observations
% 4V @ 18 rad/s triggered the click.
click_force_mag = 4 * norm(evalfr(G_force, 18j));
F_limit = click_force_mag * 0.85; % 15% safety margin

% 3. Hardware Friction Compensation Values
% IMPORTANT: Replace these with your actual measured bench values!
ud_pos = 0.8; 
ud_neg = 0.7;
v_friction_max = max(ud_pos, ud_neg); 

% 4. Enforce limits
v_safe = zeros(size(w_out));
v_total_peak = zeros(size(w_out));

for i = 1:length(w_out)
    w = w_out(i);
    
    % Force generated per 1 Volt input at this frequency
    force_per_volt = norm(evalfr(G_force, w*1j)); 
    
    % MAXIMUM TOTAL voltage the gears can safely handle
    v_max_total = F_limit / force_per_volt;
    
    % Available voltage budget for the sine wave (subtract V_comp penalty)
    v_max_sine = max(0, v_max_total - v_friction_max);
    
    % Clamp the originally planned voltage to the safety ceiling
    v_safe(i) = min(v_out(i), floor(v_max_sine));
    
    % Record what the absolute peak voltage will be hitting the motor
    v_total_peak(i) = v_safe(i) + v_friction_max;
end

% 5. Final Output Table
disp('');
disp('Suggested Frequency and Voltage Input for frequency validation');
disp(' -----------------------------------------------');
disp('Frequency (rad/s) | Safe Sine (V) | Max Peak (V)');
disp('------------------------------------------------');
for i = 1:length(w_out)
    fprintf('%17.1f | %13d | %12.2f\n', w_out(i), v_safe(i), v_total_peak(i));
end
