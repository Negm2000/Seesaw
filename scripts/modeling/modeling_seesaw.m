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
    error('data/data.mat not found. Run Section 3 on hardware first.');
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
plot(t_hw, V_cmd_hw, 'k-'); ylabel('V_{cmd} [V]'); title('Chirp Input');
ylim([-V_sat, V_sat]);
grid on;
subplot(3,1,2);
plot(t_hw, xc_hw*100, 'r-'); ylabel('x_c [cm]'); title('Cart Position (Hardware)');
grid on;
subplot(3,1,3);
plot(tdot_hw, xcdot_hw*100, 'r-'); ylabel('dx_c/dt [cm/s]'); title('Cart Velocity (Hardware)');
xlabel('Time [s]'); grid on;
sgtitle('Raw Hardware Step Response Data');

%% TUNE BASED ON VELOCITY STEADY-STATE AFTER STEP RESPONSE
% Mathematically tuning eta_g and B_eq.
% This does not mean that their real physical values will be these ones.
% It means that we consider that the model behave as it would if the
% hardware had these values.

V_step = 3;              % The step input applied

% keep datasheet limits
etag_min = eta_g_nominal * 0.9; % -10%
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

% STEP RESPONSE TUNING INSPECTION
figure
subplot(2, 1, 1);
plot(tdot_hw, xcdot_hw, 'k.', 'DisplayName', 'Hardware Data (diff)'); hold on;
plot(tdot_hw, v_sim(model_params), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Initial Model');
plot(tdot_hw, v_sim(best_params), 'r-', 'LineWidth', 2.5, 'DisplayName', 'Optimized Model');
title('Velocity Tracking');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('Location', 'southeast'); grid on;

% --- Subplot 2: Position ---
subplot(2, 1, 2);
plot(t_hw, xc_hw, 'k.', 'DisplayName', 'Hardware Data (Raw)'); hold on;
plot(t_hw, x_sim(model_params), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Initial Model');
plot(t_hw, x_sim(best_params), 'r-', 'LineWidth', 2.5, 'DisplayName', 'Optimized Model');
title('Position Tracking');
xlabel('Time (s)'); ylabel('Position (m)');
legend('Location', 'southeast'); grid on;

%% 7. APPLY TUNED PARAMETERS & REBUILD MODEL
%  Overwrite B_eq with the tuned value, recompute all derived quantities,
%  and rebuild both state-space models (cart-only and full seesaw).

fprintf('\n--- Applying Tuned Parameters ---\n');

% Recompute total damping (alpha_f and B_emf are constants -- not affected by B_eq)
B_total = B_eq + B_emf;

fprintf('  B_eq (tuned)  = %.4f N*s/m\n', B_eq);
fprintf('  B_total       = %.4f N*s/m\n', B_total);

% --- Phase 1: Cart on Table ---
A_cart = [0, 1; 0, -B_total/M_c];
B_cart = [0; alpha_f*eta_m/M_c];
C_cart = eye(2);
D_cart = zeros(2,1);

% --- Phase 2: Cart on Seesaw (linearised) ---
% Must match seesaw_params.m linearisation exactly (Good ref, page 6).
M_eff = [M_c,          -M_c*D_T;
         -M_c*D_T,      J_pivot + M_c*D_T^2];
M_inv = inv(M_eff);

G_rhs = [0, -B_total,  -g*M_c,                        0;
         -g*M_c, 0,     g*(M_c*D_T + M_SW*D_C),  -B_SW];

A_sw = [0, 1, 0, 0;
        M_inv(1,:) * G_rhs;
        0, 0, 0, 1;
        M_inv(2,:) * G_rhs];

G_inp = [alpha_f*eta_m; 0];
B_sw = [0; M_inv(1,:)*G_inp; 0; M_inv(2,:)*G_inp];
C_sw = eye(4);
D_sw = zeros(4,1);

% Updated transfer functions
Gx = minreal(Kdc_opt / (tau_opt*s^2 + s));
Gv = s * Gx;
Ga = s^2 * Gx;

num_x = alpha_f;
den_x = [M_e B_total 0];

fprintf('  State-space models rebuilt (A_cart, B_cart, A_sw, B_sw)\n');

% Seesaw eigenvalues
ev = eig(A_sw);
fprintf('  Seesaw eigenvalues:\n');
for k = 1:length(ev)
    if imag(ev(k)) ~= 0
        fprintf('    lambda_%d = %.4f %+.4fi\n', k, real(ev(k)), imag(ev(k)));
    else
        fprintf('    lambda_%d = %.4f\n', k, real(ev(k)));
    end
end

%% 8. FREQUENCY RESPONSE COMPARISON (TUNED)
%  Final Bode plot: nominal vs tuned vs hardware — the money plot.

[mag_t, phase_t] = bode(G_xc_tuned, 2*pi*freq_range);
mag_t_dB   = 20*log10(squeeze(mag_t)*100);
phase_t_deg = squeeze(phase_t);

figure('Name', 'FRF: Tuned Model vs Hardware', 'Position', [150 120 1000 600]);
subplot(2,1,1);
semilogx(freq_range, mag_an_dB, 'b--', 'LineWidth', 1); hold on;
semilogx(freq_range, mag_t_dB, 'g-', 'LineWidth', 2);
semilogx(hw_freq, 20*log10(abs(hw_H_xc)*100), 'r-', 'LineWidth', 1.5);
grid on; ylabel('Magnitude [dB cm/V]'); xlim([f_chirp_start f_chirp_end]);
legend('Nominal', 'TUNED', 'Hardware', 'Location', 'best');
subplot(2,1,2);
semilogx(freq_range, phase_an_deg, 'b--', 'LineWidth', 1); hold on;
semilogx(freq_range, phase_t_deg, 'g-', 'LineWidth', 2);
semilogx(hw_freq, unwrap(angle(hw_H_xc))*180/pi, 'r-', 'LineWidth', 1.5);
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]'); xlim([f_chirp_start f_chirp_end]);
sgtitle(sprintf('Frequency Response — AFTER Tuning (B_{eq} = %.2f \\rightarrow %.2f)', B_eq_nominal, B_eq));

%% 9. TIME-DOMAIN VALIDATION
%  Simulate the tuned model with the same chirp input that was fed to the
%  hardware, then overlay the two position traces and compute RMS error.

sys_tuned = ss(A_cart, B_cart, C_cart, D_cart);
V_in = max(-V_sat, min(V_sat, K_a * V_cmd_hw));
x0 = [xc_hw(1); xcdot_hw(1)];  % match hardware initial conditions
[y_sim, ~] = lsim(sys_tuned, V_in, t_hw, x0);
xc_sim = y_sim(:,1);  % position [m]

err_cm = (xc_sim - xc_hw) * 100;
rmse_tuned = rms(err_cm(mask));

figure('Name', 'Time-Domain Validation', 'Position', [100 100 1000 700]);
subplot(3,1,1);
plot(t_hw, xc_hw*100, 'r-', 'LineWidth', 1.2); hold on;
plot(t_hw, xc_sim*100, 'b-', 'LineWidth', 1.2);
ylabel('x_c [cm]'); title('Cart Position: Hardware vs Tuned Model');
legend('Hardware', 'Tuned Model', 'Location', 'best');
grid on;

subplot(3,1,2);
plot(t_hw, err_cm, 'k-', 'LineWidth', 1);
ylabel('Error [cm]');
title(sprintf('Position Error (RMS = %.3f cm, excluding first 2s)', rmse_tuned));
grid on;

subplot(3,1,3);
plot(t_hw, V_cmd_hw, 'k-', 'LineWidth', 0.8);
ylabel('V_{cmd} [V]'); xlabel('Time [s]');
title('Input Signal'); grid on;

sgtitle('Time-Domain Validation', 'FontWeight', 'bold');

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
fprintf('  eta_g [-]          %8.3f   %8.3f   (fixed)\n', eta_g_nominal, eta_g);
fprintf('\n  Validation:\n');
fprintf('    RMS position error = %.3f cm\n', rmse_tuned);
fprintf('    Velocity pole      = %.2f Hz\n', B_total/M_e/(2*pi));

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

%% === LOCAL FUNCTIONS ===

function [freq_out, H_xc, H_xcdot] = compute_frf(t, u, xc, xcdot, dt)
% COMPUTE_FRF  Welch's method FRF estimate (H1 estimator).
%   Uses MATLAB's tfestimate for robust cross-spectral estimation.
%   Segment size is kept moderate (4096 samples) so that the chirp
%   signal is approximately stationary within each window.  This avoids
%   the old bug where huge windows caused the chirp to sweep across
%   many bins, diluting Suu and poisoning the H1 estimator at high
%   frequencies.
    Fs = 1/dt;
    n_seg = min(4096, 2^nextpow2(length(t)/8));  % >= 8 segments
    n_seg = max(n_seg, 512);                       % floor for very short records

    [H_xc_raw,    freq_fft] = tfestimate(u, xc,    hanning(n_seg), n_seg/2, n_seg, Fs);
    [H_xcdot_raw, ~       ] = tfestimate(u, xcdot, hanning(n_seg), n_seg/2, n_seg, Fs);

    % Use chirp range from workspace; fall back to defaults if absent
    try f_lo = evalin('base','f_chirp_start'); catch, f_lo = 0.1;  end
    try f_hi = evalin('base','f_chirp_end');   catch, f_hi = 12.0; end
    valid = freq_fft >= f_lo & freq_fft <= f_hi;
    freq_out = freq_fft(valid);
    H_xc     = H_xc_raw(valid);
    H_xcdot  = H_xcdot_raw(valid);
end

function cost = tune_cost_Beq(B_try, V_cmd, t, xcdot_hw, mask, p)
% TUNE_COST_BEQ  RMS velocity error for a candidate B_eq value.
%   Velocity is used instead of position because:
%   (a) B_eq is a damping coefficient that directly sets the velocity pole;
%   (b) position integrates any DC drift (static friction, track tilt) that
%       would bias the optimizer to compensate slope rather than damping.
    af    = (p.eta_g * p.K_g * p.k_t) / (p.R_m * p.r_mp);
    B_tot = B_try + af * p.K_g * p.k_m / p.r_mp;
    sys   = ss([0, 1; 0, -B_tot/p.M_c], [0; af*p.eta_m/p.M_c], [0, 1], 0);
    x0    = [0; xcdot_hw(1)];  % states: [x_c; x_c_dot]; position irrelevant to velocity cost
    xcdot_sim = lsim(sys, max(-p.V_sat, min(p.V_sat, p.K_a*V_cmd)), t, x0);
    cost  = sqrt(mean((xcdot_sim(mask) - xcdot_hw(mask)).^2));
end
