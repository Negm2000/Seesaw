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
fprintf('\n--- Nominal Model ---\n');
fprintf('  B_eq     = %.2f N*s/m (will be tuned)\n', B_eq);
fprintf('  B_emf    = %.2f N*s/m (from back-EMF)\n', B_emf);
fprintf('  B_total  = %.2f N*s/m\n', B_total);
fprintf('  alpha_f  = %.4f (motor force constant)\n', alpha_f);

%% 2. ANALYTICAL TRANSFER FUNCTION
%  Derive and plot the cart position transfer function G(s) = X_c(s)/V_m(s).
%  This is the "prediction" we'll compare against hardware.

s_tf = tf('s');
G_xc    = K_a * alpha_f * eta_m / (M_c * s_tf^2 + B_total * s_tf);
G_xcdot = K_a * alpha_f * eta_m / (M_c * s_tf + B_total);

freq_range = logspace(log10(f_chirp_start), log10(f_chirp_end), 200);
[mag_an, phase_an] = bode(G_xc, 2*pi*freq_range);
mag_an_dB   = 20*log10(squeeze(mag_an) * 100);  % convert m/V to cm/V
phase_an_deg = squeeze(phase_an);

figure('Name', 'Analytical Bode: Cart on Table', 'Position', [50 50 1000 600]);
subplot(2,1,1);
semilogx(freq_range, mag_an_dB, 'b-', 'LineWidth', 2);
grid on; ylabel('Magnitude [dB cm/V]');
title('Analytical Model: V_{cmd} \rightarrow x_c');
subplot(2,1,2);
semilogx(freq_range, phase_an_deg, 'b-', 'LineWidth', 2);
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
sgtitle(sprintf('Nominal Model (B_{eq} = %.1f N{\\cdot}s/m)', B_eq));

fprintf('\nDC gain (velocity): %.4f (cm/s)/V\n', dcgain(G_xcdot)*100);
fprintf('Velocity pole: %.2f Hz\n', B_total/M_c / (2*pi));

%% 3. BUILD QUARC FREQUENCY TEST MODEL
%  Programmatically build IP02_FreqTest.slx with chirp input.
%  This model runs on hardware via QUARC External Mode.

frequency_setup;  % builds models/IP02_FreqTest.slx

fprintf('\n========================================\n');
fprintf(' NEXT STEP: Go to hardware!\n');
fprintf('========================================\n');
fprintf(' 1. Open models/IP02_FreqTest.slx\n');
fprintf(' 2. Connect to QUARC (External Mode)\n');
fprintf(' 3. Run the model — wait %d seconds\n', chirp_duration);
fprintf(' 4. Data saves automatically to data/data.mat\n');
fprintf(' 5. Come back here and run Section 4\n');
fprintf('========================================\n');

%% 4. LOAD & INSPECT HARDWARE DATA
%  Load the frequency sweep data collected from QUARC.
%  Plot raw time traces to sanity-check before analysis.

if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
data_file = fullfile(SEESAW_ROOT, 'data', 'data.mat');

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
xcdot_hw  = raw(4, :)';      % [m/s]
dt_hw     = mean(diff(t_hw));
Fs_hw     = 1 / dt_hw;

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
grid on;
subplot(3,1,2);
plot(t_hw, xc_hw*100, 'r-'); ylabel('x_c [cm]'); title('Cart Position (Hardware)');
grid on;
subplot(3,1,3);
plot(t_hw, xcdot_hw*100, 'r-'); ylabel('dx_c/dt [cm/s]'); title('Cart Velocity (Hardware)');
xlabel('Time [s]'); grid on;
sgtitle('Raw Hardware Frequency Sweep Data');

%% 5. FREQUENCY RESPONSE COMPARISON (UNTUNED)
%  Compute the empirical FRF from hardware data using Welch's method,
%  then overlay against the analytical model.

[hw_freq, hw_H_xc, ~] = compute_frf(t_hw, V_cmd_hw, xc_hw, xcdot_hw, dt_hw);

figure('Name', 'FRF: Model vs Hardware (Untuned)', 'Position', [100 100 1000 600]);
subplot(2,1,1);
semilogx(freq_range, mag_an_dB, 'b-', 'LineWidth', 1.5); hold on;
semilogx(hw_freq, 20*log10(abs(hw_H_xc)*100), 'r-', 'LineWidth', 1.5);
grid on; ylabel('Magnitude [dB cm/V]'); xlim([f_chirp_start f_chirp_end]);
title('V_{cmd} \rightarrow x_c');
legend('Analytical (nominal)', 'Hardware', 'Location', 'best');
subplot(2,1,2);
semilogx(freq_range, phase_an_deg, 'b-', 'LineWidth', 1.5); hold on;
semilogx(hw_freq, unwrap(angle(hw_H_xc))*180/pi, 'r-', 'LineWidth', 1.5);
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]'); xlim([f_chirp_start f_chirp_end]);
sgtitle(sprintf('Frequency Response — BEFORE Tuning (B_{eq} = %.1f)', B_eq));

%% 6. AUTO-TUNE B_eq
%  Use fminsearch to find the B_eq that minimizes time-domain RMS error
%  between the linear model simulation and hardware position data.
%  eta_g is FIXED at the hardware spec (0.90).

fprintf('\n--- Auto-Tuning B_eq ---\n');
fprintf('  Initial B_eq = %.2f N*s/m\n', B_eq);
fprintf('  eta_g = %.2f (FIXED at hardware spec)\n', eta_g);

p_tune = struct('K_a',K_a, 'V_sat',V_sat, 'R_m',R_m, 'k_t',k_t, ...
    'k_m',k_m, 'eta_m',eta_m, 'eta_g',eta_g, 'K_g',K_g, 'r_mp',r_mp, 'M_c',M_c);

% Cost function: RMS velocity error (skip first 2s of transient)
% Velocity is used because B_eq is a damping coefficient -- it directly
% governs the velocity dynamics, and velocity is immune to DC drift from
% static friction or track tilt that would bias a position-based cost.
mask = t_hw > 2.0;
cost_fn = @(B) tune_cost_Beq(B, V_cmd_hw, t_hw, xcdot_hw, mask, p_tune);

opts = optimset('Display', 'iter', 'TolX', 1e-3, 'TolFun', 1e-6);
[B_eq_opt, cost_opt] = fminsearch(cost_fn, B_eq, opts);

fprintf('\n  RESULT: B_eq = %.4f N*s/m (was %.2f)\n', B_eq_opt, B_eq_nominal);
fprintf('  Change: %+.1f%%\n', (B_eq_opt - B_eq_nominal)/B_eq_nominal * 100);
fprintf('  RMS velocity error: %.4f cm/s\n', cost_opt * 100);

%% 7. APPLY TUNED PARAMETERS & REBUILD MODEL
%  Overwrite B_eq with the tuned value, recompute all derived quantities,
%  and rebuild both state-space models (cart-only and full seesaw).

fprintf('\n--- Applying Tuned Parameters ---\n');

% Overwrite
B_eq = B_eq_opt;

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
s_tf = tf('s');
G_xc_tuned    = K_a * alpha_f * eta_m / (M_c * s_tf^2 + B_total * s_tf);
G_xcdot_tuned = K_a * alpha_f * eta_m / (M_c * s_tf + B_total);

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
rms_err = rms(err_cm(mask));

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
title(sprintf('Position Error (RMS = %.3f cm, excluding first 2s)', rms_err));
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
fprintf('  MODELING PIPELINE COMPLETE\n');
fprintf('============================================================\n');
fprintf('\n  Parameter          Nominal    Tuned      Change\n');
fprintf('  -----------------  ---------  ---------  ------\n');
fprintf('  B_eq [N*s/m]       %8.3f   %8.3f   %+.1f%%\n', ...
    B_eq_nominal, B_eq, (B_eq-B_eq_nominal)/B_eq_nominal*100);
fprintf('  B_total [N*s/m]    %8.3f   %8.3f\n', ...
    B_eq_nominal + B_emf, B_total);
fprintf('  eta_g [-]          %8.3f   %8.3f   (fixed)\n', eta_g, eta_g);
fprintf('\n  Validation:\n');
fprintf('    RMS position error = %.3f cm\n', rms_err);
fprintf('    Velocity pole      = %.2f Hz\n', B_total/M_c/(2*pi));

if rms_err < 1.0
    fprintf('\n  MODEL VALIDATED (RMS < 1 cm)\n');
    fprintf('    Ready for LQR controller design.\n');
else
    fprintf('\n  WARNING: RMS error > 1 cm -- consider:\n');
    fprintf('    - Reducing chirp amplitude (currently %.1f V)\n', A_chirp);
    fprintf('    - Checking for end-stop clipping\n');
    fprintf('    - Manual B_eq adjustment\n');
end

% Save tuned parameters
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
save_file = fullfile(SEESAW_ROOT, 'data', 'tuned_params.mat');
save(save_file, 'B_eq', 'B_eq_nominal', 'B_total', 'alpha_f', 'B_emf', ...
     'eta_g', 'A_cart', 'B_cart', 'C_cart', 'D_cart', ...
     'A_sw', 'B_sw', 'C_sw', 'D_sw', 'rms_err');
fprintf('\n  Tuned parameters saved to: data/tuned_params.mat\n');
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
