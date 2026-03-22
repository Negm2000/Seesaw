%% frequency_analysis.m
%  -----------------------------------------------------------------------
%  STEP 2: Data Processing & Auto-Tuning
%  -----------------------------------------------------------------------
%  Run this script AFTER collecting hardware data from IP02_FreqTest.slx.
%  -----------------------------------------------------------------------

if ~exist('K_a', 'var'), seesaw_params; end

% --- RE-CALCULATE ANALYTICAL MODEL ---
alpha_f = (eta_g * K_g * k_t) / (R_m * r_mp);
B_emf = alpha_f * K_g * k_m / r_mp;
B_total = B_eq + B_emf;
s = tf('s');
G_xc = K_a * alpha_f * eta_m / (M_c * s^2 + B_total * s);
freq_range = logspace(-1, 1.5, 200);   
[mag, phase] = bode(G_xc, 2*pi*freq_range);
G_xc_dB = 20*log10(squeeze(mag)*100);
G_xc_phase = squeeze(phase);

% --- LOAD HARDWARE DATA ---
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(fileparts(fileparts(mfilename('fullpath')))); end
data_file = fullfile(SEESAW_ROOT, 'data', 'data.mat');

if exist(data_file, 'file')
    fprintf('Loading hardware data (data/data.mat)...\n');
    load(data_file); 
    % ... check for variable names
    vars = who('-file', data_file);
    if ismember('ip02_freq_data', vars)
        % Using the data directly
    elseif ismember('data', vars)
        ip02_freq_data = data;
    end
    
    if exist('ip02_freq_data', 'var')
        t_hw = ip02_freq_data(1, :)';
        V_cmd_hw = ip02_freq_data(2, :)';
        xc_hw = ip02_freq_data(3, :)'; % m
        xcdot_hw = ip02_freq_data(4, :)'; % m/s
        dt_hw = mean(diff(t_hw));
        [hw_freq, hw_xc_H, ~] = compute_frf(t_hw, V_cmd_hw, xc_hw, xcdot_hw, dt_hw);
    else
        error('Data file found but required variable not found inside.');
    end
else
    error('Hardware data file (data/data.mat) not found. Run the SLX first.');
end

% --- PLOT COMPARISON (UNTUNED) ---
figure('Name', 'Validation: Model vs Hardware', 'Position', [100 100 1000 600]);
subplot(2,1,1);
semilogx(freq_range, G_xc_dB, 'b-', 'LineWidth', 1.5); hold on;
semilogx(hw_freq, 20*log10(abs(hw_xc_H)*100), 'r-', 'LineWidth', 1.5);
grid on; ylabel('Mag [dB]'); title('V_cmd -> x_c [cm/V]');
legend('Analytical (seesaw\_params)', 'Hardware Data');
subplot(2,1,2);
semilogx(freq_range, G_xc_phase, 'b-', 'LineWidth', 1.5); hold on;
semilogx(hw_freq, angle(hw_xc_H)*180/pi, 'r-', 'LineWidth', 1.5);
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
sgtitle('Frequency Response Validation (UNTUNED)');

% --- SECTION 6: AUTO-TUNE ---
fprintf('\n--- Running Auto-Tune (fminsearch) ---\n');
p_tune = struct('K_a',K_a, 'V_sat',V_sat, 'R_m',R_m, 'k_t',k_t, 'k_m',k_m, 'eta_m',eta_m, 'eta_g',eta_g, 'K_g',K_g, 'r_mp',r_mp, 'M_c',M_c);
cost_fn = @(p) tune_cost(p, V_cmd_hw, t_hw, xc_hw, t_hw > 2.0, p_tune, false);

[x_opt, cost_opt] = fminsearch(cost_fn, B_eq);
B_eq_opt = x_opt(1);
eta_g_opt = eta_g;  % fixed at hardware spec

fprintf('\nOPTIMIZED VALUES:\n');
fprintf('  B_eq  = %.4f (was %.2f)\n', B_eq_opt, B_eq);
fprintf('  eta_g = %.4f (fixed at hardware spec)\n', eta_g_opt);

% --- PLOT COMPARISON (AFTER TUNING) ---
% Re-calculate model with optimized parameters
af_opt = (eta_g_opt * K_g * k_t) / (R_m * r_mp);
B_tot_opt = B_eq_opt + af_opt * K_g * k_m / r_mp;
G_xc_opt = K_a * af_opt * eta_m / (M_c * s^2 + B_tot_opt * s);
[mag_opt, phase_opt] = bode(G_xc_opt, 2*pi*freq_range);
G_xc_dB_opt = 20*log10(squeeze(mag_opt)*100);
G_xc_phase_opt = squeeze(phase_opt);

figure('Name', 'Validation: Tuned Model vs Hardware', 'Position', [150 120 1000 600]);
subplot(2,1,1);
semilogx(freq_range, G_xc_dB, 'b--', 'LineWidth', 1); hold on;
semilogx(freq_range, G_xc_dB_opt, 'g-', 'LineWidth', 1.5);
semilogx(hw_freq, 20*log10(abs(hw_xc_H)*100), 'r-', 'LineWidth', 1.5);
grid on; ylabel('Mag [dB]'); title('V_cmd -> x_c [cm/V]');
legend('Initial Model', 'TUNED Model', 'Hardware Data', 'Location', 'best');
subplot(2,1,2);
semilogx(freq_range, G_xc_phase, 'b--', 'LineWidth', 1); hold on;
semilogx(freq_range, G_xc_phase_opt, 'g-', 'LineWidth', 1.5);
semilogx(hw_freq, angle(hw_xc_H)*180/pi, 'r-', 'LineWidth', 1.5);
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
sgtitle('Frequency Response Validation (AFTER AUTO-TUNE)');

% --- LOCAL FUNCTIONS ---
function [freq_out, H_xc, H_xcdot] = compute_frf(t, u, xc, xcdot, dt)
    N = length(t); Fs = 1/dt;
    n_seg = 2048; win = hanning(n_seg);
    n_step = round(n_seg * 0.5); n_segs = floor((N - n_seg) / n_step);
    freq_fft = (0:n_seg/2) * Fs / n_seg;
    Suu = zeros(n_seg/2+1, 1); Syu_xc = zeros(n_seg/2+1, 1); Syu_xcdot = zeros(n_seg/2+1, 1);
    for k = 0:n_segs-1
        idx = k*n_step + (1:n_seg);
        u_seg = (u(idx) - mean(u(idx))) .* win;
        xc_seg = (xc(idx) - mean(xc(idx))) .* win;
        xcdot_seg = (xcdot(idx) - mean(xcdot(idx))) .* win;
        U = fft(u_seg); Xc = fft(xc_seg); Xcdot = fft(xcdot_seg);
        U_h = U(1:n_seg/2+1);
        Suu = Suu + abs(U_h).^2;
        Syu_xc = Syu_xc + Xc(1:n_seg/2+1) .* conj(U_h);
        Syu_xcdot = Syu_xcdot + Xcdot(1:n_seg/2+1) .* conj(U_h);
    end
    valid = freq_fft >= 0.1 & freq_fft <= 25;
    freq_out = freq_fft(valid)'; H_xc = Syu_xc(valid) ./ Suu(valid); H_xcdot = Syu_xcdot(valid) ./ Suu(valid);
end

function cost = tune_cost(params, V_cmd, t, xc_hw, idx, p, tune_eta)
    B_try = params(1); eta_try = ternary(tune_eta, params(2), p.eta_g);
    af = (eta_try * p.K_g * p.k_t) / (p.R_m * p.r_mp);
    B_tot = B_try + af * p.K_g * p.k_m / p.r_mp;
    sys = ss([0, 1; 0, -B_tot/p.M_c], [0; af*p.eta_m/p.M_c], [1, 0], 0);
    xc_sim = lsim(sys, max(-p.V_sat, min(p.V_sat, p.K_a*V_cmd)), t);
    cost = sqrt(mean((xc_sim(idx) - xc_hw(idx)).^2));
end

function val = ternary(cond, y, n)
    if cond, val = y; else, val = n; end
end
