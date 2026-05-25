function results = analyze_single_run(valdir, figdir)
%ANALYZE_SINGLE_RUN  Phase 2 analysis for the single-run validation protocol.
%
% Expects validation/data/hw_single_run.mat containing:
%   hw_t, hw_seg_id, hw_xc, hw_alpha, hw_u_ctrl, hw_u_presat, hw_vm,
%   hw_d, hw_xc_dot_raw, hw_alpha_dot_raw, hw_x_fb(4),
%   hw_x_obs_active(4), hw_x_obs_luenb(4), hw_x_obs_kalm(4)
%
% Also expects validation/data/hw_test_signals.mat containing protocol
% parameters: sine_freqs_Hz, d_amp_V, n_cycles, seg_table, exp_fs_Hz

root = fileparts(fileparts(valdir));
if ~exist('SEESAW_ROOT', 'var')
    run(fullfile(root, 'startup.m'));
end

%% Load data
run_data = load(fullfile(valdir, 'data', 'hw_single_run.mat'));
protocol = load(fullfile(valdir, 'data', 'hw_test_signals.mat'));

t       = run_data.hw_t(:);
seg_id  = run_data.hw_seg_id(:);
xc      = run_data.hw_xc(:);
alpha   = run_data.hw_alpha(:);
u_ctrl  = run_data.hw_u_ctrl(:);
u_presat = run_data.hw_u_presat(:);
vm      = run_data.hw_vm(:);
d_hw    = run_data.hw_d(:);

% Observer states (if available)
has_obs_luenb = isfield(run_data, 'hw_x_obs_luenb');
has_obs_kalm  = isfield(run_data, 'hw_x_obs_kalm');
has_obs_active = isfield(run_data, 'hw_x_obs_active');

dt = median(diff(t));
Fs = 1/dt;
sine_freqs_Hz = protocol.sine_freqs_Hz;
d_amp_V = protocol.d_amp_V;
n_cycles = protocol.n_cycles;

fprintf('\n========================================\n');
fprintf(' Single-Run Validation Analysis\n');
fprintf('========================================\n');
fprintf('  Duration: %.1f s  |  Fs: %.0f Hz  |  N: %d\n', t(end)-t(1), Fs, length(t));
fprintf('  Segments detected: %s\n', mat2str(unique(seg_id)'));

results = struct();

%% =====================================================================
%  SECTION A — BASELINE FREE-RUN ENVELOPE (Segment 1)
%  =====================================================================
fprintf('\n--- Section A: Baseline Free-Run Envelope (Segment 1) ---\n');

mask_bl = seg_id == 1;
if sum(mask_bl) < 100
    fprintf('  WARNING: Baseline segment too short or missing.\n');
else
    alpha_bl = alpha(mask_bl);
    xc_bl = xc(mask_bl);
    vm_bl = vm(mask_bl);
    
    % Limit-cycle envelope metrics
    alpha_rms_bl    = rms(alpha_bl);
    alpha_p95_bl    = prctile(abs(alpha_bl), 95);
    alpha_pp_bl     = max(alpha_bl) - min(alpha_bl);
    xc_rms_bl       = rms(xc_bl);
    vm_rms_bl       = rms(vm_bl);
    vm_max_bl       = max(abs(vm_bl));
    
    fprintf('  Angle RMS:   %.4f rad (%.2f deg)\n', alpha_rms_bl, rad2deg(alpha_rms_bl));
    fprintf('  Angle 95%%:   %.4f rad (%.2f deg)\n', alpha_p95_bl, rad2deg(alpha_p95_bl));
    fprintf('  Angle P-P:   %.4f rad (%.2f deg)\n', alpha_pp_bl, rad2deg(alpha_pp_bl));
    fprintf('  Cart RMS:    %.3f cm\n', xc_rms_bl*100);
    fprintf('  Voltage RMS: %.3f V  |  Peak: %.3f V\n', vm_rms_bl, vm_max_bl);
    
    results.baseline.alpha_rms_rad = alpha_rms_bl;
    results.baseline.alpha_p95_rad = alpha_p95_bl;
    results.baseline.alpha_pp_rad  = alpha_pp_bl;
    results.baseline.xc_rms_m      = xc_rms_bl;
    results.baseline.vm_rms_V      = vm_rms_bl;
    results.baseline.vm_max_V      = vm_max_bl;
end

%% =====================================================================
%  SECTION B — TIME-DOMAIN: DISTURBANCE STEP VALIDATION (Segments 2, 4)
%  =====================================================================
fprintf('\n--- Section B: Disturbance Step Validation ---\n');

% Load tuned linear model for comparison
tuned = load(fullfile(root, 'data', 'tuned', 'tuned_params.mat'));
ctrl  = load(fullfile(root, 'data', 'controllers', 'controller_freq.mat'));
B_eq = tuned.B_eq;
K_fb = ctrl.Kf;
V_sat = 6.0 * sqrt(3);  % Motor saturation limit [V] (V_nom * sqrt(3))

% Rebuild closed-loop model for step comparison
[sys_d2alpha, sys_d2xc] = rebuild_cl_model( ...
    tuned.A_sw, tuned.B_sw, tuned.C_sw, tuned.D_sw, K_fb);

step_segments = [2, 4];
step_amps = [d_amp_V, -d_amp_V];
step_names = {'+1V Step', '-1V Step'};

for si = 1:length(step_segments)
    seg = step_segments(si);
    mask_step = seg_id == seg;
    mask_pre  = seg_id == (seg - 1);  % preceding recovery/baseline
    
    if sum(mask_step) < 50
        fprintf('  Segment %d (%s): too short, skipping.\n', seg, step_names{si});
        continue;
    end
    
    t_step = t(mask_step) - t(find(mask_step, 1));
    alpha_step = alpha(mask_step);
    xc_step = xc(mask_step);
    vm_step = vm(mask_step);
    u_presat_step = u_presat(mask_step);
    
    % Baseline subtraction
    if sum(mask_pre) > 10
        alpha_pre_mean = mean(alpha(mask_pre));
        xc_pre_mean = mean(xc(mask_pre));
    else
        alpha_pre_mean = 0;
        xc_pre_mean = 0;
    end
    
    alpha_delta = alpha_step - alpha_pre_mean;
    xc_delta = xc_step - xc_pre_mean;
    
    % Metrics
    alpha_peak = max(abs(alpha_delta));
    xc_peak = max(abs(xc_delta));
    vm_peak = max(abs(vm_step));
    
    % Saturation check
    sat_count = sum(abs(u_presat_step) > V_sat * 0.99);
    sat_frac = sat_count / length(u_presat_step) * 100;
    
    % Recovery: time until |alpha_delta| returns within 1.25 * baseline envelope
    if exist('alpha_p95_bl', 'var')
        recovery_threshold = 1.25 * alpha_p95_bl;
        below_threshold = abs(alpha_delta) < recovery_threshold;
        % Find first sample that stays below for 2 seconds
        stay_samples = ceil(2 / dt);
        idx_recovery = NaN;
        for ri = 1:(length(below_threshold) - stay_samples)
            if all(below_threshold(ri:ri+stay_samples-1))
                idx_recovery = ri;
                break;
            end
        end
        if ~isnan(idx_recovery)
            t_recovery = t_step(idx_recovery);
        else
            t_recovery = NaN;
        end
    else
        t_recovery = NaN;
    end
    
    fprintf('\n  %s (Segment %d):\n', step_names{si}, seg);
    fprintf('    Peak |alpha|:     %.3f deg\n', rad2deg(alpha_peak));
    fprintf('    Peak |x_c|:       %.2f cm\n', xc_peak*100);
    fprintf('    Peak |V_m|:       %.2f V\n', vm_peak);
    fprintf('    Saturation:       %.1f%% of samples\n', sat_frac);
    if ~isnan(t_recovery)
        fprintf('    Recovery time:    %.2f s (to 1.25x baseline envelope)\n', t_recovery);
    else
        fprintf('    Recovery time:    did not fully recover\n');
    end
    
    % Store results
    field = sprintf('step_%s', regexprep(step_names{si}, '[^a-zA-Z0-9]', ''));
    results.(field).alpha_peak_deg = rad2deg(alpha_peak);
    results.(field).xc_peak_cm = xc_peak * 100;
    results.(field).vm_peak_V = vm_peak;
    results.(field).sat_frac_pct = sat_frac;
    results.(field).t_recovery_s = t_recovery;
end

%% =====================================================================
%  SECTION C — TIME-DOMAIN: PULSE (IMPULSE SURROGATE) (Segment 6)
%  =====================================================================
fprintf('\n--- Section C: Pulse (Impulse Surrogate) Validation ---\n');

mask_pulse = seg_id == 6;
mask_pre_pulse = seg_id == 5;  % recovery before pulse

if sum(mask_pulse) < 5
    fprintf('  Pulse segment too short or missing.\n');
else
    % Include some of the following recovery for analysis
    mask_after = seg_id == 7;
    t_pulse_full = t([find(mask_pulse); find(mask_after)]);
    alpha_pulse_full = alpha([find(mask_pulse); find(mask_after)]);
    xc_pulse_full = xc([find(mask_pulse); find(mask_after)]);
    
    t_pulse_full = t_pulse_full - t_pulse_full(1);
    
    if sum(mask_pre_pulse) > 10
        alpha_pre = mean(alpha(mask_pre_pulse));
        xc_pre = mean(xc(mask_pre_pulse));
    else
        alpha_pre = 0; xc_pre = 0;
    end
    
    alpha_pulse_delta = alpha_pulse_full - alpha_pre;
    xc_pulse_delta = xc_pulse_full - xc_pre;
    
    fprintf('  Pulse peak |alpha|: %.3f deg\n', rad2deg(max(abs(alpha_pulse_delta))));
    fprintf('  Pulse peak |x_c|:   %.2f cm\n', max(abs(xc_pulse_delta))*100);
    
    results.pulse.alpha_peak_deg = rad2deg(max(abs(alpha_pulse_delta)));
    results.pulse.xc_peak_cm = max(abs(xc_pulse_delta)) * 100;
end

%% =====================================================================
%  SECTION D — FREQUENCY-DOMAIN: STEPPED SINE BODE (Segments 10-21)
%  =====================================================================
fprintf('\n--- Section D: Stepped Sine Frequency Validation ---\n');

n_freqs = length(sine_freqs_Hz);
bode_results = struct('freq_Hz', [], 'gain_alpha_dB', [], 'phase_alpha_deg', [], ...
    'gain_xc_dB', [], 'phase_xc_deg', [], 'valid', [], ...
    'sat_frac', [], 'fit_residual', []);

for k = 1:n_freqs
    f_k = sine_freqs_Hz(k);
    seg_k = 10 + (k-1);
    mask_k = seg_id == seg_k;
    
    if sum(mask_k) < 20
        fprintf('  Freq %.2f Hz (Seg %d): too short, skipping.\n', f_k, seg_k);
        bode_results.freq_Hz(k) = f_k;
        bode_results.valid(k) = false;
        continue;
    end
    
    t_k = t(mask_k) - t(find(mask_k, 1));
    d_k = d_hw(mask_k);
    alpha_k = alpha(mask_k);
    xc_k = xc(mask_k);
    u_presat_k = u_presat(mask_k);
    
    % Mean-subtract for AC analysis
    d_k = d_k - mean(d_k);
    alpha_k = alpha_k - mean(alpha_k);
    xc_k = xc_k - mean(xc_k);
    
    % Identify analysis window: cycles 4-6 of 7
    % Cycle boundaries based on frequency
    T_cycle = 1/f_k;
    t_analyze_start = 3 * T_cycle;  % after cycles 1-3
    t_analyze_end   = 6 * T_cycle;  % end of cycle 6
    analyze_mask = t_k >= t_analyze_start & t_k < t_analyze_end;
    
    if sum(analyze_mask) < 10
        bode_results.freq_Hz(k) = f_k;
        bode_results.valid(k) = false;
        continue;
    end
    
    t_an = t_k(analyze_mask);
    d_an = d_k(analyze_mask);
    alpha_an = alpha_k(analyze_mask);
    xc_an = xc_k(analyze_mask);
    u_presat_an = u_presat_k(analyze_mask);
    
    % Saturation check in analysis window
    sat_count_k = sum(abs(u_presat_an) > V_sat * 0.99);
    sat_frac_k = sat_count_k / length(u_presat_an) * 100;
    
    % Sine fitting: fit A*sin(2*pi*f*t + phi) to each signal
    % Using least-squares: y = a*cos(wt) + b*sin(wt) => A = sqrt(a^2+b^2), phi = atan2(a,b)
    w_k = 2*pi*f_k;
    cos_basis = cos(w_k * t_an);
    sin_basis = sin(w_k * t_an);
    X_basis = [cos_basis, sin_basis];
    
    % Fit input d
    coeff_d = X_basis \ d_an;
    A_d = sqrt(coeff_d(1)^2 + coeff_d(2)^2);
    phi_d = atan2(coeff_d(1), coeff_d(2));
    
    % Fit output alpha
    coeff_alpha = X_basis \ alpha_an;
    A_alpha = sqrt(coeff_alpha(1)^2 + coeff_alpha(2)^2);
    phi_alpha = atan2(coeff_alpha(1), coeff_alpha(2));
    
    % Fit output xc
    coeff_xc = X_basis \ xc_an;
    A_xc = sqrt(coeff_xc(1)^2 + coeff_xc(2)^2);
    phi_xc = atan2(coeff_xc(1), coeff_xc(2));
    
    % Gain and phase
    gain_alpha = A_alpha / A_d;
    gain_xc = A_xc / A_d;
    phase_alpha = phi_alpha - phi_d;
    phase_xc = phi_xc - phi_d;
    
    % Wrap phase to [-pi, pi]
    phase_alpha = mod(phase_alpha + pi, 2*pi) - pi;
    phase_xc = mod(phase_xc + pi, 2*pi) - pi;
    
    % Fit residual (quality metric)
    alpha_fit = coeff_alpha(1)*cos_basis + coeff_alpha(2)*sin_basis;
    residual_alpha = rms(alpha_an - alpha_fit) / A_alpha;
    
    % Check if response is above baseline floor
    if exist('alpha_p95_bl', 'var')
        above_floor = A_alpha > 0.5 * alpha_p95_bl;
    else
        above_floor = true;
    end
    
    % Validity: no saturation AND meaningful response
    valid_k = (sat_frac_k == 0) && above_floor;
    
    % Store
    bode_results.freq_Hz(k) = f_k;
    bode_results.gain_alpha_dB(k) = 20*log10(gain_alpha);
    bode_results.phase_alpha_deg(k) = rad2deg(phase_alpha);
    bode_results.gain_xc_dB(k) = 20*log10(gain_xc);
    bode_results.phase_xc_deg(k) = rad2deg(phase_xc);
    bode_results.valid(k) = valid_k;
    bode_results.sat_frac(k) = sat_frac_k;
    bode_results.fit_residual(k) = residual_alpha;
    
    status_str = '';
    if sat_frac_k > 0, status_str = ' [SATURATED]'; end
    if ~above_floor, status_str = [status_str ' [BELOW FLOOR]']; end
    if valid_k, status_str = ' OK'; end
    
    fprintf('  %.2f Hz: |G_alpha|=%.1f dB, phase=%.1f deg, residual=%.2f%s\n', ...
        f_k, 20*log10(gain_alpha), rad2deg(phase_alpha), residual_alpha, status_str);
end

results.bode = bode_results;

%% =====================================================================
%  SECTION E — OBSERVER VALIDATION (all segments)
%  =====================================================================
fprintf('\n--- Section E: Observer Validation ---\n');

% Compare all observer estimates against measured encoder signals
% Use all non-prep segments for comparison
mask_valid = seg_id >= 1;

if has_obs_luenb
    obs_luenb = [run_data.hw_x_obs_luenb];
    xc_hat_l = obs_luenb(mask_valid, 1);
    alpha_hat_l = obs_luenb(mask_valid, 3);
    
    e_xc_l = xc_hat_l - xc(mask_valid);
    e_alpha_l = alpha_hat_l - alpha(mask_valid);
    
    fprintf('  Luenberger:\n');
    fprintf('    x_c RMS err:   %.4f cm\n', rms(e_xc_l)*100);
    fprintf('    alpha RMS err: %.4f deg\n', rad2deg(rms(e_alpha_l)));
    
    results.obs_luenb.e_xc_rms_cm = rms(e_xc_l)*100;
    results.obs_luenb.e_alpha_rms_deg = rad2deg(rms(e_alpha_l));
end

if has_obs_kalm
    obs_kalm = [run_data.hw_x_obs_kalm];
    xc_hat_k = obs_kalm(mask_valid, 1);
    alpha_hat_k = obs_kalm(mask_valid, 3);
    
    e_xc_k = xc_hat_k - xc(mask_valid);
    e_alpha_k = alpha_hat_k - alpha(mask_valid);
    
    fprintf('  Kalman:\n');
    fprintf('    x_c RMS err:   %.4f cm\n', rms(e_xc_k)*100);
    fprintf('    alpha RMS err: %.4f deg\n', rad2deg(rms(e_alpha_k)));
    
    results.obs_kalm.e_xc_rms_cm = rms(e_xc_k)*100;
    results.obs_kalm.e_alpha_rms_deg = rad2deg(rms(e_alpha_k));
end

%% =====================================================================
%  SECTION F — FIGURES
%  =====================================================================
fprintf('\n--- Section F: Generating Figures ---\n');

% F1. Full-run overview
figure('Name', 'Single-Run Overview', 'Position', [50 50 1200 800]);

subplot(5,1,1);
plot(t, d_hw, 'Color', [0.5 0 0.5], 'LineWidth', 0.8); grid on;
ylabel('d [V]'); title('Validation Excitation (disturbance injection)');

subplot(5,1,2);
plot(t, rad2deg(alpha), 'r-', 'LineWidth', 0.8); grid on;
ylabel('\alpha [deg]'); title('Seesaw Angle');

subplot(5,1,3);
plot(t, xc*100, 'b-', 'LineWidth', 0.8); grid on;
ylabel('x_c [cm]'); title('Cart Position');

subplot(5,1,4);
plot(t, vm, 'Color', [0 0.6 0], 'LineWidth', 0.8); grid on; hold on;
yline(V_sat, 'r--'); yline(-V_sat, 'r--');
ylabel('V_m [V]'); title('Motor Voltage (post-saturation)');

subplot(5,1,5);
plot(t, seg_id, 'k-', 'LineWidth', 1.0); grid on;
ylabel('Seg ID'); xlabel('Time [s]'); title('Segment ID');

sgtitle('Hardware Validation — Single-Run Protocol Overview', 'FontWeight', 'bold');
saveas(gcf, fullfile(figdir, 'Validation-SingleRun-Overview.png'));
fprintf('  Saved: Validation-SingleRun-Overview.png\n');

% F2. Step comparison (model vs hardware)
figure('Name', 'Step Validation', 'Position', [100 100 1000 600]);
for si = 1:2
    seg = step_segments(si);
    mask_s = seg_id == seg;
    mask_pre_s = seg_id == (seg - 1);
    if sum(mask_s) < 50, continue; end
    
    t_s = t(mask_s) - t(find(mask_s, 1));
    alpha_s = alpha(mask_s) - mean(alpha(mask_pre_s));
    
    subplot(2,2,si);
    plot(t_s, rad2deg(alpha_s), 'r-', 'LineWidth', 1.2); hold on;
    % Model overlay: linear step response
    [y_mdl_alpha, t_mdl] = step(step_amps(si) * sys_d2alpha, t_s(end));
    plot(t_mdl, rad2deg(y_mdl_alpha), 'k--', 'LineWidth', 1.2);
    legend('Hardware', 'Linear model', 'Location', 'best');
    grid on; ylabel('\alpha [deg]');
    title(sprintf('%s: angle response', step_names{si}));
    
    subplot(2,2,si+2);
    xc_s = xc(mask_s) - mean(xc(mask_pre_s));
    plot(t_s, xc_s*100, 'b-', 'LineWidth', 1.2); hold on;
    [y_mdl_xc, ~] = step(step_amps(si) * sys_d2xc, t_s(end));
    plot(t_mdl, y_mdl_xc*100, 'k--', 'LineWidth', 1.2);
    legend('Hardware', 'Linear model', 'Location', 'best');
    grid on; ylabel('x_c [cm]'); xlabel('Time [s]');
    title(sprintf('%s: cart response', step_names{si}));
end
sgtitle('Time Validation — Disturbance Step Response', 'FontWeight', 'bold');
saveas(gcf, fullfile(figdir, 'Validation-StepResponse.png'));
fprintf('  Saved: Validation-StepResponse.png\n');

% F3. Bode plot (measured points vs model)
valid_idx = find(bode_results.valid);
if ~isempty(valid_idx)
    figure('Name', 'Bode Validation', 'Position', [150 150 1000 700]);
    
    % Compute model Bode curve over frequency range
    f_bode = logspace(log10(0.05), log10(20), 200);
    w_bode = 2*pi*f_bode;
    [mag_alpha_mdl, phase_alpha_mdl] = bode(sys_d2alpha, w_bode);
    [mag_xc_mdl, phase_xc_mdl] = bode(sys_d2xc, w_bode);
    mag_alpha_mdl = squeeze(mag_alpha_mdl);
    phase_alpha_mdl = squeeze(phase_alpha_mdl);
    mag_xc_mdl = squeeze(mag_xc_mdl);
    phase_xc_mdl = squeeze(phase_xc_mdl);
    
    subplot(2,2,1);
    semilogx(f_bode, 20*log10(mag_alpha_mdl), 'k-', 'LineWidth', 1.0); hold on;
    semilogx(bode_results.freq_Hz(valid_idx), bode_results.gain_alpha_dB(valid_idx), ...
        'ro', 'MarkerSize', 8, 'LineWidth', 2); grid on;
    legend('Linear model', 'Hardware', 'Location', 'best');
    ylabel('|G_\alpha| [dB rad/V]'); title('d \rightarrow \alpha: Magnitude');
    
    subplot(2,2,3);
    semilogx(f_bode, phase_alpha_mdl, 'k-', 'LineWidth', 1.0); hold on;
    semilogx(bode_results.freq_Hz(valid_idx), bode_results.phase_alpha_deg(valid_idx), ...
        'ro', 'MarkerSize', 8, 'LineWidth', 2); grid on;
    legend('Linear model', 'Hardware', 'Location', 'best');
    ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
    title('d \rightarrow \alpha: Phase');
    
    subplot(2,2,2);
    semilogx(f_bode, 20*log10(mag_xc_mdl), 'k-', 'LineWidth', 1.0); hold on;
    semilogx(bode_results.freq_Hz(valid_idx), bode_results.gain_xc_dB(valid_idx), ...
        'bs', 'MarkerSize', 8, 'LineWidth', 2); grid on;
    legend('Linear model', 'Hardware', 'Location', 'best');
    ylabel('|G_{xc}| [dB m/V]'); title('d \rightarrow x_c: Magnitude');
    
    subplot(2,2,4);
    semilogx(f_bode, phase_xc_mdl, 'k-', 'LineWidth', 1.0); hold on;
    semilogx(bode_results.freq_Hz(valid_idx), bode_results.phase_xc_deg(valid_idx), ...
        'bs', 'MarkerSize', 8, 'LineWidth', 2); grid on;
    legend('Linear model', 'Hardware', 'Location', 'best');
    ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
    title('d \rightarrow x_c: Phase');
    
    sgtitle('Frequency Validation — Stepped Sine Bode', 'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Validation-BodeStepped.png'));
    fprintf('  Saved: Validation-BodeStepped.png\n');
end

% F4. Data validity summary
figure('Name', 'Validity Summary', 'Position', [200 200 800 400]);
subplot(1,2,1);
bar(bode_results.freq_Hz, bode_results.sat_frac, 'FaceColor', [0.8 0.2 0.2]);
set(gca, 'XScale', 'log'); grid on;
xlabel('Frequency [Hz]'); ylabel('Saturation [%]');
title('Per-Frequency Saturation');

subplot(1,2,2);
bar(bode_results.freq_Hz, bode_results.fit_residual, 'FaceColor', [0.2 0.2 0.8]);
set(gca, 'XScale', 'log'); grid on;
xlabel('Frequency [Hz]'); ylabel('Fit Residual (norm)');
title('Sine Fit Quality');

sgtitle('Data Validity Diagnostics', 'FontWeight', 'bold');
saveas(gcf, fullfile(figdir, 'Validation-DataValidity.png'));
fprintf('  Saved: Validation-DataValidity.png\n');

%% Save results
results.protocol = protocol;
results.total_duration_s = t(end) - t(1);
results.Fs = Fs;

save(fullfile(valdir, 'data', 'validation_results_single_run.mat'), 'results');
fprintf('\n  Saved: validation/data/validation_results_single_run.mat\n');

fprintf('\n========================================\n');
fprintf(' SINGLE-RUN VALIDATION COMPLETE\n');
fprintf('========================================\n\n');

end

%% --- Nested helper: rebuild closed-loop model ---
function [sys_d2alpha, sys_d2xc] = rebuild_cl_model(A_sw, B_sw, C_sw, D_sw, K_fb)
    % Build closed-loop TF from disturbance voltage d to outputs.
    %
    % Signal chain: u_total = u_controller + d = -K_fb*x + d
    % Closed-loop: x_dot = (A - B*K)*x + B*d,  y = C*x
    %
    % Outputs:
    %   sys_d2alpha — SS model from d [V] to alpha [rad]
    %   sys_d2xc    — SS model from d [V] to x_c [m]
    
    A_cl = A_sw - B_sw * K_fb;
    B_cl = B_sw;  % disturbance enters at actuator input
    C_cl = C_sw;  % outputs: [x_c; alpha]
    D_cl = D_sw;  % zeros(2,1)
    
    sys_cl = ss(A_cl, B_cl, C_cl, D_cl);
    sys_d2xc    = sys_cl(1,1);  % first output = x_c
    sys_d2alpha = sys_cl(2,1);  % second output = alpha
end
