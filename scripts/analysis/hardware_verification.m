%% HARDWARE VERIFICATION PIPELINE — Closed-Loop Seesaw Control
%  =====================================================================
%  Master script for verifying the closed-loop seesaw controller against
%  hardware data.  Covers both time-domain (step response, bounded
%  oscillation analysis) and frequency-domain (closed-loop Bode from
%  chirp perturbation) verification.
%
%  KEY INSIGHT:  The seesaw does NOT achieve asymptotic stability at
%  theta = 0.  Nonlinearities (Coulomb friction, encoder quantization,
%  backlash, static friction) produce a bounded limit cycle.  The
%  controller constrains rocking between ±theta_bound — the better the
%  controller, the tighter the bound.  This script quantifies that bound
%  rather than chasing an unachievable zero steady-state error.
%
%  =====================================================================
%  HARDWARE DATA COLLECTION (before running this script):
%  =====================================================================
%
%  Model:   models/PolePlacementObserver2024.slx
%  Control: u = -K*x (regulator — drives x_c to zero by default)
%
%  For Experiments B & C you need a one-time model modification:
%    Replace u = -K*xhat with u = -K*(xhat - [r; 0; 0; 0]).
%    Add a "From Workspace" block for r and a manual switch so you
%    can toggle between r=0 (normal operation) and an injected signal
%    (step or chirp).  Log r alongside everything else.
%
%  Experiment A — Free-Run (unperturbed rocking, DD-in-loop):
%    1. Open models/PolePlacementObserver2024.slx (QUARC External)
%    2. Set r=0 (no reference injection)
%    3. Hold seesaw level, Start controller, release gently
%    4. Let run for 30-60 s, logging: t, x_c, alpha, V_m
%    5. Save as: data/hw_free_run.mat
%       Variables: hw_t, hw_xc, hw_alpha, hw_vm  (each Nx1)
%
%  Experiment B — Step Response (cart step, observer-in-loop):
%    1. Switch to observer-in-loop variant of model
%    2. Inject a step via r: 0 -> 5 cm at t=2 s
%    3. Hold seesaw level, Start controller, release gently
%    4. Let run 20 s, logging: t, r, x_c, alpha, V_m
%    5. Save as: data/hw_step_response.mat
%       Variables: hw_t, hw_r, hw_xc, hw_alpha, hw_vm
%
%  Experiment C — Chirp Perturbation (frequency response):
%    1. Same model as B. Replace step with chirp:
%       - Amplitude: 2-3 cm (too large may destabilize)
%       - Frequency: 0.1 to 10 Hz (linear chirp)
%       - Duration: 60 s
%    2. Log: t, r, x_c, alpha, V_m
%    3. Save as: data/hw_chirp_response.mat
%       Variables: hw_t, hw_r, hw_xc, hw_alpha, hw_vm
%
%  Experiment D — Observer free-run (observer-in-loop, for Section F):
%    1. Observer-in-loop variant, r=0
%    2. Log observer outputs alongside measurements:
%         xc_hat, xcdot_hat, alpha_hat, alphadot_hat
%    3. Free-run 30-60 s, save as: data/hw_obs_free.mat
%       Variables: hw_t, hw_xc, hw_alpha, hw_vm,
%                  hw_xc_hat, hw_xcdot_hat, hw_alpha_hat, hw_alphadot_hat
%
%  Run individual sections (Ctrl+Enter) as data is available.
%  =====================================================================
%
%  Requires:  startup.m  (or at minimum  seesaw_params.m and path setup)
%  Outputs:   docs/figures/Verification-*.png
%             data/verification_results.mat
%  =====================================================================

%% 0. SETUP — Load parameters and paths
close all; clc;

root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if ~exist('SEESAW_ROOT', 'var') || ~strcmp(SEESAW_ROOT, root)
    run(fullfile(root, 'startup.m'));
end

figdir = fullfile(root, 'docs', 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

tuned  = load(fullfile(root, 'data', 'tuned_params.mat'));
ctrl   = load(fullfile(root, 'data', 'controller_freq.mat'));

B_eq  = tuned.B_eq;
K_fb  = ctrl.Kf;
p_cl  = ctrl.p_final;

fprintf('\n========================================\n');
fprintf(' Hardware Verification Pipeline\n');
fprintf('========================================\n');
fprintf('  B_eq          = %.4f N*s/m\n', B_eq);
fprintf('  K_fb          = [%.3f, %.3f, %.3f, %.3f]\n', K_fb);
fprintf('  Dominant pole = %.2f rad/s\n', ctrl.sigma_th);

% Rebuild seesaw plant for eigenvalue reference
M_c_use = M_c + ctrl.M_c_added;
B_total = B_eq + B_emf;
M_eff = [M_c_use, -M_c_use*D_T; -M_c_use*D_T, J_pivot + M_c_use*D_T^2];
M_inv = inv(M_eff);
G_rhs = [0, -B_total, -g*M_c_use, 0; -g*M_c_use, 0, g*(M_c_use*D_T + M_SW*D_C), -B_SW];
A_sw_plant = [0 1 0 0; M_inv(1,:)*G_rhs; 0 0 0 1; M_inv(2,:)*G_rhs];
B_sw_plant = [0; M_inv(1,:)*[alpha_f*eta_m; 0]; 0; M_inv(2,:)*[alpha_f*eta_m; 0]];
ev = eig(A_sw_plant);
p_unstable_hint = max(real(ev));
fprintf('  Plant RHP pole = +%.3f rad/s\n', p_unstable_hint);
fprintf('\n');

% ---- Scanners for what data files exist ----
has_free_run  = exist(fullfile(root, 'data', 'hw_free_run.mat'), 'file') == 2;
has_step      = exist(fullfile(root, 'data', 'hw_step_response.mat'), 'file') == 2;
has_chirp     = exist(fullfile(root, 'data', 'hw_chirp_response.mat'), 'file') == 2;
has_obs       = exist(fullfile(root, 'data', 'hw_obs_free.mat'), 'file') == 2;

if ~has_free_run && ~has_step && ~has_chirp && ~has_obs
    fprintf('No hardware verification data found in data/.\n');
    fprintf('Run at least one experiment (see header comments) then re-run.\n');
    return;
end

fprintf('Data available:  free-run=%d  step=%d  chirp=%d  obs=%d\n', ...
    has_free_run, has_step, has_chirp, has_obs);


%% =====================================================================
%  SECTION A — TIME-DOMAIN: FREE-RUN BOUNDED OSCILLATION ANALYSIS
%  =====================================================================
%  The seesaw never truly stabilizes at zero.  Coulomb friction in the
%  pivot and encoder quantization create a dead-zone where small angle
%  errors produce no corrective torque.  The system rocks back and forth
%  across this dead-zone at a frequency set by the closed-loop dynamics.
%
%  The key metric is the oscillation envelope:  tighter is better.
%  We quantify: RMS angle, peak-to-peak, 95th-percentile bound,
%  oscillation frequency, and voltage utilization.
%  =====================================================================

if ~has_free_run
    fprintf('\n*** SECTION A SKIPPED — hw_free_run.mat not found ***\n');
else
    fprintf('\n========================================\n');
    fprintf(' SECTION A: Free-Run Bounded Oscillation\n');
    fprintf('========================================\n');

    d = load(fullfile(root, 'data', 'hw_free_run.mat'));

    % ---- Convert to time-series friendly arrays ----
    t     = d.hw_t(:);
    xc    = d.hw_xc(:);
    alpha = d.hw_alpha(:);
    vm    = d.hw_vm(:);

    dt = mean(diff(t));
    Fs = 1/dt;
    fprintf('  Duration: %.1f s  |  Fs: %.0f Hz  |  N: %d\n', t(end), Fs, length(t));

    % ---- Trim startup transient (first 3 s) ----
    trim = t > 3.0;
    t_t     = t(trim) - t(find(trim, 1, 'first'));
    xc_t    = xc(trim);
    alpha_t = alpha(trim);
    vm_t    = vm(trim);

    %% A1. Bounded oscillation statistics
    % These replace "settling time" and "steady-state error" which assume
    % asymptotic convergence.  The system is bounded, not convergent.

    % Angle envelope metrics
    alpha_rms    = rms(alpha_t);
    alpha_std    = std(alpha_t);
    alpha_pp     = max(alpha_t) - min(alpha_t);
    alpha_p95    = prctile(abs(alpha_t), 95);
    alpha_p99    = prctile(abs(alpha_t), 99);
    alpha_mean   = mean(alpha_t);
    alpha_max_abs = max(abs(alpha_t));

    % Cart envelope metrics
    xc_rms   = rms(xc_t);
    xc_std   = std(xc_t);
    xc_pp    = max(xc_t) - min(xc_t);
    xc_p95   = prctile(abs(xc_t), 95);

    % Voltage utilization
    vm_rms   = rms(vm_t);
    vm_max   = max(abs(vm_t));
    vm_frac  = vm_max / V_sat * 100;

    fprintf('\n  --- Angle Envelope (bounded, not asymptotic) ---\n');
    fprintf('    RMS:        %.4f rad  (%.2f deg)\n', alpha_rms, rad2deg(alpha_rms));
    fprintf('    Std:        %.4f rad  (%.2f deg)\n', alpha_std, rad2deg(alpha_std));
    fprintf('    Peak-peak:  %.4f rad  (%.2f deg)\n', alpha_pp, rad2deg(alpha_pp));
    fprintf('    95%% bound:  %.4f rad  (%.2f deg)\n', alpha_p95, rad2deg(alpha_p95));
    fprintf('    99%% bound:  %.4f rad  (%.2f deg)\n', alpha_p99, rad2deg(alpha_p99));
    fprintf('    Max |angle|:%.4f rad  (%.2f deg)\n', alpha_max_abs, rad2deg(alpha_max_abs));
    fprintf('    Mean bias:  %+.4f rad (%+.2f deg)  <-- should be near 0\n', ...
        alpha_mean, rad2deg(alpha_mean));

    % Compare against physical stops (±11.5 deg) and RHP pole timescale
    fprintf('\n    Physical stop:   ±%.1f deg\n', rad2deg(alpha_max));
    fprintf('    RHP pole:        +%.2f rad/s  (%.1f ms time constant)\n', ...
        p_unstable_hint, 1000/p_unstable_hint);
    fprintf('    Safety margin:   %.1f deg  (%.0f%%%% of physical limit)\n', ...
        rad2deg(alpha_max) - rad2deg(alpha_max_abs), ...
        (1 - alpha_max_abs/alpha_max)*100);

    fprintf('\n  --- Cart Envelope ---\n');
    fprintf('    RMS:        %.3f cm\n', xc_rms*100);
    fprintf('    Peak-peak:  %.3f cm\n', xc_pp*100);
    fprintf('    95%% bound:  %.3f cm\n', xc_p95*100);

    fprintf('\n  --- Voltage Utilization ---\n');
    fprintf('    RMS:        %.3f V\n', vm_rms);
    fprintf('    Peak:       %.3f V  (%.0f%%%% of %.1f V saturation)\n', ...
        vm_max, vm_frac, V_sat);
    if vm_frac > 90
        fprintf('    ⚠ SATURATED — controller is hitting voltage rails!\n');
        fprintf('      Reduce sigma_th or increase zeta_th.\n');
    end

    %% A2. Oscillation frequency from zero-crossing
    % The rocking occurs at a well-defined frequency set by the closed-loop
    % dominant pole pair and the dead-zone width.
    alpha_demean = alpha_t - mean(alpha_t);
    zc = find(diff(sign(alpha_demean)) ~= 0);
    if length(zc) >= 4
        % Half-periods between successive zero crossings
        half_periods = diff(t_t(zc));
        osc_period   = mean(half_periods) * 2;
        osc_freq     = 1 / osc_period;
        osc_freq_std = std(1./(half_periods*2));
        fprintf('\n  --- Oscillation (Zero-Crossing) ---\n');
        fprintf('    Crossings:      %d\n', length(zc));
        fprintf('    Osc frequency:  %.3f Hz  (%.3f rad/s)\n', osc_freq, 2*pi*osc_freq);
        fprintf('    Osc period:     %.3f s\n', osc_period);
        fprintf('    Freq std:       %.3f Hz\n', osc_freq_std);
    end

    %% A3. Spectral analysis of the oscillation
    % FFT to identify the dominant rocking frequency and harmonics
    % (harmonics = signature of dead-zone nonlinearity).
    n_fft     = 2^nextpow2(length(alpha_demean));
    win       = hanning(length(alpha_demean));
    alpha_fft = fft(alpha_demean .* win, n_fft);
    alpha_psd = abs(alpha_fft(1:n_fft/2+1)).^2 / (Fs * sum(win.^2));
    f_psd     = Fs * (0:n_fft/2)' / n_fft;

    % Find dominant peak (excluding DC)
    f_range = f_psd >= 0.2 & f_psd <= 15;
    [~, idx_dom] = max(alpha_psd(f_range));
    f_dom_psd = f_psd(find(f_range, 1, 'first') + idx_dom - 1);
    psd_ratio = alpha_psd(f_range);
    psd_ratio = max(psd_ratio) / mean(psd_ratio);

    fprintf('\n  --- Spectral Analysis ---\n');
    fprintf('    Dominant freq:  %.3f Hz  (from PSD peak)\n', f_dom_psd);
    fprintf('    PSD peak/mean:  %.1f  (>10 = clean oscillation)\n', psd_ratio);

    %% A4. Bound quality: fraction of time within tight/medium/wide bounds
    tight_pct  = mean(abs(alpha_t) < rad2deg(0.5)  * pi/180) * 100;
    medium_pct = mean(abs(alpha_t) < rad2deg(2.0)  * pi/180) * 100;
    wide_pct   = mean(abs(alpha_t) < rad2deg(5.0)  * pi/180) * 100;

    fprintf('\n  --- Bound Quality (time within envelope) ---\n');
    fprintf('    |α| < 0.5 deg:  %.1f%%%%\n', tight_pct);
    fprintf('    |α| < 2.0 deg:  %.1f%%%%\n', medium_pct);
    fprintf('    |α| < 5.0 deg:  %.1f%%%%\n', wide_pct);

    bound_score = tight_pct * 0.5 + medium_pct * 0.3 + wide_pct * 0.2;
    fprintf('    Bound score:    %.1f/100  (weighted, higher=better)\n', bound_score);

    %% A5. Figures — Free-Run
    figure('Name', 'Verification: Free-Run Bounded Oscillation', ...
        'Position', [50 50 1100 750]);

    subplot(4,1,1);
    plot(t, xc*100, 'b-', 'LineWidth', 1.0); grid on; hold on;
    yline( xc_p95*100, 'k--', '95% bound');
    yline(-xc_p95*100, 'k--');
    ylabel('x_c [cm]');
    title(sprintf('Free-Run: Bounded Oscillation (RMS angle = %.2f deg, 95%% bound = %.2f deg)', ...
        rad2deg(alpha_rms), rad2deg(alpha_p95)));

    subplot(4,1,2);
    plot(t, rad2deg(alpha), 'r-', 'LineWidth', 1.0); grid on; hold on;
    yline( rad2deg(alpha_p95), 'k--');
    yline(-rad2deg(alpha_p95), 'k--');
    yline( rad2deg(alpha_max), 'm:', 'Physical stop');
    yline(-rad2deg(alpha_max), 'm:');
    ylabel('\alpha [deg]');

    subplot(4,1,3);
    plot(t, vm, 'Color', [0 0.6 0], 'LineWidth', 0.8); grid on; hold on;
    yline( V_sat, 'r--');
    yline(-V_sat, 'r--');
    ylabel('V_m [V]');
    title(sprintf('Voltage: peak=%.2f V (%.0f%%%% of ±%.1f V)', vm_max, vm_frac, V_sat));

    subplot(4,1,4);
    semilogy(f_psd, alpha_psd, 'r-', 'LineWidth', 1.2); grid on;
    xlim([0.01 20]); ylabel('PSD [rad^2/Hz]'); xlabel('Frequency [Hz]');
    hold on; xline(f_dom_psd, 'k--', sprintf('%.2f Hz', f_dom_psd));
    if exist('osc_freq', 'var'), xline(osc_freq, 'b--', sprintf('ZC: %.2f Hz', osc_freq)); end
    title(sprintf('Angle PSD (dominant rocking at %.3f Hz)', f_dom_psd));

    sgtitle('Hardware Verification — Free-Run (Unperturbed)', 'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Verification-FreeRun.png'));
    fprintf('  Saved: docs/figures/Verification-FreeRun.png\n');

    %% A6. Bound-score histogram
    figure('Name', 'Verification: Bound Quality', ...
        'Position', [100 100 800 400]);
    h = histogram(rad2deg(alpha_t), 'BinWidth', 0.25, 'FaceColor', 'r', ...
        'EdgeColor', 'none', 'Normalization', 'count');
    hold on; grid on;
    xline(0, 'k-', 'LineWidth', 1.5);
    xline( rad2deg(alpha_rms), 'k--', sprintf('±RMS = ±%.2f°', rad2deg(alpha_rms)));
    xline(-rad2deg(alpha_rms), 'k--');
    xline( rad2deg(alpha_p95), 'm--', sprintf('±95%% = ±%.2f°', rad2deg(alpha_p95)));
    xline(-rad2deg(alpha_p95), 'm--');
    xlabel('\alpha [deg]'); ylabel('Count');
    title(sprintf('Angle Distribution — %.1f%%%% within ±0.5°, %.1f%%%% within ±2°', ...
        tight_pct, medium_pct));
    saveas(gcf, fullfile(figdir, 'Verification-BoundQuality.png'));
    fprintf('  Saved: docs/figures/Verification-BoundQuality.png\n');
end


%% =====================================================================
%  SECTION B — TIME-DOMAIN: STEP RESPONSE
%  =====================================================================
%  A cart position step tests tracking performance.  The step size must
%  be small (3-5 cm) — larger steps may destabilize the seesaw.
%
%  Metrics: rise time, overshoot of x_c, peak alpha excursion during the
%  transient, and oscillation envelope before/after the step.
%  =====================================================================

if ~has_step
    fprintf('\n*** SECTION B SKIPPED — hw_step_response.mat not found ***\n');
else
    fprintf('\n========================================\n');
    fprintf(' SECTION B: Step Response\n');
    fprintf('========================================\n');

    d = load(fullfile(root, 'data', 'hw_step_response.mat'));

    t       = d.hw_t(:);
    xc_ref  = d.hw_r(:);
    xc      = d.hw_xc(:);
    alpha   = d.hw_alpha(:);
    vm      = d.hw_vm(:);

    dt = mean(diff(t));
    fprintf('  Duration: %.1f s  |  Fs: %.0f Hz\n', t(end), 1/dt);

    %% B1. Detect step time from reference
    % Find when x_c_ref changes (rise above noise floor)
    dref = diff(xc_ref);
    [~, step_idx] = max(abs(dref));
    t_step = t(step_idx);
    step_amp = xc_ref(end) - xc_ref(1);

    fprintf('\n  Step detected at t = %.2f s, amplitude = %.2f cm\n', ...
        t_step, step_amp * 100);

    %% B2. Pre-step and post-step bounded oscillation
    pre_mask  = t >= 1.0 & t < t_step - 0.2;
    post_mask = t > t_step + 1.0;

    alpha_pre_pp  = max(alpha(pre_mask))  - min(alpha(pre_mask));
    alpha_post_pp = max(alpha(post_mask)) - min(alpha(post_mask));
    alpha_pre_rms  = rms(alpha(pre_mask));
    alpha_post_rms = rms(alpha(post_mask));

    fprintf('\n  --- Oscillation Envelope Change ---\n');
    fprintf('    Pre-step  P-P:  %.2f deg  |  RMS: %.2f deg\n', ...
        rad2deg(alpha_pre_pp), rad2deg(alpha_pre_rms));
    fprintf('    Post-step P-P:  %.2f deg  |  RMS: %.2f deg\n', ...
        rad2deg(alpha_post_pp), rad2deg(alpha_post_rms));
    fprintf('    Change:         P-P %+.2f deg,  RMS %+.2f deg\n', ...
        rad2deg(alpha_post_pp - alpha_pre_pp), ...
        rad2deg(alpha_post_rms - alpha_pre_rms));

    %% B3. Step response on x_c: rise time, overshoot, steady-state bound
    % Trim to step window ± 5 s
    step_window = t >= t_step - 0.5 & t <= t_step + 5.0;
    tw = t(step_window) - t_step;
    xcw = xc(step_window) - xc_ref(step_window);

    % Rise time (10%-90% of step magnitude, referenced to pre-step x_c)
    xc_pre = mean(xc(pre_mask));
    [~, i10] = min(abs(xc(step_window) - xc_pre - 0.10*step_amp));
    [~, i90] = min(abs(xc(step_window) - xc_pre - 0.90*step_amp));
    t_rise = tw(i90) - tw(i10);

    % Peak overshoot (cart)
    xc_post_mean = mean(xc(post_mask));
    xc_err = xc - xc_ref;
    xc_overshoot = (max(xc(post_mask)) - xc_ref(post_mask(end))) / step_amp * 100;

    % Post-step cart bound
    xc_p95_post = prctile(abs(xc(post_mask) - mean(xc(post_mask))), 95);

    % Peak angle excursion during transient
    alpha_peak_transient = max(abs(alpha(step_window))) * 180/pi;

    fprintf('\n  --- Step Response Metrics ---\n');
    fprintf('    Rise time (10-90%%):  %.3f s\n', t_rise);
    fprintf('    Overshoot (cart):    %.1f%%%%\n', xc_overshoot);
    fprintf('    Peak |alpha| during: %.2f deg\n', alpha_peak_transient);
    fprintf('    Post-step x_c bound: ±%.2f cm\n', xc_p95_post*100);

    %% B4. Figures — Step Response
    figure('Name', 'Verification: Step Response', ...
        'Position', [50 50 1100 750]);

    subplot(4,1,1);
    yyaxis left;
    plot(t, xc*100, 'b-', 'LineWidth', 1.2); hold on;
    plot(t, xc_ref*100, 'k--', 'LineWidth', 1.5);
    ylabel('x_c [cm]');
    yyaxis right;
    plot(t, (xc - xc_ref)*100, 'r-', 'LineWidth', 0.8);
    ylabel('Error [cm]');
    xline(t_step, 'g--', 'Step');
    grid on; title(sprintf('Step Response: %.0f cm step', step_amp*100));
    legend('x_c', 'x_c^{ref}', 'Error', 'Location', 'best');

    subplot(4,1,2);
    plot(t, rad2deg(alpha), 'r-', 'LineWidth', 1.2); grid on; hold on;
    yline(0, 'k-');
    yline( rad2deg(alpha_max), 'm:', 'Physical stop');
    yline(-rad2deg(alpha_max), 'm:');
    xline(t_step, 'g--');
    ylabel('\alpha [deg]');
    title(sprintf('Angle: pre-step PP=%.2f°, post-step PP=%.2f°', ...
        rad2deg(alpha_pre_pp), rad2deg(alpha_post_pp)));

    subplot(4,1,3);
    plot(t, vm, 'Color', [0 0.6 0], 'LineWidth', 0.8); grid on; hold on;
    yline( V_sat, 'r--');
    yline(-V_sat, 'r--');
    xline(t_step, 'g--');
    ylabel('V_m [V]');
    title(sprintf('Control Voltage (peak = %.2f V)', max(abs(vm))));

    subplot(4,1,4);
    h_pre = histogram(rad2deg(alpha(pre_mask)), 'BinWidth', 0.25, ...
        'FaceColor', 'b', 'EdgeColor', 'none', 'Normalization', 'pdf');
    hold on;
    h_post = histogram(rad2deg(alpha(post_mask)), 'BinWidth', 0.25, ...
        'FaceColor', 'r', 'EdgeColor', 'none', 'Normalization', 'pdf');
    grid on;
    xlabel('\alpha [deg]'); ylabel('PDF');
    legend('Pre-step', 'Post-step');
    title('Angle Distribution: Before vs After Step');

    sgtitle('Hardware Verification — Step Response', 'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Verification-StepResponse.png'));
    fprintf('  Saved: docs/figures/Verification-StepResponse.png\n');

    %% B5. Zoomed step window
    figure('Name', 'Verification: Step Zoom', ...
        'Position', [100 100 1000 500]);
    subplot(2,1,1);
    plot(tw, xc(step_window)*100, 'b-', 'LineWidth', 1.5); hold on;
    plot(tw, xc_ref(step_window)*100, 'k--', 'LineWidth', 1.5);
    grid on; xlim([-0.2 2.0]);
    ylabel('x_c [cm]');
    title(sprintf('Cart Step Zoom (t_{rise} = %.3f s, overshoot = %.1f%%)', ...
        t_rise, xc_overshoot));
    legend('x_c', 'x_c^{ref}');

    subplot(2,1,2);
    plot(tw, rad2deg(alpha(step_window)), 'r-', 'LineWidth', 1.2); hold on;
    grid on; xlim([-0.2 2.0]);
    yline(0, 'k-');
    ylabel('\alpha [deg]'); xlabel('Time from step [s]');
    title(sprintf('Angle Transient (peak = %.2f deg)', alpha_peak_transient));

    saveas(gcf, fullfile(figdir, 'Verification-StepZoom.png'));
    fprintf('  Saved: docs/figures/Verification-StepZoom.png\n');
end


%% =====================================================================
%  SECTION C — FREQUENCY-DOMAIN: CLOSED-LOOP BODE FROM CHIRP
%  =====================================================================
%  While the controller is running, a small-amplitude linear chirp is
%  added to x_c_ref.  From the logged data we estimate the closed-loop
%  transfer functions:
%
%    T(s)  = X_c(s) / X_c_ref(s)    (complementary sensitivity)
%    G_vm(s) = V_m(s) / X_c_ref(s)  (control sensitivity — maps reference to effort)
%    G_alpha(s) = α(s) / X_c_ref(s) (reference-to-angle coupling)
%
%  These reveal bandwidth, peaking, and disturbance rejection.
%
%  H1 estimator (Welch's method) via tfestimate is used for robust
%  cross-spectral estimation in the presence of the unmeasured rocking
%  disturbance (which acts as uncorrelated output noise).
%  =====================================================================

if ~has_chirp
    fprintf('\n*** SECTION C SKIPPED — hw_chirp_response.mat not found ***\n');
else
    fprintf('\n========================================\n');
    fprintf(' SECTION C: Closed-Loop Frequency Response\n');
    fprintf('========================================\n');

    d = load(fullfile(root, 'data', 'hw_chirp_response.mat'));

    t       = d.hw_t(:);
    xc_ref  = d.hw_r(:);
    xc      = d.hw_xc(:);
    alpha   = d.hw_alpha(:);
    vm      = d.hw_vm(:);

    dt = mean(diff(t));
    Fs = 1/dt;
    fprintf('  Duration: %.1f s  |  Fs: %.0f Hz  |  N: %d\n', t(end), Fs, length(t));

    %% C1. Estimate closed-loop transfer functions via Welch's method
    % Window segment: target ~8-16 segments across duration.
    % 4096 samples/segment at 500 Hz = ~8 s/segment → ~7 segments for 60 s
    % Good balance of frequency resolution and variance reduction.
    n_seg = min(4096, 2^nextpow2(length(t)/10));
    n_seg = max(n_seg, 512);
    n_overlap = n_seg / 2;

    fprintf('\n  Welch parameters: n_seg=%d, overlap=%d, segments≈%d\n', ...
        n_seg, n_overlap, floor(length(t)/(n_seg - n_overlap)));

    % H1 estimator: T_xy = P_xy / P_xx
    % Best when input (x_c_ref) is noise-free and noise is on output
    [H_xc, f_tfe] = tfestimate(xc_ref, xc,    hanning(n_seg), n_overlap, n_seg, Fs);
    [H_alpha, ~]  = tfestimate(xc_ref, alpha, hanning(n_seg), n_overlap, n_seg, Fs);
    [H_vm, ~]     = tfestimate(xc_ref, vm,    hanning(n_seg), n_overlap, n_seg, Fs);

    % Coherence — how much of the output variance is explained by the input
    [C_xc, ~]    = mscohere(xc_ref, xc,    hanning(n_seg), n_overlap, n_seg, Fs);
    [C_alpha, ~] = mscohere(xc_ref, alpha, hanning(n_seg), n_overlap, n_seg, Fs);

    % Constrain to chirp frequency range
    f_lo = 0.1; f_hi = 10.0;
    f_mask = f_tfe >= f_lo & f_tfe <= f_hi;
    f_use = f_tfe(f_mask);

    %% C2. Closed-loop bandwidth (cart)
    % Bandwidth = frequency where |T(s)| first crosses -3 dB from DC
    mag_xc_db = 20*log10(abs(H_xc(f_mask)));
    dc_gain_db = mag_xc_db(find(f_use >= f_lo, 1, 'first'));
    % Find first crossing below -3 dB from DC
    idx_bw = find(mag_xc_db < dc_gain_db - 3, 1, 'first');
    if isempty(idx_bw)
        bw_hz = f_use(end);
        bw_warning = ' (not found — beyond chirp range)';
    else
        bw_hz = f_use(idx_bw);
        bw_warning = '';
    end

    % Peak magnitude (resonance) and peaking frequency
    [mag_peak_db, idx_peak] = max(mag_xc_db);
    f_peak = f_use(idx_peak);

    % Phase margin from complementary sensitivity
    % Approximate: phase margin ≈ phase(T) at crossover where |T|=1 (0 dB)
    phase_xc_deg = unwrap(angle(H_xc(f_mask))) * 180/pi;
    idx_cross = find(mag_xc_db >= 0, 1, 'last');
    if ~isempty(idx_cross) && idx_cross < length(f_use)
        pm_from_t = 180 + phase_xc_deg(idx_cross);
    else
        pm_from_t = NaN;
    end

    fprintf('\n  --- Closed-Loop Frequency Metrics ---\n');
    fprintf('    Bandwidth (-3 dB):  %.2f Hz%s\n', bw_hz, bw_warning);
    fprintf('    Peak |T(s)|:        %.1f dB  at %.2f Hz\n', mag_peak_db, f_peak);
    fprintf('    DC gain (cart):     %.1f dB\n', dc_gain_db);
    if ~isnan(pm_from_t)
        fprintf('    Phase at 0 dB:      %.1f deg  (~phase margin from T)\n', pm_from_t);
    end

    % Coherence quality
    coh_xc_avg = mean(C_xc(f_mask));
    coh_alpha_avg = mean(C_alpha(f_mask));
    fprintf('\n  --- Coherence ---\n');
    fprintf('    Mean coh (x_c):     %.2f  (>0.7 = good FRF)\n', coh_xc_avg);
    fprintf('    Mean coh (alpha):   %.2f  (lower = rocking dominates)\n', coh_alpha_avg);
    if coh_alpha_avg < 0.5
        fprintf('    Note: alpha coherence is low because the unmeasured\n');
        fprintf('          rocking disturbance is uncorrelated with x_c_ref.\n');
        fprintf('          This is expected — the chirp is NOT driving the oscillation.\n');
    end

    %% C3. Figures — Closed-Loop Bode
    figure('Name', 'Verification: Closed-Loop Bode', ...
        'Position', [50 50 1100 800]);

    % --- Tile 1: Complementary sensitivity T(s) = x_c / x_c_ref ---
    subplot(3,2,1);
    semilogx(f_use, mag_xc_db(f_mask), 'b-', 'LineWidth', 1.5); grid on; hold on;
    yline(dc_gain_db - 3, 'r--', sprintf('-3 dB → %.1f Hz', bw_hz));
    xlim([f_lo f_hi]);
    ylabel('Magnitude [dB]');
    title(sprintf('T(s) = X_c / X_c^{ref}  (BW = %.2f Hz)', bw_hz));

    subplot(3,2,2);
    semilogx(f_use, phase_xc_deg(f_mask), 'b-', 'LineWidth', 1.5); grid on;
    xlim([f_lo f_hi]);
    ylabel('Phase [deg]');
    title('Phase T(s)');

    % --- Tile 2: Control sensitivity V_m / x_c_ref ---
    subplot(3,2,3);
    mag_vm_db = 20*log10(abs(H_vm(f_mask)));
    semilogx(f_use, mag_vm_db, 'Color', [0 0.6 0], 'LineWidth', 1.5); grid on;
    xlim([f_lo f_hi]);
    ylabel('Magnitude [dB V/m]');
    title('Control Effort: V_m / X_c^{ref}');

    subplot(3,2,4);
    phase_vm_deg = unwrap(angle(H_vm(f_mask))) * 180/pi;
    semilogx(f_use, phase_vm_deg, 'Color', [0 0.6 0], 'LineWidth', 1.5); grid on;
    xlim([f_lo f_hi]);
    ylabel('Phase [deg]');
    title('Phase V_m / X_c^{ref}');

    % --- Tile 3: Reference-to-angle coupling (disturbance cross-talk) ---
    subplot(3,2,5);
    mag_alpha_db = 20*log10(abs(H_alpha(f_mask)));
    semilogx(f_use, mag_alpha_db, 'r-', 'LineWidth', 1.5); grid on;
    xlim([f_lo f_hi]);
    ylabel('Magnitude [dB rad/m]');
    xlabel('Frequency [Hz]');
    title('Coupling: \alpha / X_c^{ref}');

    subplot(3,2,6);
    semilogx(f_use, C_xc(f_mask), 'b-', 'LineWidth', 1.2); hold on;
    semilogx(f_use, C_alpha(f_mask), 'r-', 'LineWidth', 1.2); grid on;
    yline(0.7, 'k--');
    xlim([f_lo f_hi]); ylim([0 1]);
    ylabel('Coherence');
    xlabel('Frequency [Hz]');
    legend('x_c', '\alpha', 'Location', 'best');
    title('Coherence (H1 estimator quality)');

    sgtitle('Hardware Verification — Closed-Loop Frequency Response', ...
        'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Verification-CLBode.png'));
    fprintf('  Saved: docs/figures/Verification-CLBode.png\n');

    %% C4. Overlay with analytical prediction
    % Rebuild the closed-loop transfer function from the controller
    % and plant, for comparison with hardware.
    M_c_added = ctrl.M_c_added;
    if ~exist('M_c_added', 'var'), M_c_added = 0; end
    M_c_use = M_c + M_c_added;
    B_total = B_eq + B_emf;
    M_eff = [M_c_use, -M_c_use*D_T; -M_c_use*D_T, J_pivot + M_c_use*D_T^2];
    M_inv = inv(M_eff);
    G_rhs = [0, -B_total, -g*M_c_use, 0; -g*M_c_use, 0, g*(M_c_use*D_T + M_SW*D_C), -B_SW];
    A_sw = [0 1 0 0; M_inv(1,:)*G_rhs; 0 0 0 1; M_inv(2,:)*G_rhs];
    B_sw = [0; M_inv(1,:)*[alpha_f*eta_m; 0]; 0; M_inv(2,:)*[alpha_f*eta_m; 0]];

    % Closed-loop A, B, C for reference tracking
    A_cl = A_sw - B_sw * K_fb;
    % x_c output = C_xc * x = [1 0 0 0] * x
    C_xc = [1 0 0 0];
    % Reference enters as a feedforward: x_dot = A_cl*x + B_sw*K_fb(1)*r
    B_ref = B_sw * K_fb(1);

    T_model = tf(ss(A_cl, B_ref, C_xc, 0));

    % Compare model vs hardware magnitude
    [mag_model, phase_model] = bode(T_model, 2*pi*f_use);
    mag_model_db = 20*log10(squeeze(mag_model));
    phase_model_deg = squeeze(phase_model);

    figure('Name', 'Verification: CL Bode Model vs Hardware', ...
        'Position', [100 100 1000 600]);
    subplot(2,1,1);
    semilogx(f_use, mag_xc_db(f_mask), 'b-', 'LineWidth', 1.5); hold on;
    semilogx(f_use, mag_model_db, 'k--', 'LineWidth', 2);
    grid on; xlim([f_lo f_hi]);
    ylabel('Magnitude [dB]');
    title('T(s) = X_c / X_c^{ref}:  Model vs Hardware');
    legend('Hardware (H1)', 'Model (linear)', 'Location', 'best');

    subplot(2,1,2);
    semilogx(f_use, phase_xc_deg(f_mask), 'b-', 'LineWidth', 1.5); hold on;
    semilogx(f_use, phase_model_deg, 'k--', 'LineWidth', 2);
    grid on; xlim([f_lo f_hi]);
    ylabel('Phase [deg]'); xlabel('Frequency [Hz]');

    % Compute RMS magnitude error in band
    mag_err_rms = rms(mag_xc_db(f_mask) - mag_model_db(:));
    fprintf('\n  --- Model-Hardware Agreement ---\n');
    fprintf('    RMS magnitude error:  %.2f dB  (in chirp band)\n', mag_err_rms);

    saveas(gcf, fullfile(figdir, 'Verification-CLBode-ModelVsHW.png'));
    fprintf('  Saved: docs/figures/Verification-CLBode-ModelVsHW.png\n');
end


%% =====================================================================
%  SECTION D — COMPOSITE VERIFICATION DASHBOARD
%  =====================================================================
%  Single-page summary combining time and frequency metrics.
%  Suitable for reports and quick pass/fail assessment.
%  =====================================================================

figure('Name', 'Verification: Composite Dashboard', ...
    'Position', [30 30 1200 850]);

% --- Tile 1: Angle time trace + bound markers ---
if has_free_run
    subplot(3,3,1);
    d = load(fullfile(root, 'data', 'hw_free_run.mat'));
    t_t = d.hw_t(:); t_t = t_t(t_t > 3) - 3;
    a_t = rad2deg(d.hw_alpha(d.hw_t > 3));
    plot(t_t, a_t, 'r-', 'LineWidth', 0.8); grid on; hold on;
    yline(0, 'k-');
    a_rms = rms(a_t);
    a_pp = max(a_t) - min(a_t);
    yline( a_rms, 'b--'); yline(-a_rms, 'b--');
    ylabel('\alpha [deg]');
    title(sprintf('Angle: RMS=%.2f°, P-P=%.2f°', a_rms, a_pp));
end

% --- Tile 2: Angle histogram ---
if has_free_run
    subplot(3,3,2);
    histogram(a_t, 'BinWidth', 0.3, 'FaceColor', 'r', 'EdgeColor', 'none');
    grid on; xline(0, 'k-', 'LineWidth', 2);
    xlabel('\alpha [deg]'); ylabel('Count');
    title(sprintf('Distribution (skew: %.2f)', skewness(a_t)));
end

% --- Tile 3: Bound quality bar ---
if has_free_run
    subplot(3,3,3);
    cats = {'<0.5°', '<2.0°', '<5.0°'};
    vals = [tight_pct, medium_pct, wide_pct];
    bar(vals, 'FaceColor', [0.2 0.6 0.2]);
    set(gca, 'XTickLabel', cats);
    ylim([0 100]); grid on;
    ylabel('% of time');
    title(sprintf('Bound Quality (score = %.1f)', bound_score));
    for i = 1:3, text(i, vals(i)+2, sprintf('%.0f%%', vals(i)), ...
            'HorizontalAlignment', 'center'); end
end

% --- Tile 4: Step response ---
if has_step
    subplot(3,3,4);
    d = load(fullfile(root, 'data', 'hw_step_response.mat'));
    t_s = d.hw_t(:); xc_s = d.hw_xc(:)*100; ref_s = d.hw_r(:)*100;
    [~, si] = max(abs(diff(ref_s)));
    t_s = t_s - d.hw_t(si);
    mask = t_s >= -1 & t_s <= 4;
    plot(t_s(mask), xc_s(mask), 'b-', 'LineWidth', 1.2); hold on;
    plot(t_s(mask), ref_s(mask), 'k--', 'LineWidth', 1.2);
    grid on; ylabel('x_c [cm]');
    title('Cart Step Response');
    legend('x_c', 'ref', 'Location', 'best');
end

% --- Tile 5: Voltage histogram ---
if has_free_run
    subplot(3,3,5);
    d = load(fullfile(root, 'data', 'hw_free_run.mat'));
    vm_t = d.hw_vm(d.hw_t > 3);
    histogram(vm_t, 'BinWidth', 0.2, 'FaceColor', [0 0.6 0], 'EdgeColor', 'none');
    grid on; xline( V_sat, 'r--'); xline(-V_sat, 'r--');
    xlabel('V_m [V]'); ylabel('Count');
    title(sprintf('Voltage (peak=%.1f V, %.0f%%%% sat)', max(abs(vm_t)), ...
        max(abs(vm_t))/V_sat*100));
end

% --- Tile 6: Oscillation PSD ---
if has_free_run
    subplot(3,3,6);
    semilogy(f_psd, alpha_psd, 'r-', 'LineWidth', 1.2); grid on;
    xlim([0.1 15]);
    ylabel('PSD [rad^2/Hz]'); xlabel('Freq [Hz]');
    title(sprintf('Angle PSD (peak at %.2f Hz)', f_dom_psd));
end

% --- Tile 7: CL Bode Magnitude ---
if has_chirp
    subplot(3,3,7);
    semilogx(f_use, mag_xc_db(f_mask), 'b-', 'LineWidth', 1.5); grid on; hold on;
    yline(dc_gain_db - 3, 'r--');
    xlim([f_lo f_hi]);
    ylabel('|T(s)| [dB]');
    xlabel('Freq [Hz]');
    title(sprintf('CL BW = %.2f Hz, peak = %.1f dB', bw_hz, mag_peak_db));
end

% --- Tile 8: Coherence ---
if has_chirp
    subplot(3,3,8);
    semilogx(f_use, C_xc(f_mask), 'b-', 'LineWidth', 1.2); hold on;
    semilogx(f_use, C_alpha(f_mask), 'r-', 'LineWidth', 1.2); grid on;
    xlim([f_lo f_hi]); ylim([0 1]);
    ylabel('Coherence'); xlabel('Freq [Hz]');
    legend('x_c', '\alpha', 'Location', 'best');
    title('H1 Coherence');
end

% --- Tile 9: Summary text ---
subplot(3,3,9); axis off;
summary_lines = {
    '═══ VERIFICATION SUMMARY ═══';
    '';
    sprintf('Controller:  σ=%.1f, ζ=%.1f', ctrl.sigma_th, ctrl.zeta_th);
    '';
    'FREE-RUN:';
    };
li = length(summary_lines);
if has_free_run
    li = li + 1; summary_lines{li} = sprintf('  Angle RMS:     %.2f deg', rad2deg(alpha_rms));
    li = li + 1; summary_lines{li} = sprintf('  Angle P-P:     %.2f deg', rad2deg(alpha_pp));
    li = li + 1; summary_lines{li} = sprintf('  95%% bound:     %.2f deg', rad2deg(alpha_p95));
    li = li + 1; summary_lines{li} = sprintf('  Bound score:   %.1f/100', bound_score);
else
    li = li + 1; summary_lines{li} = '  (no data)';
end

li = li + 1; summary_lines{li} = '';
li = li + 1; summary_lines{li} = 'STEP RESPONSE:';
if has_step
    li = li + 1; summary_lines{li} = sprintf('  Rise time:     %.3f s', t_rise);
    li = li + 1; summary_lines{li} = sprintf('  Overshoot:     %.1f%%%%', xc_overshoot);
else
    li = li + 1; summary_lines{li} = '  (no data)';
end

if has_obs
    li = li + 1; summary_lines{li} = '';
    li = li + 1; summary_lines{li} = 'OBSERVER:';
    li = li + 1; summary_lines{li} = sprintf('  RMS err xc:    %.3f cm', e_xc_rms*100);
    li = li + 1; summary_lines{li} = sprintf('  RMS err alpha: %.3f deg', rad2deg(e_alpha_rms));
    li = li + 1; summary_lines{li} = sprintf('  Conv time:     %.2f s', t_conv);
end

li = li + 1; summary_lines{li} = '';
li = li + 1; summary_lines{li} = 'FREQUENCY:';
if has_chirp
    li = li + 1; summary_lines{li} = sprintf('  CL bandwidth:  %.2f Hz', bw_hz);
    li = li + 1; summary_lines{li} = sprintf('  Model-HW err:  %.2f dB', mag_err_rms);
else
    li = li + 1; summary_lines{li} = '  (no data)';
end

li = li + 1; summary_lines{li} = '';
li = li + 1; summary_lines{li} = 'PASS CRITERIA:';
if has_free_run
    li = li + 1; summary_lines{li} = sprintf('  RMS < 2 deg?   %s', cond_str(rad2deg(alpha_rms) < 2));
    li = li + 1; summary_lines{li} = sprintf('  Peak V < 5 V?  %s', cond_str(vm_max < 5));
else
    li = li + 1; summary_lines{li} = '  (no free-run data)';
end
if has_chirp
    li = li + 1; summary_lines{li} = sprintf('  BW > 1 Hz?     %s', cond_str(bw_hz > 1));
else
    li = li + 1; summary_lines{li} = '  BW > 1 Hz?     N/A';
end

for i = 1:length(summary_lines)
    if ~isempty(summary_lines{i})
        text(0.05, 1 - i*0.045, summary_lines{i}, 'FontName', 'FixedWidth', ...
            'FontSize', 9, 'VerticalAlignment', 'top');
    end
end

sgtitle('Seesaw Hardware Verification — Composite Dashboard', ...
    'FontWeight', 'bold', 'FontSize', 14);
saveas(gcf, fullfile(figdir, 'Verification-Dashboard.png'));
fprintf('  Saved: docs/figures/Verification-Dashboard.png\n');


%% =====================================================================
%  SECTION F — OBSERVER VERIFICATION
%  =====================================================================
%  Compares observer estimates against encoder measurements and assesses
%  estimation quality: tracking error, convergence time, innovation
%  (evidence of model mismatch), and noise floor via PSD.
%
%  Observer State-Space block (from data/observer.mat):
%    Inputs:  [u; x_c_measured; theta_measured]
%    Outputs: xhat = [x_c_hat; x_c_dot_hat; theta_hat; theta_dot_hat]
%  =====================================================================

if ~has_obs
    fprintf('\n*** SECTION F SKIPPED — hw_obs_free.mat not found ***\n');
else
    fprintf('\n========================================\n');
    fprintf(' SECTION F: Observer Verification\n');
    fprintf('========================================\n');

    obs = load(fullfile(root, 'data', 'hw_obs_free.mat'));

    t       = obs.hw_t(:);
    xc      = obs.hw_xc(:);
    alpha   = obs.hw_alpha(:);
    vm      = obs.hw_vm(:);
    xc_hat  = obs.hw_xc_hat(:);
    xcd_hat = obs.hw_xcdot_hat(:);
    al_hat  = obs.hw_alpha_hat(:);
    ald_hat = obs.hw_alphadot_hat(:);

    dt = mean(diff(t));
    Fs = 1/dt;
    fprintf('  Duration: %.1f s  |  Fs: %.0f Hz  |  N: %d\n', t(end), Fs, length(t));

    %% F1. Tracking error: observer estimate vs encoder measurement
    % The observer should converge to x_c_meas and alpha_meas within its
    % time constant (≈ 20/k_obs/σ_th ~ 0.8 s for k_obs=5).
    e_xc    = xc_hat - xc;
    e_alpha = al_hat - alpha;

    e_xc_rms    = rms(e_xc);
    e_alpha_rms = rms(e_alpha);
    e_xc_max    = max(abs(e_xc));
    e_alpha_max = max(abs(e_alpha));

    % Convergence time: time until both errors drop below encoder resolution
    % (K_ec = 2.54e-4 m/count for cart, K_E_SW / K_gs for alpha)
    q_xc    = K_ec;
    q_theta = K_E_SW / K_gs;
    e_combined = abs(e_xc) / q_xc + abs(e_alpha) / q_theta;
    idx_conv = find(e_combined < 3, 1, 'first');
    if ~isempty(idx_conv)
        t_conv = t(idx_conv);
    else
        t_conv = t(end);
    end

    fprintf('\n  --- Observer Tracking Error ---\n');
    fprintf('    Cart RMS error:   %.4f cm  (encoder res = %.4f cm)\n', ...
        e_xc_rms*100, q_xc*100);
    fprintf('    Cart max |error|: %.4f cm\n', e_xc_max*100);
    fprintf('    Alpha RMS error:  %.4f deg  (encoder res = %.4f deg)\n', ...
        rad2deg(e_alpha_rms), rad2deg(q_theta));
    fprintf('    Alpha max |err|:  %.4f deg\n', rad2deg(e_alpha_max));
    fprintf('    Convergence time: %.2f s  (error < 3× encoder res)\n', t_conv);

    %% F2. Innovation analysis (y - C_meas * xhat)
    % The innovation is the difference between the measurement and the
    % observer's prediction. If the linear model is accurate and
    % friction is negligible, the innovation should be zero-mean white
    % noise. Systematic patterns = model mismatch (wrong B_eq, inertia,
    % or unmodeled friction).
    innov_xc    = xc - xc_hat;
    innov_alpha = alpha - al_hat;

    innov_xc_rms    = rms(innov_xc);
    innov_alpha_rms = rms(innov_alpha);
    innov_xc_bias   = mean(innov_xc);
    innov_alpha_bias = mean(innov_alpha);

    % Whiteness check via autocorrelation lag-1
    innov_xc_dm = innov_xc - mean(innov_xc);
    innov_alpha_dm = innov_alpha - mean(innov_alpha);
    rho_xc_lag1    = innov_xc_dm(2:end)' * innov_xc_dm(1:end-1) / (innov_xc_dm' * innov_xc_dm);
    rho_alpha_lag1 = innov_alpha_dm(2:end)' * innov_alpha_dm(1:end-1) / (innov_alpha_dm' * innov_alpha_dm);

    fprintf('\n  --- Innovation (Measurement - Prediction) ---\n');
    fprintf('    Cart innov RMS:    %.4f cm  | bias: %+.4f cm  | lag-1 ρ: %+.3f\n', ...
        innov_xc_rms*100, innov_xc_bias*100, rho_xc_lag1);
    fprintf('    Alpha innov RMS:   %.4f deg | bias: %+.4f deg | lag-1 ρ: %+.3f\n', ...
        rad2deg(innov_alpha_rms), rad2deg(innov_alpha_bias), rho_alpha_lag1);

    if abs(rho_xc_lag1) > 0.3 || abs(rho_alpha_lag1) > 0.3
        fprintf('    ⚠ High innovation autocorrelation — model mismatch suspected.\n');
        fprintf('      Possible causes: B_eq error, unmodeled Coulomb friction,\n');
        fprintf('      incorrect pivot inertia, or D_C error.\n');
    end

    %% F3. PSD of observer velocity vs numerical derivative
    % Compare the noise floor of the observer's velocity estimate with
    % a filtered numerical derivative of the encoder signal.
    n_fft = 2^nextpow2(length(t));

    % Observer velocity PSD
    win_len = min(length(t), 4096);
    win = hanning(win_len);
    n_seg_psd = max(win_len/2, 256);
    [P_xcd_obs, f_psd] = pwelch(xcd_hat, win, n_seg_psd, n_fft, Fs);
    [P_ald_obs, ~]     = pwelch(ald_hat, win, n_seg_psd, n_fft, Fs);

    % Filtered numerical derivative for comparison
    fc_diff = 30;  % Hz — same as dirty-derivative LPF cutoff
    [b_diff, a_diff] = butter(2, fc_diff/(Fs/2));
    xc_filt  = filtfilt(b_diff, a_diff, xc);
    al_filt  = filtfilt(b_diff, a_diff, alpha);
    xcd_num  = [diff(xc_filt); 0] / dt;
    ald_num  = [diff(al_filt); 0] / dt;
    xcd_num  = filtfilt(b_diff, a_diff, xcd_num);
    ald_num  = filtfilt(b_diff, a_diff, ald_num);

    [P_xcd_num, ~] = pwelch(xcd_num, win, n_seg_psd, n_fft, Fs);
    [P_ald_num, ~] = pwelch(ald_num, win, n_seg_psd, n_fft, Fs);

    % RMS velocity (power in the motion band 0.1-10 Hz)
    band_mask = f_psd >= 0.05 & f_psd <= 15;
    v_xc_obs_rms = sqrt(trapz(f_psd(band_mask), P_xcd_obs(band_mask)));
    v_xc_num_rms = sqrt(trapz(f_psd(band_mask), P_xcd_num(band_mask)));
    v_al_obs_rms = sqrt(trapz(f_psd(band_mask), P_ald_obs(band_mask)));
    v_al_num_rms = sqrt(trapz(f_psd(band_mask), P_ald_num(band_mask)));

    fprintf('\n  --- Velocity Estimate Comparison ---\n');
    fprintf('    Cart vel RMS (observer):  %.3f m/s  |  (numerical): %.3f m/s\n', ...
        v_xc_obs_rms, v_xc_num_rms);
    fprintf('    Alpha vel RMS (observer): %.3f rad/s |  (numerical): %.3f rad/s\n', ...
        v_al_obs_rms, v_al_num_rms);

    %% F4. Figures — Observer Verification
    figure('Name', 'Verification: Observer Tracking', ...
        'Position', [50 50 1100 800]);

    subplot(4,1,1);
    yyaxis left;
    plot(t, xc*100, 'b-', 'LineWidth', 1.0); hold on;
    plot(t, xc_hat*100, 'r--', 'LineWidth', 1.2);
    ylabel('x_c [cm]');
    yyaxis right;
    plot(t, e_xc*100, 'k-', 'LineWidth', 0.5);
    ylabel('Error [cm]');
    grid on; legend('Measured', 'Observer', 'Error', 'Location', 'best');
    title(sprintf('Cart Tracking (RMS err = %.3f cm)', e_xc_rms*100));

    subplot(4,1,2);
    yyaxis left;
    plot(t, rad2deg(alpha), 'b-', 'LineWidth', 1.0); hold on;
    plot(t, rad2deg(al_hat), 'r--', 'LineWidth', 1.2);
    ylabel('\alpha [deg]');
    yyaxis right;
    plot(t, rad2deg(e_alpha), 'k-', 'LineWidth', 0.5);
    ylabel('Error [deg]');
    grid on; legend('Measured', 'Observer', 'Error', 'Location', 'best');
    title(sprintf('Alpha Tracking (RMS err = %.3f deg)', rad2deg(e_alpha_rms)));

    subplot(4,1,3);
    plot(t, e_xc*100, 'b-', 'LineWidth', 0.6); hold on;
    plot(t, rad2deg(e_alpha), 'r-', 'LineWidth', 0.6); grid on;
    yline(0, 'k-');
    ylabel('Error');
    legend('Cart [cm]', 'Alpha [deg]', 'Location', 'best');
    title(sprintf('Innovation: convergence in %.2f s', t_conv));
    xline(t_conv, 'g--', sprintf('%.1f s', t_conv));

    subplot(4,1,4);
    semilogy(f_psd, P_xcd_obs, 'b-', 'LineWidth', 1.2); hold on;
    semilogy(f_psd, P_xcd_num, 'k--', 'LineWidth', 1.0);
    semilogy(f_psd, P_ald_obs, 'r-', 'LineWidth', 1.2);
    semilogy(f_psd, P_ald_num, 'm--', 'LineWidth', 1.0);
    grid on; xlim([0.05 30]);
    ylabel('PSD'); xlabel('Frequency [Hz]');
    legend('x_c-dot (obs)', 'x_c-dot (num)', '\alpha-dot (obs)', '\alpha-dot (num)');
    title('Velocity Estimate PSD: Observer vs Numerical Derivative');

    sgtitle('Hardware Verification — Observer Performance', 'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Verification-Observer.png'));
    fprintf('  Saved: docs/figures/Verification-Observer.png\n');

    %% F5. Track each observer state
    figure('Name', 'Verification: Observer Full State', ...
        'Position', [100 100 1100 700]);

    subplot(2,2,1);
    plot(t, xc*100, 'b-', 'LineWidth', 1.0); hold on;
    plot(t, xc_hat*100, 'r--', 'LineWidth', 1.2); grid on;
    ylabel('x_c [cm]'); legend('Enc', 'Obs');
    title('Cart Position');

    subplot(2,2,2);
    plot(t, xcd_hat, 'r-', 'LineWidth', 1.2); grid on;
    ylabel('x_c-dot [m/s]');
    title('Cart Velocity (observer only)');

    subplot(2,2,3);
    plot(t, rad2deg(alpha), 'b-', 'LineWidth', 1.0); hold on;
    plot(t, rad2deg(al_hat), 'r--', 'LineWidth', 1.2); grid on;
    ylabel('\alpha [deg]'); xlabel('Time [s]');
    legend('Enc', 'Obs');
    title('Seesaw Angle');

    subplot(2,2,4);
    plot(t, rad2deg(ald_hat), 'r-', 'LineWidth', 1.2); grid on;
    ylabel('\alpha-dot [deg/s]'); xlabel('Time [s]');
    title('Angular Velocity (observer only)');

    sgtitle('Full Observer State Estimation', 'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Verification-ObserverState.png'));
    fprintf('  Saved: docs/figures/Verification-ObserverState.png\n');

    %% F6. Observer PSD detail (cart velocity noise floor comparison)
    figure('Name', 'Verification: Observer Velocity PSD Detail', ...
        'Position', [100 100 1000 500]);

    subplot(1,2,1);
    loglog(f_psd, P_xcd_obs, 'b-', 'LineWidth', 1.5); hold on;
    loglog(f_psd, P_xcd_num, 'k--', 'LineWidth', 1.0); grid on;
    xlim([0.05 30]);
    xlabel('Frequency [Hz]'); ylabel('PSD [(m/s)^2/Hz]');
    legend('Observer', 'Numerical diff', 'Location', 'best');
    title(sprintf('Cart Velocity PSD (obs RMS = %.3f m/s)', v_xc_obs_rms));

    subplot(1,2,2);
    loglog(f_psd, P_ald_obs, 'r-', 'LineWidth', 1.5); hold on;
    loglog(f_psd, P_ald_num, 'm--', 'LineWidth', 1.0); grid on;
    xlim([0.05 30]);
    xlabel('Frequency [Hz]'); ylabel('PSD [(rad/s)^2/Hz]');
    legend('Observer', 'Numerical diff', 'Location', 'best');
    title(sprintf('Angular Velocity PSD (obs RMS = %.3f rad/s)', v_al_obs_rms));

    sgtitle('Velocity PSD: Observer vs Filtered Numerical Derivative', ...
        'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Verification-ObserverPSD.png'));
    fprintf('  Saved: docs/figures/Verification-ObserverPSD.png\n');
end


%% =====================================================================
%  SECTION E — SAVE RESULTS & PRINT REPORT
%  =====================================================================
fprintf('\n========================================\n');
fprintf(' Saving verification results...\n');
fprintf('========================================\n');

results = struct();

% Free-run metrics
if has_free_run
    results.free_run = struct(...
        'alpha_rms_deg',    rad2deg(alpha_rms), ...
        'alpha_pp_deg',     rad2deg(alpha_pp), ...
        'alpha_p95_deg',    rad2deg(alpha_p95), ...
        'alpha_p99_deg',    rad2deg(alpha_p99), ...
        'alpha_max_deg',    rad2deg(alpha_max_abs), ...
        'xc_rms_cm',        xc_rms*100, ...
        'xc_pp_cm',         xc_pp*100, ...
        'vm_max',           vm_max, ...
        'vm_frac_pct',      vm_frac, ...
        'bound_score',      bound_score, ...
        'tight_pct',        tight_pct, ...
        'medium_pct',       medium_pct, ...
        'wide_pct',         wide_pct);
    if exist('osc_freq', 'var')
        results.free_run.osc_freq_hz = osc_freq;
    end
    if exist('f_dom_psd', 'var')
        results.free_run.psd_dom_freq_hz = f_dom_psd;
    end
end

% Step response metrics
if has_step
    results.step = struct(...
        'step_time_s',      t_step, ...
        'step_amp_cm',      step_amp*100, ...
        'rise_time_s',      t_rise, ...
        'overshoot_pct',    xc_overshoot, ...
        'alpha_peak_deg',   alpha_peak_transient, ...
        'alpha_pre_pp_deg', rad2deg(alpha_pre_pp), ...
        'alpha_post_pp_deg', rad2deg(alpha_post_pp), ...
        'alpha_pre_rms_deg', rad2deg(alpha_pre_rms), ...
        'alpha_post_rms_deg', rad2deg(alpha_post_rms));
end

% Frequency response metrics
if has_chirp
    results.freq = struct(...
        'bandwidth_hz',     bw_hz, ...
        'peak_mag_db',      mag_peak_db, ...
        'peak_freq_hz',     f_peak, ...
        'dc_gain_db',       dc_gain_db, ...
        'coh_xc_mean',      coh_xc_avg, ...
        'coh_alpha_mean',   coh_alpha_avg, ...
        'model_hw_rms_db',  mag_err_rms);
    if ~isnan(pm_from_t)
        results.freq.phase_margin_deg = pm_from_t;
    end
end

% Observer metrics
if has_obs
    results.observer = struct(...
        'e_xc_rms_cm',        e_xc_rms*100, ...
        'e_alpha_rms_deg',    rad2deg(e_alpha_rms), ...
        'e_xc_max_cm',        e_xc_max*100, ...
        'e_alpha_max_deg',    rad2deg(e_alpha_max), ...
        't_conv_s',           t_conv, ...
        'innov_xc_rms_cm',    innov_xc_rms*100, ...
        'innov_alpha_rms_deg', rad2deg(innov_alpha_rms), ...
        'innov_xc_bias_cm',   innov_xc_bias*100, ...
        'innov_alpha_bias_deg', rad2deg(innov_alpha_bias), ...
        'rho_xc_lag1',        rho_xc_lag1, ...
        'rho_alpha_lag1',     rho_alpha_lag1, ...
        'v_xc_obs_rms',       v_xc_obs_rms, ...
        'v_xc_num_rms',       v_xc_num_rms, ...
        'v_al_obs_rms',       v_al_obs_rms, ...
        'v_al_num_rms',       v_al_num_rms);
end

% Observer metrics
if has_obs
    results.observer = struct(...
        'e_xc_rms_cm',        e_xc_rms*100, ...
        'e_alpha_rms_deg',    rad2deg(e_alpha_rms), ...
        'e_xc_max_cm',        e_xc_max*100, ...
        'e_alpha_max_deg',    rad2deg(e_alpha_max), ...
        't_conv_s',           t_conv, ...
        'innov_xc_rms_cm',    innov_xc_rms*100, ...
        'innov_alpha_rms_deg', rad2deg(innov_alpha_rms), ...
        'innov_xc_bias_cm',   innov_xc_bias*100, ...
        'innov_alpha_bias_deg', rad2deg(innov_alpha_bias), ...
        'rho_xc_lag1',        rho_xc_lag1, ...
        'rho_alpha_lag1',     rho_alpha_lag1, ...
        'v_xc_obs_rms',       v_xc_obs_rms, ...
        'v_xc_num_rms',       v_xc_num_rms, ...
        'v_al_obs_rms',       v_al_obs_rms, ...
        'v_al_num_rms',       v_al_num_rms);
end

% Controller info
results.controller = struct(...
    'sigma_th', ctrl.sigma_th, ...
    'zeta_th',  ctrl.zeta_th, ...
    'K_fb',     K_fb);

save(fullfile(root, 'data', 'verification_results.mat'), 'results');
fprintf('  Saved: data/verification_results.mat\n');

fprintf('\n========================================\n');
fprintf(' VERIFICATION COMPLETE\n');
fprintf('========================================\n\n');


%% =====================================================================
%  LOCAL HELPERS
%  =====================================================================

function s = cond_str(cond)
    if cond, s = 'PASS'; else, s = 'FAIL'; end
end

function s = skewness(x)
    % Sample skewness (bias-corrected)
    n = length(x);
    x = x(:) - mean(x);
    s = (sqrt(n*(n-1))/(n-2)) * (mean(x.^3) / (mean(x.^2))^(3/2));
end
