%% HARDWARE VERIFICATION PIPELINE — Closed-Loop Seesaw Control
%  =====================================================================
%  ONE script, two phases.  Run it before AND after each experiment.
%
%  Phase 1 (BEFORE testing) — Section 0.5 runs unconditionally:
%    - Controller slot is plug-and-play: pole_placement, LQR, PID
%    - Observer slot is plug-and-play: raw derivatives, Leuenberger, Kalman
%    - Saves data/hw_test_signals.mat with d_inj presets:
%        d_free   (Exp A: zero perturbation)
%        d_step   (Exp B: 0.5 V step at t=2 s)
%        d_prbs   (Exp C: PRBS up to 18 rad/s, +-0.5 V, 90 s)  <- preferred
%        d_chirp  (Exp C fallback: linear chirp 0.1 Hz to 18 rad/s)
%
%  Phase 2 (AFTER each experiment) — re-run this script:
%    - Detects whichever hw_*.mat files are in data/
%    - Time-domain (A, B), frequency-domain (C: CL + IV plant),
%      Hammerstein-Wiener identification from total voltage (C6), observer (F)
%    - Saves data/verification_results.mat
%
%  KEY INSIGHT:  The seesaw does NOT achieve asymptotic stability at
%  alpha = 0.  Nonlinearities (Coulomb friction, encoder quantization,
%  backlash, static friction) produce a bounded limit cycle.  The
%  controller constrains rocking between +-alpha_bound — the better the
%  controller, the tighter the bound.  This script quantifies that bound
%  rather than chasing an unachievable zero steady-state error.
%
%  =====================================================================
%  HARDWARE PROCEDURE (on the QUARC PC, per experiment)
%  =====================================================================
%
%  1. >> load data/hw_test_signals.mat       % loads d_inj_ts, segment_id_ts
%  2. Configure the model:
%       >> load_hardware_validation_config('pole_placement')
%       >> open_system(fullfile(SEESAW_ROOT, 'models', 'Seesaw_Validation.slx'))
%     Swap in other designs without changing the model, e.g.:
%       >> load_hardware_validation_config('lqr')
%  4. QUARC External -> Build (Ctrl+B) -> Connect (Ctrl+T) -> Start.
%     Hold seesaw level; release gently after Start.
%  5. After the run, To Host File output columns are:
%       [time | seg_id | xc | alpha | xc_dot | alpha_dot | u_ctrl | u_presat | Vm | d_inj]
%  6. Import the raw To Host File matrix:
%       >> import_hardware_validation_log('data/hw_raw_single.mat', 'single')
%     This saves data/ files with the variables listed below.
%  7. Named analysis files consumed by this script:
%       Exp A: hw_free_run.mat      with hw_t,hw_xc,hw_alpha,hw_vm
%       Exp B: hw_step_response.mat with hw_t,hw_d,hw_xc,hw_alpha,hw_vm
%       Exp C: hw_prbs_response.mat or hw_chirp_response.mat (same vars as B)
%       Exp D: hw_obs_free.mat      with hw_t,hw_xc,hw_alpha,hw_vm,
%              hw_xc_hat,hw_xcdot_hat,hw_alpha_hat,hw_alphadot_hat
%  8. Re-run this script to get the analysis.
%
%  =====================================================================
%  Requires:  startup.m  (paths + seesaw_params)
%             data/tuned_params.mat, data/controller_freq.mat
%  Outputs:   data/hw_test_signals.mat                       (Phase 1)
%             docs/figures/Verification-*.png                (Phase 2)
%             data/verification_results.mat                  (Phase 2)
%  =====================================================================

%% 0. SETUP — Load parameters and paths
close all; clc;

root   = SEESAW_ROOT;
valdir = fullfile(root, 'validation');
if ~exist('SEESAW_ROOT', 'var') || ~strcmp(SEESAW_ROOT, root)
    run(fullfile(root, 'startup.m'));
end

figdir = fullfile(valdir, 'docs', 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

tuned  = load(fullfile(root, 'data', 'tuned', 'tuned_params.mat'));
ctrl   = load(fullfile(root, 'data', 'controllers', 'controller_freq.mat'));

B_eq  = tuned.B_eq;
K_fb  = ctrl.Kf;
p_cl  = ctrl.p_final;
alpha_max = theta_max;

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

%% 0.5 PRE-TEST SETUP — Generate Single-Run Protocol Signals
% Generates one concatenated d_inj timeseries and segment_id_ts for the
% single-run validation protocol.  One hardware run collects all data.
%
% Protocol (all times approximate):
%   Segment 0: Prep/offset-reset + mini-liftoff (excluded from analysis)
%   Segment 1: Baseline free-run (d = 0)
%   Segment 2: +1 V disturbance step
%   Segment 3: Recovery (d = 0)
%   Segment 4: -1 V disturbance step
%   Segment 5: Recovery (d = 0)
%   Segment 6: +1 V pulse (impulse surrogate)
%   Segment 7: Recovery (d = 0)
%   Segments 10-21: Stepped sine frequencies (1 V, zero-mean)
%   Segment 99: Final zero recovery
%
% Sine rule: 7 cycles per frequency
%   cycle 1: ramp in (half-cosine taper)
%   cycles 2-3: discard/settle
%   cycles 4-6: analyze
%   cycle 7: ramp out (half-cosine taper)

fprintf('Pre-test setup: generating single-run protocol signals...\n');

% --- Protocol parameters ---
exp_fs_Hz       = 500;         % QUARC sample rate (Ts = 2 ms)
Ts              = 1/exp_fs_Hz;
d_amp_V         = 1.0;         % Validation excitation amplitude
step_duration_s = 15;          % Finite step hold time
recovery_s      = 15;          % Zero-recovery between segments
pulse_width_s   = 0.3;         % Impulse-surrogate pulse width
baseline_s      = 10;          % Free-run baseline window
prep_s          = 5;           % Offset reset + mini-liftoff
final_recovery_s = 10;         % Final zero window
n_cycles        = 7;           % Total cycles per sine frequency

% Sine frequencies (Hz) — logarithmically spaced, 0.1 to 10 Hz
sine_freqs_Hz = [0.10, 0.16, 0.25, 0.40, 0.63, 1.00, ...
                 1.60, 2.50, 4.00, 6.30, 8.00, 10.00];

% --- Build the concatenated protocol ---
d_segments = {};    % cell array of {time_local, d_values, segment_id}
seg_table = [];     % [seg_id, t_start, t_end, description]

t_cursor = 0;      % running time

% Helper: append a constant segment
append_const = @(dur, amp, seg_id) deal( ...
    (0:Ts:dur-Ts)', amp*ones(ceil(dur/Ts), 1), seg_id);

% Segment 0: Prep (d = 0)
dur = prep_s;
n = ceil(dur / Ts);
t_local = (0:Ts:(n-1)*Ts)';
d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
    'd', zeros(n,1), 'seg_id', 0);
seg_table = [seg_table; 0, t_cursor, t_cursor+dur];
t_cursor = t_cursor + dur;

% Segment 1: Baseline (d = 0)
dur = baseline_s;
n = ceil(dur / Ts);
d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
    'd', zeros(n,1), 'seg_id', 1);
seg_table = [seg_table; 1, t_cursor, t_cursor+dur];
t_cursor = t_cursor + dur;

% Segment 2: +1 V step
dur = step_duration_s;
n = ceil(dur / Ts);
d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
    'd', d_amp_V*ones(n,1), 'seg_id', 2);
seg_table = [seg_table; 2, t_cursor, t_cursor+dur];
t_cursor = t_cursor + dur;

% Segment 3: Recovery
dur = recovery_s;
n = ceil(dur / Ts);
d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
    'd', zeros(n,1), 'seg_id', 3);
seg_table = [seg_table; 3, t_cursor, t_cursor+dur];
t_cursor = t_cursor + dur;

% Segment 4: -1 V step
dur = step_duration_s;
n = ceil(dur / Ts);
d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
    'd', -d_amp_V*ones(n,1), 'seg_id', 4);
seg_table = [seg_table; 4, t_cursor, t_cursor+dur];
t_cursor = t_cursor + dur;

% Segment 5: Recovery
dur = recovery_s;
n = ceil(dur / Ts);
d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
    'd', zeros(n,1), 'seg_id', 5);
seg_table = [seg_table; 5, t_cursor, t_cursor+dur];
t_cursor = t_cursor + dur;

% Segment 6: +1 V pulse
dur = pulse_width_s;
n = max(1, ceil(dur / Ts));
d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
    'd', d_amp_V*ones(n,1), 'seg_id', 6);
seg_table = [seg_table; 6, t_cursor, t_cursor+dur];
t_cursor = t_cursor + dur;

% Segment 7: Recovery after pulse
dur = 10;
n = ceil(dur / Ts);
d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
    'd', zeros(n,1), 'seg_id', 7);
seg_table = [seg_table; 7, t_cursor, t_cursor+dur];
t_cursor = t_cursor + dur;

% Segments 10-21: Stepped sine frequencies
for k = 1:length(sine_freqs_Hz)
    f_k = sine_freqs_Hz(k);
    T_cycle = 1/f_k;
    dur = n_cycles * T_cycle;
    n = ceil(dur / Ts);
    t_local = (0:Ts:(n-1)*Ts)';
    
    % Generate sine with half-cosine ramp in/out over first and last cycle
    raw_sine = d_amp_V * sin(2*pi*f_k * t_local);
    
    % Ramp envelope: cycle 1 ramps 0->1, cycle 7 ramps 1->0
    envelope = ones(n, 1);
    n_ramp = ceil(T_cycle / Ts);  % samples in one cycle
    
    % Ramp in: first cycle
    ramp_in = 0.5*(1 - cos(pi*(0:n_ramp-1)'/n_ramp));
    envelope(1:min(n_ramp, n)) = ramp_in(1:min(n_ramp, n));
    
    % Ramp out: last cycle
    ramp_out = 0.5*(1 + cos(pi*(0:n_ramp-1)'/n_ramp));
    idx_start = max(1, n - n_ramp + 1);
    envelope(idx_start:n) = ramp_out(1:(n-idx_start+1));
    
    d_sine = raw_sine .* envelope;
    
    seg_id_k = 10 + (k-1);  % segments 10, 11, 12, ..., 21
    d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
        'd', d_sine, 'seg_id', seg_id_k);
    seg_table = [seg_table; seg_id_k, t_cursor, t_cursor+dur];
    t_cursor = t_cursor + dur;
end

% Segment 99: Final zero recovery
dur = final_recovery_s;
n = ceil(dur / Ts);
d_segments{end+1} = struct('t_start', t_cursor, 'n', n, ...
    'd', zeros(n,1), 'seg_id', 99);
seg_table = [seg_table; 99, t_cursor, t_cursor+dur];
t_cursor = t_cursor + dur;

% --- Concatenate into single timeseries ---
total_n = sum(cellfun(@(s) s.n, d_segments));
t_vec = zeros(total_n, 1);
d_vec = zeros(total_n, 1);
seg_vec = zeros(total_n, 1);
idx = 1;
for k = 1:length(d_segments)
    s = d_segments{k};
    t_local = s.t_start + (0:Ts:(s.n-1)*Ts)';
    t_vec(idx:idx+s.n-1) = t_local;
    d_vec(idx:idx+s.n-1) = s.d;
    seg_vec(idx:idx+s.n-1) = s.seg_id;
    idx = idx + s.n;
end

% Create timeseries for FromWorkspace blocks
d_inj = [t_vec, d_vec];
segment_id_ts = [t_vec, seg_vec];

% Total duration
total_duration_s = t_vec(end);

% Save protocol signals
sig_path = fullfile(valdir, 'data', 'hw_test_signals.mat');
save(sig_path, 'd_inj', 'segment_id_ts', 'seg_table', 'sine_freqs_Hz', ...
    'd_amp_V', 'n_cycles', 'exp_fs_Hz', 'total_duration_s');

% Also assign to base workspace for immediate model use
assignin('base', 'd_inj', d_inj);
assignin('base', 'segment_id_ts', segment_id_ts);

fprintf('  Saved: validation/data/hw_test_signals.mat\n');
fprintf('  Total run duration: %.1f s (%.1f min)\n', total_duration_s, total_duration_s/60);
fprintf('  Sine frequencies: %s Hz\n', num2str(sine_freqs_Hz, '%.2f '));
fprintf('  Excitation amplitude: %.1f V\n', d_amp_V);
fprintf('  Sine rule: %d cycles (ramp-in 1, discard 2-3, analyze 4-6, ramp-out 7)\n', n_cycles);
fprintf('\n  Segment table:\n');
seg_names = {'Prep', 'Baseline', '+1V Step', 'Recovery', '-1V Step', ...
    'Recovery', 'Pulse +1V', 'Recovery'};
for k = 1:size(seg_table, 1)
    sid = seg_table(k, 1);
    if sid <= 7
        sname = seg_names{sid+1};
    elseif sid >= 10 && sid <= 21
        sname = sprintf('Sine %.2f Hz', sine_freqs_Hz(sid-9));
    elseif sid == 99
        sname = 'Final Recovery';
    else
        sname = '?';
    end
    fprintf('    Seg %2d: %6.1f - %6.1f s  (%s)\n', sid, ...
        seg_table(k,2), seg_table(k,3), sname);
end

fprintf('\n----------------------------------------\n');
fprintf(' Pre-test ready. On the QUARC PC:\n');
fprintf('----------------------------------------\n');
fprintf('   >> load validation/data/hw_test_signals.mat\n');
fprintf('   >> load_hardware_validation_config(''pole_placement'')\n');
fprintf('   Open models/Seesaw_Validation.slx\n');
fprintf('   QUARC External -> Build -> Connect -> Start.\n');
fprintf('   Hold seesaw level, release gently after Start.\n');
fprintf('   Single run collects ALL validation data (~%.0f s).\n', total_duration_s);
fprintf('----------------------------------------\n');

% ---- Check for single-run validation data ----
single_run_file = fullfile(valdir, 'data', 'hw_single_run.mat');
has_single_run = exist(single_run_file, 'file') == 2;

% Legacy file support (from previous multi-experiment protocol)
has_free_run  = exist(fullfile(valdir, 'data', 'hw_free_run.mat'), 'file') == 2;
has_step      = exist(fullfile(valdir, 'data', 'hw_step_response.mat'), 'file') == 2;
has_obs       = exist(fullfile(valdir, 'data', 'hw_obs_free.mat'), 'file') == 2;
freq_file_prbs  = fullfile(valdir, 'data', 'hw_prbs_response.mat');
freq_file_chirp = fullfile(valdir, 'data', 'hw_chirp_response.mat');
if exist(freq_file_prbs, 'file') == 2
    freq_file = freq_file_prbs; freq_label = 'PRBS';
elseif exist(freq_file_chirp, 'file') == 2
    freq_file = freq_file_chirp; freq_label = 'chirp/PRBS legacy';
else
    freq_file = freq_file_chirp; freq_label = 'missing';
end
has_chirp = exist(freq_file, 'file') == 2;
has_hammerstein_wiener = false;

if ~has_single_run && ~has_free_run && ~has_step && ~has_obs
    fprintf('\nNo hardware data in data/ yet — Phase 1 complete.\n');
    fprintf('Run the single-run experiment, import the log, then re-run for Phase 2.\n\n');
    return;
end

if has_single_run
    fprintf('\nPhase 2: Single-Run Protocol Analysis\n');
    fprintf('  Loading: %s\n', single_run_file);
    results = analyze_single_run(valdir, figdir);
    return;  % Skip legacy sections below
else
    fprintf('\nPhase 2: Legacy multi-file analysis.\n');
    fprintf('  free-run=%d  step=%d  obs=%d\n', has_free_run, has_step, has_obs);
    fprintf('  NOTE: Consider re-running with the new single-run protocol.\n');
end


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

    d = load(fullfile(valdir, 'data', 'hw_free_run.mat'));

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
    win       = my_hanning(length(alpha_demean));
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
%  SECTION B — TIME-DOMAIN: DISTURBANCE REJECTION STEP
%  =====================================================================
%  A small disturbance voltage step is injected into the control signal:
%    u = sat(-K*x + d),   d = 0 -> ±0.5 V step at t ≈ 2 s.
%  This tests disturbance rejection: how does the controller respond to
%  an external perturbation?
%
%  Metrics: peak angle excursion during transient, cart displacement from
%  zero, control effort to reject the disturbance, and oscillation
%  envelope before/after the transient.
%  =====================================================================

if ~has_step
    fprintf('\n*** SECTION B SKIPPED — hw_step_response.mat not found ***\n');
else
    fprintf('\n========================================\n');
    fprintf(' SECTION B: Disturbance Rejection Step\n');
    fprintf('========================================\n');

    d = load(fullfile(valdir, 'data', 'hw_step_response.mat'));

    t      = d.hw_t(:);
    d_inj  = d.hw_d(:);
    xc     = d.hw_xc(:);
    alpha  = d.hw_alpha(:);
    vm     = d.hw_vm(:);

    dt = mean(diff(t));
    fprintf('  Duration: %.1f s  |  Fs: %.0f Hz\n', t(end), 1/dt);

    %% B1. Detect step time from disturbance signal
    dd = diff(d_inj);
    [~, step_idx] = max(abs(dd));
    t_step = t(step_idx);
    d_amp  = d_inj(end) - d_inj(1);

    fprintf('\n  Step detected at t = %.2f s, amplitude = %.3f V\n', ...
        t_step, d_amp);

    %% B2. Pre-step and post-step bounded oscillation
    pre_mask  = t >= 1.0 & t < t_step - 0.2;
    post_mask = t > t_step + 1.0;

    alpha_pre_pp   = max(alpha(pre_mask))  - min(alpha(pre_mask));
    alpha_post_pp  = max(alpha(post_mask)) - min(alpha(post_mask));
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

    %% B3. Disturbance rejection metrics on cart and angle
    step_window = t >= t_step - 0.5 & t <= t_step + 5.0;
    tw = t(step_window) - t_step;

    % Cart displacement (from zero, since regulator drives to zero)
    xc_pre = mean(xc(pre_mask));
    xc_deviation = max(abs(xc(step_window) - xc_pre));

    % Peak angle excursion during transient
    alpha_peak_transient = max(abs(alpha(step_window)));
    alpha_peak_transient_deg = rad2deg(alpha_peak_transient);

    % Voltage effort after step (how hard the controller fights back)
    vm_rms_pre  = rms(vm(pre_mask));
    vm_rms_post = rms(vm(post_mask));
    vm_peak_transient = max(abs(vm(step_window)));

    % Post-step cart bound
    xc_p95_post = prctile(abs(xc(post_mask) - mean(xc(post_mask))), 95);

    fprintf('\n  --- Disturbance Rejection Metrics ---\n');
    fprintf('    Disturbance step:     %+.3f V\n', d_amp);
    fprintf('    Cart deviation:       %.2f cm\n', xc_deviation*100);
    fprintf('    Peak |alpha| during:  %.2f deg\n', alpha_peak_transient_deg);
    fprintf('    V_m RMS: pre=%.2f V → post=%.2f V\n', vm_rms_pre, vm_rms_post);
    fprintf('    Post-step x_c bound:  ±%.2f cm\n', xc_p95_post*100);

    %% B4. Figures — Disturbance Rejection Step
    figure('Name', 'Verification: Disturbance Rejection Step', ...
        'Position', [50 50 1100 750]);

    subplot(4,1,1);
    yyaxis left;
    plot(t, xc*100, 'b-', 'LineWidth', 1.2); hold on;
    ylabel('x_c [cm]');
    yyaxis right;
    plot(t, d_inj, 'Color', [0.5 0 0.5], 'LineWidth', 1.2);
    ylabel('d [V]');
    xline(t_step, 'g--', 'Step');
    grid on;
    title(sprintf('Disturbance Rejection: d = %+.2f V step', d_amp));
    legend('x_c', 'd (perturbation)', 'Location', 'best');

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
    title(sprintf('Control Voltage (RMS pre=%.2f V, post=%.2f V)', ...
        vm_rms_pre, vm_rms_post));

    subplot(4,1,4);
    h_pre = histogram(rad2deg(alpha(pre_mask)), 'BinWidth', 0.25, ...
        'FaceColor', 'b', 'EdgeColor', 'none', 'Normalization', 'pdf');
    hold on;
    h_post = histogram(rad2deg(alpha(post_mask)), 'BinWidth', 0.25, ...
        'FaceColor', 'r', 'EdgeColor', 'none', 'Normalization', 'pdf');
    grid on;
    xlabel('\alpha [deg]'); ylabel('PDF');
    legend('Pre-step', 'Post-step');
    title('Angle Distribution: Before vs After Disturbance');

    sgtitle('Hardware Verification — Disturbance Rejection', 'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Verification-StepResponse.png'));
    fprintf('  Saved: docs/figures/Verification-StepResponse.png\n');

    %% B5. Zoomed disturbance window
    figure('Name', 'Verification: Disturbance Zoom', ...
        'Position', [100 100 1000 500]);
    subplot(2,1,1);
    plot(tw, xc(step_window)*100, 'b-', 'LineWidth', 1.5); hold on;
    plot(tw, d_inj(step_window)*100, '-', 'Color', [0.5 0 0.5], 'LineWidth', 1.5);
    grid on; xlim([-0.2 2.0]);
    ylabel('x_c [cm] / d [V]');
    title(sprintf('Cart Response to %.2f V Disturbance Step', d_amp));
    legend('x_c [cm]', 'd [V]');

    subplot(2,1,2);
    plot(tw, rad2deg(alpha(step_window)), 'r-', 'LineWidth', 1.2); hold on;
    grid on; xlim([-0.2 2.0]);
    yline(0, 'k-');
    ylabel('\alpha [deg]'); xlabel('Time from disturbance [s]');
    title(sprintf('Angle Transient (peak = %.2f deg)', alpha_peak_transient_deg));

    saveas(gcf, fullfile(figdir, 'Verification-StepZoom.png'));
    fprintf('  Saved: docs/figures/Verification-StepZoom.png\n');
end


%% =====================================================================
%  SECTION C — FREQUENCY-DOMAIN: CLOSED-LOOP AND PLANT BODE
%  =====================================================================
%  A small-amplitude broadband disturbance d (PRBS preferred, chirp OK)
%  is injected into the control signal:  u = sat(-K*x + d).
%
%  From logged d, V_m, x_c, alpha we estimate two families of FRFs:
%
%  (1) Closed-loop disturbance sensitivity (what the script originally did):
%        X_c(s)/d(s)   = G_xc * S      <- cart disturbance rejection
%        alpha(s)/d(s) = G_alpha * S   <- angle disturbance rejection
%        V_m(s)/d(s)   = S             <- input sensitivity
%      Compared to (sI - A + BK)^-1 B from the model -> CL-rejection check.
%
%  (2) Open-loop PLANT FRF via the indirect / IV method:
%        G_xc(jw)    = (X_c/d) / (V_m/d) = H_xc(jw) / H_vm(jw)
%        G_alpha(jw) = (alpha/d) / (V_m/d)
%      Compared to (sI - A)^-1 B from the model -> plant-model check.
%      d acts as an instrumental variable, so this is asymptotically
%      unbiased despite the unstable plant being in closed loop.
%      Trustworthy only where coherence(d,V_m) AND coherence(d,y) are
%      both high; the rest is NaN-masked.
%  =====================================================================

if ~has_chirp
    fprintf('\n*** SECTION C SKIPPED — hw_prbs_response.mat/hw_chirp_response.mat not found ***\n');
else
    fprintf('\n========================================\n');
    fprintf(' SECTION C: Disturbance-to-Output Frequency Response\n');
    fprintf('========================================\n');

    fprintf('  Using %s data: %s\n', freq_label, freq_file);
    d = load(freq_file);

    t      = d.hw_t(:);
    d_inj  = d.hw_d(:);
    xc     = d.hw_xc(:);
    alpha  = d.hw_alpha(:);
    vm     = d.hw_vm(:);

    dt = mean(diff(t));
    Fs = 1/dt;
    fprintf('  Duration: %.1f s  |  Fs: %.0f Hz  |  N: %d\n', t(end), Fs, length(t));

    %% C1. Estimate disturbance-to-output transfer functions (Welch)
    n_seg = min(4096, 2^nextpow2(length(t)/10));
    n_seg = max(n_seg, 512);
    n_overlap = n_seg / 2;

    fprintf('\n  Welch parameters: n_seg=%d, overlap=%d, segments≈%d\n', ...
        n_seg, n_overlap, floor(length(t)/(n_seg - n_overlap)));

    % H1 estimator from disturbance d to each output
    [H_xc, f_tfe]    = tfestimate(d_inj, xc,    hanning(n_seg), n_overlap, n_seg, Fs);
    [H_alpha, ~]     = tfestimate(d_inj, alpha, hanning(n_seg), n_overlap, n_seg, Fs);
    [H_vm, ~]        = tfestimate(d_inj, vm,    hanning(n_seg), n_overlap, n_seg, Fs);

    [C_xc, ~]    = mscohere(d_inj, xc,    hanning(n_seg), n_overlap, n_seg, Fs);
    [C_alpha, ~] = mscohere(d_inj, alpha, hanning(n_seg), n_overlap, n_seg, Fs);
    [C_vm, ~]    = mscohere(d_inj, vm,    hanning(n_seg), n_overlap, n_seg, Fs);

    % --- Indirect / IV plant FRF (closed-loop ID) -----------------------
    % With u = -Kx + d and plant G:    y/d = G*S,   V_m/d = S
    % => G(jw) = (y/d)/(V_m/d) = H_yd / H_vm
    % Recovers the OPEN-LOOP plant from CLOSED-LOOP data without ever
    % opening the (unstable) loop. d is the instrumental variable, so
    % the estimate is unbiased even though u is correlated with noise.
    G_xc_plant    = H_xc    ./ H_vm;
    G_alpha_plant = H_alpha ./ H_vm;

    coh_thresh    = 0.5;
    mask_xc_iv    = (C_vm > coh_thresh) & (C_xc    > coh_thresh);
    mask_alpha_iv = (C_vm > coh_thresh) & (C_alpha > coh_thresh);

    f_lo = 0.1;
    f_hi = validation_f_hi_Hz;
    f_mask = f_tfe >= f_lo & f_tfe <= f_hi;
    f_use = f_tfe(f_mask);

    %% C2. Cart disturbance sensitivity bandwidth
    mag_xc_db = 20*log10(abs(H_xc(f_mask)));
    dc_gain_db = mag_xc_db(find(f_use >= f_lo, 1, 'first'));
    idx_bw = find(mag_xc_db < dc_gain_db - 3, 1, 'first');
    if isempty(idx_bw)
        bw_hz = f_use(end);
        bw_warning = ' (not found — beyond validation band)';
    else
        bw_hz = f_use(idx_bw);
        bw_warning = '';
    end

    [mag_peak_db, idx_peak] = max(mag_xc_db);
    f_peak = f_use(idx_peak);

    % Mean sensitivity magnitude (lower = better rejection)
    mag_xc_mean = mean(abs(H_xc(f_mask)));

    fprintf('\n  --- Disturbance-to-Output Frequency Metrics ---\n');
    fprintf('    Sensitivity BW (-3 dB):  %.2f Hz%s\n', bw_hz, bw_warning);
    fprintf('    Peak |G_xc(s)|:          %.1f dB (%.1e m/V) at %.2f Hz\n', ...
        mag_peak_db, 10^(mag_peak_db/20), f_peak);
    fprintf('    Mean |G_xc|:             %.1e m/V  (lower = better rejection)\n', ...
        mag_xc_mean);

    % Coherence quality
    coh_xc_avg    = mean(C_xc(f_mask));
    coh_alpha_avg = mean(C_alpha(f_mask));
    coh_vm_avg    = mean(C_vm(f_mask));
    fprintf('\n  --- Coherence ---\n');
    fprintf('    Mean coh (x_c):     %.2f  (>0.7 = good CL FRF)\n', coh_xc_avg);
    fprintf('    Mean coh (alpha):   %.2f  (lower = rocking dominates)\n', coh_alpha_avg);
    fprintf('    Mean coh (V_m):     %.2f  (>0.7 = good IV plant FRF)\n', coh_vm_avg);
    if coh_alpha_avg < 0.5
        fprintf('    Note: alpha coherence may be low because the unmeasured\n');
        fprintf('          rocking disturbance is uncorrelated with injected d(t).\n');
        fprintf('          This is expected.\n');
    end

    %% C2b. Open-loop plant FRF coverage (indirect estimate)
    iv_coverage_xc    = sum(mask_xc_iv(f_mask))    / sum(f_mask) * 100;
    iv_coverage_alpha = sum(mask_alpha_iv(f_mask)) / sum(f_mask) * 100;
    fprintf('\n  --- Indirect Plant FRF Coverage (coh > %.1f) ---\n', coh_thresh);
    fprintf('    X_c/V_m reliable:    %.0f%%%% of [%.2f, %.2f] Hz\n', ...
        iv_coverage_xc, f_lo, f_hi);
    fprintf('    alpha/V_m reliable:  %.0f%%%% of [%.2f, %.2f] Hz\n', ...
        iv_coverage_alpha, f_lo, f_hi);
    if iv_coverage_xc < 30
        fprintf('    Warning: indirect estimate has poor coverage. Use PRBS\n');
        fprintf('             and/or a longer record (>= 90 s).\n');
    end

    %% C3. Figures — Disturbance-to-Output Bode
    figure('Name', 'Verification: Disturbance-to-Output Bode', ...
        'Position', [50 50 1100 800]);

    subplot(3,2,1);
    semilogx(f_use, mag_xc_db(f_mask), 'b-', 'LineWidth', 1.5); grid on; hold on;
    yline(dc_gain_db - 3, 'r--', sprintf('-3 dB → %.1f Hz', bw_hz));
    xlim([f_lo f_hi]);
    ylabel('Magnitude [dB m/V]');
    title(sprintf('G_{xc}(s) = X_c / d  (BW = %.2f Hz)', bw_hz));

    subplot(3,2,2);
    phase_xc_deg = unwrap(angle(H_xc(f_mask))) * 180/pi;
    semilogx(f_use, phase_xc_deg, 'b-', 'LineWidth', 1.5); grid on;
    xlim([f_lo f_hi]);
    ylabel('Phase [deg]');
    title('Phase G_{xc}(s)');

    subplot(3,2,3);
    mag_vm_db = 20*log10(abs(H_vm(f_mask)));
    semilogx(f_use, mag_vm_db, 'Color', [0 0.6 0], 'LineWidth', 1.5); grid on;
    xlim([f_lo f_hi]);
    ylabel('Magnitude [dB V/V]');
    title('Control Response: V_m / d');

    subplot(3,2,4);
    phase_vm_deg = unwrap(angle(H_vm(f_mask))) * 180/pi;
    semilogx(f_use, phase_vm_deg, 'Color', [0 0.6 0], 'LineWidth', 1.5); grid on;
    xlim([f_lo f_hi]);
    ylabel('Phase [deg]');
    title('Phase V_m / d');

    subplot(3,2,5);
    mag_alpha_db = 20*log10(abs(H_alpha(f_mask)));
    semilogx(f_use, mag_alpha_db, 'r-', 'LineWidth', 1.5); grid on;
    xlim([f_lo f_hi]);
    ylabel('Magnitude [dB rad/V]');
    xlabel('Frequency [Hz]');
    title('Angle Sensitivity: α / d');

    subplot(3,2,6);
    semilogx(f_use, C_xc(f_mask), 'b-', 'LineWidth', 1.2); hold on;
    semilogx(f_use, C_alpha(f_mask), 'r-', 'LineWidth', 1.2); grid on;
    yline(0.7, 'k--');
    xlim([f_lo f_hi]); ylim([0 1]);
    ylabel('Coherence');
    xlabel('Frequency [Hz]');
    legend('x_c', '\alpha', 'Location', 'best');
    title('Coherence (H1 estimator quality)');

    sgtitle('Hardware Verification — Disturbance-to-Output Frequency Response', ...
        'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Verification-CLBode.png'));
    fprintf('  Saved: docs/figures/Verification-CLBode.png\n');

    %% C4. Overlay with analytical prediction
    M_c_added = ctrl.M_c_added;
    if ~exist('M_c_added', 'var'), M_c_added = 0; end
    M_c_use = M_c + M_c_added;
    B_total = B_eq + B_emf;
    M_eff = [M_c_use, -M_c_use*D_T; -M_c_use*D_T, J_pivot + M_c_use*D_T^2];
    M_inv = inv(M_eff);
    G_rhs = [0, -B_total, -g*M_c_use, 0; -g*M_c_use, 0, g*(M_c_use*D_T + M_SW*D_C), -B_SW];
    A_sw = [0 1 0 0; M_inv(1,:)*G_rhs; 0 0 0 1; M_inv(2,:)*G_rhs];
    B_sw = [0; M_inv(1,:)*[alpha_f*eta_m; 0]; 0; M_inv(2,:)*[alpha_f*eta_m; 0]];

    % Closed-loop with disturbance input: x_dot = (A - B*K)*x + B*d
    A_cl = A_sw - B_sw * K_fb;
    C_xc_out = [1 0 0 0];

    G_xc_model = tf(ss(A_cl, B_sw, C_xc_out, 0));

    [mag_model, phase_model] = bode(G_xc_model, 2*pi*f_use);
    mag_model_db = 20*log10(squeeze(mag_model));
    phase_model_deg = squeeze(phase_model);

    figure('Name', 'Verification: Dist. Bode Model vs Hardware', ...
        'Position', [100 100 1000 600]);
    subplot(2,1,1);
    semilogx(f_use, mag_xc_db(f_mask), 'b-', 'LineWidth', 1.5); hold on;
    semilogx(f_use, mag_model_db, 'k--', 'LineWidth', 2);
    grid on; xlim([f_lo f_hi]);
    ylabel('Magnitude [dB m/V]');
    title('G_{xc}(s) = X_c / d:  Model vs Hardware');
    legend('Hardware (H1)', 'Model (linear)', 'Location', 'best');

    subplot(2,1,2);
    semilogx(f_use, phase_xc_deg, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(f_use, phase_model_deg, 'k--', 'LineWidth', 2);
    grid on; xlim([f_lo f_hi]);
    ylabel('Phase [deg]'); xlabel('Frequency [Hz]');

    mag_err_rms = rms(mag_xc_db(f_mask) - mag_model_db(:));
    fprintf('\n  --- Model-Hardware Agreement ---\n');
    fprintf('    RMS magnitude error:  %.2f dB  (in validation band)\n', mag_err_rms);

    saveas(gcf, fullfile(figdir, 'Verification-CLBode-ModelVsHW.png'));
    fprintf('  Saved: docs/figures/Verification-CLBode-ModelVsHW.png\n');

    %% C5. Open-loop PLANT Bode: indirect IV estimate vs analytical model
    % This is the plant-model check. (sI-A)^-1 B is the OPEN-LOOP plant,
    % recovered from closed-loop data via G(jw) = H_xc(jw) / H_vm(jw).
    G_xc_plant_model    = tf(ss(A_sw, B_sw, [1 0 0 0], 0));
    G_alpha_plant_model = tf(ss(A_sw, B_sw, [0 0 1 0], 0));

    [mag_p_xc,    phase_p_xc]    = bode(G_xc_plant_model,    2*pi*f_use);
    [mag_p_alpha, phase_p_alpha] = bode(G_alpha_plant_model, 2*pi*f_use);
    mag_p_xc_db       = 20*log10(squeeze(mag_p_xc));
    mag_p_alpha_db    = 20*log10(squeeze(mag_p_alpha));
    phase_p_xc_deg    = squeeze(phase_p_xc);
    phase_p_alpha_deg = squeeze(phase_p_alpha);

    % NaN-mask the indirect estimate where coherence is too low to trust
    G_xc_iv_plot    = G_xc_plant(f_mask);
    G_alpha_iv_plot = G_alpha_plant(f_mask);
    G_xc_iv_plot(~mask_xc_iv(f_mask))       = NaN;
    G_alpha_iv_plot(~mask_alpha_iv(f_mask)) = NaN;

    mag_xc_iv_db       = 20*log10(abs(G_xc_iv_plot));
    mag_alpha_iv_db    = 20*log10(abs(G_alpha_iv_plot));
    phase_xc_iv_deg    = unwrap(angle(G_xc_iv_plot)) * 180/pi;
    phase_alpha_iv_deg = unwrap(angle(G_alpha_iv_plot)) * 180/pi;

    figure('Name', 'Verification: OL Plant Bode (Indirect) vs Model', ...
        'Position', [100 100 1100 700]);

    subplot(2,2,1);
    semilogx(f_use, mag_xc_iv_db, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(f_use, mag_p_xc_db,  'k--', 'LineWidth', 2);
    grid on; xlim([f_lo f_hi]);
    ylabel('|G_{xc}| [dB m/V]');
    legend('Hardware (IV indirect)', 'Model (sI-A)^{-1}B', 'Location', 'best');
    title('Plant: X_c / V_m');

    subplot(2,2,2);
    semilogx(f_use, phase_xc_iv_deg, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(f_use, phase_p_xc_deg,  'k--', 'LineWidth', 2);
    grid on; xlim([f_lo f_hi]);
    ylabel('Phase [deg]');
    title('Phase X_c / V_m');

    subplot(2,2,3);
    semilogx(f_use, mag_alpha_iv_db, 'r-', 'LineWidth', 1.5); hold on;
    semilogx(f_use, mag_p_alpha_db,  'k--', 'LineWidth', 2);
    grid on; xlim([f_lo f_hi]);
    ylabel('|G_{\alpha}| [dB rad/V]'); xlabel('Frequency [Hz]');
    legend('Hardware (IV indirect)', 'Model (sI-A)^{-1}B', 'Location', 'best');
    title('Plant: \alpha / V_m');

    subplot(2,2,4);
    semilogx(f_use, phase_alpha_iv_deg, 'r-', 'LineWidth', 1.5); hold on;
    semilogx(f_use, phase_p_alpha_deg,  'k--', 'LineWidth', 2);
    grid on; xlim([f_lo f_hi]);
    ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
    title('Phase \alpha / V_m');

    % Mag-error metrics (in the coherent band only)
    err_xc    = mag_xc_iv_db    - mag_p_xc_db;
    err_alpha = mag_alpha_iv_db - mag_p_alpha_db;
    plant_err_xc_rms    = rms(err_xc(~isnan(err_xc)));
    plant_err_alpha_rms = rms(err_alpha(~isnan(err_alpha)));

    fprintf('\n  --- Plant Model vs Indirect Estimate ---\n');
    fprintf('    RMS mag error (X_c/V_m):    %.2f dB  (coherent band)\n', ...
        plant_err_xc_rms);
    fprintf('    RMS mag error (alpha/V_m):  %.2f dB  (coherent band)\n', ...
        plant_err_alpha_rms);

    sgtitle('Hardware Verification — Open-Loop PLANT Bode (Indirect)', ...
        'FontWeight', 'bold');
    saveas(gcf, fullfile(figdir, 'Verification-PlantBode-Indirect.png'));
    fprintf('  Saved: docs/figures/Verification-PlantBode-Indirect.png\n');

    %% C6. Hammerstein-Wiener identification from total motor voltage
    % The FRF sections use injected d as the instrumental variable.  For a
    % nonlinear predictive model, use the total applied voltage V_m as the
    % input so the identified input nonlinearity can absorb dead-zone,
    % stiction, and directional gain bias seen by the motor.
    fprintf('\n========================================\n');
    fprintf(' SECTION C6: Hammerstein-Wiener Identification\n');
    fprintf('========================================\n');

    if ~license('test', 'Identification_Toolbox') || exist('nlhw', 'file') ~= 2
        fprintf('  Skipped: System Identification Toolbox nlhw() is not available.\n');
    else
        id_trim_s     = 3.0;
        id_target_fs  = 100;
        id_split_frac = 0.70;
        id_orders     = [4 4 1];  % nb, nf, nk for the voltage-to-output dynamics

        finite_mask = isfinite(t) & isfinite(vm) & isfinite(xc) & isfinite(alpha);
        id_mask = finite_mask & t >= id_trim_s;
        id_idx = find(id_mask);
        id_decim = max(1, round(Fs / id_target_fs));
        id_idx = id_idx(1:id_decim:end);

        if numel(id_idx) < 500
            fprintf('  Skipped: only %d usable samples after trim/downsample.\n', numel(id_idx));
        else
            t_id = t(id_idx) - t(id_idx(1));
            u_id = vm(id_idx);
            y_xc_id = xc(id_idx);
            y_alpha_id = alpha(id_idx);
            Ts_id = median(diff(t_id));

            % Demean the closed-loop record so nlhw fits the perturbed
            % voltage-to-motion dynamics rather than static encoder offsets.
            u_id = u_id - mean(u_id);
            y_xc_id = y_xc_id - mean(y_xc_id);
            y_alpha_id = y_alpha_id - mean(y_alpha_id);

            split_idx = floor(id_split_frac * numel(t_id));
            est_idx = 1:split_idx;
            val_idx = split_idx+1:numel(t_id);

            ze_xc = iddata(y_xc_id(est_idx), u_id(est_idx), Ts_id);
            zv_xc = iddata(y_xc_id(val_idx), u_id(val_idx), Ts_id);
            ze_alpha = iddata(y_alpha_id(est_idx), u_id(est_idx), Ts_id);
            zv_alpha = iddata(y_alpha_id(val_idx), u_id(val_idx), Ts_id);

            ze_xc.InputName = {'V_m_total'}; ze_xc.OutputName = {'x_c'};
            zv_xc.InputName = {'V_m_total'}; zv_xc.OutputName = {'x_c'};
            ze_alpha.InputName = {'V_m_total'}; ze_alpha.OutputName = {'alpha'};
            zv_alpha.InputName = {'V_m_total'}; zv_alpha.OutputName = {'alpha'};

            [in_nl_xc, out_nl_xc] = make_hw_nonlinearities();
            [in_nl_alpha, out_nl_alpha] = make_hw_nonlinearities();

            fprintf('  Input: logged total V_m, not injected d.\n');
            fprintf('  Fit data: %.1f s to %.1f s | Validation: %.1f s to %.1f s | Fs ~= %.0f Hz\n', ...
                t_id(est_idx(1)), t_id(est_idx(end)), t_id(val_idx(1)), t_id(val_idx(end)), 1/Ts_id);
            fprintf('  Orders: nb=%d, nf=%d, nk=%d\n', id_orders(1), id_orders(2), id_orders(3));

            try
                if exist('nlhwOptions', 'file') == 2
                    opt = nlhwOptions;
                    opt.Display = 'off';
                    hw_xc_model = nlhw(ze_xc, id_orders, in_nl_xc, out_nl_xc, opt);
                    hw_alpha_model = nlhw(ze_alpha, id_orders, in_nl_alpha, out_nl_alpha, opt);
                else
                    hw_xc_model = nlhw(ze_xc, id_orders, in_nl_xc, out_nl_xc);
                    hw_alpha_model = nlhw(ze_alpha, id_orders, in_nl_alpha, out_nl_alpha);
                end

                [y_xc_hat, fit_xc] = compare(zv_xc, hw_xc_model);
                [y_alpha_hat, fit_alpha] = compare(zv_alpha, hw_alpha_model);
                if iscell(y_xc_hat), y_xc_hat = y_xc_hat{1}; end
                if iscell(y_alpha_hat), y_alpha_hat = y_alpha_hat{1}; end

                hw_fit_xc_pct = fit_xc(1);
                hw_fit_alpha_pct = fit_alpha(1);
                has_hammerstein_wiener = true;

                fprintf('\n  --- Validation Fit ---\n');
                fprintf('    x_c:    %.1f%%%%\n', hw_fit_xc_pct);
                fprintf('    alpha:  %.1f%%%%\n', hw_fit_alpha_pct);

                t_val = t_id(val_idx);
                xc_val = iddata_output_vector(zv_xc);
                alpha_val = iddata_output_vector(zv_alpha);
                xc_hat = iddata_output_vector(y_xc_hat);
                alpha_hat = iddata_output_vector(y_alpha_hat);

                figure('Name', 'Verification: Hammerstein-Wiener ID', ...
                    'Position', [80 80 1100 750]);

                subplot(3,1,1);
                plot(t_val, u_id(val_idx), 'Color', [0 0.6 0], 'LineWidth', 0.9);
                grid on; ylabel('V_m [V]');
                title(sprintf('Identification Input: Total Motor Voltage (%s record)', freq_label));

                subplot(3,1,2);
                plot(t_val, xc_val*100, 'b-', 'LineWidth', 1.0); hold on;
                plot(t_val, xc_hat*100, 'k--', 'LineWidth', 1.1);
                grid on; ylabel('x_c [cm]');
                legend('Hardware', 'HW model', 'Location', 'best');
                title(sprintf('Hammerstein-Wiener Validation: x_c fit = %.1f%%%%', hw_fit_xc_pct));

                subplot(3,1,3);
                plot(t_val, rad2deg(alpha_val), 'r-', 'LineWidth', 1.0); hold on;
                plot(t_val, rad2deg(alpha_hat), 'k--', 'LineWidth', 1.1);
                grid on; ylabel('\alpha [deg]'); xlabel('Time [s]');
                legend('Hardware', 'HW model', 'Location', 'best');
                title(sprintf('Hammerstein-Wiener Validation: alpha fit = %.1f%%%%', hw_fit_alpha_pct));

                sgtitle('Hardware Verification — Hammerstein-Wiener Model ID', ...
                    'FontWeight', 'bold');
                saveas(gcf, fullfile(figdir, 'Verification-HammersteinWiener.png'));
                fprintf('  Saved: docs/figures/Verification-HammersteinWiener.png\n');

                hw_id_info = struct( ...
                    'source_file', freq_file, ...
                    'source_label', freq_label, ...
                    'input', 'logged total motor voltage hw_vm', ...
                    'outputs', {{'x_c', 'alpha'}}, ...
                    'trim_s', id_trim_s, ...
                    'sample_time_s', Ts_id, ...
                    'decimation', id_decim, ...
                    'orders_nb_nf_nk', id_orders, ...
                    'split_fraction', id_split_frac, ...
                    'fit_xc_pct', hw_fit_xc_pct, ...
                    'fit_alpha_pct', hw_fit_alpha_pct);
                save(fullfile(valdir, 'data', 'hammerstein_wiener_hw.mat'), ...
                    'hw_xc_model', 'hw_alpha_model', 'hw_id_info');
                fprintf('  Saved: data/hammerstein_wiener_hw.mat\n');
            catch ME
                fprintf('  Hammerstein-Wiener identification failed: %s\n', ME.message);
                fprintf('  Frequency-domain validation results are unaffected.\n');
            end
        end
    end
end


%% =====================================================================
%  SECTION D — COMPOSITE VERIFICATION DASHBOARD
%  =====================================================================
%  Single-page summary combining time and frequency metrics.
%  Suitable for reports and quick pass/fail assessment.
%  =====================================================================

if has_obs
    % Pre-compute observer metrics for dashboard
    obs_temp = load(fullfile(valdir, 'data', 'hw_obs_free.mat'));
    t_temp = obs_temp.hw_t(:);
    xc_temp = obs_temp.hw_xc(:);
    alpha_temp = obs_temp.hw_alpha(:);
    xc_hat_temp = obs_temp.hw_xc_hat(:);
    al_hat_temp = obs_temp.hw_alpha_hat(:);
    
    e_xc_rms = rms(xc_hat_temp - xc_temp);
    e_alpha_rms = rms(al_hat_temp - alpha_temp);
    
    q_xc = K_ec;
    q_theta = K_E_SW / K_gs;
    e_combined = abs(xc_hat_temp - xc_temp) / q_xc + abs(al_hat_temp - alpha_temp) / q_theta;
    idx_conv = find(e_combined < 3, 1, 'first');
    if ~isempty(idx_conv)
        t_conv = t_temp(idx_conv);
    else
        t_conv = t_temp(end);
    end
end

figure('Name', 'Verification: Composite Dashboard', ...
    'Position', [30 30 1200 850]);

% --- Tile 1: Angle time trace + bound markers ---
if has_free_run
    subplot(3,3,1);
    d = load(fullfile(valdir, 'data', 'hw_free_run.mat'));
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

% --- Tile 4: Disturbance rejection step ---
if has_step
    subplot(3,3,4);
    d = load(fullfile(valdir, 'data', 'hw_step_response.mat'));
    t_s = d.hw_t(:); xc_s = d.hw_xc(:)*100; d_s = d.hw_d(:);
    [~, si] = max(abs(diff(d_s)));
    t_s = t_s - d.hw_t(si);
    mask = t_s >= -1 & t_s <= 4;
    yyaxis left;
    plot(t_s(mask), xc_s(mask), 'b-', 'LineWidth', 1.2); hold on;
    ylabel('x_c [cm]');
    yyaxis right;
    plot(t_s(mask), d_s(mask), '-', 'Color', [0.5 0 0.5], 'LineWidth', 1.2);
    ylabel('d [V]');
    grid on;
    title('Disturbance Rejection');
    legend('x_c [cm]', 'd [V]', 'Location', 'best');
end

% --- Tile 5: Voltage histogram ---
if has_free_run
    subplot(3,3,5);
    d = load(fullfile(valdir, 'data', 'hw_free_run.mat'));
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

% --- Tile 7: Disturbance-to-output Bode magnitude ---
if has_chirp
    subplot(3,3,7);
    semilogx(f_use, mag_xc_db(f_mask), 'b-', 'LineWidth', 1.5); grid on; hold on;
    yline(dc_gain_db - 3, 'r--');
    xlim([f_lo f_hi]);
    ylabel('|G_{xc}| [dB m/V]');
    xlabel('Freq [Hz]');
    title(sprintf('Sensitivity BW = %.2f Hz, peak = %.1f dB', bw_hz, mag_peak_db));
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
li = li + 1; summary_lines{li} = 'DISTURBANCE STEP:';
if has_step
    li = li + 1; summary_lines{li} = sprintf('  Cart deviation: %.2f cm', xc_deviation*100);
    li = li + 1; summary_lines{li} = sprintf('  Peak |alpha|:   %.2f deg', alpha_peak_transient_deg);
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
li = li + 1; summary_lines{li} = 'FREQUENCY (dist-to-output):';
if has_chirp
    li = li + 1; summary_lines{li} = sprintf('  Sensitivity BW: %.2f Hz', bw_hz);
    li = li + 1; summary_lines{li} = sprintf('  Model-HW err:   %.2f dB', mag_err_rms);
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

    obs = load(fullfile(valdir, 'data', 'hw_obs_free.mat'));

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
    win = my_hanning(win_len);
    n_seg_psd = max(win_len/2, 256);
    [P_xcd_obs, f_psd] = my_pwelch(xcd_hat, win, n_seg_psd, n_fft, Fs);
    [P_ald_obs, ~]     = my_pwelch(ald_hat, win, n_seg_psd, n_fft, Fs);

    % Filtered numerical derivative for comparison
    fc_diff = 30;  % Hz — same as dirty-derivative LPF cutoff
    [b_diff, a_diff] = my_butter(2, fc_diff/(Fs/2));
    xc_filt  = my_filtfilt(b_diff, a_diff, xc);
    al_filt  = my_filtfilt(b_diff, a_diff, alpha);
    xcd_num  = [diff(xc_filt); 0] / dt;
    ald_num  = [diff(al_filt); 0] / dt;
    xcd_num  = my_filtfilt(b_diff, a_diff, xcd_num);
    ald_num  = my_filtfilt(b_diff, a_diff, ald_num);

    [P_xcd_num, ~] = my_pwelch(xcd_num, win, n_seg_psd, n_fft, Fs);
    [P_ald_num, ~] = my_pwelch(ald_num, win, n_seg_psd, n_fft, Fs);

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

% Disturbance step metrics
if has_step
    results.step = struct(...
        'd_amp_V',              d_amp, ...
        'step_time_s',          t_step, ...
        'xc_deviation_cm',      xc_deviation*100, ...
        'alpha_peak_deg',       alpha_peak_transient_deg, ...
        'vm_rms_pre',           vm_rms_pre, ...
        'vm_rms_post',          vm_rms_post, ...
        'vm_peak_transient',    vm_peak_transient, ...
        'alpha_pre_pp_deg',     rad2deg(alpha_pre_pp), ...
        'alpha_post_pp_deg',    rad2deg(alpha_post_pp), ...
        'alpha_pre_rms_deg',    rad2deg(alpha_pre_rms), ...
        'alpha_post_rms_deg',   rad2deg(alpha_post_rms));
end

% Frequency response metrics
if has_chirp
    results.freq = struct(...
        'bandwidth_hz',           bw_hz, ...
        'peak_mag_db',            mag_peak_db, ...
        'peak_freq_hz',           f_peak, ...
        'dc_gain_db',             dc_gain_db, ...
        'coh_xc_mean',            coh_xc_avg, ...
        'coh_alpha_mean',         coh_alpha_avg, ...
        'coh_vm_mean',            coh_vm_avg, ...
        'cl_model_hw_rms_db',     mag_err_rms, ...
        'plant_err_xc_rms_db',    plant_err_xc_rms, ...
        'plant_err_alpha_rms_db', plant_err_alpha_rms, ...
        'iv_coverage_xc_pct',     iv_coverage_xc, ...
        'iv_coverage_alpha_pct',  iv_coverage_alpha);
end

% Hammerstein-Wiener identification metrics
if has_hammerstein_wiener
    results.hammerstein_wiener = hw_id_info;
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

save(fullfile(valdir, 'data', 'verification_results.mat'), 'results');
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

function [input_nl, output_nl] = make_hw_nonlinearities()
    % Flexible enough to capture motor dead-zone and asymmetric gain without
    % forcing a hand-tuned dead-zone width before identification.
    input_nl = 'pwlinear';
    output_nl = 'pwlinear';
end

function y = iddata_output_vector(z)
    y = z.OutputData;
    if iscell(y), y = y{1}; end
    y = y(:);
end

function s = skewness(x)
    % Sample skewness (bias-corrected)
    n = length(x);
    x = x(:) - mean(x);
    s = (sqrt(n*(n-1))/(n-2)) * (mean(x.^3) / (mean(x.^2))^(3/2));
end

function w = my_hanning(N)
    w = 0.5 * (1 - cos(2*pi*(0:N-1)' / (N-1)));
end

function [P, f] = my_pwelch(x, win, noverlap, nfft, Fs)
    if exist('pwelch', 'file') == 2 || exist('pwelch', 'builtin') == 5
        [P, f] = pwelch(x, win, noverlap, nfft, Fs);
    else
        N = length(win);
        n_shift = N - noverlap;
        n_segments = floor((length(x) - noverlap) / n_shift);
        if n_segments < 1
            x_win = x(1:min(length(x), N)) .* win(1:min(length(x), N));
            X = fft(x_win, nfft);
            P = abs(X(1:nfft/2+1)).^2 / (Fs * sum(win.^2));
            f = Fs * (0:nfft/2)' / nfft;
        else
            P_accum = zeros(nfft/2+1, 1);
            for idx = 1:n_segments
                start_idx = (idx-1)*n_shift + 1;
                x_seg = x(start_idx : start_idx + N - 1) .* win;
                X = fft(x_seg, nfft);
                P_accum = P_accum + abs(X(1:nfft/2+1)).^2;
            end
            P = P_accum / (n_segments * Fs * sum(win.^2));
            f = Fs * (0:nfft/2)' / nfft;
        end
    end
end

function [b, a] = my_butter(n, Wn)
    if exist('butter', 'file') == 2 || exist('butter', 'builtin') == 5
        [b, a] = butter(n, Wn);
    else
        T = 0.5;
        w = 4 * tan(pi * Wn / 4);
        c1 = 16;
        c2 = 4 * sqrt(2) * w;
        c3 = w^2;
        
        a0 = c1 + c2 + c3;
        a1 = -2*c1 + 2*c3;
        a2 = c1 - c2 + c3;
        
        b0 = c3;
        b1 = 2*c3;
        b2 = c3;
        
        b = [b0, b1, b2] / a0;
        a = [a0, a1, a2] / a0;
    end
end

function y = my_filtfilt(b, a, x)
    if exist('filtfilt', 'file') == 2 || exist('filtfilt', 'builtin') == 5
        y = filtfilt(b, a, x);
    else
        mx = mean(x);
        x_demean = x - mx;
        y1 = filter(b, a, x_demean);
        y1_flip = flipud(y1);
        y2 = filter(b, a, y1_flip);
        y = flipud(y2) + mx;
    end
end
