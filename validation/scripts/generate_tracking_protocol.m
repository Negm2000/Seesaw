function generate_tracking_protocol()
%GENERATE_TRACKING_PROTOCOL  Build r_theta(t) tracking-validation protocol.
%
% Produces a single concatenated reference-angle command r_theta(t) [rad]
% and a co-timed segment_id(t) signal, both formatted for FromWorkspace
% blocks (Nx2 = [time, value]).  One ~4.2-minute run captures all four
% evidence pieces in the Hardware Validation Protocol (HVP):
%
%   Time-series tracking, Bode plot, controller comparison, observer
%   benchmark.
%
% Segment map (matches HVP labels):
%    0  prep / offset reset       (5  s, r=0, excluded from analysis)
%    1  free-run baseline         (10 s, r=0)
%    2  +1.0 deg step              (12 s)
%    3  recovery                  (8  s, r=0)
%    4  -1.0 deg step              (12 s)
%    5  recovery                  (8  s, r=0)
%    6  +1.0 deg pulse (0.3 s)     (then 8 s settle at r=0 within same seg)
%    7  recovery                  (8  s, r=0)
%   10-21  stepped sines 0.10..10 Hz, 0.5 deg, 7 cycles ea (taper in/out)
%   30  final recovery             (10 s, r=0)
%
% Saves: validation/data/tracking_protocol.mat
%   r_theta_ts     Nx2 [t, r_theta_rad]   FromWorkspace format
%   segment_id_ts  Nx2 [t, seg_id]
%   seg_table      [seg_id, t_start, t_end, label]
%   sine_freqs_Hz  vector of sine frequencies
%   r_step_deg     step amplitude (1.0 deg)
%   r_sine_deg     sine amplitude (0.5 deg)
%   exp_fs_Hz      sample rate (500 Hz)
%   total_duration_s
%
% Also assigns r_theta_ts and segment_id_ts to the base workspace so the
% FromWorkspace blocks in the *_test.slx models resolve immediately.

%% Parameters
exp_fs_Hz      = 500;        % QUARC sample rate (Ts = 2 ms)
Ts             = 1/exp_fs_Hz;

r_step_deg     = 1.0;        % step amplitude
r_pulse_deg    = 1.0;        % pulse amplitude
r_sine_deg     = 0.5;        % sine amplitude (tapered to keep cart safe @ HF)
deg2rad_       = pi/180;

prep_s         = 5;
baseline_s     = 10;
step_dur_s     = 12;
recovery_s     = 8;
pulse_width_s  = 0.3;
pulse_settle_s = 8;          % included inside seg 6
final_rec_s   = 10;
n_cycles       = 7;          % per sine freq (taper in 1, settle 2-3, analyse 4-6, taper out 7)

sine_freqs_Hz  = [0.10, 0.16, 0.25, 0.40, 0.63, 1.00, ...
                  1.60, 2.50, 4.00, 6.30, 8.00, 10.00];

seg_labels = containers.Map('KeyType','double','ValueType','char');
seg_labels(0)  = 'prep';
seg_labels(1)  = 'free_run';
seg_labels(2)  = 'step_pos_1deg';
seg_labels(3)  = 'recovery';
seg_labels(4)  = 'step_neg_1deg';
seg_labels(5)  = 'recovery';
seg_labels(6)  = 'pulse_pos_1deg';
seg_labels(7)  = 'recovery';
for k = 1:numel(sine_freqs_Hz)
    seg_labels(9+k) = sprintf('sine_%.2fHz', sine_freqs_Hz(k));
end
seg_labels(30) = 'final_recovery';

%% Build segments in order
S = {};   % each entry: struct(n, r_rad, seg_id)

append_const = @(dur, amp_rad, sid) struct( ...
    'n',     max(1, ceil(dur/Ts)), ...
    'r_rad', amp_rad*ones(max(1, ceil(dur/Ts)),1), ...
    'seg_id', sid);

% Seg 0: prep
S{end+1} = append_const(prep_s,     0, 0);
% Seg 1: baseline
S{end+1} = append_const(baseline_s, 0, 1);
% Seg 2: +1 deg step
S{end+1} = append_const(step_dur_s,  r_step_deg*deg2rad_, 2);
% Seg 3: recovery
S{end+1} = append_const(recovery_s,  0, 3);
% Seg 4: -1 deg step
S{end+1} = append_const(step_dur_s, -r_step_deg*deg2rad_, 4);
% Seg 5: recovery
S{end+1} = append_const(recovery_s,  0, 5);

% Seg 6: +1 deg pulse (short HIGH then LOW, all tagged seg 6)
n_hi = max(1, ceil(pulse_width_s/Ts));
n_lo = max(1, ceil(pulse_settle_s/Ts));
S{end+1} = struct( ...
    'n',     n_hi + n_lo, ...
    'r_rad', [r_pulse_deg*deg2rad_*ones(n_hi,1); zeros(n_lo,1)], ...
    'seg_id', 6);

% Seg 7: recovery
S{end+1} = append_const(recovery_s, 0, 7);

% Segs 10..21: stepped sines (0.5 deg, 7 cycles ea, tapered first/last cycle)
for k = 1:numel(sine_freqs_Hz)
    f_k = sine_freqs_Hz(k);
    T_cyc = 1/f_k;
    n_seg = max(1, ceil(n_cycles*T_cyc/Ts));
    t_loc = (0:n_seg-1).' * Ts;
    raw   = r_sine_deg*deg2rad_ * sin(2*pi*f_k * t_loc);

    env   = ones(n_seg,1);
    n_ramp = max(1, ceil(T_cyc/Ts));
    % ramp-in (cycle 1): half-cosine 0 -> 1
    n_in = min(n_ramp, n_seg);
    env(1:n_in) = 0.5*(1 - cos(pi*(0:n_in-1).'/n_in));
    % ramp-out (cycle 7): half-cosine 1 -> 0
    n_out_start = max(1, n_seg - n_ramp + 1);
    n_out       = n_seg - n_out_start + 1;
    env(n_out_start:n_seg) = 0.5*(1 + cos(pi*(0:n_out-1).'/n_out));

    S{end+1} = struct( ...
        'n',     n_seg, ...
        'r_rad', raw .* env, ...
        'seg_id', 9 + k);   %#ok<AGROW>
end

% Seg 30: final recovery
S{end+1} = append_const(final_rec_s, 0, 30);

%% Concatenate into single timeseries
total_n  = sum(cellfun(@(s) s.n, S));
t_vec    = zeros(total_n,1);
r_vec    = zeros(total_n,1);
seg_vec  = zeros(total_n,1);

idx = 1;
t_cursor = 0;
seg_table_cells = {};
for k = 1:numel(S)
    s = S{k};
    n = s.n;
    t_local = t_cursor + (0:n-1).'*Ts;
    t_vec(idx:idx+n-1)   = t_local;
    r_vec(idx:idx+n-1)   = s.r_rad;
    seg_vec(idx:idx+n-1) = s.seg_id;

    seg_table_cells(end+1, :) = { ...
        s.seg_id, t_cursor, t_cursor + n*Ts, seg_labels(s.seg_id) }; %#ok<AGROW>

    idx      = idx + n;
    t_cursor = t_cursor + n*Ts;
end

% Append a trailing sample so FromWorkspace holds the last value cleanly
t_vec(end+1)   = t_vec(end) + Ts;
r_vec(end+1)   = 0;
seg_vec(end+1) = 30;

r_theta_ts    = [t_vec, r_vec];      % FromWorkspace format
segment_id_ts = [t_vec, seg_vec];
total_duration_s = t_vec(end);

%% Print summary
fprintf('Tracking protocol generated.\n');
fprintf('  Sample rate     : %d Hz (Ts = %.3f ms)\n', exp_fs_Hz, Ts*1e3);
fprintf('  Total duration  : %.1f s (%.2f min)\n', total_duration_s, total_duration_s/60);
fprintf('  Step amplitude  : %.2f deg\n', r_step_deg);
fprintf('  Pulse amplitude : %.2f deg, width %.2f s\n', r_pulse_deg, pulse_width_s);
fprintf('  Sine amplitude  : %.2f deg, %d cycles per freq\n', r_sine_deg, n_cycles);
fprintf('  Sine frequencies: %s Hz\n', num2str(sine_freqs_Hz, '%.2f '));
fprintf('\n  Segment table:\n');
fprintf('  %4s | %8s | %8s | %s\n', 'ID', 't_start', 't_end', 'label');
for k = 1:size(seg_table_cells,1)
    fprintf('  %4d | %8.2f | %8.2f | %s\n', ...
        seg_table_cells{k,1}, seg_table_cells{k,2}, seg_table_cells{k,3}, seg_table_cells{k,4});
end

%% Save
seesaw_root = getenv('SEESAW_ROOT');
if isempty(seesaw_root)
    seesaw_root = pwd;
end
out_dir = fullfile(seesaw_root, 'validation', 'data');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
out_path = fullfile(out_dir, 'tracking_protocol.mat');

seg_table = seg_table_cells; %#ok<NASGU>
save(out_path, 'r_theta_ts', 'segment_id_ts', 'seg_table', ...
    'sine_freqs_Hz', 'r_step_deg', 'r_pulse_deg', 'r_sine_deg', ...
    'exp_fs_Hz', 'total_duration_s');
fprintf('\n  Saved: %s\n', out_path);

% Push to base workspace so the _test.slx models can resolve immediately
assignin('base', 'r_theta_ts',    r_theta_ts);
assignin('base', 'segment_id_ts', segment_id_ts);
assignin('base', 'tracking_total_duration_s', total_duration_s);
end
