function protocol = generate_validation_protocol(varargin)
%GENERATE_VALIDATION_PROTOCOL Build the theta-reference validation signal.
%
% The protocol is the common input for controller validation:
%   r_theta(t) -> theta(t)
%
% It contains free-run, positive/negative theta steps, a short pulse, and a
% stepped sine sweep. Cart position is never commanded externally.

opts = parse_inputs(varargin{:});

dt = opts.sample_time_s;
t = [];
r = [];
segment_id = [];
segment_frequency_hz = [];
analyze_window = [];
segments = struct('id', {}, 'name', {}, 't_start', {}, 't_end', {}, ...
    'frequency_hz', {}, 'analyze_t_start', {}, 'analyze_t_end', {});

append_constant(1, "free_run", opts.free_run_s, 0, NaN, NaN, NaN);
append_constant(2, "positive_theta_step", opts.step_s, opts.theta_step_rad, NaN, NaN, NaN);
append_constant(3, "recovery_after_positive_step", opts.recovery_s, 0, NaN, NaN, NaN);
append_constant(4, "negative_theta_step", opts.step_s, -opts.theta_step_rad, NaN, NaN, NaN);
append_constant(5, "recovery_after_negative_step", opts.recovery_s, 0, NaN, NaN, NaN);
append_constant(6, "positive_theta_pulse", opts.pulse_s, opts.theta_step_rad, NaN, NaN, NaN);
append_constant(7, "recovery_after_pulse", opts.pulse_recovery_s, 0, NaN, NaN, NaN);

freqs = opts.sine_frequencies_hz(:)';
for k = 1:numel(freqs)
    f = freqs(k);
    id = 9 + k;
    period = 1 / f;
    duration = opts.sine_cycles_per_frequency * period;
    local_t = (0:dt:duration-dt)';
    cycle_phase = local_t / period;

    envelope = ones(size(local_t));
    ramp_in = cycle_phase < 1;
    ramp_out = cycle_phase >= opts.sine_cycles_per_frequency - 1;
    envelope(ramp_in) = 0.5 - 0.5*cos(pi * cycle_phase(ramp_in));
    envelope(ramp_out) = 0.5 + 0.5*cos(pi * (cycle_phase(ramp_out) - (opts.sine_cycles_per_frequency - 1)));

    local_r = opts.theta_sine_rad * envelope .* sin(2*pi*f*local_t);
    analyze_start = current_time() + opts.sine_discard_cycles * period;
    analyze_end = current_time() + (opts.sine_discard_cycles + opts.sine_analyze_cycles) * period;
    append_series(id, sprintf('sine_%.2f_hz', f), local_t, local_r, f, analyze_start, analyze_end);
end

append_constant(30, "final_recovery", opts.final_recovery_s, 0, NaN, NaN, NaN);

protocol = struct();
protocol.sample_time_s = dt;
protocol.t = t;
protocol.r_theta = r;
protocol.segment_id = segment_id;
protocol.segment_frequency_hz = segment_frequency_hz;
protocol.analyze_window = analyze_window;
protocol.segments = segments;
protocol.r_theta_ts = timeseries(r, t, 'Name', 'r_theta');
protocol.segment_id_ts = timeseries(segment_id, t, 'Name', 'segment_id');

out_dir = fullfile(opts.root, 'validation', 'data', 'protocol');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
out_file = fullfile(out_dir, 'reference_tracking_protocol.mat');

t_protocol = t;
r_theta = r;
r_theta_ts = protocol.r_theta_ts;
segment_id_ts = protocol.segment_id_ts;
save(out_file, 'protocol', 't_protocol', 'r_theta', 'r_theta_ts', 'segment_id_ts');
fprintf('Saved theta-reference validation protocol: %s\n', out_file);

    function append_constant(id, name, duration, value, frequency_hz, analyze_start, analyze_end)
        local_t = (0:dt:duration-dt)';
        local_r = value * ones(size(local_t));
        append_series(id, name, local_t, local_r, frequency_hz, analyze_start, analyze_end);
    end

    function append_series(id, name, local_t, local_r, frequency_hz, analyze_start, analyze_end)
        t0 = current_time();
        t_abs = t0 + local_t;
        t = [t; t_abs];
        r = [r; local_r(:)];
        segment_id = [segment_id; id * ones(numel(local_t), 1)];
        segment_frequency_hz = [segment_frequency_hz; frequency_hz * ones(numel(local_t), 1)];
        analyze_window = [analyze_window; local_t*0 + double(t_abs >= analyze_start & t_abs < analyze_end)];
        segments(end+1) = struct( ...
            'id', id, 'name', char(name), 't_start', t0, 't_end', t0 + local_t(end) + dt, ...
            'frequency_hz', frequency_hz, 'analyze_t_start', analyze_start, 'analyze_t_end', analyze_end);
    end

    function val = current_time()
        if isempty(t)
            val = 0;
        else
            val = t(end) + dt;
        end
    end
end

function opts = parse_inputs(varargin)
if evalin('base', 'exist(''SEESAW_ROOT'', ''var'')')
    default_root = evalin('base', 'SEESAW_ROOT');
else
    default_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end

p = inputParser;
addParameter(p, 'root', default_root, @(x) ischar(x) || isstring(x));
addParameter(p, 'sample_time_s', 0.002, @isscalar);
addParameter(p, 'theta_step_rad', deg2rad(1.0), @isscalar);
addParameter(p, 'theta_sine_rad', deg2rad(1.0), @isscalar);
addParameter(p, 'free_run_s', 10, @isscalar);
addParameter(p, 'step_s', 15, @isscalar);
addParameter(p, 'recovery_s', 15, @isscalar);
addParameter(p, 'pulse_s', 0.3, @isscalar);
addParameter(p, 'pulse_recovery_s', 10, @isscalar);
addParameter(p, 'final_recovery_s', 10, @isscalar);
addParameter(p, 'sine_frequencies_hz', [0.10 0.16 0.25 0.40 0.63 1.00 1.60 2.50 4.00 6.30 8.00 10.00], @isnumeric);
addParameter(p, 'sine_cycles_per_frequency', 7, @isscalar);
addParameter(p, 'sine_discard_cycles', 3, @isscalar);
addParameter(p, 'sine_analyze_cycles', 3, @isscalar);
parse(p, varargin{:});
opts = p.Results;
opts.root = char(opts.root);
end
