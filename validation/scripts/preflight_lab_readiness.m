function report = preflight_lab_readiness(varargin)
%PREFLIGHT_LAB_READINESS Fail-fast checks before relying on validation models.
%
% Recommended lab command:
%   startup
%   preflight_lab_readiness('level','step')
%
% Levels:
%   smoke  - compile all quick cases and simulate 2 s
%   step   - compile all quick cases and simulate through the first +theta step
%   full   - run the full reference protocol for all quick cases
%
% Case profiles:
%   lab_minimum - only cases that must be safe before lab use: PP and SMC
%   quick       - PP, LQI, LQG, PID, SMC

opts = parse_inputs(varargin{:});
report = struct();
report.ok = false;
report.level = opts.level;
report.case_profile = opts.case_profile;
report.root = opts.root;
report.generated_at = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
report.checks = struct('name', {}, 'status', {}, 'message', {});

fprintf('\n=== Seesaw Lab Readiness Preflight (%s, %s) ===\n', opts.level, opts.case_profile);

try
    run_startup(opts.root);
    record('startup', 'PASS', 'startup.m completed');
catch ME
    record('startup', 'FAIL', ME.message);
    finish();
    return;
end

check_file('protocol_script', fullfile(opts.root, 'validation', 'scripts', 'generate_validation_protocol.m'));
check_file('case_loader', fullfile(opts.root, 'validation', 'scripts', 'load_controller_validation_case.m'));
check_file('harness_builder', fullfile(opts.root, 'validation', 'scripts', 'build_controller_validation_harness.m'));
check_file('tuned_params', fullfile(opts.root, 'data', 'tuned', 'tuned_params.mat'));
check_file('pp_controller_data', fullfile(opts.root, 'data', 'controllers', 'controller_freq.mat'));
check_model_file('fallback_pp_hardware_model', fullfile('models', 'controllers', 'pole_placement', 'PolePlacementObserver2024.slx'));
check_model_file('fallback_validation_hardware_model', fullfile('validation', 'models', 'HardwareValidation_HWTest_2024_Proper.slx'));
check_quarc();

try
    protocol = generate_validation_protocol('root', opts.root);
    assert(isfield(protocol, 'r_theta_ts') && isfield(protocol, 'segment_id_ts'), 'Protocol timeseries fields are missing.');
    assert(all(diff(protocol.t) > 0), 'Protocol time vector is not strictly increasing.');
    required_segments = [1 2 3 4 5 6 7 10:21 30];
    missing_segments = setdiff(required_segments, unique(protocol.segment_id(:))');
    assert(isempty(missing_segments), 'Protocol is missing segment IDs: %s', mat2str(missing_segments));
    record('reference_protocol', 'PASS', sprintf('%.1f s protocol, %d samples', protocol.t(end), numel(protocol.t)));
catch ME
    record('reference_protocol', 'FAIL', ME.message);
end

stop_time_s = choose_stop_time(opts.level, protocol);
cases = preflight_cases(opts.case_profile);
for k = 1:numel(cases)
    controller_id = cases(k).controller_id;
    feedback_source_id = cases(k).feedback_source_id;
    case_name = sprintf('%s_%s', controller_id, feedback_source_id);
    try
        build_controller_validation_harness('root', opts.root, ...
            'controller_id', controller_id, ...
            'feedback_source_id', feedback_source_id);
        cfg = load_controller_validation_case(controller_id, feedback_source_id, 'root', opts.root);
        mdl = 'ControllerValidationHarness';
        set_param(mdl, 'SimulationCommand', 'update');
        check_goto_tags(mdl, case_name);

        in = Simulink.SimulationInput(mdl);
        in = in.setModelParameter('StopTime', num2str(stop_time_s));
        out = sim(in);
        log = out.get('validation_log');
        validate_log(log, cfg, case_name);
        record(['simulate_' case_name], 'PASS', sprintf('ran %.1f s without NaN/Inf or voltage-limit violation', log.Time(end)));
    catch ME
        record(['simulate_' case_name], 'FAIL', ME.message);
        if opts.fail_fast
            break;
        end
    end
end

finish();

    function record(name, status, message)
        entry = struct('name', char(name), 'status', char(status), 'message', char(message));
        report.checks(end+1) = entry;
        fprintf('[%s] %s - %s\n', entry.status, entry.name, entry.message);
    end

    function check_file(name, file_path)
        if exist(file_path, 'file') == 2
            record(name, 'PASS', file_path);
        else
            record(name, 'FAIL', ['Missing file: ' file_path]);
        end
    end

    function check_model_file(name, rel_path)
        file_path = fullfile(opts.root, rel_path);
        if exist(file_path, 'file') == 2
            record(name, 'PASS', file_path);
            return;
        end
        alt = which(rel_path);
        if ~isempty(alt)
            record(name, 'PASS', alt);
        else
            record(name, 'FAIL', ['Missing file: ' file_path]);
        end
    end

    function check_quarc()
        has_quarc_callback = exist('qc_set_target_type', 'file') == 2;
        if has_quarc_callback
            record('quarc_callback', 'PASS', which('qc_set_target_type'));
        elseif opts.require_quarc
            record('quarc_callback', 'FAIL', 'qc_set_target_type is unavailable; hardware models will not configure QUARC target callbacks.');
        else
            record('quarc_callback', 'WARN', 'qc_set_target_type is unavailable in this MATLAB session; simulation checks can run, QUARC deployability must be checked on the lab PC.');
        end
    end

    function check_goto_tags(mdl, case_name)
        goto_blocks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Goto');
        visibilities = get_param(goto_blocks, 'TagVisibility');
        if ischar(visibilities)
            visibilities = {visibilities};
        end
        if all(strcmp(visibilities, 'local'))
            record(['local_goto_tags_' case_name], 'PASS', sprintf('%d local tags', numel(goto_blocks)));
        else
            record(['local_goto_tags_' case_name], 'FAIL', 'One or more Goto tags are not local.');
        end
    end

    function validate_log(log, cfg, case_name)
        assert(isa(log, 'timeseries'), 'validation_log was not returned as a timeseries.');
        data = log.Data;
        assert(size(data, 2) >= numel(cfg.validation_log_columns), ...
            'validation_log has %d columns, expected at least %d.', size(data, 2), numel(cfg.validation_log_columns));
        assert(all(isfinite(data(:))), 'validation_log contains NaN or Inf.');

        [V_m_idx, theta_idx] = validation_log_indices(cfg.validation_log_columns);
        V_m = squeeze(data(:, V_m_idx));
        theta = squeeze(data(:, theta_idx));
        assert(max(abs(V_m)) <= cfg.V_sat_hw + 1e-9, ...
            'V_m exceeded %.2f V in %s.', cfg.V_sat_hw, case_name);
        max_theta_deg = rad2deg(max(abs(theta)));
        assert(max(abs(theta)) <= deg2rad(11.5), ...
            'theta exceeded physical stop estimate in %s: %.2f deg.', case_name, max_theta_deg);
        record(['theta_range_' case_name], 'PASS', sprintf('max |theta| %.2f deg', max_theta_deg));
    end

    function finish()
        statuses = {report.checks.status};
        report.ok = ~any(strcmp(statuses, 'FAIL'));
        out_dir = fullfile(opts.root, 'validation', 'data', 'preflight');
        if exist(out_dir, 'dir') ~= 7
            mkdir(out_dir);
        end
        report_stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
        report_file = fullfile(out_dir, ['preflight_' report_stamp '.mat']);
        save(report_file, 'report');
        fprintf('Saved preflight report: %s\n', report_file);
        if report.ok
            fprintf('PREFLIGHT PASS\n');
        else
            fprintf('PREFLIGHT FAIL\n');
            if opts.throw_on_fail
                error('preflight_lab_readiness:Failed', 'Preflight failed. Do not rely on the generated validation path until failures are fixed.');
            end
        end
    end
end

function cases = preflight_cases(case_profile)
switch lower(string(case_profile))
    case "lab_minimum"
        rows = {
            'PP', 'measured'
            'SMC', 'measured'};
    case "quick"
        rows = {
            'PP', 'measured'
            'LQI', 'measured'
            'LQG', 'kalman'
            'PID', 'measured'
            'SMC', 'measured'};
    otherwise
        error('Unknown preflight case_profile: %s', case_profile);
end
cases = struct('controller_id', {}, 'feedback_source_id', {});
for i = 1:size(rows, 1)
    cases(i).controller_id = rows{i, 1};
    cases(i).feedback_source_id = rows{i, 2};
end
end

function stop_time_s = choose_stop_time(level, protocol)
switch lower(string(level))
    case "smoke"
        stop_time_s = 2;
    case "step"
        stop_time_s = 25;
    case "full"
        stop_time_s = protocol.t(end);
    otherwise
        error('Unknown preflight level: %s', level);
end
end

function run_startup(root)
startup_file = fullfile(root, 'startup.m');
assignin('base', 'preflight_startup_file', startup_file);
evalin('base', 'run(preflight_startup_file); clear preflight_startup_file');
end

function opts = parse_inputs(varargin)
if evalin('base', 'exist(''SEESAW_ROOT'', ''var'')')
    default_root = evalin('base', 'SEESAW_ROOT');
else
    default_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end

p = inputParser;
addParameter(p, 'root', default_root, @(x) ischar(x) || isstring(x));
addParameter(p, 'level', 'smoke', @(x) any(strcmpi(string(x), ["smoke", "step", "full"])));
addParameter(p, 'case_profile', 'lab_minimum', @(x) any(strcmpi(string(x), ["lab_minimum", "quick"])));
addParameter(p, 'require_quarc', false, @islogical);
addParameter(p, 'fail_fast', false, @islogical);
addParameter(p, 'throw_on_fail', true, @islogical);
parse(p, varargin{:});
opts = p.Results;
opts.root = char(opts.root);
opts.level = char(lower(string(opts.level)));
opts.case_profile = char(lower(string(opts.case_profile)));
end

function [V_m_idx, theta_idx] = validation_log_indices(columns)
widths = zeros(1, numel(columns));
for i = 1:numel(columns)
    switch columns{i}
        case {'x_ref', 'x_feedback', 'x_measured', 'x_luenberger', 'x_kalman', 'x_state'}
            widths(i) = 4;
        otherwise
            widths(i) = 1;
    end
end
starts = [1 1 + cumsum(widths(1:end-1))];
V_m_idx = starts(strcmp(columns, 'V_m'));
theta_idx = starts(strcmp(columns, 'theta'));
end
