function results = run_controller_validation_suite(varargin)
%RUN_CONTROLLER_VALIDATION_SUITE Run configured theta-tracking simulations.

opts = parse_inputs(varargin{:});
ensure_ready(opts.root);

protocol = generate_validation_protocol('root', opts.root);
cases = default_cases(opts.quick);

out_dir = fullfile(opts.root, 'validation', 'data', 'sim_results');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

results = struct('case_id', {}, 'controller_id', {}, 'feedback_source_id', {}, ...
    'result_file', {}, 'status', {}, 'message', {});

for k = 1:numel(cases)
    c = cases(k);
    case_id = sprintf('%s_%s_theta', c.controller_id, c.feedback_source_id);
    fprintf('\n[%d/%d] Running %s\n', k, numel(cases), case_id);
    try
        build_controller_validation_harness('root', opts.root, ...
            'controller_id', c.controller_id, ...
            'feedback_source_id', c.feedback_source_id);
        cfg = load_controller_validation_case(c.controller_id, c.feedback_source_id, 'root', opts.root);
        in = Simulink.SimulationInput('ControllerValidationHarness');
        in = in.setModelParameter('StopTime', num2str(protocol.t(end)));
        out = sim(in);
        result_file = fullfile(out_dir, [case_id '.mat']);
        validation_log_columns = cfg.validation_log_columns;
        save(result_file, 'out', 'c', 'cfg', 'protocol', 'validation_log_columns', '-v7.3');
        results(end+1) = make_result(case_id, c, result_file, 'ok', ''); %#ok<AGROW>
    catch ME
        results(end+1) = make_result(case_id, c, '', 'failed', ME.message); %#ok<AGROW>
        warning('Case %s failed: %s', case_id, ME.message);
        if opts.stop_on_failure
            rethrow(ME);
        end
    end
end

summary_file = fullfile(out_dir, 'suite_results_index.mat');
save(summary_file, 'results', 'cases');
fprintf('\nSaved suite index: %s\n', summary_file);
end

function cases = default_cases(quick)
if quick
    rows = {
        'PP', 'measured'
        'LQI', 'measured'
        'LQG', 'kalman'
        'PID', 'measured'
        'SMC', 'measured'};
else
    rows = {
        'PP', 'measured'
        'PP', 'luenberger'
        'PP', 'kalman'
        'LQI', 'measured'
        'LQG', 'kalman'
        'PID', 'measured'
        'PID', 'kalman'
        'SMC', 'measured'
        'SMC', 'kalman'};
end
cases = struct('controller_id', {}, 'feedback_source_id', {});
for i = 1:size(rows, 1)
    cases(i).controller_id = rows{i, 1};
    cases(i).feedback_source_id = rows{i, 2};
end
end

function r = make_result(case_id, c, result_file, status, message)
r = struct('case_id', case_id, 'controller_id', c.controller_id, ...
    'feedback_source_id', c.feedback_source_id, 'result_file', result_file, ...
    'status', status, 'message', message);
end

function ensure_ready(root)
if evalin('base', 'exist(''SEESAW_ROOT'', ''var'')') == 0 || ~strcmp(evalin('base', 'SEESAW_ROOT'), root)
    run(fullfile(root, 'startup.m'));
end
model_file = fullfile(root, 'validation', 'models', 'ControllerValidationHarness.slx');
if exist(model_file, 'file') ~= 2
    build_controller_validation_harness;
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
addParameter(p, 'quick', false, @islogical);
addParameter(p, 'stop_on_failure', false, @islogical);
parse(p, varargin{:});
opts = p.Results;
opts.root = char(opts.root);
end
