function report = prebuild_required_controllers(varargin)
%PREBUILD_REQUIRED_CONTROLLERS Stage all controllers required by the Seesaw brief.
%
% Requirements source:
%   docs/references/Requirements and Timeline.pdf, slides 41-42
%
% Seesaw objectives extracted from the brief:
%   1. Cart position control (FB)
%   2. Body horizontal stabilization (FB & SS)
%   3. Periodic body trajectory tracking / lift-up (SS & AC)
%
% This script stages the controller/observer artifacts consumed by the
% generated validation harness, compiles the supported cases, and runs the
% lab-minimum readiness preflight.

opts = parse_inputs(varargin{:});
run_startup(opts.root);
tune_validation_case('verbose', false);

report = struct();
report.ok = false;
report.root = opts.root;
report.generated_at = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
report.requirements_source = fullfile(opts.root, 'docs', 'references', 'Requirements and Timeline.pdf');
report.requirements = seesaw_requirements();
report.artifacts = struct('id', {}, 'requirement', {}, 'file', {}, 'status', {}, 'message', {});
report.cases = struct('case_id', {}, 'controller_id', {}, 'feedback_source_id', {}, 'status', {}, 'message', {});

fprintf('\n=== Prebuild Required Seesaw Controllers ===\n');

artifact_specs = required_artifacts();
prebuilt = struct();
prebuilt.generated_at = report.generated_at;
prebuilt.requirements = report.requirements;

for i = 1:numel(artifact_specs)
    spec = artifact_specs(i);
    [file_path, status, message] = stage_artifact(opts.root, spec);
    entry = struct('id', spec.id, 'requirement', spec.requirement, ...
        'file', file_path, 'status', status, 'message', message);
    report.artifacts(end+1) = entry;
    fprintf('[%s] %s - %s\n', status, spec.id, message);

    if ~isempty(file_path) && (strcmp(status, 'PASS') || strcmp(status, 'WARN'))
        prebuilt.(spec.id) = load(file_path);
    end
end

generate_validation_protocol('root', opts.root);

cases = required_validation_cases();
for i = 1:numel(cases)
    c = cases(i);
    case_id = sprintf('%s_%s', lower(c.controller_id), lower(c.feedback_source_id));
    try
        load_controller_validation_case(c.controller_id, c.feedback_source_id, 'root', opts.root);
        build_controller_validation_harness('root', opts.root, ...
            'controller_id', c.controller_id, ...
            'feedback_source_id', c.feedback_source_id);
        set_param('ControllerValidationHarness', 'SimulationCommand', 'update');
        report.cases(end+1) = make_case_result(case_id, c, 'PASS', 'compiled generated harness case');
        fprintf('[PASS] compile_%s - generated harness compiles\n', case_id);
    catch ME
        report.cases(end+1) = make_case_result(case_id, c, 'FAIL', ME.message);
        fprintf('[FAIL] compile_%s - %s\n', case_id, ME.message);
        if opts.fail_fast
            break;
        end
    end
end

if opts.run_preflight
    report.lab_preflight = preflight_lab_readiness('root', opts.root, ...
        'level', 'step', 'case_profile', 'lab_minimum', 'throw_on_fail', false);
else
    report.lab_preflight = [];
end

out_dir = fullfile(opts.root, 'validation', 'data', 'prebuilt');
if exist(out_dir, 'dir') ~= 7
    mkdir(out_dir);
end
artifact_statuses = {report.artifacts.status};
case_statuses = {report.cases.status};
report.ok = ~any(strcmp(artifact_statuses, 'FAIL')) && ~any(strcmp(case_statuses, 'FAIL'));
if isstruct(report.lab_preflight)
    report.ok = report.ok && report.lab_preflight.ok;
end

prebuilt_file = fullfile(out_dir, 'required_controllers.mat');
report_file = fullfile(out_dir, ['prebuild_report_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) '.mat']);
save(prebuilt_file, 'prebuilt', '-v7.3');
save(report_file, 'report', '-v7.3');

fprintf('Saved prebuilt controllers: %s\n', prebuilt_file);
fprintf('Saved prebuild report: %s\n', report_file);
if report.ok
    fprintf('PREBUILD PASS\n');
else
    fprintf('PREBUILD FAIL\n');
    if opts.throw_on_fail
        error('prebuild_required_controllers:Failed', 'Required controller prebuild failed. Review report before lab use.');
    end
end
end

function requirements = seesaw_requirements()
requirements = struct('id', {}, 'text', {}, 'technique', {});
requirements(1) = struct('id', 'seesaw_cart_fb', ...
    'text', 'Position control for the cart', 'technique', 'FB');
requirements(2) = struct('id', 'seesaw_body_stabilization', ...
    'text', 'Stabilization of the body in the horizontal position', 'technique', 'FB & SS');
requirements(3) = struct('id', 'seesaw_body_tracking_liftup', ...
    'text', 'Periodic trajectory tracking for the body and lift-up', 'technique', 'SS & AC');
end

function specs = required_artifacts()
specs = struct('id', {}, 'requirement', {}, 'rel_paths', {}, 'fields', {}, 'optional', {});
specs(end+1) = spec('tuned_plant', 'all', ...
    {fullfile('data','tuned','tuned_params.mat'), fullfile('data','tuned_params.mat')}, ...
    {'A_sw','B_sw','B_eq'}, false);
specs(end+1) = spec('nonlinear_params', 'all', ...
    {fullfile('data','params','param_nonlinear.mat'), fullfile('data','param_nonlinear.mat')}, ...
    {'ud_pos','ud_neg'}, false);
specs(end+1) = spec('cart_fb_inner_pid', 'seesaw_cart_fb', ...
    {fullfile('data','controllers','controller_inner_pid.mat'), fullfile('data','controller_inner_pid.mat')}, ...
    {'Kp_in','Ki_in','Kd_in','N_in'}, false);
specs(end+1) = spec('body_fb_outer_pid', 'seesaw_body_stabilization', ...
    {fullfile('data','controllers','controller_outer_pid.mat'), fullfile('data','controller_outer_pid.mat')}, ...
    {'Kp_out','Kd_out','N_out'}, false);
specs(end+1) = spec('body_fb_pole_placement', 'seesaw_body_stabilization', ...
    {fullfile('data','controllers','controller_freq.mat'), fullfile('data','controller_freq.mat')}, ...
    {'Kf'}, false);
specs(end+1) = spec('body_ss_lqr_lqi', 'seesaw_body_stabilization', ...
    {fullfile('data','controllers','controller_lqr.mat')}, ...
    {'K_lqr'}, false);
specs(end+1) = spec('body_ss_luenberger', 'seesaw_body_stabilization', ...
    {fullfile('data','params','observer.mat'), fullfile('data','observer.mat')}, ...
    {'A_obs','B_obs','C_obs','D_obs'}, false);
specs(end+1) = spec('body_ss_kalman', 'seesaw_body_stabilization', ...
    {fullfile('data','params','observer_kalman.mat'), fullfile('data','params','kalman_observer.mat')}, ...
    {'A_obs','B_obs','C_obs','D_obs'}, false);
specs(end+1) = spec('body_ac_smc', 'seesaw_body_tracking_liftup', ...
    {fullfile('data','controllers','controller_smc.mat'), fullfile('data','controller_smc.mat')}, ...
    {'surface.S','surface.K_eq','sta_design.k1','sta_design.k2','sta_design.phi_bl'}, false);
specs(end+1) = spec('body_ac_mpc_optional', 'seesaw_body_tracking_liftup', ...
    {fullfile('data','controllers','controller_mpc.mat')}, ...
    {}, true);
specs(end+1) = spec('liftup_optional', 'seesaw_body_tracking_liftup', ...
    {fullfile('data','params','liftoff_params.mat')}, ...
    {}, true);
end

function s = spec(id, requirement, rel_paths, fields, optional)
s = struct('id', id, 'requirement', requirement, 'rel_paths', {rel_paths}, ...
    'fields', {fields}, 'optional', optional);
end

function cases = required_validation_cases()
rows = {
    'PP', 'measured'
    'LQI', 'measured'
    'LQG', 'kalman'
    'PID', 'measured'
    'SMC', 'measured'};
cases = struct('controller_id', {}, 'feedback_source_id', {});
for i = 1:size(rows, 1)
    cases(i).controller_id = rows{i, 1};
    cases(i).feedback_source_id = rows{i, 2};
end
end

function [file_path, status, message] = stage_artifact(root, spec)
file_path = '';
for i = 1:numel(spec.rel_paths)
    candidate = fullfile(root, spec.rel_paths{i});
    if exist(candidate, 'file') == 2
        file_path = candidate;
        break;
    end
end

if isempty(file_path)
    status = ternary(spec.optional, 'WARN', 'FAIL');
    message = ['missing artifact: ' spec.rel_paths{1}];
    return;
end

try
    data = load(file_path);
    missing = missing_fields(data, spec.fields);
    if isempty(missing)
        status = 'PASS';
        message = file_path;
    else
        status = ternary(spec.optional, 'WARN', 'FAIL');
        message = ['missing fields: ' strjoin(missing, ', ')];
    end
catch ME
    status = ternary(spec.optional, 'WARN', 'FAIL');
    message = ME.message;
end
end

function missing = missing_fields(data, fields)
missing = {};
for i = 1:numel(fields)
    if ~has_field_path(data, fields{i})
        missing{end+1} = fields{i}; %#ok<AGROW>
    end
end
end

function ok = has_field_path(data, field_path)
parts = split(string(field_path), '.');
val = data;
ok = true;
for i = 1:numel(parts)
    p = char(parts(i));
    if isstruct(val) && isfield(val, p)
        val = val.(p);
    else
        ok = false;
        return;
    end
end
end

function result = make_case_result(case_id, c, status, message)
result = struct('case_id', case_id, 'controller_id', c.controller_id, ...
    'feedback_source_id', c.feedback_source_id, 'status', status, 'message', message);
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function run_startup(root)
startup_file = fullfile(root, 'startup.m');
assignin('base', 'prebuild_startup_file', startup_file);
evalin('base', 'run(prebuild_startup_file); clear prebuild_startup_file');
end

function opts = parse_inputs(varargin)
if evalin('base', 'exist(''SEESAW_ROOT'', ''var'')')
    default_root = evalin('base', 'SEESAW_ROOT');
else
    default_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
p = inputParser;
addParameter(p, 'root', default_root, @(x) ischar(x) || isstring(x));
addParameter(p, 'run_preflight', true, @islogical);
addParameter(p, 'fail_fast', false, @islogical);
addParameter(p, 'throw_on_fail', false, @islogical);
parse(p, varargin{:});
opts = p.Results;
opts.root = char(opts.root);
end
