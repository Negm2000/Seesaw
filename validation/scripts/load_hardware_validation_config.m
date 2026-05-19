function cfg = load_hardware_validation_config(controller_name, observer_name)
%LOAD_HARDWARE_VALIDATION_CONFIG Configure the generic HW validation model.
%
%   load_hardware_validation_config('pole_placement', 'none')
%   load_hardware_validation_config('lqr', 'leuenberger')
%   load_hardware_validation_config('lqi_discrete', 'kalman_discrete')
%   load_hardware_validation_config('pid', 'kalman')
%
% The Simulink model uses a fixed interface:
%   controller input: x_fb = [x_c; x_c_dot; alpha; alpha_dot]
%   augmented input:  [x_fb; integral(alpha)]
%   observer input:   [V_m; x_c; alpha]
%
% Final-model controller data uses [x_c; alpha; x_c_dot; alpha_dot].
% This loader reorders final-model gains and observers to the validation
% model interface above.

if nargin < 1 || isempty(controller_name)
    controller_name = 'pole_placement';
end
if nargin < 2 || isempty(observer_name)
    observer_name = 'none';
end

controller_name = validatestring(lower(controller_name), ...
    {'pole_placement', 'pp', 'lqr', 'lqi', 'lqr_discrete', ...
     'lqi_discrete', 'lqd', 'pid'}, mfilename, 'controller_name');
observer_name = validatestring(lower(observer_name), ...
    {'none', 'raw', 'leuenberger', 'kalman', 'kalman_discrete', 'dlqe'}, ...
    mfilename, 'observer_name');

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
run_project_startup(root);

cfg = struct();
cfg.controller_name = char(controller_name);
cfg.observer_name = char(observer_name);

switch char(controller_name)
    case {'pole_placement', 'pp'}
        [K_aug_hw, K_hw, ctrl_file] = load_controller_gain(root, ...
            {'data/pole_placement.mat', 'data/controller_freq.mat'}, ...
            {'K4d', 'K4', 'Kf'}, false);
        cfg.controller_display_name = 'Pole placement state feedback';

    case 'lqr'
        [K_aug_hw, K_hw, ctrl_file] = load_controller_gain(root, ...
            {'data/controller_lqr.mat'}, {'K4', 'K_lqr'}, false);
        cfg.controller_display_name = 'LQR state feedback';

    case 'lqi'
        [K_aug_hw, K_hw, ctrl_file] = load_controller_gain(root, ...
            {'data/controller_lqr.mat'}, {'K5', 'K_lqr'}, true);
        cfg.controller_display_name = 'LQI augmented state feedback';

    case {'lqr_discrete', 'lqd'}
        [K_aug_hw, K_hw, ctrl_file] = load_controller_gain(root, ...
            {'data/controller_lqr.mat'}, {'K4d', 'K4', 'K_lqr'}, false);
        cfg.controller_display_name = 'Discrete LQR state feedback';

    case 'lqi_discrete'
        [K_aug_hw, K_hw, ctrl_file] = load_controller_gain(root, ...
            {'data/controller_lqr.mat'}, {'K5d', 'K5', 'K_lqr'}, true);
        cfg.controller_display_name = 'Discrete LQI augmented state feedback';

    case 'pid'
        if exist(fullfile(root, 'data', 'controller_pid.mat'), 'file') == 2
            [K_aug_hw, K_hw, ctrl_file] = load_controller_gain(root, ...
                {'data/controller_pid.mat'}, {'K_aug'}, true);
        else
            ctrl_file = sprintf('%s + %s', fullfile(root, 'data', 'controller_inner_pid.mat'), ...
                fullfile(root, 'data', 'controller_outer_pid.mat'));
            K_hw = zeros(1, 4);
            K_aug_hw = zeros(1, 5);
        end
        load_pid_cascade_if_available(root);
        cfg.controller_display_name = 'PID/cascade augmented feedback';
end

hw_use_augmented_controller = double(numel(K_aug_hw) > 4);

switch char(observer_name)
    case {'none', 'raw'}
        A_obs_hw = -eye(4);
        B_obs_hw = zeros(4, 3);
        C_obs_hw = eye(4);
        D_obs_hw = zeros(4, 3);
        hw_use_observer = 0;
        cfg.observer_display_name = 'Raw encoder derivatives';

    case 'leuenberger'
        obs = load(fullfile(root, 'data', 'observer.mat'));
        [A_obs_hw, B_obs_hw, C_obs_hw, D_obs_hw, used_discrete] = observer_matrices(obs, false);
        [A_obs_hw, B_obs_hw, C_obs_hw, D_obs_hw] = discretize_observer_if_needed( ...
            A_obs_hw, B_obs_hw, C_obs_hw, D_obs_hw, used_discrete);
        hw_use_observer = 1;
        cfg.observer_display_name = 'Leuenberger observer';

    case {'kalman', 'kalman_discrete', 'dlqe'}
        prefer_discrete = any(strcmp(char(observer_name), {'kalman_discrete', 'dlqe'}));
        candidates = { ...
            fullfile(root, 'data', 'observer_kalman.mat'), ...
            fullfile(root, 'data', 'kalman_observer.mat'), ...
            fullfile(root, 'data', 'controller_lqr.mat'), ...
            fullfile(root, 'data', 'observer.mat')};
        kalman_file = '';
        for k = 1:numel(candidates)
            if exist(candidates{k}, 'file') == 2
                obs_try = load(candidates{k});
                if has_observer_matrices(obs_try, prefer_discrete)
                    kalman_file = candidates{k};
                    obs = obs_try;
                    break;
                end
            end
        end
        if isempty(kalman_file)
            error(['Kalman observer matrices not found. Expected A_obs/B_obs/C_obs/D_obs ' ...
                   'or Aobs/Bobs/Cobs/Dobs, optionally with _d suffix, in data/.']);
        end
        [A_obs_hw, B_obs_hw, C_obs_hw, D_obs_hw, used_discrete] = ...
            observer_matrices(obs, prefer_discrete);
        [A_obs_hw, B_obs_hw, C_obs_hw, D_obs_hw] = discretize_observer_if_needed( ...
            A_obs_hw, B_obs_hw, C_obs_hw, D_obs_hw, used_discrete);
        hw_use_observer = 1;
        if used_discrete
            cfg.observer_display_name = 'Discrete Kalman observer';
        else
            cfg.observer_display_name = 'Kalman observer';
        end
end

assignin('base', 'K_hw', K_hw);
assignin('base', 'K_aug_hw', K_aug_hw);
assignin('base', 'hw_use_augmented_controller', hw_use_augmented_controller);
assignin('base', 'hw_controller_type', char(controller_name));
assignin('base', 'hw_use_observer', hw_use_observer);
assignin('base', 'A_obs_hw', A_obs_hw);
assignin('base', 'B_obs_hw', B_obs_hw);
assignin('base', 'C_obs_hw', C_obs_hw);
assignin('base', 'D_obs_hw', D_obs_hw);
assignin('base', 'hw_observer_sample_time', evalin('base', 'Ts'));

if evalin('base', 'exist(''d_inj'', ''var'')') == 0
    assignin('base', 'd_inj', [0 0; 1e6 0]);
end

cfg.K_hw = K_hw;
cfg.K_aug_hw = K_aug_hw;
cfg.controller_file = ctrl_file;
cfg.hw_use_augmented_controller = hw_use_augmented_controller;
cfg.hw_use_observer = hw_use_observer;

fprintf('Hardware validation config:\n');
fprintf('  Controller: %s\n', cfg.controller_display_name);
fprintf('  Source:     %s\n', ctrl_file);
fprintf('  Observer:   %s\n', cfg.observer_display_name);
fprintf('  K_hw:       [%s]\n', num2str(K_hw, ' %.4g'));
if hw_use_augmented_controller
    fprintf('  K_aug_hw:   [%s]\n', num2str(K_aug_hw, ' %.4g'));
end
end

function run_project_startup(root)
startup_candidates = {'seesawstartup.m', 'seesaw_startup.m', 'startup.m'};
needs_startup = evalin('base', 'exist(''SEESAW_ROOT'', ''var'')') == 0 || ...
    ~strcmp(evalin('base', 'SEESAW_ROOT'), root);
if ~needs_startup
    return;
end
for k = 1:numel(startup_candidates)
    startup_file = fullfile(root, startup_candidates{k});
    if exist(startup_file, 'file') == 2
        run(startup_file);
        return;
    end
end
error('No project startup script found under %s.', root);
end

function [K_aug_hw, K_hw, source_file] = load_controller_gain(root, rel_files, fields, needs_augmented)
for i = 1:numel(rel_files)
    source_file = fullfile(root, rel_files{i});
    if exist(source_file, 'file') ~= 2
        continue;
    end
    ctrl = load(source_file);
    for j = 1:numel(fields)
        if isfield(ctrl, fields{j})
            K = ctrl.(fields{j});
            source_order = gain_source_order(source_file, fields{j});
            K_aug_hw = gain_to_validation_order(K(:).', source_order);
            K_hw = K_aug_hw(1:4);
            if needs_augmented
                if numel(K_aug_hw) < 5
                    K_aug_hw = [K_hw, 0];
                end
            else
                K_aug_hw = K_hw;
            end
            return;
        end
    end
end
error('No compatible gain (%s) found in %s.', strjoin(fields, ', '), strjoin(rel_files, ', '));
end

function source_order = gain_source_order(source_file, field_name)
[~, name] = fileparts(source_file);
if any(strcmp(field_name, {'K4', 'K5', 'K4d', 'K5d'})) || strcmp(name, 'pole_placement')
    source_order = 'final';
else
    source_order = 'validation';
end
end

function K_out = gain_to_validation_order(K_in, source_order)
if strcmp(source_order, 'final')
    if numel(K_in) < 4
        error('Final-model gain must have at least four elements.');
    end
    K_out = [K_in([1 3 2 4]), K_in(5:end)];
else
    K_out = K_in;
end
end

function load_pid_cascade_if_available(root)
inner_file = fullfile(root, 'data', 'controller_inner_pid.mat');
outer_file = fullfile(root, 'data', 'controller_outer_pid.mat');
if exist(inner_file, 'file') == 2
    inner = load(inner_file);
    assign_struct_fields('hw_inner_pid', inner);
end
if exist(outer_file, 'file') == 2
    outer = load(outer_file);
    assign_struct_fields('hw_outer_pid', outer);
end
end

function assign_struct_fields(prefix, s)
names = fieldnames(s);
for k = 1:numel(names)
    assignin('base', sprintf('%s_%s', prefix, names{k}), s.(names{k}));
end
end

function tf = has_observer_matrices(obs, prefer_discrete)
tf = false;
try
    observer_matrices(obs, prefer_discrete);
    tf = true;
catch
end
end

function [A, B, C, D, used_discrete] = observer_matrices(obs, prefer_discrete)
used_discrete = false;
if prefer_discrete && all(isfield(obs, {'Aobs_d', 'Bobs_d', 'Cobs_d', 'Dobs_d'}))
    [A, B, C, D] = deal(obs.Aobs_d, obs.Bobs_d, obs.Cobs_d, obs.Dobs_d);
    used_discrete = true;
elseif prefer_discrete && all(isfield(obs, {'A_obs_d', 'B_obs_d', 'C_obs_d', 'D_obs_d'}))
    [A, B, C, D] = deal(obs.A_obs_d, obs.B_obs_d, obs.C_obs_d, obs.D_obs_d);
    used_discrete = true;
elseif all(isfield(obs, {'A_obs', 'B_obs', 'C_obs', 'D_obs'}))
    [A, B, C, D] = deal(obs.A_obs, obs.B_obs, obs.C_obs, obs.D_obs);
elseif all(isfield(obs, {'Aobs', 'Bobs', 'Cobs', 'Dobs'}))
    [A, B, C, D] = deal(obs.Aobs, obs.Bobs, obs.Cobs, obs.Dobs);
elseif all(isfield(obs, {'Aobs_d', 'Bobs_d', 'Cobs_d', 'Dobs_d'}))
    [A, B, C, D] = deal(obs.Aobs_d, obs.Bobs_d, obs.Cobs_d, obs.Dobs_d);
    used_discrete = true;
elseif all(isfield(obs, {'A_obs_d', 'B_obs_d', 'C_obs_d', 'D_obs_d'}))
    [A, B, C, D] = deal(obs.A_obs_d, obs.B_obs_d, obs.C_obs_d, obs.D_obs_d);
    used_discrete = true;
else
    error('No compatible observer matrices found.');
end

if is_final_order_observer(A)
    P = final_to_validation_permutation();
    A = P * A * P.';
    B = P * B;
    C = P * C * P.';
    D = P * D;
end
end

function [A, B, C, D] = discretize_observer_if_needed(A, B, C, D, used_discrete)
if used_discrete
    return;
end
Ts = evalin('base', 'Ts');
sysd = c2d(ss(A, B, C, D), Ts, 'zoh');
[A, B, C, D] = ssdata(sysd);
end

function tf = is_final_order_observer(A)
tf = size(A, 1) == 4 && abs(A(1, 3) - 1) < 1e-9 && abs(A(1, 2)) < 1e-9;
end

function P = final_to_validation_permutation()
P = [1 0 0 0;
     0 0 1 0;
     0 1 0 0;
     0 0 0 1];
end
