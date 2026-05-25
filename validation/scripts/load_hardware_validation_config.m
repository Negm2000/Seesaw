function cfg = load_hardware_validation_config(controller_name, observer_name)
%LOAD_HARDWARE_VALIDATION_CONFIG Configure the generic HW validation model.
%
%   load_hardware_validation_config('pole_placement', 'none')
%   load_hardware_validation_config('lqr', 'leuenberger')
%   load_hardware_validation_config('pid', 'kalman')
%
% The Simulink model uses a fixed interface:
%   controller input: x_fb = [x_c; x_c_dot; alpha; alpha_dot]
%   augmented input:  [x_fb; integral(alpha)]
%   observer input:   [V_m; x_c; alpha]
%
% PARALLEL OBSERVERS: The model now runs Luenberger and Kalman observers
% in parallel for logging, regardless of which observer is the active
% feedback source.  The active feedback source is set by observer_name.

if nargin < 1 || isempty(controller_name)
    controller_name = 'pole_placement';
end
if nargin < 2 || isempty(observer_name)
    observer_name = 'none';
end

controller_name = validatestring(lower(controller_name), ...
    {'pole_placement', 'pp', 'lqr', 'pid'}, mfilename, 'controller_name');
observer_name = validatestring(lower(observer_name), ...
    {'none', 'raw', 'leuenberger', 'kalman'}, mfilename, 'observer_name');

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if evalin('base', 'exist(''SEESAW_ROOT'', ''var'')') == 0 || ...
        ~strcmp(evalin('base', 'SEESAW_ROOT'), root)
    run(fullfile(root, 'startup.m'));
end

cfg = struct();
cfg.controller_name = char(controller_name);
cfg.observer_name = char(observer_name);

switch char(controller_name)
    case {'pole_placement', 'pp'}
        ctrl = load(fullfile(root, 'data', 'controllers', 'controller_freq.mat'));
        K_hw = ctrl.Kf;
        K_aug_hw = [ctrl.Kf, 0];
        hw_use_augmented_controller = 0;
        cfg.controller_display_name = 'Pole placement state feedback';

    case 'lqr'
        ctrl = load(fullfile(root, 'data', 'controllers', 'controller_lqr.mat'));
        K_aug_hw = ctrl.K_lqr;
        K_hw = ctrl.K_lqr(1:4);
        hw_use_augmented_controller = 1;
        cfg.controller_display_name = 'LQR augmented state feedback';

    case 'pid'
        ctrl = load(fullfile(root, 'data', 'controllers', 'controller_pid.mat'));
        K_aug_hw = ctrl.K_aug;
        K_hw = ctrl.K_aug(1:4);
        hw_use_augmented_controller = 1;
        cfg.controller_display_name = 'PID/cascade augmented feedback';
end

%% Active observer (feedback source)
switch char(observer_name)
    case {'none', 'raw'}
        A_obs_hw = -eye(4);
        B_obs_hw = zeros(4, 3);
        C_obs_hw = eye(4);
        D_obs_hw = zeros(4, 3);
        hw_use_observer = 0;
        cfg.observer_display_name = 'Raw encoder derivatives';

    case 'leuenberger'
        obs = load(fullfile(root, 'data', 'params', 'observer.mat'));
        A_obs_hw = obs.A_obs;
        B_obs_hw = obs.B_obs;
        C_obs_hw = obs.C_obs;
        D_obs_hw = obs.D_obs;
        hw_use_observer = 1;
        cfg.observer_display_name = 'Leuenberger observer';

    case 'kalman'
        candidates = { ...
            fullfile(root, 'data', 'params', 'observer_kalman.mat'), ...
            fullfile(root, 'data', 'kalman_observer.mat')};
        kalman_file = '';
        for k = 1:numel(candidates)
            if exist(candidates{k}, 'file') == 2
                kalman_file = candidates{k};
                break;
            end
        end
        if isempty(kalman_file)
            error(['Kalman observer file not found. Save A_obs, B_obs, C_obs, ' ...
                   'D_obs to data/observer_kalman.mat or data/kalman_observer.mat.']);
        end
        obs = load(kalman_file);
        required = {'A_obs', 'B_obs', 'C_obs', 'D_obs'};
        for k = 1:numel(required)
            assert(isfield(obs, required{k}), 'Missing %s in %s.', required{k}, kalman_file);
        end
        A_obs_hw = obs.A_obs;
        B_obs_hw = obs.B_obs;
        C_obs_hw = obs.C_obs;
        D_obs_hw = obs.D_obs;
        hw_use_observer = 1;
        cfg.observer_display_name = 'Kalman observer';
end

%% Parallel observers for logging (always loaded)
% Luenberger
obs_luenb_file = fullfile(root, 'data', 'params', 'observer.mat');
if exist(obs_luenb_file, 'file') == 2
    obs_l = load(obs_luenb_file);
    A_obs_luenb = obs_l.A_obs;
    B_obs_luenb = obs_l.B_obs;
    C_obs_luenb = obs_l.C_obs;
    D_obs_luenb = obs_l.D_obs;
else
    % Passthrough if not available
    A_obs_luenb = -eye(4);
    B_obs_luenb = zeros(4, 3);
    C_obs_luenb = eye(4);
    D_obs_luenb = zeros(4, 3);
    warning('Luenberger observer file not found; parallel logging will output passthrough.');
end

% Kalman
kalman_candidates = { ...
    fullfile(root, 'data', 'params', 'observer_kalman.mat'), ...
    fullfile(root, 'data', 'kalman_observer.mat')};
kalman_found = false;
for k = 1:numel(kalman_candidates)
    if exist(kalman_candidates{k}, 'file') == 2
        obs_k = load(kalman_candidates{k});
        A_obs_kalm = obs_k.A_obs;
        B_obs_kalm = obs_k.B_obs;
        C_obs_kalm = obs_k.C_obs;
        D_obs_kalm = obs_k.D_obs;
        kalman_found = true;
        break;
    end
end
if ~kalman_found
    A_obs_kalm = -eye(4);
    B_obs_kalm = zeros(4, 3);
    C_obs_kalm = eye(4);
    D_obs_kalm = zeros(4, 3);
    warning('Kalman observer file not found; parallel logging will output passthrough.');
end

%% Assign all to base workspace
assignin('base', 'K_hw', K_hw);
assignin('base', 'K_aug_hw', K_aug_hw);
assignin('base', 'hw_use_augmented_controller', hw_use_augmented_controller);
assignin('base', 'hw_use_observer', hw_use_observer);
assignin('base', 'A_obs_hw', A_obs_hw);
assignin('base', 'B_obs_hw', B_obs_hw);
assignin('base', 'C_obs_hw', C_obs_hw);
assignin('base', 'D_obs_hw', D_obs_hw);

% Parallel observers
assignin('base', 'A_obs_luenb', A_obs_luenb);
assignin('base', 'B_obs_luenb', B_obs_luenb);
assignin('base', 'C_obs_luenb', C_obs_luenb);
assignin('base', 'D_obs_luenb', D_obs_luenb);
assignin('base', 'A_obs_kalm', A_obs_kalm);
assignin('base', 'B_obs_kalm', B_obs_kalm);
assignin('base', 'C_obs_kalm', C_obs_kalm);
assignin('base', 'D_obs_kalm', D_obs_kalm);

if evalin('base', 'exist(''d_inj'', ''var'')') == 0
    assignin('base', 'd_inj', [0 0; 1e6 0]);
end
if evalin('base', 'exist(''segment_id_ts'', ''var'')') == 0
    assignin('base', 'segment_id_ts', [0 0; 1e6 0]);
end

cfg.K_hw = K_hw;
cfg.K_aug_hw = K_aug_hw;
cfg.hw_use_augmented_controller = hw_use_augmented_controller;
cfg.hw_use_observer = hw_use_observer;

fprintf('Hardware validation config:\n');
fprintf('  Controller: %s\n', cfg.controller_display_name);
fprintf('  Observer (feedback): %s\n', cfg.observer_display_name);
fprintf('  Parallel observers: Luenberger + Kalman (logging only)\n');
fprintf('  K_hw:       [%s]\n', num2str(K_hw, ' %.4g'));
if hw_use_augmented_controller
    fprintf('  K_aug_hw:   [%s]\n', num2str(K_aug_hw, ' %.4g'));
end
end
