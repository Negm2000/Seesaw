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
        ctrl = load(fullfile(root, 'data', 'controller_freq.mat'));
        K_hw = ctrl.Kf;
        K_aug_hw = [ctrl.Kf, 0];
        hw_use_augmented_controller = 0;
        cfg.controller_display_name = 'Pole placement state feedback';

    case 'lqr'
        ctrl = load(fullfile(root, 'data', 'controller_lqr.mat'));
        K_aug_hw = ctrl.K_lqr;
        K_hw = ctrl.K_lqr(1:4);
        hw_use_augmented_controller = 1;
        cfg.controller_display_name = 'LQR augmented state feedback';

    case 'pid'
        ctrl = load(fullfile(root, 'data', 'controller_pid.mat'));
        K_aug_hw = ctrl.K_aug;
        K_hw = ctrl.K_aug(1:4);
        hw_use_augmented_controller = 1;
        cfg.controller_display_name = 'PID/cascade augmented feedback';
end

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
        A_obs_hw = obs.A_obs;
        B_obs_hw = obs.B_obs;
        C_obs_hw = obs.C_obs;
        D_obs_hw = obs.D_obs;
        hw_use_observer = 1;
        cfg.observer_display_name = 'Leuenberger observer';

    case 'kalman'
        candidates = { ...
            fullfile(root, 'data', 'observer_kalman.mat'), ...
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

assignin('base', 'K_hw', K_hw);
assignin('base', 'K_aug_hw', K_aug_hw);
assignin('base', 'hw_use_augmented_controller', hw_use_augmented_controller);
assignin('base', 'hw_use_observer', hw_use_observer);
assignin('base', 'A_obs_hw', A_obs_hw);
assignin('base', 'B_obs_hw', B_obs_hw);
assignin('base', 'C_obs_hw', C_obs_hw);
assignin('base', 'D_obs_hw', D_obs_hw);

if evalin('base', 'exist(''d_inj'', ''var'')') == 0
    assignin('base', 'd_inj', [0 0; 1e6 0]);
end

cfg.K_hw = K_hw;
cfg.K_aug_hw = K_aug_hw;
cfg.hw_use_augmented_controller = hw_use_augmented_controller;
cfg.hw_use_observer = hw_use_observer;

fprintf('Hardware validation config:\n');
fprintf('  Controller: %s\n', cfg.controller_display_name);
fprintf('  Observer:   %s\n', cfg.observer_display_name);
fprintf('  K_hw:       [%s]\n', num2str(K_hw, ' %.4g'));
if hw_use_augmented_controller
    fprintf('  K_aug_hw:   [%s]\n', num2str(K_aug_hw, ' %.4g'));
end
end
