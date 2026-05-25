function cfg = load_hardware_validation_config(controller_name)
%LOAD_HARDWARE_VALIDATION_CONFIG Configure Seesaw_Validation.slx for a run.
%
%   load_hardware_validation_config('pole_placement')
%   load_hardware_validation_config('lqr')
%
% Assigns to base workspace:
%   K_hw          — state feedback gain [1x4]
%   K_ec          — cart encoder scale [m/count]
%   K_E_SW, K_gs  — seesaw encoder calibration
%   alpha_f       — derivative filter bandwidth [rad/s]
%   V_sat         — voltage saturation limit [V]
%   d_inj_ts      — disturbance timeseries (default zero if not loaded)
%   segment_id_ts — segment ID timeseries (default zero if not loaded)

if nargin < 1 || isempty(controller_name)
    controller_name = 'pole_placement';
end

controller_name = validatestring(lower(controller_name), ...
    {'pole_placement', 'pp', 'lqr'}, mfilename, 'controller_name');

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if evalin('base', 'exist(''SEESAW_ROOT'', ''var'')') == 0 || ...
        ~strcmp(evalin('base', 'SEESAW_ROOT'), root)
    run(fullfile(root, 'startup.m'));
end

cfg = struct();
cfg.controller_name = char(controller_name);

switch char(controller_name)
    case {'pole_placement', 'pp'}
        ctrl = load(fullfile(root, 'data', 'controllers', 'controller_freq.mat'));
        K_hw = ctrl.Kf;
        cfg.display_name = 'Pole placement state feedback';

    case 'lqr'
        ctrl = load(fullfile(root, 'data', 'controllers', 'controller_lqr.mat'));
        K_hw = ctrl.K_lqr(1:4);
        cfg.display_name = 'LQR state feedback';
end

%% Assign to base workspace
assignin('base', 'K_hw', K_hw);

% Ensure FromWorkspace defaults exist (overwritten when hw_test_signals loaded)
if evalin('base', 'exist(''d_inj_ts'', ''var'')') == 0
    assignin('base', 'd_inj_ts', timeseries(0, 0));
end
if evalin('base', 'exist(''segment_id_ts'', ''var'')') == 0
    assignin('base', 'segment_id_ts', timeseries(0, 0));
end

cfg.K_hw = K_hw;

fprintf('Hardware validation config loaded:\n');
fprintf('  Controller: %s\n', cfg.display_name);
fprintf('  K_hw: [%s]\n', num2str(K_hw, ' %.4g'));
fprintf('  Model: models/Seesaw_Validation.slx\n');
end
