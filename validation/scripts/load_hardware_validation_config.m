function cfg = load_hardware_validation_config(controller_name)
%LOAD_HARDWARE_VALIDATION_CONFIG Configure Seesaw_Validation.slx for a run.
%
%   load_hardware_validation_config('K_pp')     % pole placement (no integral)
%   load_hardware_validation_config('K_ppi')    % pole placement + integral
%   load_hardware_validation_config('LQR')      % LQR (5-state, integral on alpha)
%   load_hardware_validation_config('LQI')      % alias for LQR
%   load_hardware_validation_config('LQG')      % LQR gain + observer in feedback (future)
%
% Assigns to base workspace:
%   K_hw          — state feedback gain [1x5]
%   ud_pos        — positive deadzone offset [V] (liftoff asymmetry)
%   ud_neg        — negative deadzone offset [V]
%   A_obs_L, B_obs_L, C_obs_L, D_obs_L — Luenberger observer matrices
%   A_obs_K, B_obs_K, C_obs_K, D_obs_K — Kalman observer matrices
%   d_inj_ts      — disturbance timeseries (default zero if not loaded)
%   segment_id_ts — segment ID timeseries (default zero if not loaded)
%
% State vector order: [xc, alpha, xc_dot, alpha_dot, int_alpha]
% For controllers without integral action, K_hw(5) = 0.

if nargin < 1 || isempty(controller_name)
    controller_name = 'K_pp';
end

valid_names = {'k_pp', 'pp', 'pole_placement', ...
               'k_ppi', 'ppi', ...
               'lqr', 'k_lqr', ...
               'lqi', 'k_lqi', ...
               'lqg', 'k_lqg'};
controller_name = validatestring(lower(controller_name), valid_names, ...
    mfilename, 'controller_name');

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if evalin('base', 'exist(''SEESAW_ROOT'', ''var'')') == 0 || ...
        ~strcmp(evalin('base', 'SEESAW_ROOT'), root)
    run(fullfile(root, 'startup.m'));
end

cfg = struct();
cfg.controller_name = char(controller_name);

%% Load controller gain
% State order: [xc, alpha, xc_dot, alpha_dot, int_alpha]
% controller_freq.mat Kf is [1x4] in order [xc, alpha, xc_dot, alpha_dot]
% controller_lqr.mat K_lqr is [1x5] in order [xc, alpha, xc_dot, alpha_dot, int_alpha]

switch char(controller_name)
    case {'k_pp', 'pp', 'pole_placement'}
        ctrl = load(fullfile(root, 'data', 'controllers', 'controller_freq.mat'));
        K_hw = [ctrl.Kf, 0];  % pad 5th element (no integral)
        cfg.display_name = 'Pole placement (K_pp)';

    case {'k_ppi', 'ppi'}
        ctrl = load(fullfile(root, 'data', 'controllers', 'controller_freq.mat'));
        % Use PP gains + manually tuned integral on alpha
        Ki_alpha = -20;  % TODO: tune this value
        K_hw = [ctrl.Kf, Ki_alpha];
        cfg.display_name = 'Pole placement + integral (K_ppi)';

    case {'lqr', 'k_lqr', 'lqi', 'k_lqi'}
        ctrl = load(fullfile(root, 'data', 'controllers', 'controller_lqr.mat'));
        K_hw = ctrl.K_lqr;  % already [1x5]
        cfg.display_name = 'LQR/LQI (5-state)';

    case {'lqg', 'k_lqg'}
        ctrl = load(fullfile(root, 'data', 'controllers', 'controller_lqr.mat'));
        K_hw = ctrl.K_lqr;  % same gain, observer handles estimation
        cfg.display_name = 'LQG (LQR + Kalman observer)';
        % NOTE: For true LQG, observer output feeds controller instead of
        % direct measurement. The model currently uses direct measurement;
        % LQG mode is logged-only until observer-in-loop is validated.
end

%% Load observer matrices
obs_L = load(fullfile(root, 'data', 'params', 'observer.mat'));
obs_K = load(fullfile(root, 'data', 'params', 'observer_kalman.mat'));

%% Deadzone offsets (symmetric default; asymmetric for liftoff)
ud_pos = 0.12;  % [V] positive direction deadzone
ud_neg = 0.12;  % [V] negative direction deadzone

%% Assign to base workspace
assignin('base', 'K_hw', K_hw);
assignin('base', 'ud_pos', ud_pos);
assignin('base', 'ud_neg', ud_neg);

% Luenberger observer
assignin('base', 'A_obs_L', obs_L.A_obs);
assignin('base', 'B_obs_L', obs_L.B_obs);
assignin('base', 'C_obs_L', obs_L.C_obs);
assignin('base', 'D_obs_L', obs_L.D_obs);

% Kalman observer
assignin('base', 'A_obs_K', obs_K.A_obs);
assignin('base', 'B_obs_K', obs_K.B_obs);
assignin('base', 'C_obs_K', obs_K.C_obs);
assignin('base', 'D_obs_K', obs_K.D_obs);

% Ensure FromWorkspace defaults exist (overwritten when hw_test_signals loaded)
if evalin('base', 'exist(''d_inj_ts'', ''var'')') == 0
    assignin('base', 'd_inj_ts', timeseries(0, 0));
end
if evalin('base', 'exist(''segment_id_ts'', ''var'')') == 0
    assignin('base', 'segment_id_ts', timeseries(0, 0));
end

cfg.K_hw = K_hw;
cfg.ud_pos = ud_pos;
cfg.ud_neg = ud_neg;

fprintf('Hardware validation config loaded:\n');
fprintf('  Controller: %s\n', cfg.display_name);
fprintf('  K_hw: [%s]\n', num2str(K_hw, ' %.4g'));
fprintf('  ud_pos=%.3f V, ud_neg=%.3f V\n', ud_pos, ud_neg);
fprintf('  Observers: Luenberger + Kalman loaded\n');
fprintf('  Model: models/Seesaw_Validation.slx\n');
end
