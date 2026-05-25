function cfg = tune_validation_case(varargin)
%TUNE_VALIDATION_CASE Single place for validation controller/observer tuning.
%
% Edit the values in this file, then run:
%   startup
%   tune_validation_case
%   build_controller_validation_harness('controller_id','PP','feedback_source_id','measured')
%   sim('ControllerValidationHarness')
%
% The case loader reapplies this script after loading saved controller data.

opts = parse_inputs(varargin{:});
cfg = default_tuning();

assignin('base', 'validation_tuning', cfg);

if opts.apply_now
    apply_validation_tuning(cfg);
end

if opts.verbose
    fprintf('Loaded validation_tuning. Edit validation/scripts/tune_validation_case.m to tune gains.\n');
end
end

function cfg = default_tuning()
cfg = struct();

% Global multipliers. Start here in the lab if the response is too aggressive.
cfg.global.controller_gain_scale = 1.0;
cfg.global.observer_gain_scale = 1.0;

% Actuator and safety behavior.
cfg.actuator.V_sat_hw = 6.0;
cfg.actuator.ud_pos_scale = 1.0;
cfg.actuator.ud_neg_scale = 1.0;

% State-feedback controllers.
cfg.pp.K_scale = 1.0;
cfg.pp.K_override = [];

cfg.lqr.K_scale = 1.0;
cfg.lqr.K_override = [];

cfg.lqi.K_scale = 1.0;
cfg.lqi.K_override = [];

% PID cascade transfer-function numerator scaling.
cfg.pid.outer_scale = 1.0;
cfg.pid.inner_scale = 1.0;
cfg.pid.outer_num_override = [];
cfg.pid.outer_den_override = [];
cfg.pid.inner_num_override = [];
cfg.pid.inner_den_override = [];

% Super-twisting SMC.
cfg.smc.surface_scale = 1.0;
cfg.smc.eq_scale = 1.0;
cfg.smc.k1_scale = 1.0;
cfg.smc.k2_scale = 1.0;
cfg.smc.phi_scale = 1.0;
cfg.smc.S_override = [];
cfg.smc.K_eq_override = [];
cfg.smc.k1_override = [];
cfg.smc.k2_override = [];
cfg.smc.phi_override = [];

% Observers. Matrix overrides are safest; scalar scaling is a coarse fallback.
cfg.observer.luenberger_scale = 1.0;
cfg.observer.kalman_scale = 1.0;
cfg.observer.A_luenberger_override = [];
cfg.observer.B_luenberger_override = [];
cfg.observer.C_luenberger_override = [];
cfg.observer.D_luenberger_override = [];
cfg.observer.A_kalman_override = [];
cfg.observer.B_kalman_override = [];
cfg.observer.C_kalman_override = [];
cfg.observer.D_kalman_override = [];
end

function apply_validation_tuning(cfg)
scale_if_exists('K_pp', cfg.global.controller_gain_scale * cfg.pp.K_scale);
scale_if_exists('K_lqr', cfg.global.controller_gain_scale * cfg.lqr.K_scale);
scale_if_exists('K_lqi', cfg.global.controller_gain_scale * cfg.lqi.K_scale);
scale_if_exists('K_aug', cfg.global.controller_gain_scale);
scale_if_exists('K_pid_aug', cfg.global.controller_gain_scale);

override_if_set('K_pp', cfg.pp.K_override);
override_if_set('K_lqr', cfg.lqr.K_override);
override_if_set('K_lqi', cfg.lqi.K_override);

scale_if_exists('pid_outer_num', cfg.global.controller_gain_scale * cfg.pid.outer_scale);
scale_if_exists('pid_inner_num', cfg.global.controller_gain_scale * cfg.pid.inner_scale);
override_if_set('pid_outer_num', cfg.pid.outer_num_override);
override_if_set('pid_outer_den', cfg.pid.outer_den_override);
override_if_set('pid_inner_num', cfg.pid.inner_num_override);
override_if_set('pid_inner_den', cfg.pid.inner_den_override);

scale_if_exists('S_smc', cfg.smc.surface_scale);
scale_if_exists('K_eq_smc', cfg.global.controller_gain_scale * cfg.smc.eq_scale);
scale_if_exists('k1_smc', cfg.global.controller_gain_scale * cfg.smc.k1_scale);
scale_if_exists('k2_smc', cfg.global.controller_gain_scale * cfg.smc.k2_scale);
scale_if_exists('phi_smc', cfg.smc.phi_scale);
override_if_set('S_smc', cfg.smc.S_override);
override_if_set('K_eq_smc', cfg.smc.K_eq_override);
override_if_set('k1_smc', cfg.smc.k1_override);
override_if_set('k2_smc', cfg.smc.k2_override);
override_if_set('phi_smc', cfg.smc.phi_override);

scale_if_exists('A_obs_l', cfg.global.observer_gain_scale * cfg.observer.luenberger_scale);
scale_if_exists('B_obs_l', cfg.global.observer_gain_scale * cfg.observer.luenberger_scale);
scale_if_exists('A_obs_k', cfg.global.observer_gain_scale * cfg.observer.kalman_scale);
scale_if_exists('B_obs_k', cfg.global.observer_gain_scale * cfg.observer.kalman_scale);
override_if_set('A_obs_l', cfg.observer.A_luenberger_override);
override_if_set('B_obs_l', cfg.observer.B_luenberger_override);
override_if_set('C_obs_l', cfg.observer.C_luenberger_override);
override_if_set('D_obs_l', cfg.observer.D_luenberger_override);
override_if_set('A_obs_k', cfg.observer.A_kalman_override);
override_if_set('B_obs_k', cfg.observer.B_kalman_override);
override_if_set('C_obs_k', cfg.observer.C_kalman_override);
override_if_set('D_obs_k', cfg.observer.D_kalman_override);

override_if_set('V_sat_hw', cfg.actuator.V_sat_hw);
scale_if_exists('ud_pos', cfg.actuator.ud_pos_scale);
scale_if_exists('ud_neg', cfg.actuator.ud_neg_scale);
end

function scale_if_exists(name, scale)
if isempty(scale) || isequal(scale, 1.0)
    return;
end
if evalin('base', sprintf('exist(''%s'', ''var'')', name))
    val = evalin('base', name);
    assignin('base', name, scale * val);
end
end

function override_if_set(name, value)
if ~isempty(value)
    assignin('base', name, value);
end
end

function opts = parse_inputs(varargin)
p = inputParser;
addParameter(p, 'apply_now', true, @islogical);
addParameter(p, 'verbose', true, @islogical);
parse(p, varargin{:});
opts = p.Results;
end
