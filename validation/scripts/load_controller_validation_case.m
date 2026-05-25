function cfg = load_controller_validation_case(controller_id, feedback_source_id, varargin)
%LOAD_CONTROLLER_VALIDATION_CASE Configure controller validation variables.
%
% Canonical state order for this validation harness:
%   x = [x_c; x_c_dot; theta; theta_dot]

if nargin < 1 || isempty(controller_id)
    controller_id = 'PP';
end
if nargin < 2 || isempty(feedback_source_id)
    feedback_source_id = 'measured';
end

opts = parse_inputs(varargin{:});
controller_id = lower(string(controller_id));
feedback_source_id = lower(string(feedback_source_id));

root = opts.root;
ensure_startup(root);

cfg = base_config(root);
cfg.controller_id = char(controller_id);
cfg.feedback_source_id = char(feedback_source_id);

switch controller_id
    case {"pp", "pole_placement"}
        cfg.controller_selector_id = 1;
        ctrl = load_first(root, {fullfile('data','controllers','controller_freq.mat'), fullfile('data','controller_freq.mat')});
        cfg.K_pp = ctrl.Kf(:)';
        cfg.K_basic = cfg.K_pp;
        cfg.K_aug = [cfg.K_pp 0];
        cfg.use_integral = 0;

    case {"pp_integral", "ppi"}
        cfg.controller_selector_id = 2;
        pp = load_first(root, {fullfile('data','controllers','controller_freq.mat'), fullfile('data','controller_freq.mat')});
        pid = maybe_load(root, {fullfile('data','controllers','controller_pid.mat')});
        cfg.K_pp = pp.Kf(:)';
        if isfield(pid, 'K_aug')
            cfg.K_aug = pid.K_aug(:)';
        else
            cfg.K_aug = [cfg.K_pp 0];
        end
        cfg.K_basic = cfg.K_pp;
        cfg.use_integral = 1;

    case "lqr"
        cfg.controller_selector_id = 3;
        lqr = load_first(root, {fullfile('data','controllers','controller_lqr.mat')});
        cfg.K_lqr = lqr.K_lqr(1:4);
        cfg.K_basic = cfg.K_lqr;
        cfg.K_aug = [cfg.K_lqr 0];
        cfg.use_integral = 0;

    case "lqi"
        cfg.controller_selector_id = 4;
        lqr = load_first(root, {fullfile('data','controllers','controller_lqr.mat')});
        cfg.K_lqi = lqr.K_lqr(:)';
        cfg.K_aug = cfg.K_lqi;
        cfg.K_basic = cfg.K_lqi(1:4);
        cfg.use_integral = 1;

    case "lqg"
        cfg.controller_selector_id = 4;
        lqr = load_first(root, {fullfile('data','controllers','controller_lqr.mat')});
        cfg.K_lqi = lqr.K_lqr(:)';
        cfg.K_aug = cfg.K_lqi;
        cfg.K_basic = cfg.K_lqi(1:4);
        cfg.use_integral = 1;
        feedback_source_id = "kalman";
        cfg.feedback_source_id = char(feedback_source_id);

    case "pid"
        cfg.controller_selector_id = 5;
        pid = load_first(root, {fullfile('data','controllers','controller_pid.mat')});
        cfg.K_pid_aug = pid.K_aug(:)';
        cfg.K_aug = cfg.K_pid_aug;
        cfg.K_basic = cfg.K_pid_aug(1:4);
        cfg.use_integral = 1;

    case "smc"
        cfg.controller_selector_id = 6;
        smc = load_first(root, {fullfile('data','controllers','controller_smc.mat')});
        cfg.S_smc = smc.surface.S(:)';
        cfg.K_eq_smc = smc.surface.K_eq(:)';
        cfg.SB_smc = smc.surface.SB;
        cfg.k1_smc = smc.sta_design.k1;
        cfg.k2_smc = smc.sta_design.k2;
        cfg.phi_smc = smc.sta_design.phi_bl;
        cfg.use_integral = 0;

    otherwise
        error('Unknown controller_id: %s', controller_id);
end

switch feedback_source_id
    case {"measured", "dirty", "raw", "dd"}
        cfg.feedback_selector_id = 1;
        cfg.feedback_source_id = 'measured';
    case {"luenberger", "leuenberger"}
        cfg.feedback_selector_id = 2;
    case "kalman"
        cfg.feedback_selector_id = 3;
    otherwise
        error('Unknown feedback_source_id: %s', feedback_source_id);
end

assign_cfg(cfg);
apply_tuning_if_present();
fprintf('Configured validation case: controller=%s, feedback=%s\n', cfg.controller_id, cfg.feedback_source_id);
end

function cfg = base_config(root)
tuned = load_first(root, {fullfile('data','tuned','tuned_params.mat')});
pp = maybe_load(root, {fullfile('data','controllers','controller_freq.mat'), fullfile('data','controller_freq.mat')});
obs_l = maybe_load(root, {fullfile('data','params','observer.mat'), fullfile('data','observer.mat')});
obs_k = maybe_load(root, {fullfile('data','params','observer_kalman.mat'), fullfile('data','params','kalman_observer.mat')});
nonlin = maybe_load(root, {fullfile('data','params','param_nonlinear.mat'), fullfile('data','param_nonlinear.mat')});
pid_outer = maybe_load(root, {fullfile('data','controllers','controller_outer_pid.mat'), fullfile('data','controllers','controller_outer_pd.mat')});
pid_inner = maybe_load(root, {fullfile('data','controllers','controller_inner_pid.mat')});

cfg = struct();
cfg.root = root;
cfg.Ts = evalin('base', 'Ts');
cfg.V_sat_hw = 6.0;
cfg.V_model_sat = evalin('base', 'V_sat');
cfg.dd_filter_N = 50;
cfg.validation_log_columns = {'segment_id','theta_ref','x_ref','u_ff', ...
    'x_feedback','V_cmd','V_m','theta','x_measured', ...
    'x_luenberger','x_kalman'};

% The tuned params and current seesaw_params variants disagree in state order.
% For validation we standardize on the controller order used by PP/SMC/observers.
cfg.A_ctrl = tuned.A_sw;
cfg.B_ctrl = tuned.B_sw;
cfg.C_ctrl = eye(4);
cfg.D_ctrl = zeros(4, 1);
cfg.C_theta = [0 0 1 0];
cfg.C_meas = [1 0 0 0; 0 0 1 0];

ss_map = [cfg.A_ctrl cfg.B_ctrl; cfg.C_theta 0] \ [zeros(4, 1); 1];
cfg.theta_xss_gain = ss_map(1:4);
cfg.theta_uff_gain = ss_map(5);

cfg.K_basic = zeros(1, 4);
cfg.K_aug = zeros(1, 5);
cfg.K_pp = zeros(1, 4);
cfg.K_lqr = zeros(1, 4);
cfg.K_lqi = zeros(1, 5);
cfg.K_pid_aug = zeros(1, 5);

if isfield(pp, 'Kf')
    cfg.K_pp = pp.Kf(:)';
end

cfg.A_obs_l = -eye(4);
cfg.B_obs_l = zeros(4, 3);
cfg.C_obs_l = eye(4);
cfg.D_obs_l = zeros(4, 3);
if isfield(obs_l, 'A_obs')
    cfg.A_obs_l = obs_l.A_obs;
    cfg.B_obs_l = obs_l.B_obs;
    cfg.C_obs_l = obs_l.C_obs;
    cfg.D_obs_l = obs_l.D_obs;
end

cfg.A_obs_k = cfg.A_obs_l;
cfg.B_obs_k = cfg.B_obs_l;
cfg.C_obs_k = cfg.C_obs_l;
cfg.D_obs_k = cfg.D_obs_l;
if isfield(obs_k, 'A_obs')
    cfg.A_obs_k = obs_k.A_obs;
    cfg.B_obs_k = obs_k.B_obs;
    cfg.C_obs_k = obs_k.C_obs;
    cfg.D_obs_k = obs_k.D_obs;
end

cfg.ud_pos = getfield_or(nonlin, 'ud_pos', 0.12);
cfg.ud_neg = getfield_or(nonlin, 'ud_neg', 0.12);

cfg.Kp_out = getfield_or(pid_outer, 'Kp_out', 0);
cfg.Ki_out = getfield_or(pid_outer, 'Ki_out', 0);
cfg.Kd_out = getfield_or(pid_outer, 'Kd_out', 0);
cfg.N_out = getfield_or(pid_outer, 'N_out', 50);
cfg.Kp_in = getfield_or(pid_inner, 'Kp_in', 0);
cfg.Ki_in = getfield_or(pid_inner, 'Ki_in', 0);
cfg.Kd_in = getfield_or(pid_inner, 'Kd_in', 0);
cfg.N_in = getfield_or(pid_inner, 'N_in', 50);

cfg.pid_outer_num = pid_num(cfg.Kp_out, cfg.Ki_out, cfg.Kd_out, cfg.N_out);
cfg.pid_outer_den = [1 cfg.N_out 0];
cfg.pid_inner_num = pid_num(cfg.Kp_in, cfg.Ki_in, cfg.Kd_in, cfg.N_in);
cfg.pid_inner_den = [1 cfg.N_in 0];

cfg.S_smc = [0 0 1 0];
cfg.K_eq_smc = zeros(1, 4);
cfg.SB_smc = 1;
cfg.k1_smc = 1;
cfg.k2_smc = 1;
cfg.phi_smc = 0.01;
cfg.smc_int_ic = 0;
end

function num = pid_num(Kp, Ki, Kd, N)
num = [Kp + Kd*N, Kp*N + Ki, Ki*N];
end

function assign_cfg(cfg)
names = fieldnames(cfg);
for i = 1:numel(names)
    assignin('base', names{i}, cfg.(names{i}));
end
end

function apply_tuning_if_present()
if evalin('base', 'exist(''validation_tuning'', ''var'')')
    tune_validation_case('verbose', false);
end
end

function out = load_first(root, rel_paths)
for i = 1:numel(rel_paths)
    file = fullfile(root, rel_paths{i});
    if exist(file, 'file') == 2
        out = load(file);
        return;
    end
end
error('None of the required data files exist: %s', strjoin(rel_paths, ', '));
end

function out = maybe_load(root, rel_paths)
out = struct();
for i = 1:numel(rel_paths)
    file = fullfile(root, rel_paths{i});
    if exist(file, 'file') == 2
        out = load(file);
        return;
    end
end
end

function val = getfield_or(s, field, default)
if isfield(s, field)
    val = s.(field);
else
    val = default;
end
end

function ensure_startup(root)
if evalin('base', 'exist(''SEESAW_ROOT'', ''var'')') == 0 || ~strcmp(evalin('base', 'SEESAW_ROOT'), root)
    run(fullfile(root, 'startup.m'));
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
parse(p, varargin{:});
opts = p.Results;
opts.root = char(opts.root);
end
