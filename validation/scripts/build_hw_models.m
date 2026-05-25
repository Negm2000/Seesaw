function build_hw_models(varargin)
%BUILD_HW_MODELS  Regenerate the four per-controller HW validation models
%                 in the SMC_STA_HW_2 layout style.
%
%   build_hw_models()                       % build all four
%   build_hw_models('Models', {'pid'})      % subset
%   build_hw_models('Force', true)
%
% Layout = three lavender area zones, top-down/left-right reading:
%
%   +---------------+ +----------------------------+
%   | Inputs        | | Outputs                    |
%   |               | |  x_bus mux, telemetry mux, |
%   | HIL, Encoders,| |  Scope                     |
%   | scale gains,  | |                            |
%   | LPF dx/dt,    | |                            |
%   | theta integral| |                            |
%   +---------------+ +----------------------------+
%   +-----------------+
%   | Controller      |
%   |                 |
%   | From[x_bus] ->  |
%   | <id>_Controller |
%   |   -> +d -> DZ   |
%   |   -> Sat -> u_sat
%   |                 |
%   | From[u_sat] ->  |
%   |   Motor Command |
%   |                 |
%   | DisturbanceBank |
%   |   -> Goto [d]   |
%   +-----------------+
%
% Goto/From tags are short and bracket-displayed: xc, theta, xc_dot,
% theta_dot, theta_int, x_bus, x_obs, u_sat, d.

opts = struct('Force', false, 'Models', {{'pid','sf','sf_obs','smc'}});
for k = 1:2:numel(varargin), opts.(varargin{k}) = varargin{k+1}; end

valdir   = fileparts(fileparts(mfilename('fullpath')));
lib_path = fullfile(valdir, 'lib', 'seesaw_hw_lib.slx');
assert(isfile(lib_path), 'Missing %s. Run build_seesaw_hw_lib first.', lib_path);

warning('off', 'all');
try, load_system(lib_path); catch, end

legacy_model = 'HardwareValidation_HWTest';
if isempty(find_system('SearchDepth', 0, 'Name', legacy_model))
    legacy_slx = fullfile(valdir, 'models', 'legacy', [legacy_model '.slx']);
    try, load_system(legacy_slx); catch, end
end

builders = struct( ...
    'pid',    @build_HW_PID, ...
    'sf',     @build_HW_StateFeedback, ...
    'sf_obs', @build_HW_StateFeedback_Observer, ...
    'smc',    @build_HW_SMC);

for k = 1:numel(opts.Models)
    key = opts.Models{k};
    fn = builders.(key);
    name = fn('NameOnly');
    out_path = fullfile(valdir, 'models', [name '.slx']);
    if isfile(out_path)
        if ~opts.Force
            fprintf('skip   %s (exists)\n', name);
            continue;
        end
        delete(out_path);
    end
    fn(out_path, legacy_model);
end
end

% =========================================================================
% LAYOUT CONSTANTS  (matches SMC_STA_HW_2 conventions)
% =========================================================================
function L = layout()
% --- Zone rectangles (area_annotation, lavender) ---
L.zone_inputs     = [-620 -575  -50  -20];
L.zone_outputs    = [  60 -610  720  130];
L.zone_controller = [-620   20   50  380];

% --- Zone title positions (note_annotation, 22pt bold) ---
L.label_inputs     = [-345 -560];
L.label_controller = [-345  335];
L.label_outputs    = [ 320 -595];

% --- INPUTS zone ---
L.hil_init     = [-500 -510 -395 -435];
L.encoders     = [-500 -345 -395 -285];
L.demux        = [-380 -340 -375 -290];
L.gain_xc      = [-345 -385 -295 -335];
L.gain_theta   = [-335 -285 -305 -255];
L.goto_xc      = [-235 -375 -195 -345];
L.goto_theta   = [-235 -285 -195 -255];

L.from_xc_d      = [-485 -185 -445 -155];
L.der_xc         = [-380 -185 -350 -155];
L.lpf_xc         = [-275 -188 -215 -152];
L.goto_xc_dot    = [-180 -185 -140 -155];

L.from_theta_d   = [-485 -120 -445  -90];
L.der_theta      = [-380 -120 -350  -90];
L.lpf_theta      = [-275 -123 -215  -87];
L.goto_theta_dot = [-180 -120 -140  -90];

L.from_theta_i   = [-485  -55 -445  -25];
L.int_theta      = [-380  -60 -350  -20];
L.goto_theta_int = [-180  -55 -140  -25];

% --- CONTROLLER zone (inside lavender) ---
L.from_xbus_ctrl  = [-580   65 -540   85];
L.controller      = [-500   50 -390  110];
L.from_d_sum      = [-380  100 -340  120];
L.sum_ud          = [-330   70 -310  110];
L.deadzone        = [-290   55 -190  135];
L.saturation      = [-170   80 -130  110];
L.goto_u_sat      = [-110   80  -60  110];

L.from_usat_mot   = [-580  175 -540  195];
L.motor_command   = [-500  155 -400  220];

L.dist_bank       = [-380  220 -260  340];
L.goto_d          = [-220  265 -170  285];

% --- OUTPUTS zone ---
% x_bus assembly (top of outputs zone)
L.from_xc_xbus      = [ 100 -510  140 -490];
L.from_xc_dot_xbus  = [ 100 -480  140 -460];
L.from_theta_xbus   = [ 100 -450  140 -430];
L.from_theta_dot_xbus = [ 100 -420  140 -400];
L.mux_xbus          = [ 200 -515  205 -395];
L.goto_xbus         = [ 240 -465  290 -445];

% Telemetry mux for To Host File (middle of outputs zone)
L.clock             = [ 320 -380  360 -360];
L.from_xc_log       = [ 320 -350  360 -330];
L.from_theta_log    = [ 320 -320  360 -300];
L.from_usat_log     = [ 320 -290  360 -270];
L.from_d_log        = [ 320 -260  360 -240];
L.from_xbus_log     = [ 320 -230  360 -210];
L.from_xobs_log     = [ 320 -200  360 -180];
L.mux_log           = [ 440 -385  445 -180];
L.to_host_file      = [ 490 -325  590 -270];

% Scope feed (bottom of outputs zone)
L.from_usat_scope   = [ 320 -100  360  -80];
L.from_xc_scope     = [ 320  -60  360  -40];
L.from_theta_scope  = [ 320  -20  360    0];
L.scope             = [ 440 -100  520    0];
end

% =========================================================================
% Skeleton helpers (Inputs and Outputs zones -- identical for all four)
% =========================================================================
function build_skeleton(model, legacy_model)
L = layout();

% --- Zone rectangles ---
add_zone(model, 'zone_inputs',     L.zone_inputs);
add_zone(model, 'zone_outputs',    L.zone_outputs);
add_zone(model, 'zone_controller', L.zone_controller);

% --- Zone titles ---
add_title(model, 'Inputs',     L.label_inputs);
add_title(model, 'Controller', L.label_controller);
add_title(model, 'Outputs',    L.label_outputs);

% =====================================================================
% INPUTS zone
% =====================================================================
add_block([legacy_model '/HIL Initialize'], [model '/HIL Initialize'], 'Position', L.hil_init);
add_block([legacy_model '/Encoders'],       [model '/Encoders'],       'Position', L.encoders);
add_block('built-in/Demux', [model '/Encoder Demux'], 'Outputs', '2', 'Position', L.demux);
add_block('built-in/Gain',  [model '/meters//tick'],  'Gain', 'K_ec',        'Position', L.gain_xc);
add_block('built-in/Gain',  [model '/radians//tick'], 'Gain', 'K_E_SW/K_gs', 'Position', L.gain_theta);
add_block('built-in/Goto',  [model '/[xc]'],     'GotoTag', 'xc',    'TagVisibility', 'global', 'Position', L.goto_xc);
add_block('built-in/Goto',  [model '/[theta]'],  'GotoTag', 'theta', 'TagVisibility', 'global', 'Position', L.goto_theta);
add_line(model, 'Encoders/1',      'Encoder Demux/1', 'autorouting', 'on');
add_line(model, 'Encoder Demux/1', 'meters//tick/1',  'autorouting', 'on');
add_line(model, 'Encoder Demux/2', 'radians//tick/1', 'autorouting', 'on');
add_line(model, 'meters//tick/1',  '[xc]/1',          'autorouting', 'on');
add_line(model, 'radians//tick/1', '[theta]/1',       'autorouting', 'on');

% xc_dot
add_block('built-in/From',        [model '/From_xc_1'],    'GotoTag', 'xc',    'Position', L.from_xc_d);
add_block('built-in/Derivative',  [model '/d//dt xc'],                          'Position', L.der_xc);
add_block('built-in/TransferFcn', [model '/LPF xc_dot'],   'Numerator', '[1]', 'Denominator', '[1/30 1]', 'Position', L.lpf_xc);
add_block('built-in/Goto',        [model '/[xc_dot]'],     'GotoTag', 'xc_dot', 'TagVisibility', 'global', 'Position', L.goto_xc_dot);
add_line(model, 'From_xc_1/1',  'd//dt xc/1',    'autorouting', 'on');
add_line(model, 'd//dt xc/1',   'LPF xc_dot/1',  'autorouting', 'on');
add_line(model, 'LPF xc_dot/1', '[xc_dot]/1',    'autorouting', 'on');

% theta_dot
add_block('built-in/From',        [model '/From_theta_1'], 'GotoTag', 'theta', 'Position', L.from_theta_d);
add_block('built-in/Derivative',  [model '/d//dt theta'],                       'Position', L.der_theta);
add_block('built-in/TransferFcn', [model '/LPF theta_dot'], 'Numerator', '[1]', 'Denominator', '[1/30 1]', 'Position', L.lpf_theta);
add_block('built-in/Goto',        [model '/[theta_dot]'],   'GotoTag', 'theta_dot', 'TagVisibility', 'global', 'Position', L.goto_theta_dot);
add_line(model, 'From_theta_1/1', 'd//dt theta/1',    'autorouting', 'on');
add_line(model, 'd//dt theta/1',  'LPF theta_dot/1',  'autorouting', 'on');
add_line(model, 'LPF theta_dot/1','[theta_dot]/1',    'autorouting', 'on');

% theta integral
add_block('built-in/From',       [model '/From_theta_2'], 'GotoTag', 'theta', 'Position', L.from_theta_i);
add_block('built-in/Integrator', [model '/integral theta'], 'InitialCondition', '0', 'Position', L.int_theta);
add_block('built-in/Goto',       [model '/[theta_int]'], 'GotoTag', 'theta_int', 'TagVisibility', 'global', 'Position', L.goto_theta_int);
add_line(model, 'From_theta_2/1',   'integral theta/1', 'autorouting', 'on');
add_line(model, 'integral theta/1', '[theta_int]/1',    'autorouting', 'on');

% =====================================================================
% OUTPUTS zone -- x_bus assembly + telemetry mux + scope
% =====================================================================
% x_bus = [xc; xc_dot; theta; theta_dot]
add_block('built-in/From', [model '/From_xc_2'],        'GotoTag', 'xc',        'Position', L.from_xc_xbus);
add_block('built-in/From', [model '/From_xc_dot_1'],    'GotoTag', 'xc_dot',    'Position', L.from_xc_dot_xbus);
add_block('built-in/From', [model '/From_theta_3'],     'GotoTag', 'theta',     'Position', L.from_theta_xbus);
add_block('built-in/From', [model '/From_theta_dot_1'], 'GotoTag', 'theta_dot', 'Position', L.from_theta_dot_xbus);
add_block('built-in/Mux',  [model '/Mux x_bus'], 'Inputs', '4', 'Position', L.mux_xbus);
add_block('built-in/Goto', [model '/[x_bus]'],   'GotoTag', 'x_bus', 'TagVisibility', 'global', 'Position', L.goto_xbus);
add_line(model, 'From_xc_2/1',        'Mux x_bus/1', 'autorouting', 'on');
add_line(model, 'From_xc_dot_1/1',    'Mux x_bus/2', 'autorouting', 'on');
add_line(model, 'From_theta_3/1',     'Mux x_bus/3', 'autorouting', 'on');
add_line(model, 'From_theta_dot_1/1', 'Mux x_bus/4', 'autorouting', 'on');
add_line(model, 'Mux x_bus/1',        '[x_bus]/1',   'autorouting', 'on');

% To Host File: 7-wide bundle [t, xc, theta, u_sat, d, x_bus(4), x_obs(4)]
add_block('built-in/Clock', [model '/Clock'], 'Position', L.clock);
add_block('built-in/From', [model '/From_xc_3'],    'GotoTag', 'xc',    'Position', L.from_xc_log);
add_block('built-in/From', [model '/From_theta_4'], 'GotoTag', 'theta', 'Position', L.from_theta_log);
add_block('built-in/From', [model '/From_usat_1'],  'GotoTag', 'u_sat', 'Position', L.from_usat_log);
add_block('built-in/From', [model '/From_d_1'],     'GotoTag', 'd',     'Position', L.from_d_log);
add_block('built-in/From', [model '/From_xbus_1'],  'GotoTag', 'x_bus', 'Position', L.from_xbus_log);
add_block('built-in/From', [model '/From_xobs_1'],  'GotoTag', 'x_obs', 'Position', L.from_xobs_log);
add_block('built-in/Mux',  [model '/Logging Bundle'], 'Inputs', '7', 'Position', L.mux_log);
add_block([legacy_model '/To Host File'], [model '/To Host File'], 'Position', L.to_host_file);
add_line(model, 'Clock/1',         'Logging Bundle/1', 'autorouting', 'on');
add_line(model, 'From_xc_3/1',     'Logging Bundle/2', 'autorouting', 'on');
add_line(model, 'From_theta_4/1',  'Logging Bundle/3', 'autorouting', 'on');
add_line(model, 'From_usat_1/1',   'Logging Bundle/4', 'autorouting', 'on');
add_line(model, 'From_d_1/1',      'Logging Bundle/5', 'autorouting', 'on');
add_line(model, 'From_xbus_1/1',   'Logging Bundle/6', 'autorouting', 'on');
add_line(model, 'From_xobs_1/1',   'Logging Bundle/7', 'autorouting', 'on');
add_line(model, 'Logging Bundle/1','To Host File/1',   'autorouting', 'on');

% Scope
add_block('built-in/From', [model '/From_usat_2'],  'GotoTag', 'u_sat', 'Position', L.from_usat_scope);
add_block('built-in/From', [model '/From_xc_4'],    'GotoTag', 'xc',    'Position', L.from_xc_scope);
add_block('built-in/From', [model '/From_theta_5'], 'GotoTag', 'theta', 'Position', L.from_theta_scope);
add_block('built-in/Mux',  [model '/Mux Scope'], 'Inputs', '3', 'Position', [400 -100 405 0]);
add_block('built-in/Scope', [model '/Scope'], 'Position', L.scope);
add_line(model, 'From_usat_2/1',  'Mux Scope/1', 'autorouting', 'on');
add_line(model, 'From_xc_4/1',    'Mux Scope/2', 'autorouting', 'on');
add_line(model, 'From_theta_5/1', 'Mux Scope/3', 'autorouting', 'on');
add_line(model, 'Mux Scope/1',    'Scope/1',     'autorouting', 'on');

% =====================================================================
% CONTROLLER zone -- shared output chain: u_lin + d -> DZ -> Sat -> [u_sat]
% =====================================================================
% (Controller subsystem itself is added by per-model builder.)
add_block('built-in/From', [model '/From_xbus_ctrl'], 'GotoTag', 'x_bus', 'Position', L.from_xbus_ctrl);

add_block('built-in/From', [model '/From_d_sum'],  'GotoTag', 'd', 'Position', L.from_d_sum);
add_block('built-in/Sum',  [model '/Sum u+d'],     'Inputs', '++', 'Position', L.sum_ud);
add_block('seesaw_hw_lib/DeadzoneInverse', [model '/DeadzoneInverse'], 'Position', L.deadzone);
add_block('built-in/Saturate', [model '/Saturation'], ...
    'UpperLimit', 'V_sat', 'LowerLimit', '-V_sat', 'Position', L.saturation);
add_block('built-in/Goto', [model '/[u_sat]'], 'GotoTag', 'u_sat', ...
    'TagVisibility', 'global', 'Position', L.goto_u_sat);
add_line(model, 'From_d_sum/1',      'Sum u+d/2',         'autorouting', 'on');
add_line(model, 'Sum u+d/1',         'DeadzoneInverse/1', 'autorouting', 'on');
add_line(model, 'DeadzoneInverse/1', 'Saturation/1',      'autorouting', 'on');
add_line(model, 'Saturation/1',      '[u_sat]/1',         'autorouting', 'on');

% Motor Command (below the control chain inside Controller zone)
add_block('built-in/From', [model '/From_usat_motor'], 'GotoTag', 'u_sat', 'Position', L.from_usat_mot);
add_block([legacy_model '/Motor Command'], [model '/Motor Command'], 'Position', L.motor_command);
add_line(model, 'From_usat_motor/1', 'Motor Command/1', 'autorouting', 'on');

% DisturbanceBank + Goto [d]
add_block('seesaw_hw_lib/DisturbanceBank', [model '/DisturbanceBank'], 'Position', L.dist_bank);
add_block('built-in/Goto', [model '/[d]'], 'GotoTag', 'd', 'TagVisibility', 'global', 'Position', L.goto_d);
add_line(model, 'DisturbanceBank/1', '[d]/1', 'autorouting', 'on');

config_solver(model);
end

function attach_x_obs_zeros(model)
% Non-observer models publish x_obs = zeros(4,1) so telemetry stays uniform.
add_block('built-in/Constant', [model '/x_obs_zeros'], ...
    'Value', 'zeros(4,1)', 'Position', [-580  -10 -540  10]);
add_block('built-in/Goto', [model '/[x_obs]'], 'GotoTag', 'x_obs', ...
    'TagVisibility', 'global', 'Position', [-510 -10 -460  10]);
add_line(model, 'x_obs_zeros/1', '[x_obs]/1', 'autorouting', 'on');
end

function wire_controller(model, ctrl_block, augmented)
% Connect From_xbus_ctrl -> Controller; then Controller -> Sum u+d/1.
% augmented==true: also feed [theta_int] into Controller input 2 (subclass
% controllers can ignore that port).
add_line(model, 'From_xbus_ctrl/1', [ctrl_block '/1'], 'autorouting', 'on');
if augmented
    L = layout();
    add_block('built-in/From', [model '/From_theta_int_ctrl'], 'GotoTag', 'theta_int', ...
        'Position', [-580 100 -540 120]);
    add_line(model, 'From_theta_int_ctrl/1', [ctrl_block '/2'], 'autorouting', 'on');
end
add_line(model, [ctrl_block '/1'], 'Sum u+d/1', 'autorouting', 'on');
end

% =========================================================================
% Annotations
% =========================================================================
function add_zone(model, name, pos)
h = add_block('built-in/Area', [model '/' name], 'Position', pos);
an = get_param(h, 'Object');
an.BackgroundColor = '[0.901961, 0.901961, 1.000000]';
an.ForegroundColor = '[0.901961, 0.901961, 1.000000]';
an.Text = '';
end

function add_title(model, text, pos_xy)
% Build a unique-safe block name from the text
nm = matlab.lang.makeValidName(['title_' text]);
a = Simulink.Annotation([model '/' nm]);
a.Text = text;
a.FontSize = 22;
a.FontWeight = 'bold';
a.BackgroundColor = 'white';
a.Position = pos_xy;
end

function config_solver(model)
set_param(model, 'Solver', 'ode1', 'SolverType', 'Fixed-step', ...
    'FixedStep', '0.002', 'StopTime', '30');
end

% =========================================================================
% HW_PID  -- cascade inner cart-PID + outer theta-PID
% =========================================================================
function out = build_HW_PID(target, legacy_model)
out = 'HW_PID';
if nargin && strcmp(target, 'NameOnly'), return; end

model = 'HW_PID';
try, bdclose(model); catch, end
new_system(model);
build_skeleton(model, legacy_model);
attach_x_obs_zeros(model);

L = layout();
add_block('built-in/Subsystem', [model '/PID_Controller'], 'Position', L.controller);
ctrl = [model '/PID_Controller'];
add_block('built-in/Inport',  [ctrl '/x_bus'], 'Position', [40 80 70 100]);
add_block('built-in/Outport', [ctrl '/u'],     'Position', [780 80 810 100]);
add_block('built-in/Demux',   [ctrl '/Demux'], 'Outputs', '4', 'Position', [110 30 115 170]);

add_block('built-in/Constant', [ctrl '/theta_ref'], 'Value', '0', 'Position', [160 30 200 50]);
add_block('built-in/Sum',      [ctrl '/Sum theta'], 'Inputs', '+-', 'Position', [240 50 260 90]);
add_block('simulink/Continuous/PID Controller', [ctrl '/Outer PID'], ...
    'P', 'Kp_out', 'I', 'Ki_out', 'D', 'Kd_out', 'N', 'N_out', ...
    'Position', [290 40 380 100]);
add_block('built-in/Gain', [ctrl '/Negate'], 'Gain', '-1', 'Position', [410 60 440 80]);

add_block('built-in/Sum', [ctrl '/Sum xc'], 'Inputs', '+-', 'Position', [490 80 510 120]);
add_block('simulink/Continuous/PID Controller', [ctrl '/Inner PID'], ...
    'P', 'Kp_in', 'I', 'Ki_in', 'D', 'Kd_in', 'N', 'N_in', ...
    'Position', [550 80 640 140]);

add_line(ctrl, 'x_bus/1',     'Demux/1',     'autorouting', 'on');
add_line(ctrl, 'theta_ref/1', 'Sum theta/1', 'autorouting', 'on');
add_line(ctrl, 'Demux/3',     'Sum theta/2', 'autorouting', 'on');
add_line(ctrl, 'Sum theta/1', 'Outer PID/1', 'autorouting', 'on');
add_line(ctrl, 'Outer PID/1', 'Negate/1',    'autorouting', 'on');
add_line(ctrl, 'Negate/1',    'Sum xc/1',    'autorouting', 'on');
add_line(ctrl, 'Demux/1',     'Sum xc/2',    'autorouting', 'on');
add_line(ctrl, 'Sum xc/1',    'Inner PID/1', 'autorouting', 'on');
add_line(ctrl, 'Inner PID/1', 'u/1',         'autorouting', 'on');

wire_controller(model, 'PID_Controller', false);
save_system(model, target);
fprintf('Saved model:   %s\n', target);
end

% =========================================================================
% HW_StateFeedback  -- u = -K * [x_bus; theta_int]  (PP, LQR, LQI)
% =========================================================================
function out = build_HW_StateFeedback(target, legacy_model)
out = 'HW_StateFeedback';
if nargin && strcmp(target, 'NameOnly'), return; end

model = 'HW_StateFeedback';
try, bdclose(model); catch, end
new_system(model);
build_skeleton(model, legacy_model);
attach_x_obs_zeros(model);

L = layout();
add_block('built-in/Subsystem', [model '/SF_Controller'], 'Position', L.controller);
ctrl = [model '/SF_Controller'];
add_block('built-in/Inport',  [ctrl '/x_bus'],     'Position', [40  60  70  80]);
add_block('built-in/Inport',  [ctrl '/theta_int'], 'Position', [40 100  70 120]);
add_block('built-in/Outport', [ctrl '/u'],         'Position', [400 80 430 100]);
add_block('built-in/Mux',     [ctrl '/Mux 5'], 'Inputs', '[4 1]', 'Position', [110 60 115 120]);
add_block('built-in/Gain',    [ctrl '/K_aug'], 'Gain', 'K_aug_hw', ...
    'Multiplication', 'Matrix(K*u)', 'Position', [150 70 240 110]);
add_block('built-in/Gain',    [ctrl '/Negate'], 'Gain', '-1', 'Position', [290 80 320 100]);
add_line(ctrl, 'x_bus/1',     'Mux 5/1', 'autorouting', 'on');
add_line(ctrl, 'theta_int/1', 'Mux 5/2', 'autorouting', 'on');
add_line(ctrl, 'Mux 5/1',     'K_aug/1', 'autorouting', 'on');
add_line(ctrl, 'K_aug/1',     'Negate/1','autorouting', 'on');
add_line(ctrl, 'Negate/1',    'u/1',     'autorouting', 'on');

wire_controller(model, 'SF_Controller', true);
save_system(model, target);
fprintf('Saved model:   %s\n', target);
end

% =========================================================================
% HW_StateFeedback_Observer  -- observer + u = -K * [x_obs; theta_int]
% =========================================================================
function out = build_HW_StateFeedback_Observer(target, legacy_model)
out = 'HW_StateFeedback_Observer';
if nargin && strcmp(target, 'NameOnly'), return; end

model = 'HW_StateFeedback_Observer';
try, bdclose(model); catch, end
new_system(model);
build_skeleton(model, legacy_model);

% --- Observer subsystem, sits above the Controller subsystem ---
L = layout();
add_block('built-in/Subsystem', [model '/Observer'], 'Position', [-580 -10 -450  50]);
obs = [model '/Observer'];
add_block('built-in/Inport',  [obs '/u_sat'],     'Position', [40  20  70  40]);
add_block('built-in/Inport',  [obs '/xc'],        'Position', [40  60  70  80]);
add_block('built-in/Inport',  [obs '/theta'],     'Position', [40 100  70 120]);
add_block('built-in/Outport', [obs '/x_obs'],     'Position', [400 60  430 80]);
add_block('built-in/Mux',     [obs '/Mux in'], 'Inputs', '3', 'Position', [130 30 135 130]);
add_block('built-in/StateSpace', [obs '/Observer SS'], ...
    'A', 'A_obs_hw', 'B', 'B_obs_hw', 'C', 'C_obs_hw', 'D', 'D_obs_hw', ...
    'InitialCondition', '0', 'Position', [180 40 320 120]);
add_line(obs, 'u_sat/1',     'Mux in/1', 'autorouting', 'on');
add_line(obs, 'xc/1',        'Mux in/2', 'autorouting', 'on');
add_line(obs, 'theta/1',     'Mux in/3', 'autorouting', 'on');
add_line(obs, 'Mux in/1',    'Observer SS/1', 'autorouting', 'on');
add_line(obs, 'Observer SS/1','x_obs/1', 'autorouting', 'on');

% Observer wiring at top level
add_block('built-in/From', [model '/From_usat_obs'],  'GotoTag', 'u_sat', 'Position', [-620  0 -590 20]);
add_block('built-in/From', [model '/From_xc_obs'],    'GotoTag', 'xc',    'Position', [-620 30 -590 50]);
add_block('built-in/From', [model '/From_theta_obs'], 'GotoTag', 'theta', 'Position', [-620 60 -590 80]);
add_block('built-in/Goto', [model '/[x_obs]'],        'GotoTag', 'x_obs', ...
    'TagVisibility', 'global', 'Position', [-410 10 -360 30]);
add_line(model, 'From_usat_obs/1',  'Observer/1', 'autorouting', 'on');
add_line(model, 'From_xc_obs/1',    'Observer/2', 'autorouting', 'on');
add_line(model, 'From_theta_obs/1', 'Observer/3', 'autorouting', 'on');
add_line(model, 'Observer/1',       '[x_obs]/1',  'autorouting', 'on');

% Controller: feeds from x_obs (not x_bus) + theta_int
add_block('built-in/Subsystem', [model '/SF_Controller'], 'Position', L.controller);
ctrl = [model '/SF_Controller'];
add_block('built-in/Inport',  [ctrl '/x_obs'],     'Position', [40  60  70  80]);
add_block('built-in/Inport',  [ctrl '/theta_int'], 'Position', [40 100  70 120]);
add_block('built-in/Outport', [ctrl '/u'],         'Position', [400 80 430 100]);
add_block('built-in/Mux',     [ctrl '/Mux 5'], 'Inputs', '[4 1]', 'Position', [110 60 115 120]);
add_block('built-in/Gain',    [ctrl '/K_aug'], 'Gain', 'K_aug_hw', ...
    'Multiplication', 'Matrix(K*u)', 'Position', [150 70 240 110]);
add_block('built-in/Gain',    [ctrl '/Negate'], 'Gain', '-1', 'Position', [290 80 320 100]);
add_line(ctrl, 'x_obs/1',     'Mux 5/1', 'autorouting', 'on');
add_line(ctrl, 'theta_int/1', 'Mux 5/2', 'autorouting', 'on');
add_line(ctrl, 'Mux 5/1',     'K_aug/1', 'autorouting', 'on');
add_line(ctrl, 'K_aug/1',     'Negate/1','autorouting', 'on');
add_line(ctrl, 'Negate/1',    'u/1',     'autorouting', 'on');

% Override: instead of From [x_bus], use From [x_obs] for the controller input
delete_block([model '/From_xbus_ctrl']);
add_block('built-in/From', [model '/From_xobs_ctrl'], 'GotoTag', 'x_obs', 'Position', L.from_xbus_ctrl);
add_block('built-in/From', [model '/From_theta_int_ctrl'], 'GotoTag', 'theta_int', 'Position', [-580 100 -540 120]);
add_line(model, 'From_xobs_ctrl/1',      'SF_Controller/1', 'autorouting', 'on');
add_line(model, 'From_theta_int_ctrl/1', 'SF_Controller/2', 'autorouting', 'on');
add_line(model, 'SF_Controller/1',       'Sum u+d/1',       'autorouting', 'on');

save_system(model, target);
fprintf('Saved model:   %s\n', target);
end

% =========================================================================
% HW_SMC  -- classical SMC, u = -K_eq*x - eta*tanh(S*x/phi)
% =========================================================================
function out = build_HW_SMC(target, legacy_model)
out = 'HW_SMC';
if nargin && strcmp(target, 'NameOnly'), return; end

model = 'HW_SMC';
try, bdclose(model); catch, end
new_system(model);
build_skeleton(model, legacy_model);
attach_x_obs_zeros(model);

L = layout();
add_block('built-in/Subsystem', [model '/SMC_Controller'], 'Position', L.controller);
ctrl = [model '/SMC_Controller'];
add_block('built-in/Inport',  [ctrl '/x_bus'], 'Position', [40 80 70 100]);
add_block('built-in/Outport', [ctrl '/u'],     'Position', [620 80 650 100]);

% u_eq branch:  -K_eq * x
add_block('built-in/Gain', [ctrl '/K_eq'], 'Gain', 'K_eq_smc_hw', ...
    'Multiplication', 'Matrix(K*u)', 'Position', [120 30 200 70]);
add_block('built-in/Gain', [ctrl '/Negate u_eq'], 'Gain', '-1', ...
    'Position', [230 40 260 60]);

% u_disc branch:  -eta * tanh( (S * x) / phi )
add_block('built-in/Gain', [ctrl '/Surface S'], 'Gain', 'S_smc_hw', ...
    'Multiplication', 'Matrix(K*u)', 'Position', [120 110 200 150]);
add_block('built-in/Gain', [ctrl '/1 over phi'], 'Gain', '1/phi_bl_smc_hw', ...
    'Position', [230 120 270 140]);
add_block('built-in/Trigonometry', [ctrl '/tanh'], 'Operator', 'tanh', ...
    'Position', [300 120 330 140]);
add_block('built-in/Gain', [ctrl '/neg eta'], 'Gain', '-eta_smc_hw', ...
    'Position', [360 120 400 140]);

add_block('built-in/Sum', [ctrl '/Sum u'], 'Inputs', '++', 'Position', [450 70 470 130]);

add_line(ctrl, 'x_bus/1',     'K_eq/1',       'autorouting', 'on');
add_line(ctrl, 'x_bus/1',     'Surface S/1',  'autorouting', 'on');
add_line(ctrl, 'K_eq/1',      'Negate u_eq/1','autorouting', 'on');
add_line(ctrl, 'Negate u_eq/1','Sum u/1',     'autorouting', 'on');
add_line(ctrl, 'Surface S/1', '1 over phi/1', 'autorouting', 'on');
add_line(ctrl, '1 over phi/1','tanh/1',       'autorouting', 'on');
add_line(ctrl, 'tanh/1',      'neg eta/1',    'autorouting', 'on');
add_line(ctrl, 'neg eta/1',   'Sum u/2',      'autorouting', 'on');
add_line(ctrl, 'Sum u/1',     'u/1',          'autorouting', 'on');

wire_controller(model, 'SMC_Controller', false);
save_system(model, target);
fprintf('Saved model:   %s\n', target);
end
