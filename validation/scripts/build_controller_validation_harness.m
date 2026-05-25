function build_controller_validation_harness()
%BUILD_CONTROLLER_VALIDATION_HARNESS Build the canonical theta-tracking harness.
%
% This model is simulation-first. It uses the shared protocol
% r_theta(t) -> theta(t), controller bank, observer bank, actuator path, and
% logger. Existing deployment models remain references; this is the common
% validation harness.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
run(fullfile(root, 'startup.m'));
generate_validation_protocol('root', root);
load_controller_validation_case('PP', 'dirty', 'root', root);

model_dir = fullfile(root, 'validation', 'models');
if ~exist(model_dir, 'dir')
    mkdir(model_dir);
end

mdl = 'ControllerValidationHarness';
model_file = fullfile(model_dir, [mdl '.slx']);
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist(model_file, 'file') == 2
    delete(model_file);
end

new_system(mdl);
open_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', 'Ts', 'StopTime', 'protocol.t(end)', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout');

add_top_level(mdl);
save_system(mdl, model_file);
fprintf('Built controller validation harness: %s\n', model_file);
end

function add_top_level(mdl)
add_subsystem(mdl, 'ProtocolSource', [40 95 170 185]);
add_subsystem(mdl, 'PlantInterface', [980 130 1130 260]);
add_subsystem(mdl, 'StateEstimation', [300 330 480 500]);
add_subsystem(mdl, 'ReferenceBuilder', [300 75 480 230]);
add_subsystem(mdl, 'ControllerBank', [560 95 760 285]);
add_subsystem(mdl, 'ControllerSelector', [805 145 920 245]);
add_subsystem(mdl, 'ActuatorInterface', [1180 145 1340 245]);
add_subsystem(mdl, 'Logger', [1410 115 1580 360]);

build_protocol_source([mdl '/ProtocolSource']);
build_reference_builder([mdl '/ReferenceBuilder']);
build_plant_interface([mdl '/PlantInterface']);
build_state_estimation([mdl '/StateEstimation']);
build_controller_bank([mdl '/ControllerBank']);
build_controller_selector([mdl '/ControllerSelector']);
build_actuator_interface([mdl '/ActuatorInterface']);
build_logger([mdl '/Logger']);

connect(mdl, 'ProtocolSource/1', 'ReferenceBuilder/1');
connect(mdl, 'ProtocolSource/2', 'Logger/1');
connect(mdl, 'ReferenceBuilder/1', 'ControllerBank/1');
connect(mdl, 'ReferenceBuilder/2', 'ControllerBank/2');
connect(mdl, 'ReferenceBuilder/3', 'ControllerBank/3');
connect(mdl, 'ReferenceBuilder/1', 'Logger/2');
connect(mdl, 'ReferenceBuilder/2', 'Logger/3');
connect(mdl, 'ReferenceBuilder/3', 'Logger/4');
connect(mdl, 'PlantInterface/1', 'StateEstimation/1');
connect(mdl, 'PlantInterface/2', 'StateEstimation/2');
connect(mdl, 'ActuatorInterface/1', 'StateEstimation/3');
connect(mdl, 'StateEstimation/1', 'ControllerBank/4');
connect(mdl, 'StateEstimation/2', 'ControllerBank/5');
connect(mdl, 'StateEstimation/3', 'ControllerBank/6');
connect(mdl, 'StateEstimation/4', 'ControllerBank/7');
connect(mdl, 'StateEstimation/4', 'Logger/5');
connect(mdl, 'StateEstimation/5', 'Logger/9');
connect(mdl, 'StateEstimation/6', 'Logger/10');
connect(mdl, 'StateEstimation/7', 'Logger/11');
connect(mdl, 'ControllerBank/1', 'ControllerSelector/1');
connect(mdl, 'ControllerBank/2', 'ControllerSelector/2');
connect(mdl, 'ControllerBank/3', 'ControllerSelector/3');
connect(mdl, 'ControllerBank/4', 'ControllerSelector/4');
connect(mdl, 'ControllerBank/5', 'ControllerSelector/5');
connect(mdl, 'ControllerBank/6', 'ControllerSelector/6');
connect(mdl, 'ControllerSelector/1', 'ActuatorInterface/1');
connect(mdl, 'ActuatorInterface/1', 'PlantInterface/1');
connect(mdl, 'ControllerSelector/1', 'Logger/6');
connect(mdl, 'ActuatorInterface/1', 'Logger/7');
connect(mdl, 'PlantInterface/1', 'Logger/8');
connect(mdl, 'PlantInterface/2', 'Logger/12');
end

function build_protocol_source(ss)
clear_system(ss);
add_block('simulink/Sources/From Workspace', [ss '/r_theta_ts'], 'VariableName', 'protocol.r_theta_ts', 'Position', [35 35 145 65]);
add_block('simulink/Sources/From Workspace', [ss '/segment_id_ts'], 'VariableName', 'protocol.segment_id_ts', 'Position', [35 100 145 130]);
add_block('simulink/Sinks/Out1', [ss '/r_theta'], 'Position', [210 40 240 60]);
add_block('simulink/Sinks/Out1', [ss '/segment_id'], 'Position', [210 105 240 125]);
connect(ss, 'r_theta_ts/1', 'r_theta/1');
connect(ss, 'segment_id_ts/1', 'segment_id/1');
end

function build_reference_builder(ss)
clear_system(ss);
add_block('simulink/Sources/In1', [ss '/r_theta'], 'Position', [30 70 60 90]);
add_block('simulink/Continuous/Derivative', [ss '/theta_ref_dot'], 'Position', [115 110 145 140]);
add_block('simulink/Continuous/Derivative', [ss '/theta_ref_ddot'], 'Position', [190 145 220 175]);
add_block('simulink/Sources/Constant', [ss '/zero'], 'Value', '0', 'Position', [110 25 140 45]);
add_block('simulink/Signal Routing/Mux', [ss '/x_ref_mux'], 'Inputs', '4', 'Position', [285 55 295 155]);
add_block('simulink/Math Operations/Gain', [ss '/theta_xss_gain'], 'Gain', 'theta_xss_gain', 'Multiplication', 'Matrix(K*u)', 'Position', [275 190 345 220]);
add_block('simulink/Math Operations/Gain', [ss '/theta_uss_gain'], 'Gain', 'theta_uss_gain', 'Position', [275 250 345 280]);
add_block('simulink/Sinks/Out1', [ss '/theta_ref'], 'Position', [420 40 450 60]);
add_block('simulink/Sinks/Out1', [ss '/x_ref'], 'Position', [420 90 450 110]);
add_block('simulink/Sinks/Out1', [ss '/u_ss'], 'Position', [420 255 450 275]);
connect(ss, 'r_theta/1', 'theta_ref/1');
connect(ss, 'r_theta/1', 'theta_ref_dot/1');
connect(ss, 'theta_ref_dot/1', 'theta_ref_ddot/1');
connect(ss, 'zero/1', 'x_ref_mux/1');
connect(ss, 'zero/1', 'x_ref_mux/2');
connect(ss, 'r_theta/1', 'x_ref_mux/3');
connect(ss, 'theta_ref_dot/1', 'x_ref_mux/4');
connect(ss, 'x_ref_mux/1', 'x_ref/1');
connect(ss, 'r_theta/1', 'theta_xss_gain/1');
connect(ss, 'r_theta/1', 'theta_uss_gain/1');
connect(ss, 'theta_uss_gain/1', 'u_ss/1');
end

function build_plant_interface(ss)
clear_system(ss);
add_block('simulink/Sources/In1', [ss '/u_motor'], 'Position', [30 65 60 85]);
add_block('simulink/Continuous/State-Space', [ss '/LinearPlant'], 'A', 'A_ctrl', 'B', 'B_ctrl', 'C', 'C_ctrl', 'D', 'D_ctrl', 'X0', '[0;0;0;0]', 'Position', [135 45 250 105]);
add_block('simulink/Signal Routing/Demux', [ss '/state_demux'], 'Outputs', '4', 'Position', [300 35 305 125]);
add_block('simulink/Sinks/Out1', [ss '/x_state'], 'Position', [360 55 390 75]);
add_block('simulink/Sinks/Out1', [ss '/theta'], 'Position', [360 100 390 120]);
connect(ss, 'u_motor/1', 'LinearPlant/1');
connect(ss, 'LinearPlant/1', 'x_state/1');
connect(ss, 'LinearPlant/1', 'state_demux/1');
connect(ss, 'state_demux/3', 'theta/1');
end

function build_state_estimation(ss)
clear_system(ss);
add_block('simulink/Sources/In1', [ss '/x_state'], 'Position', [25 50 55 70]);
add_block('simulink/Sources/In1', [ss '/theta'], 'Position', [25 115 55 135]);
add_block('simulink/Sources/In1', [ss '/u_motor'], 'Position', [25 180 55 200]);
add_block('simulink/Signal Routing/Demux', [ss '/state_demux'], 'Outputs', '4', 'Position', [95 35 100 125]);
add_block('simulink/Signal Routing/Mux', [ss '/measurement_mux'], 'Inputs', '2', 'Position', [180 145 190 205]);
add_block('simulink/Signal Routing/Mux', [ss '/observer_input_mux'], 'Inputs', '3', 'Position', [250 145 260 225]);
add_block('simulink/Continuous/State-Space', [ss '/Luenberger'], 'A', 'A_obs_l', 'B', 'B_obs_l', 'C', 'C_obs_l', 'D', 'D_obs_l', 'X0', '[0;0;0;0]', 'Position', [330 135 430 185]);
add_block('simulink/Continuous/State-Space', [ss '/Kalman'], 'A', 'A_obs_k', 'B', 'B_obs_k', 'C', 'C_obs_k', 'D', 'D_obs_k', 'X0', '[0;0;0;0]', 'Position', [330 215 430 265]);
add_block('simulink/Signal Routing/Multiport Switch', [ss '/FeedbackSelector'], 'DataPortOrder', 'One-based contiguous', 'Inputs', '3', 'Position', [520 80 585 220]);
add_block('simulink/Sources/Constant', [ss '/feedback_source_id'], 'Value', 'feedback_selector_id', 'Position', [405 55 475 80]);
add_block('simulink/Sinks/Out1', [ss '/x_dirty'], 'Position', [660 40 690 60]);
add_block('simulink/Sinks/Out1', [ss '/x_luenberger'], 'Position', [660 90 690 110]);
add_block('simulink/Sinks/Out1', [ss '/x_kalman'], 'Position', [660 140 690 160]);
add_block('simulink/Sinks/Out1', [ss '/x_feedback'], 'Position', [660 195 690 215]);
add_block('simulink/Sinks/Out1', [ss '/xc'], 'Position', [660 250 690 270]);
add_block('simulink/Sinks/Out1', [ss '/theta_out'], 'Position', [660 300 690 320]);
add_block('simulink/Sinks/Out1', [ss '/theta_dot'], 'Position', [660 350 690 370]);
connect(ss, 'x_state/1', 'state_demux/1');
connect(ss, 'x_state/1', 'x_dirty/1');
connect(ss, 'x_state/1', 'FeedbackSelector/2');
connect(ss, 'u_motor/1', 'observer_input_mux/1');
connect(ss, 'state_demux/1', 'measurement_mux/1');
connect(ss, 'state_demux/3', 'measurement_mux/2');
connect(ss, 'state_demux/1', 'observer_input_mux/2');
connect(ss, 'state_demux/3', 'observer_input_mux/3');
connect(ss, 'observer_input_mux/1', 'Luenberger/1');
connect(ss, 'observer_input_mux/1', 'Kalman/1');
connect(ss, 'Luenberger/1', 'x_luenberger/1');
connect(ss, 'Kalman/1', 'x_kalman/1');
connect(ss, 'Luenberger/1', 'FeedbackSelector/3');
connect(ss, 'Kalman/1', 'FeedbackSelector/4');
connect(ss, 'feedback_source_id/1', 'FeedbackSelector/1');
connect(ss, 'FeedbackSelector/1', 'x_feedback/1');
connect(ss, 'state_demux/1', 'xc/1');
connect(ss, 'state_demux/3', 'theta_out/1');
connect(ss, 'state_demux/4', 'theta_dot/1');
end

function build_controller_bank(ss)
clear_system(ss);
ins = {'theta_ref','x_ref','u_ss','x_dirty','x_luenberger','x_kalman','x_feedback'};
for i = 1:numel(ins)
    add_block('simulink/Sources/In1', [ss '/' ins{i}], 'Position', [30 30+45*i 60 50+45*i]);
end

add_state_feedback_controller(ss, 'PP', 'K_pp', 150, 65, false);
add_augmented_controller(ss, 'PP_Integral', 'K_aug', 150, 140);
add_state_feedback_controller(ss, 'LQR', 'K_lqr', 150, 215, false);
add_augmented_controller(ss, 'LQI_LQG', 'K_lqi', 150, 290);
add_pid_controller(ss, 150, 380);
add_smc_controller(ss, 150, 500);

outs = {'u_pp','u_pp_integral','u_lqr','u_lqi_lqg','u_pid','u_smc'};
for i = 1:numel(outs)
    add_block('simulink/Sinks/Out1', [ss '/' outs{i}], 'Position', [780 65+75*(i-1) 810 85+75*(i-1)]);
end
connect(ss, 'PP/1', 'u_pp/1');
connect(ss, 'PP_Integral/1', 'u_pp_integral/1');
connect(ss, 'LQR/1', 'u_lqr/1');
connect(ss, 'LQI_LQG/1', 'u_lqi_lqg/1');
connect(ss, 'PID_Cascade/1', 'u_pid/1');
connect(ss, 'SMC_STA/1', 'u_smc/1');

% Common inputs to subsystems.
connect(ss, 'theta_ref/1', 'PP_Integral/1');
connect(ss, 'theta_ref/1', 'LQI_LQG/1');
connect(ss, 'theta_ref/1', 'PID_Cascade/1');
connect(ss, 'theta_ref/1', 'SMC_STA/1');
connect(ss, 'x_ref/1', 'PP/1');
connect(ss, 'x_ref/1', 'LQR/1');
connect(ss, 'x_ref/1', 'SMC_STA/2');
connect(ss, 'u_ss/1', 'PP/2');
connect(ss, 'u_ss/1', 'LQR/2');
connect(ss, 'x_feedback/1', 'PP/3');
connect(ss, 'x_feedback/1', 'PP_Integral/2');
connect(ss, 'x_feedback/1', 'LQR/3');
connect(ss, 'x_feedback/1', 'LQI_LQG/2');
connect(ss, 'x_feedback/1', 'PID_Cascade/2');
connect(ss, 'x_feedback/1', 'SMC_STA/3');
end

function add_state_feedback_controller(parent, name, gain_var, x, y, ~)
ss = [parent '/' name];
add_subsystem(parent, name, [x y x+180 y+90]);
clear_system(ss);
add_block('simulink/Sources/In1', [ss '/x_ref'], 'Position', [25 30 55 50]);
add_block('simulink/Sources/In1', [ss '/u_ss'], 'Position', [25 80 55 100]);
add_block('simulink/Sources/In1', [ss '/x_feedback'], 'Position', [25 130 55 150]);
add_block('simulink/Math Operations/Sum', [ss '/x_error'], 'Inputs', '+-', 'Position', [120 65 150 115]);
add_block('simulink/Math Operations/Gain', [ss '/K'], 'Gain', ['-' gain_var], 'Multiplication', 'Matrix(K*u)', 'Position', [210 75 265 105]);
add_block('simulink/Math Operations/Sum', [ss '/add_u_ss'], 'Inputs', '++', 'Position', [315 75 345 105]);
add_block('simulink/Sinks/Out1', [ss '/u'], 'Position', [400 85 430 105]);
connect(ss, 'x_ref/1', 'x_error/1');
connect(ss, 'x_feedback/1', 'x_error/2');
connect(ss, 'x_error/1', 'K/1');
connect(ss, 'u_ss/1', 'add_u_ss/1');
connect(ss, 'K/1', 'add_u_ss/2');
connect(ss, 'add_u_ss/1', 'u/1');
end

function add_augmented_controller(parent, name, gain_var, x, y)
ss = [parent '/' name];
add_subsystem(parent, name, [x y x+210 y+100]);
clear_system(ss);
add_block('simulink/Sources/In1', [ss '/theta_ref'], 'Position', [25 30 55 50]);
add_block('simulink/Sources/In1', [ss '/x_feedback'], 'Position', [25 95 55 115]);
add_block('simulink/Signal Routing/Demux', [ss '/state_demux'], 'Outputs', '4', 'Position', [100 75 105 145]);
add_block('simulink/Math Operations/Sum', [ss '/theta_error'], 'Inputs', '+-', 'Position', [165 35 195 65]);
add_block('simulink/Continuous/Integrator', [ss '/integral_theta_error'], 'InitialCondition', '0', 'Position', [235 35 265 65]);
add_block('simulink/Signal Routing/Mux', [ss '/aug_state'], 'Inputs', '5', 'Position', [325 70 335 165]);
add_block('simulink/Math Operations/Gain', [ss '/K_aug'], 'Gain', ['-' gain_var], 'Multiplication', 'Matrix(K*u)', 'Position', [390 100 450 130]);
add_block('simulink/Sinks/Out1', [ss '/u'], 'Position', [500 105 530 125]);
connect(ss, 'x_feedback/1', 'state_demux/1');
connect(ss, 'theta_ref/1', 'theta_error/1');
connect(ss, 'state_demux/3', 'theta_error/2');
connect(ss, 'theta_error/1', 'integral_theta_error/1');
for i = 1:4
    connect(ss, sprintf('state_demux/%d', i), sprintf('aug_state/%d', i));
end
connect(ss, 'integral_theta_error/1', 'aug_state/5');
connect(ss, 'aug_state/1', 'K_aug/1');
connect(ss, 'K_aug/1', 'u/1');
end

function add_pid_controller(parent, x, y)
ss = [parent '/PID_Cascade'];
add_subsystem(parent, 'PID_Cascade', [x y x+235 y+110]);
clear_system(ss);
add_block('simulink/Sources/In1', [ss '/theta_ref'], 'Position', [25 30 55 50]);
add_block('simulink/Sources/In1', [ss '/x_feedback'], 'Position', [25 90 55 110]);
add_block('simulink/Signal Routing/Demux', [ss '/state_demux'], 'Outputs', '4', 'Position', [95 75 100 145]);
add_block('simulink/Math Operations/Sum', [ss '/theta_error'], 'Inputs', '+-', 'Position', [150 35 180 65]);
add_block('simulink/Continuous/Transfer Fcn', [ss '/Outer_PID'], 'Numerator', 'pid_outer_num', 'Denominator', 'pid_outer_den', 'Position', [225 30 310 70]);
add_block('simulink/Math Operations/Sum', [ss '/cart_error'], 'Inputs', '+-', 'Position', [365 50 395 80]);
add_block('simulink/Continuous/Transfer Fcn', [ss '/Inner_PID'], 'Numerator', 'pid_inner_num', 'Denominator', 'pid_inner_den', 'Position', [445 45 530 85]);
add_block('simulink/Sinks/Out1', [ss '/u'], 'Position', [590 55 620 75]);
connect(ss, 'x_feedback/1', 'state_demux/1');
connect(ss, 'theta_ref/1', 'theta_error/1');
connect(ss, 'state_demux/3', 'theta_error/2');
connect(ss, 'theta_error/1', 'Outer_PID/1');
connect(ss, 'Outer_PID/1', 'cart_error/1');
connect(ss, 'state_demux/1', 'cart_error/2');
connect(ss, 'cart_error/1', 'Inner_PID/1');
connect(ss, 'Inner_PID/1', 'u/1');
end

function add_smc_controller(parent, x, y)
ss = [parent '/SMC_STA'];
add_subsystem(parent, 'SMC_STA', [x y x+250 y+115]);
clear_system(ss);
add_block('simulink/Sources/In1', [ss '/theta_ref'], 'Position', [25 25 55 45]);
add_block('simulink/Sources/In1', [ss '/x_ref'], 'Position', [25 75 55 95]);
add_block('simulink/Sources/In1', [ss '/x_feedback'], 'Position', [25 125 55 145]);
add_block('simulink/Math Operations/Sum', [ss '/x_error'], 'Inputs', '+-', 'Position', [105 85 135 135]);
add_block('simulink/Math Operations/Gain', [ss '/SlidingSurface'], 'Gain', 'S_smc', 'Multiplication', 'Matrix(K*u)', 'Position', [180 90 250 120]);
add_block('simulink/Math Operations/Gain', [ss '/EquivalentControl'], 'Gain', '-K_eq_smc', 'Multiplication', 'Matrix(K*u)', 'Position', [180 150 250 180]);
add_block('simulink/Math Operations/Gain', [ss '/BoundaryScale'], 'Gain', '1/phi_smc', 'Position', [300 85 360 115]);
add_block('simulink/Discontinuities/Saturation', [ss '/BoundarySaturation'], 'UpperLimit', '1', 'LowerLimit', '-1', 'Position', [400 85 460 115]);
add_block('simulink/Math Operations/Gain', [ss '/ReachingGain'], 'Gain', '-k1_smc', 'Position', [510 80 570 110]);
add_block('simulink/Math Operations/Gain', [ss '/IntegratorGain'], 'Gain', '-k2_smc', 'Position', [510 135 570 165]);
add_block('simulink/Continuous/Integrator', [ss '/STAIntegrator'], 'InitialCondition', 'smc_int_ic', 'Position', [610 135 640 165]);
add_block('simulink/Math Operations/Sum', [ss '/ReachingSum'], 'Inputs', '++', 'Position', [685 95 715 140]);
add_block('simulink/Math Operations/Gain', [ss '/InputNormalize'], 'Gain', '1/SB_smc', 'Position', [755 100 815 130]);
add_block('simulink/Math Operations/Sum', [ss '/TotalControl'], 'Inputs', '++', 'Position', [860 125 890 170]);
add_block('simulink/Sinks/Out1', [ss '/u'], 'Position', [950 140 980 160]);
connect(ss, 'x_feedback/1', 'x_error/1');
connect(ss, 'x_ref/1', 'x_error/2');
connect(ss, 'x_error/1', 'SlidingSurface/1');
connect(ss, 'x_error/1', 'EquivalentControl/1');
connect(ss, 'SlidingSurface/1', 'BoundaryScale/1');
connect(ss, 'BoundaryScale/1', 'BoundarySaturation/1');
connect(ss, 'BoundarySaturation/1', 'ReachingGain/1');
connect(ss, 'BoundarySaturation/1', 'IntegratorGain/1');
connect(ss, 'IntegratorGain/1', 'STAIntegrator/1');
connect(ss, 'ReachingGain/1', 'ReachingSum/1');
connect(ss, 'STAIntegrator/1', 'ReachingSum/2');
connect(ss, 'ReachingSum/1', 'InputNormalize/1');
connect(ss, 'EquivalentControl/1', 'TotalControl/1');
connect(ss, 'InputNormalize/1', 'TotalControl/2');
connect(ss, 'TotalControl/1', 'u/1');
end

function build_controller_selector(ss)
clear_system(ss);
for i = 1:6
    add_block('simulink/Sources/In1', [ss '/u' num2str(i)], 'Position', [35 20+40*i 65 40+40*i]);
end
add_block('simulink/Sources/Constant', [ss '/controller_selector_id'], 'Value', 'controller_selector_id', 'Position', [35 20 110 45]);
add_block('simulink/Signal Routing/Multiport Switch', [ss '/selector'], 'DataPortOrder', 'One-based contiguous', 'Inputs', '6', 'Position', [175 60 245 275]);
add_block('simulink/Sinks/Out1', [ss '/u_controller'], 'Position', [315 160 345 180]);
connect(ss, 'controller_selector_id/1', 'selector/1');
for i = 1:6
    connect(ss, ['u' num2str(i) '/1'], sprintf('selector/%d', i+1));
end
connect(ss, 'selector/1', 'u_controller/1');
end

function build_actuator_interface(ss)
clear_system(ss);
add_block('simulink/Sources/In1', [ss '/u_controller'], 'Position', [30 55 60 75]);
add_block('simulink/Sources/Constant', [ss '/ud_pos'], 'Value', 'ud_pos', 'Position', [95 10 145 30]);
add_block('simulink/Sources/Constant', [ss '/ud_neg'], 'Value', '-ud_neg', 'Position', [95 120 145 140]);
add_block('simulink/Sources/Constant', [ss '/zero'], 'Value', '0', 'Position', [95 175 145 195]);
add_block('simulink/Math Operations/Sum', [ss '/add_positive_comp'], 'Inputs', '++', 'Position', [190 25 220 55]);
add_block('simulink/Math Operations/Sum', [ss '/add_negative_comp'], 'Inputs', '++', 'Position', [190 105 220 135]);
add_block('simulink/Signal Routing/Switch', [ss '/positive_switch'], 'Threshold', '0.1', 'Criteria', 'u2 > Threshold', 'Position', [285 45 335 95]);
add_block('simulink/Math Operations/Gain', [ss '/negate_u'], 'Gain', '-1', 'Position', [285 125 335 155]);
add_block('simulink/Signal Routing/Switch', [ss '/negative_switch'], 'Threshold', '0.1', 'Criteria', 'u2 > Threshold', 'Position', [390 65 440 115]);
add_block('simulink/Discontinuities/Saturation', [ss '/FinalSaturation'], 'UpperLimit', 'V_sat_hw', 'LowerLimit', '-V_sat_hw', 'Position', [495 75 555 115]);
add_block('simulink/Sinks/Out1', [ss '/u_motor'], 'Position', [620 85 650 105]);
connect(ss, 'u_controller/1', 'add_positive_comp/1');
connect(ss, 'ud_pos/1', 'add_positive_comp/2');
connect(ss, 'u_controller/1', 'add_negative_comp/1');
connect(ss, 'ud_neg/1', 'add_negative_comp/2');
connect(ss, 'add_positive_comp/1', 'positive_switch/1');
connect(ss, 'u_controller/1', 'positive_switch/2');
connect(ss, 'zero/1', 'positive_switch/3');
connect(ss, 'add_negative_comp/1', 'negative_switch/1');
connect(ss, 'u_controller/1', 'negate_u/1');
connect(ss, 'negate_u/1', 'negative_switch/2');
connect(ss, 'positive_switch/1', 'negative_switch/3');
connect(ss, 'negative_switch/1', 'FinalSaturation/1');
connect(ss, 'FinalSaturation/1', 'u_motor/1');
end

function build_logger(ss)
clear_system(ss);
names = {'segment_id','theta_ref','x_ref','u_ss','x_feedback','u_controller','u_motor','theta','x_dirty','x_luenberger','x_kalman','x_state'};
add_block('simulink/Signal Routing/Mux', [ss '/log_mux'], 'Inputs', num2str(numel(names)), 'Position', [270 40 280 40+25*numel(names)]);
for i = 1:numel(names)
    add_block('simulink/Sources/In1', [ss '/' names{i}], 'Position', [35 20+25*i 65 40+25*i]);
    connect(ss, [names{i} '/1'], sprintf('log_mux/%d', i));
end
add_block('simulink/Sinks/To Workspace', [ss '/validation_log'], 'VariableName', 'validation_log', 'SaveFormat', 'Timeseries', 'Position', [360 150 445 180]);
connect(ss, 'log_mux/1', 'validation_log/1');
end

function add_subsystem(parent, name, pos)
add_block('simulink/Ports & Subsystems/Subsystem', [parent '/' name], 'Position', pos);
end

function clear_system(ss)
blocks = find_system(ss, 'SearchDepth', 1, 'Type', 'Block');
for i = 2:numel(blocks)
    delete_block(blocks{i});
end
lines = find_system(ss, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');
for i = 1:numel(lines)
    delete_line(lines(i));
end
end

function connect(sys, src, dst)
try
    add_line(sys, src, dst, 'autorouting', 'smart');
catch
    add_line(sys, src, dst);
end
end
