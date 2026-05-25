function build_controller_validation_harness(varargin)
%BUILD_CONTROLLER_VALIDATION_HARNESS Build a clean theta-tracking harness.
%
% The model contains one Controller subsystem at a time. Batch tests
% rebuild that subsystem for each controller case, which keeps the diagram
% readable and prevents inactive controller branches from compiling.

opts = parse_inputs(varargin{:});
run(fullfile(opts.root, 'startup.m'));
generate_validation_protocol('root', opts.root);
load_controller_validation_case(opts.controller_id, opts.feedback_source_id, 'root', opts.root);

model_dir = fullfile(opts.root, 'validation', 'models');
if ~exist(model_dir, 'dir')
    mkdir(model_dir);
end

mdl = 'ControllerValidationHarness';
model_file = fullfile(model_dir, [mdl '.slx']);
shadow_warning = warning('off', 'Simulink:Engine:MdlFileShadowing');
restore_shadow_warning = onCleanup(@() warning(shadow_warning));

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist(model_file, 'file') == 2
    open_system(model_file);
    clear_system(mdl);
else
    new_system(mdl);
    open_system(mdl);
end
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', 'Ts', 'StopTime', 'protocol.t(end)', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout');

build_top_level(mdl, opts.controller_id);
save_system(mdl, model_file);
fprintf('Built %s with controller=%s, feedback=%s\n', model_file, opts.controller_id, opts.feedback_source_id);
end

function build_top_level(mdl, controller_id)
add_subsystem(mdl, 'Theta_Protocol', [90 230 230 320]);
add_subsystem(mdl, 'Steady_State_Targets', [420 220 590 340]);
add_subsystem(mdl, 'Controller', [800 210 1000 355]);
add_subsystem(mdl, 'Motor_Voltage', [1230 220 1410 340]);
add_subsystem(mdl, 'Seesaw_Plant', [1640 220 1810 340]);
add_subsystem(mdl, 'State_Feedback', [1640 520 1830 700]);
add_subsystem(mdl, 'Validation_Log', [2160 265 2350 625]);

build_protocol_source([mdl '/Theta_Protocol']);
build_reference_builder([mdl '/Steady_State_Targets']);
build_controller_under_test([mdl '/Controller'], controller_id);
build_actuator_interface([mdl '/Motor_Voltage']);
build_plant_interface([mdl '/Seesaw_Plant']);
build_state_estimation([mdl '/State_Feedback']);
build_logger([mdl '/Validation_Log']);

style_top_level(mdl, controller_id);
wire_top_level_with_tags(mdl);
end

function wire_top_level_with_tags(mdl)
publish_signal(mdl, 'Theta_Protocol/1', 'r_theta', 'tag_r_theta', [255 235 315 255]);
publish_signal(mdl, 'Theta_Protocol/2', 'segment_id', 'tag_segment_id', [255 290 315 310]);
publish_signal(mdl, 'Steady_State_Targets/1', 'theta_ref', 'tag_theta_ref', [620 225 695 245]);
publish_signal(mdl, 'Steady_State_Targets/2', 'x_ref', 'tag_x_ref', [620 265 695 285]);
publish_signal(mdl, 'Steady_State_Targets/3', 'u_ff', 'tag_u_ff', [620 305 695 325]);
publish_signal(mdl, 'Controller/1', 'V_cmd', 'tag_V_cmd', [1025 265 1115 285]);
publish_signal(mdl, 'Motor_Voltage/1', 'V_m', 'tag_V_m', [1440 270 1525 290]);
publish_signal(mdl, 'Seesaw_Plant/1', 'x_state', 'tag_x_state', [1845 240 1935 260]);
publish_signal(mdl, 'Seesaw_Plant/2', 'theta', 'tag_theta', [1845 295 1935 315]);
publish_signal(mdl, 'State_Feedback/1', 'x_measured', 'tag_x_measured', [1875 535 1975 555]);
publish_signal(mdl, 'State_Feedback/2', 'x_luenberger', 'tag_x_luenberger', [1875 580 1995 600]);
publish_signal(mdl, 'State_Feedback/3', 'x_kalman', 'tag_x_kalman', [1875 625 1975 645]);
publish_signal(mdl, 'State_Feedback/4', 'x_feedback', 'tag_x_feedback', [1875 670 1995 690]);

subscribe_signal(mdl, 'r_theta', 'r_theta_to_targets', 'Steady_State_Targets/1', [360 255 390 275]);
subscribe_signal(mdl, 'theta_ref', 'theta_ref_to_controller', 'Controller/1', [745 220 775 240]);
subscribe_signal(mdl, 'x_ref', 'x_ref_to_controller', 'Controller/2', [745 255 775 275]);
subscribe_signal(mdl, 'u_ff', 'u_ff_to_controller', 'Controller/3', [745 290 775 310]);
subscribe_signal(mdl, 'x_feedback', 'x_feedback_to_controller', 'Controller/4', [745 325 775 345]);
subscribe_signal(mdl, 'V_cmd', 'V_cmd_to_motor_voltage', 'Motor_Voltage/1', [1170 270 1205 290]);
subscribe_signal(mdl, 'V_m', 'V_m_to_plant', 'Seesaw_Plant/1', [1585 255 1615 275]);
subscribe_signal(mdl, 'x_state', 'x_state_to_feedback', 'State_Feedback/1', [1585 560 1615 580]);
subscribe_signal(mdl, 'V_m', 'V_m_to_feedback', 'State_Feedback/2', [1585 620 1615 640]);

subscribe_signal(mdl, 'segment_id', 'log_segment_id', 'Validation_Log/1', [2100 290 2140 310]);
subscribe_signal(mdl, 'theta_ref', 'log_theta_ref', 'Validation_Log/2', [2100 315 2140 335]);
subscribe_signal(mdl, 'x_ref', 'log_x_ref', 'Validation_Log/3', [2100 340 2140 360]);
subscribe_signal(mdl, 'u_ff', 'log_u_ff', 'Validation_Log/4', [2100 365 2140 385]);
subscribe_signal(mdl, 'x_feedback', 'log_x_feedback', 'Validation_Log/5', [2100 390 2140 410]);
subscribe_signal(mdl, 'V_cmd', 'log_V_cmd', 'Validation_Log/6', [2100 415 2140 435]);
subscribe_signal(mdl, 'V_m', 'log_V_m', 'Validation_Log/7', [2100 440 2140 460]);
subscribe_signal(mdl, 'theta', 'log_theta', 'Validation_Log/8', [2100 465 2140 485]);
subscribe_signal(mdl, 'x_measured', 'log_x_measured', 'Validation_Log/9', [2100 490 2140 510]);
subscribe_signal(mdl, 'x_luenberger', 'log_x_luenberger', 'Validation_Log/10', [2100 515 2140 535]);
subscribe_signal(mdl, 'x_kalman', 'log_x_kalman', 'Validation_Log/11', [2100 540 2140 560]);
end

function style_top_level(mdl, controller_id)
style_block(mdl, 'Theta_Protocol', 'lightBlue');
style_block(mdl, 'Steady_State_Targets', 'cyan');
style_block(mdl, 'Controller', 'yellow');
style_block(mdl, 'Motor_Voltage', 'orange');
style_block(mdl, 'Seesaw_Plant', 'green');
style_block(mdl, 'State_Feedback', 'magenta');
style_block(mdl, 'Validation_Log', 'gray');

add_note(mdl, sprintf('Theta Tracking Validation Harness\nController: %s', controller_id), [80 35 620 90], 'white');
add_note(mdl, sprintf('Use this model\nBuild/select case: build_controller_validation_harness(''controller_id'',''PP'',''feedback_source_id'',''measured'')\nRun current case: sim(''ControllerValidationHarness'')\nBatch: run_controller_validation_suite(''quick'',true); analyze_controller_validation_suite'), [690 25 2050 115], 'white');
add_note(mdl, sprintf('01 Theta Protocol\nfree-run, theta steps, pulse, stepped sine sweep'), [75 145 310 205], 'lightBlue');
add_note(mdl, sprintf('02 Steady-State Targets\ncreates theta_ref, x_ref, u_ff'), [395 145 645 205], 'cyan');
add_note(mdl, sprintf('03 Controller\none selected controller per generated model'), [770 145 1060 200], 'yellow');
add_note(mdl, sprintf('04 Motor Voltage\ndead-zone compensation + final +/-6 V saturation'), [1140 145 1480 200], 'orange');
add_note(mdl, sprintf('05 Plant + State Feedback\nlinear plant; measured, Luenberger, Kalman state'), [1585 145 1945 200], 'green');
add_note(mdl, sprintf('06 Validation Log\nwrites validation_log timeseries'), [2075 180 2400 240], 'gray');
end

function build_protocol_source(ss)
clear_system(ss);
add_note(ss, sprintf('Theta reference protocol\nfree-run, steps, pulse, sine sweep'), [20 10 260 45], 'lightBlue');
add_block('simulink/Sources/From Workspace', [ss '/r_theta_ts'], 'VariableName', 'protocol.r_theta_ts', 'Position', [45 70 175 100]);
add_block('simulink/Sources/From Workspace', [ss '/segment_id_ts'], 'VariableName', 'protocol.segment_id_ts', 'Position', [45 145 175 175]);
add_out(ss, 'r_theta', [270 75 300 95]);
add_out(ss, 'segment_id', [270 150 300 170]);
connect(ss, 'r_theta_ts/1', 'r_theta/1');
connect(ss, 'segment_id_ts/1', 'segment_id/1');
end

function build_reference_builder(ss)
clear_system(ss);
add_note(ss, sprintf('Steady-state target map\nr_theta -> theta_ref, x_ref, u_ff'), [25 10 335 45], 'cyan');
add_in(ss, 'r_theta', [35 115 65 135]);
add_block('simulink/Math Operations/Gain', [ss '/theta_to_x_ref'], 'Gain', 'theta_xss_gain', 'Multiplication', 'Matrix(K*u)', 'Position', [165 80 285 120]);
add_block('simulink/Math Operations/Gain', [ss '/theta_to_u_ff'], 'Gain', 'theta_uff_gain', 'Position', [165 170 285 200]);
add_out(ss, 'theta_ref', [395 70 425 90]);
add_out(ss, 'x_ref', [395 115 425 135]);
add_out(ss, 'u_ff', [395 180 425 200]);
connect(ss, 'r_theta/1', 'theta_ref/1');
connect(ss, 'r_theta/1', 'theta_to_x_ref/1');
connect(ss, 'theta_to_x_ref/1', 'x_ref/1');
connect(ss, 'r_theta/1', 'theta_to_u_ff/1');
connect(ss, 'theta_to_u_ff/1', 'u_ff/1');
end

function build_controller_under_test(ss, controller_id)
clear_system(ss);
add_note(ss, sprintf('Controller: %s\nInputs are reference targets plus selected feedback state.', upper(char(controller_id))), [25 10 460 55], 'yellow');
add_in(ss, 'theta_ref', [25 75 55 95]);
add_in(ss, 'x_ref', [25 135 55 155]);
add_in(ss, 'u_ff', [25 195 55 215]);
add_in(ss, 'x_feedback', [25 255 55 275]);

switch lower(string(controller_id))
    case {"pp", "pole_placement"}
        add_state_feedback_controller(ss, 'K_pp');
    case {"lqr"}
        add_state_feedback_controller(ss, 'K_lqr');
    case {"pp_integral", "ppi"}
        add_integral_state_feedback_controller(ss, 'K_aug');
    case {"lqi", "lqg"}
        add_integral_state_feedback_controller(ss, 'K_lqi');
    case "pid"
        add_pid_cascade_controller(ss);
    case "smc"
        add_smc_controller(ss);
    otherwise
        error('Unsupported controller_id for harness build: %s', controller_id);
end
end

function add_state_feedback_controller(ss, gain_var)
add_block('simulink/Math Operations/Sum', [ss '/x_error'], 'Inputs', '+-', 'Position', [155 205 190 260]);
add_block('simulink/Math Operations/Gain', [ss '/K'], 'Gain', ['-' gain_var], 'Multiplication', 'Matrix(K*u)', 'Position', [285 215 365 245]);
add_block('simulink/Math Operations/Sum', [ss '/add_u_ff'], 'Inputs', '++', 'Position', [465 215 500 260]);
add_out(ss, 'V_cmd', [600 225 630 245]);
add_terminator(ss, 'unused_theta_ref', [115 75 140 95]);
connect(ss, 'theta_ref/1', 'unused_theta_ref/1');
connect(ss, 'x_feedback/1', 'x_error/1');
connect(ss, 'x_ref/1', 'x_error/2');
connect(ss, 'x_error/1', 'K/1');
connect(ss, 'u_ff/1', 'add_u_ff/1');
connect(ss, 'K/1', 'add_u_ff/2');
connect(ss, 'add_u_ff/1', 'V_cmd/1');
end

function add_integral_state_feedback_controller(ss, gain_var)
add_block('simulink/Signal Routing/Demux', [ss '/state_demux'], 'Outputs', '4', 'Position', [135 225 140 315]);
add_block('simulink/Math Operations/Sum', [ss '/theta_error'], 'Inputs', '+-', 'Position', [220 80 255 115]);
add_block('simulink/Continuous/Integrator', [ss '/integral_theta_error'], 'InitialCondition', '0', 'Position', [330 80 370 115]);
add_block('simulink/Math Operations/Sum', [ss '/x_error'], 'Inputs', '+-', 'Position', [220 190 255 245]);
add_block('simulink/Signal Routing/Mux', [ss '/augmented_error'], 'Inputs', '2', 'Position', [445 175 455 265]);
add_block('simulink/Math Operations/Gain', [ss '/K_augmented'], 'Gain', ['-' gain_var], 'Multiplication', 'Matrix(K*u)', 'Position', [550 205 650 235]);
add_out(ss, 'V_cmd', [740 210 770 230]);
add_terminator(ss, 'unused_u_ff', [115 195 140 215]);
connect(ss, 'u_ff/1', 'unused_u_ff/1');
connect(ss, 'x_feedback/1', 'state_demux/1');
connect(ss, 'theta_ref/1', 'theta_error/1');
connect(ss, 'state_demux/3', 'theta_error/2');
connect(ss, 'theta_error/1', 'integral_theta_error/1');
connect(ss, 'x_feedback/1', 'x_error/1');
connect(ss, 'x_ref/1', 'x_error/2');
connect(ss, 'x_error/1', 'augmented_error/1');
connect(ss, 'integral_theta_error/1', 'augmented_error/2');
connect(ss, 'augmented_error/1', 'K_augmented/1');
connect(ss, 'K_augmented/1', 'V_cmd/1');
end

function add_pid_cascade_controller(ss)
add_block('simulink/Signal Routing/Demux', [ss '/state_demux'], 'Outputs', '4', 'Position', [135 245 140 335]);
add_block('simulink/Math Operations/Sum', [ss '/theta_error'], 'Inputs', '+-', 'Position', [220 75 255 110]);
add_block('simulink/Continuous/Transfer Fcn', [ss '/Outer_PID'], 'Numerator', 'pid_outer_num', 'Denominator', 'pid_outer_den', 'Position', [330 70 430 115]);
add_block('simulink/Math Operations/Sum', [ss '/cart_error'], 'Inputs', '+-', 'Position', [515 105 550 140]);
add_block('simulink/Continuous/Transfer Fcn', [ss '/Inner_PID'], 'Numerator', 'pid_inner_num', 'Denominator', 'pid_inner_den', 'Position', [640 100 740 145]);
add_out(ss, 'V_cmd', [835 110 865 130]);
add_terminator(ss, 'unused_x_ref', [115 135 140 155]);
add_terminator(ss, 'unused_u_ff', [115 195 140 215]);
connect(ss, 'x_ref/1', 'unused_x_ref/1');
connect(ss, 'u_ff/1', 'unused_u_ff/1');
connect(ss, 'x_feedback/1', 'state_demux/1');
connect(ss, 'theta_ref/1', 'theta_error/1');
connect(ss, 'state_demux/3', 'theta_error/2');
connect(ss, 'theta_error/1', 'Outer_PID/1');
connect(ss, 'Outer_PID/1', 'cart_error/1');
connect(ss, 'state_demux/1', 'cart_error/2');
connect(ss, 'cart_error/1', 'Inner_PID/1');
connect(ss, 'Inner_PID/1', 'V_cmd/1');
end

function add_smc_controller(ss)
add_block('simulink/Math Operations/Sum', [ss '/x_error'], 'Inputs', '+-', 'Position', [155 220 190 275]);
add_block('simulink/Math Operations/Gain', [ss '/SlidingSurface'], 'Gain', 'S_smc', 'Multiplication', 'Matrix(K*u)', 'Position', [285 190 375 220]);
add_block('simulink/Math Operations/Gain', [ss '/EquivalentControl'], 'Gain', '-K_eq_smc', 'Multiplication', 'Matrix(K*u)', 'Position', [285 285 375 315]);
add_block('simulink/Math Operations/Gain', [ss '/BoundaryScale'], 'Gain', '1/phi_smc', 'Position', [455 190 535 220]);
add_block('simulink/Discontinuities/Saturation', [ss '/BoundarySaturation'], 'UpperLimit', '1', 'LowerLimit', '-1', 'Position', [610 190 690 220]);
add_block('simulink/Math Operations/Gain', [ss '/ReachingGain'], 'Gain', '-k1_smc', 'Position', [760 165 835 195]);
add_block('simulink/Math Operations/Gain', [ss '/IntegratorGain'], 'Gain', '-k2_smc', 'Position', [760 240 835 270]);
add_block('simulink/Continuous/Integrator', [ss '/STAIntegrator'], 'InitialCondition', 'smc_int_ic', 'Position', [910 240 950 270]);
add_block('simulink/Math Operations/Sum', [ss '/ReachingSum'], 'Inputs', '++', 'Position', [1025 185 1060 235]);
add_block('simulink/Math Operations/Gain', [ss '/InputNormalize'], 'Gain', '1/SB_smc', 'Position', [1135 195 1215 225]);
add_block('simulink/Math Operations/Sum', [ss '/TotalControl'], 'Inputs', '++', 'Position', [1290 240 1325 290]);
add_out(ss, 'V_cmd', [1415 255 1445 275]);
add_terminator(ss, 'unused_theta_ref', [115 75 140 95]);
add_terminator(ss, 'unused_u_ff', [115 195 140 215]);
connect(ss, 'theta_ref/1', 'unused_theta_ref/1');
connect(ss, 'u_ff/1', 'unused_u_ff/1');
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
connect(ss, 'TotalControl/1', 'V_cmd/1');
end

function build_actuator_interface(ss)
clear_system(ss);
add_note(ss, sprintf('Motor voltage path\ndead-zone compensation, then hard +/-6 V limit'), [25 10 410 45], 'orange');
add_in(ss, 'V_cmd', [35 145 65 165]);
add_block('simulink/Sources/Constant', [ss '/ud_pos'], 'Value', 'ud_pos', 'Position', [35 215 95 240]);
add_block('simulink/Sources/Constant', [ss '/ud_neg'], 'Value', 'ud_neg', 'Position', [35 275 95 300]);
add_block('simulink/User-Defined Functions/MATLAB Function', [ss '/DeadzoneCompensation'], 'Position', [200 115 390 225]);
set_matlab_function_script([ss '/DeadzoneCompensation'], sprintf([ ...
    'function V_comp = fcn(V_cmd, ud_pos, ud_neg)\n' ...
    '%% Add just enough voltage to overcome measured motor dead-zone.\n' ...
    'threshold = 0.1;\n' ...
    'if V_cmd > threshold\n' ...
    '    V_comp = V_cmd + ud_pos;\n' ...
    'elseif V_cmd < -threshold\n' ...
    '    V_comp = V_cmd - ud_neg;\n' ...
    'else\n' ...
    '    V_comp = 0;\n' ...
    'end\n']));
add_block('simulink/Discontinuities/Saturation', [ss '/FinalSaturation'], 'UpperLimit', 'V_sat_hw', 'LowerLimit', '-V_sat_hw', 'Position', [515 150 610 190]);
add_out(ss, 'V_m', [735 160 765 180]);
connect(ss, 'V_cmd/1', 'DeadzoneCompensation/1');
connect(ss, 'ud_pos/1', 'DeadzoneCompensation/2');
connect(ss, 'ud_neg/1', 'DeadzoneCompensation/3');
connect(ss, 'DeadzoneCompensation/1', 'FinalSaturation/1');
connect(ss, 'FinalSaturation/1', 'V_m/1');
end

function build_plant_interface(ss)
clear_system(ss);
add_note(ss, sprintf('Linear simulation plant\nx = [x_c; x_c_dot; theta; theta_dot]'), [25 10 430 45], 'green');
add_in(ss, 'V_m', [35 115 65 135]);
add_block('simulink/Continuous/State-Space', [ss '/LinearPlant'], 'A', 'A_ctrl', 'B', 'B_ctrl', 'C', 'C_ctrl', 'D', 'D_ctrl', 'X0', '[0;0;0;0]', 'Position', [165 85 300 145]);
add_block('simulink/Signal Routing/Demux', [ss '/state_demux'], 'Outputs', '4', 'Position', [385 70 390 170]);
add_out(ss, 'x_state', [500 95 530 115]);
add_out(ss, 'theta', [500 150 530 170]);
connect(ss, 'V_m/1', 'LinearPlant/1');
connect(ss, 'LinearPlant/1', 'x_state/1');
connect(ss, 'LinearPlant/1', 'state_demux/1');
connect(ss, 'state_demux/3', 'theta/1');
end

function build_state_estimation(ss)
clear_system(ss);
add_note(ss, sprintf('State feedback selection\nUse x_state directly, or select an observer estimate.'), [25 10 560 45], 'magenta');
add_in(ss, 'x_state', [35 145 65 165]);
add_in(ss, 'V_m', [35 255 65 275]);
add_block('simulink/Signal Routing/Demux', [ss '/state_demux'], 'Outputs', '4', 'Position', [150 115 155 225]);
add_block('simulink/Signal Routing/Mux', [ss '/observer_input'], 'Inputs', '3', 'Position', [285 205 295 335]);
add_block('simulink/Continuous/State-Space', [ss '/Luenberger'], 'A', 'A_obs_l', 'B', 'B_obs_l', 'C', 'C_obs_l', 'D', 'D_obs_l', 'X0', '[0;0;0;0]', 'Position', [435 185 565 235]);
add_block('simulink/Continuous/State-Space', [ss '/Kalman'], 'A', 'A_obs_k', 'B', 'B_obs_k', 'C', 'C_obs_k', 'D', 'D_obs_k', 'X0', '[0;0;0;0]', 'Position', [435 300 565 350]);
add_block('simulink/Sources/Constant', [ss '/feedback_source_id'], 'Value', 'feedback_selector_id', 'Position', [650 80 745 105]);
add_block('simulink/Signal Routing/Multiport Switch', [ss '/SelectFeedbackState'], 'DataPortOrder', 'One-based contiguous', 'Inputs', '3', 'Position', [790 145 870 295]);
add_out(ss, 'x_measured', [980 90 1010 110]);
add_out(ss, 'x_luenberger', [980 155 1010 175]);
add_out(ss, 'x_kalman', [980 220 1010 240]);
add_out(ss, 'x_feedback', [980 285 1010 305]);
connect(ss, 'x_state/1', 'x_measured/1');
connect(ss, 'x_state/1', 'state_demux/1');
connect(ss, 'V_m/1', 'observer_input/1');
connect(ss, 'state_demux/1', 'observer_input/2');
connect(ss, 'state_demux/3', 'observer_input/3');
connect(ss, 'observer_input/1', 'Luenberger/1');
connect(ss, 'observer_input/1', 'Kalman/1');
connect(ss, 'Luenberger/1', 'x_luenberger/1');
connect(ss, 'Kalman/1', 'x_kalman/1');
connect(ss, 'feedback_source_id/1', 'SelectFeedbackState/1');
connect(ss, 'x_state/1', 'SelectFeedbackState/2');
connect(ss, 'Luenberger/1', 'SelectFeedbackState/3');
connect(ss, 'Kalman/1', 'SelectFeedbackState/4');
connect(ss, 'SelectFeedbackState/1', 'x_feedback/1');
end

function build_logger(ss)
clear_system(ss);
names = {'segment_id','theta_ref','x_ref','u_ff','x_feedback','V_cmd','V_m','theta','x_measured','x_luenberger','x_kalman'};
add_note(ss, sprintf('Validation log order is exported as validation_log_columns.'), [25 10 430 45], 'gray');
add_block('simulink/Signal Routing/Mux', [ss '/log_mux'], 'Inputs', num2str(numel(names)), 'Position', [330 65 340 65+32*numel(names)]);
for i = 1:numel(names)
    add_in(ss, names{i}, [45 45+32*i 75 65+32*i]);
    connect(ss, [names{i} '/1'], sprintf('log_mux/%d', i));
end
add_block('simulink/Sinks/To Workspace', [ss '/validation_log'], 'VariableName', 'validation_log', 'SaveFormat', 'Timeseries', 'Position', [460 220 565 250]);
connect(ss, 'log_mux/1', 'validation_log/1');
end

function add_subsystem(parent, name, pos)
add_block('simulink/Ports & Subsystems/Subsystem', [parent '/' name], 'Position', pos);
end

function publish_signal(parent, source_port, tag, block_name, pos)
add_block('simulink/Signal Routing/Goto', [parent '/' block_name], ...
    'GotoTag', tag, 'TagVisibility', 'local', 'ShowName', 'off', 'Position', pos);
connect(parent, source_port, [block_name '/1']);
end

function subscribe_signal(parent, tag, block_name, destination_port, pos)
add_block('simulink/Signal Routing/From', [parent '/' block_name], ...
    'GotoTag', tag, 'ShowName', 'off', 'Position', pos);
connect(parent, [block_name '/1'], destination_port);
end

function style_block(parent, block_name, color)
set_param([parent '/' block_name], 'BackgroundColor', color, 'ForegroundColor', 'black');
end

function add_note(parent, text, pos, color)
annotation = Simulink.Annotation(parent, text);
annotation.Position = pos;
annotation.BackgroundColor = color;
annotation.ForegroundColor = 'black';
annotation.FontWeight = 'bold';
end

function set_matlab_function_script(block_path, script)
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
if isempty(chart)
    error('Could not find MATLAB Function block: %s', block_path);
end
chart.Script = script;
end

function add_in(parent, name, pos)
add_block('simulink/Sources/In1', [parent '/' name], 'Position', pos);
end

function add_out(parent, name, pos)
add_block('simulink/Sinks/Out1', [parent '/' name], 'Position', pos);
end

function add_terminator(parent, name, pos)
add_block('simulink/Sinks/Terminator', [parent '/' name], 'Position', pos);
end

function clear_system(ss)
lines = find_system(ss, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');
for i = 1:numel(lines)
    delete_line(lines(i));
end
blocks = find_system(ss, 'SearchDepth', 1, 'Type', 'Block');
for i = 1:numel(blocks)
    if ~strcmp(blocks{i}, ss)
        delete_block(blocks{i});
    end
end
annotations = find_system(ss, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'annotation');
for i = 1:numel(annotations)
    delete(annotations(i));
end
end

function connect(sys, src, dst)
try
    add_line(sys, src, dst, 'autorouting', 'smart');
catch
    add_line(sys, src, dst);
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
addParameter(p, 'controller_id', 'PP', @(x) ischar(x) || isstring(x));
addParameter(p, 'feedback_source_id', 'dirty', @(x) ischar(x) || isstring(x));
parse(p, varargin{:});
opts = p.Results;
opts.root = char(opts.root);
opts.controller_id = char(opts.controller_id);
opts.feedback_source_id = char(opts.feedback_source_id);
end
