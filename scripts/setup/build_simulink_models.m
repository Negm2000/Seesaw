%% build_simulink_models.m
%  -----------------------------------------------------------------------
%  Creates Simulink models for the Quanser IP02 + SEESAW-E system.
%
%  Phase 1: IP02_CartOnTable  - Cart on flat table (linearised, State-Space)
%  Phase 2: Seesaw_Full       - Cart on seesaw    (linearised, State-Space)
%
%  Signal chain (both models):
%    Sine_Vcmd --> Gain(K_a) --> Saturation(V_sat) --> State-Space --> outputs
%
%  The State-Space block uses the matrices computed in seesaw_params.m:
%    Phase 1: A_cart, B_cart, C_cart, D_cart   (2 states: x_c, x_c_dot)
%    Phase 2: A_sw,   B_sw,   C_sw,   D_sw     (4 states: x_c, x_c_dot, alpha, alpha_dot)
%
%  No S-Functions are used, so QUARC can compile without TLC files.
%
%  PREREQUISITE: Run seesaw_params.m first.
%  USAGE:        >> seesaw_params; build_simulink_models;
%  -----------------------------------------------------------------------

if ~exist('A_cart', 'var')
    error('Run seesaw_params.m first (A_cart not found)!');
end

fprintf('\n========================================\n');
fprintf(' Building Simulink Models (State-Space)\n');
fprintf('========================================\n');

% Check if QUARC is available
quarc_available = exist('quarc_library', 'file') == 4 || ...
                  ~isempty(which('hil_initialize_block'));
if ~quarc_available
    try
        quarc_available = ~isempty(ver('quarc'));
    catch
        quarc_available = false;
    end
end

if quarc_available
    fprintf('  QUARC detected - hardware blocks will be added.\n');
else
    fprintf('  QUARC not detected - simulation-only models.\n');
end

%% =====================================================================
%  PHASE 1: Cart on Flat Table  (2-state State-Space)
%  States:  [x_c; x_c_dot]
%  =====================================================================
mdl1 = 'IP02_CartOnTable';
fprintf('\n--- Building Phase 1: %s ---\n', mdl1);

if bdIsLoaded(mdl1), close_system(mdl1, 0); end
new_system(mdl1);
open_system(mdl1);

% -- Solver settings --
set_param(mdl1, 'Solver', 'ode45', ...
    'StopTime', '10', ...
    'MaxStep',  '1e-3', ...
    'RelTol',   '1e-4', ...
    'AbsTol',   '1e-6');

% ---- SIGNAL GENERATOR ----
add_block('simulink/Sources/Sine Wave', [mdl1 '/Sine_Vcmd'], ...
    'Amplitude',  'A_input', ...
    'Frequency',  '2*pi*f_input', ...
    'SampleTime', '0', ...
    'Position', [50 145 90 185]);

if ~evalin('base', 'exist(''A_input'',''var'')')
    assignin('base', 'A_input', 3.0);
end
if ~evalin('base', 'exist(''f_input'',''var'')')
    assignin('base', 'f_input', 0.5);
end

% ---- AMPLIFIER GAIN (K_a) ----
% Represents the VoltPAQ-X1 voltage amplifier (unity gain, switch = 1x)
add_block('simulink/Math Operations/Gain', [mdl1 '/Amp_Gain'], ...
    'Gain',     'K_a', ...
    'Position', [155 148 195 182]);

% ---- VOLTAGE SATURATION (+/- V_sat) ----
% Clamps amplifier output to the VoltPAQ rail voltage
add_block('simulink/Discontinuities/Saturation', [mdl1 '/V_Sat'], ...
    'UpperLimit', 'V_sat', ...
    'LowerLimit', '-V_sat', ...
    'Position', [220 148 270 182]);

% ---- STATE-SPACE PLANT (Phase 1) ----
% Uses A_cart, B_cart, C_cart, D_cart from seesaw_params.m
% Input:  V_m (1 signal)
% Output: [x_c; x_c_dot] (2 signals via C_cart = eye(2))
add_block('simulink/Continuous/State-Space', [mdl1 '/Cart_SS'], ...
    'A', 'A_cart', ...
    'B', 'B_cart', ...
    'C', 'C_cart', ...
    'D', 'D_cart', ...
    'X0', '[0; 0]', ...
    'Position', [310 130 420 200]);

% ---- DEMUX to split [x_c; x_c_dot] ----
add_block('simulink/Signal Routing/Demux', [mdl1 '/Demux_SS'], ...
    'Outputs',  '2', ...
    'Position', [455 140 460 190]);

% ---- UNIT CONVERSION: m -> cm ----
add_block('simulink/Math Operations/Gain', [mdl1 '/m_to_cm'], ...
    'Gain',     '100', ...
    'Position', [490 140 530 165]);

% ---- SCOPES ----
% Scope 1: V_cmd and V_m (saturated voltage)
add_block('simulink/Sinks/Scope', [mdl1 '/Scope_Voltage'], ...
    'NumInputPorts', '2', ...
    'Position', [620 30 660 70]);
set_param([mdl1 '/Scope_Voltage'], 'Name', 'Voltage');

% Scope 2: Cart position [cm]
add_block('simulink/Sinks/Scope', [mdl1 '/Scope_Position'], ...
    'NumInputPorts', '1', ...
    'Position', [620 140 660 165]);
set_param([mdl1 '/Scope_Position'], 'Name', 'Cart Position [cm]');

% Scope 3: Cart velocity [m/s]
add_block('simulink/Sinks/Scope', [mdl1 '/Scope_Velocity'], ...
    'NumInputPorts', '1', ...
    'Position', [620 175 660 205]);
set_param([mdl1 '/Scope_Velocity'], 'Name', 'Cart Velocity');

% ---- TO WORKSPACE ----
add_block('simulink/Sinks/To Workspace', [mdl1 '/ToWS_xc'], ...
    'VariableName', 'sim_xc', 'SaveFormat', 'Timeseries', ...
    'Position', [490 175 550 195]);

add_block('simulink/Sinks/To Workspace', [mdl1 '/ToWS_xdot'], ...
    'VariableName', 'sim_xdot', 'SaveFormat', 'Timeseries', ...
    'Position', [490 215 550 235]);

% ---- WIRING: Phase 1 ----
% Sine --> Amp_Gain --> V_Sat --> Cart_SS
add_line(mdl1, 'Sine_Vcmd/1', 'Amp_Gain/1',  'autorouting', 'smart');
add_line(mdl1, 'Amp_Gain/1',  'V_Sat/1',     'autorouting', 'smart');
add_line(mdl1, 'V_Sat/1',     'Cart_SS/1',   'autorouting', 'smart');

% Cart_SS --> Demux
add_line(mdl1, 'Cart_SS/1', 'Demux_SS/1', 'autorouting', 'smart');

% Demux port 1 = x_c --> m_to_cm --> Scope_Position + ToWS_xc
add_line(mdl1, 'Demux_SS/1', 'm_to_cm/1',           'autorouting', 'smart');
add_line(mdl1, 'm_to_cm/1',  'Cart Position [cm]/1', 'autorouting', 'smart');
add_line(mdl1, 'Demux_SS/1', 'ToWS_xc/1',           'autorouting', 'smart');

% Demux port 2 = x_c_dot --> Scope_Velocity + ToWS_xdot
add_line(mdl1, 'Demux_SS/2', 'Cart Velocity/1', 'autorouting', 'smart');
add_line(mdl1, 'Demux_SS/2', 'ToWS_xdot/1',    'autorouting', 'smart');

% V_cmd --> Voltage scope port 1 | V_Sat --> Voltage scope port 2
add_line(mdl1, 'Sine_Vcmd/1', 'Voltage/1', 'autorouting', 'smart');
add_line(mdl1, 'V_Sat/1',     'Voltage/2', 'autorouting', 'smart');

% ---- QUARC HARDWARE BLOCKS (Phase 1) ----
if quarc_available
    fprintf('  Adding QUARC hardware blocks to %s...\n', mdl1);

    set_param(mdl1, 'SolverType', 'Fixed-step', ...
        'Solver',    'ode1', ...
        'FixedStep', '0.002', ...
        'StopTime',  'inf');

    try
        add_block('quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize', ...
            [mdl1 '/HIL Initialize'], ...
            'Position', [50 350 134 425]);

        add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Write Analog', ...
            [mdl1 '/Motor Command'], ...
            'Position', [300 350 385 412]);

        add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Read Encoder', ...
            [mdl1 '/Encoders'], ...
            'Position', [300 430 385 492]);

        add_block('simulink/Math Operations/Gain', [mdl1 '/Enc_to_m'], ...
            'Gain',     'K_ec', ...
            'Position', [430 445 480 475]);

        add_block('simulink/Math Operations/Gain', [mdl1 '/HW_m_to_cm'], ...
            'Gain',     '100', ...
            'Position', [510 445 540 475]);

        add_block('simulink/Sinks/Scope', [mdl1 '/HW_Position'], ...
            'NumInputPorts', '2', ...
            'Position', [620 435 660 485]);
        set_param([mdl1 '/HW_Position'], 'Name', 'Sim vs HW Position [cm]');

        add_block('simulink/Sinks/To Workspace', [mdl1 '/ToWS_hw_xc'], ...
            'VariableName', 'hw_xc', 'SaveFormat', 'Timeseries', ...
            'Position', [510 500 570 520]);

        % Wire hardware blocks
        add_line(mdl1, 'V_Sat/1',       'Motor Command/1',           'autorouting', 'smart');
        add_line(mdl1, 'Encoders/1',    'Enc_to_m/1',                'autorouting', 'smart');
        add_line(mdl1, 'Enc_to_m/1',   'HW_m_to_cm/1',              'autorouting', 'smart');
        add_line(mdl1, 'HW_m_to_cm/1', 'Sim vs HW Position [cm]/2', 'autorouting', 'smart');
        add_line(mdl1, 'Enc_to_m/1',   'ToWS_hw_xc/1',              'autorouting', 'smart');
        add_line(mdl1, 'm_to_cm/1',    'Sim vs HW Position [cm]/1', 'autorouting', 'smart');

        fprintf('  QUARC blocks added successfully.\n');
    catch ME
        fprintf('  Warning: Could not add QUARC blocks: %s\n', ME.message);
        fprintf('  Model will work in simulation-only mode.\n');
    end
end

if ~exist('SEESAW_ROOT', 'var')
    [curr_path, ~, ~] = fileparts(mfilename('fullpath'));
    SEESAW_ROOT = fileparts(fileparts(curr_path)); 
end
save_system(mdl1, fullfile(SEESAW_ROOT, 'models', [mdl1 '.slx']));
fprintf('  Phase 1 model saved: models/%s.slx\n', mdl1);

%% =====================================================================
%  PHASE 2: Cart on Seesaw  (4-state State-Space, linearised)
%  States:  [x_c; x_c_dot; alpha; alpha_dot]
%  =====================================================================
mdl2 = 'Seesaw_Full';
fprintf('\n--- Building Phase 2: %s ---\n', mdl2);

if bdIsLoaded(mdl2), close_system(mdl2, 0); end
new_system(mdl2);
open_system(mdl2);

set_param(mdl2, 'Solver', 'ode45', ...
    'StopTime', '10', ...
    'MaxStep',  '1e-3', ...
    'RelTol',   '1e-4', ...
    'AbsTol',   '1e-6');

% ---- SIGNAL GENERATOR ----
add_block('simulink/Sources/Sine Wave', [mdl2 '/Sine_Vcmd'], ...
    'Amplitude',  'A_input', ...
    'Frequency',  '2*pi*f_input', ...
    'SampleTime', '0', ...
    'Position', [50 195 90 235]);

% ---- AMPLIFIER GAIN (K_a) ----
add_block('simulink/Math Operations/Gain', [mdl2 '/Amp_Gain'], ...
    'Gain',     'K_a', ...
    'Position', [155 198 195 232]);

% ---- VOLTAGE SATURATION (+/- V_sat) ----
add_block('simulink/Discontinuities/Saturation', [mdl2 '/V_Sat'], ...
    'UpperLimit', 'V_sat', ...
    'LowerLimit', '-V_sat', ...
    'Position', [220 198 270 232]);

% ---- STATE-SPACE PLANT (Phase 2) ----
% Uses A_sw, B_sw, C_sw, D_sw from seesaw_params.m
% Input:  V_m (1 signal)
% Output: [x_c; x_c_dot; alpha; alpha_dot] (4 signals via C_sw = eye(4))
add_block('simulink/Continuous/State-Space', [mdl2 '/Seesaw_SS'], ...
    'A',  'A_sw', ...
    'B',  'B_sw', ...
    'C',  'C_sw', ...
    'D',  'D_sw', ...
    'X0', '[0; 0; 0; 0]', ...
    'Position', [310 170 450 260]);

% ---- DEMUX: split [x_c; x_c_dot; alpha; alpha_dot] ----
add_block('simulink/Signal Routing/Demux', [mdl2 '/Demux_SS'], ...
    'Outputs',  '4', ...
    'Position', [490 150 495 285]);

% ---- UNIT CONVERSIONS ----
add_block('simulink/Math Operations/Gain', [mdl2 '/m_to_cm'], ...
    'Gain',     '100', ...
    'Position', [530 148 570 173]);

add_block('simulink/Math Operations/Gain', [mdl2 '/rad_to_deg'], ...
    'Gain',     '180/pi', ...
    'Position', [530 218 570 243]);

% ---- SCOPES ----
add_block('simulink/Sinks/Scope', [mdl2 '/Scope_Voltage'], ...
    'NumInputPorts', '2', ...
    'Position', [680 30 720 70]);
set_param([mdl2 '/Scope_Voltage'], 'Name', 'Voltage');

add_block('simulink/Sinks/Scope', [mdl2 '/Scope_Position'], ...
    'NumInputPorts', '1', ...
    'Position', [680 148 720 173]);
set_param([mdl2 '/Scope_Position'], 'Name', 'Cart Position [cm]');

add_block('simulink/Sinks/Scope', [mdl2 '/Scope_Alpha'], ...
    'NumInputPorts', '1', ...
    'Position', [680 218 720 243]);
set_param([mdl2 '/Scope_Alpha'], 'Name', 'Seesaw Angle [deg]');

add_block('simulink/Sinks/Scope', [mdl2 '/Scope_Velocity'], ...
    'NumInputPorts', '1', ...
    'Position', [680 175 720 200]);
set_param([mdl2 '/Scope_Velocity'], 'Name', 'Cart Velocity');

add_block('simulink/Sinks/Scope', [mdl2 '/Scope_AlphaDot'], ...
    'NumInputPorts', '1', ...
    'Position', [680 250 720 275]);
set_param([mdl2 '/Scope_AlphaDot'], 'Name', 'Seesaw Rate [rad/s]');

% ---- TO WORKSPACE ----
add_block('simulink/Sinks/To Workspace', [mdl2 '/ToWS_xc'], ...
    'VariableName', 'sim_xc',    'SaveFormat', 'Timeseries', ...
    'Position', [580 148 640 168]);

add_block('simulink/Sinks/To Workspace', [mdl2 '/ToWS_xdot'], ...
    'VariableName', 'sim_xdot',  'SaveFormat', 'Timeseries', ...
    'Position', [580 178 640 198]);

add_block('simulink/Sinks/To Workspace', [mdl2 '/ToWS_alpha'], ...
    'VariableName', 'sim_alpha', 'SaveFormat', 'Timeseries', ...
    'Position', [580 218 640 238]);

add_block('simulink/Sinks/To Workspace', [mdl2 '/ToWS_adot'], ...
    'VariableName', 'sim_adot',  'SaveFormat', 'Timeseries', ...
    'Position', [580 248 640 268]);

% ---- WIRING: Phase 2 ----
% Sine --> Amp_Gain --> V_Sat --> Seesaw_SS
add_line(mdl2, 'Sine_Vcmd/1', 'Amp_Gain/1',   'autorouting', 'smart');
add_line(mdl2, 'Amp_Gain/1',  'V_Sat/1',      'autorouting', 'smart');
add_line(mdl2, 'V_Sat/1',     'Seesaw_SS/1',  'autorouting', 'smart');

% Seesaw_SS --> Demux
add_line(mdl2, 'Seesaw_SS/1', 'Demux_SS/1', 'autorouting', 'smart');

% Port 1 = x_c --> m_to_cm --> Scope + ToWS
add_line(mdl2, 'Demux_SS/1', 'm_to_cm/1',           'autorouting', 'smart');
add_line(mdl2, 'm_to_cm/1',  'Cart Position [cm]/1', 'autorouting', 'smart');
add_line(mdl2, 'Demux_SS/1', 'ToWS_xc/1',           'autorouting', 'smart');

% Port 2 = x_c_dot --> Scope + ToWS
add_line(mdl2, 'Demux_SS/2', 'Cart Velocity/1', 'autorouting', 'smart');
add_line(mdl2, 'Demux_SS/2', 'ToWS_xdot/1',    'autorouting', 'smart');

% Port 3 = alpha --> rad_to_deg --> Scope + ToWS
add_line(mdl2, 'Demux_SS/3', 'rad_to_deg/1',        'autorouting', 'smart');
add_line(mdl2, 'rad_to_deg/1', 'Seesaw Angle [deg]/1', 'autorouting', 'smart');
add_line(mdl2, 'Demux_SS/3', 'ToWS_alpha/1',        'autorouting', 'smart');

% Port 4 = alpha_dot --> Scope + ToWS
add_line(mdl2, 'Demux_SS/4', 'Seesaw Rate [rad/s]/1', 'autorouting', 'smart');
add_line(mdl2, 'Demux_SS/4', 'ToWS_adot/1',           'autorouting', 'smart');

% V_cmd --> Voltage scope port 1 | V_Sat --> Voltage scope port 2
add_line(mdl2, 'Sine_Vcmd/1', 'Voltage/1', 'autorouting', 'smart');
add_line(mdl2, 'V_Sat/1',     'Voltage/2', 'autorouting', 'smart');

% ---- QUARC HARDWARE BLOCKS (Phase 2) ----
if quarc_available
    fprintf('  Adding QUARC hardware blocks to %s...\n', mdl2);

    set_param(mdl2, 'SolverType', 'Fixed-step', ...
        'Solver',    'ode1', ...
        'FixedStep', '0.002', ...
        'StopTime',  'inf');

    try
        add_block('quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize', ...
            [mdl2 '/HIL Initialize'], ...
            'Position', [50 400 134 475]);

        add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Write Analog', ...
            [mdl2 '/Motor Command'], ...
            'Position', [300 400 385 462]);

        % HIL Read Encoder: ch[0,1] -- output 1=cart, output 2=seesaw
        add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Read Encoder', ...
            [mdl2 '/Encoders'], ...
            'Position', [300 490 385 552]);

        add_block('simulink/Math Operations/Gain', [mdl2 '/Enc_Cart_to_m'], ...
            'Gain',     'K_ec', ...
            'Position', [430 495 480 525]);

        add_block('simulink/Math Operations/Gain', [mdl2 '/Enc_Seesaw_to_rad'], ...
            'Gain',     'K_E_SW / K_gs', ...
            'Position', [430 545 480 575]);

        add_block('simulink/Math Operations/Gain', [mdl2 '/HW_m_to_cm'], ...
            'Gain',     '100', ...
            'Position', [510 495 540 525]);

        add_block('simulink/Math Operations/Gain', [mdl2 '/HW_rad_to_deg'], ...
            'Gain',     '180/pi', ...
            'Position', [510 545 540 575]);

        add_block('simulink/Sinks/Scope', [mdl2 '/Scope_Cmp_Pos'], ...
            'NumInputPorts', '2', ...
            'Position', [680 485 720 535]);
        set_param([mdl2 '/Scope_Cmp_Pos'], 'Name', 'Sim vs HW Cart [cm]');

        add_block('simulink/Sinks/Scope', [mdl2 '/Scope_Cmp_Alpha'], ...
            'NumInputPorts', '2', ...
            'Position', [680 535 720 585]);
        set_param([mdl2 '/Scope_Cmp_Alpha'], 'Name', 'Sim vs HW Angle [deg]');

        add_block('simulink/Sinks/To Workspace', [mdl2 '/ToWS_hw_xc'], ...
            'VariableName', 'hw_xc',    'SaveFormat', 'Timeseries', ...
            'Position', [580 610 640 630]);
        add_block('simulink/Sinks/To Workspace', [mdl2 '/ToWS_hw_alpha'], ...
            'VariableName', 'hw_alpha', 'SaveFormat', 'Timeseries', ...
            'Position', [580 640 640 660]);

        hHostFile = add_block('quarc_library/Sinks/To Host/To Host File', ...
            [mdl2 '/To Host File'], ...
            'Position', [680 620 760 680]);
        
        data_path = fullfile(SEESAW_ROOT, 'data', 'data.mat');
        
        try
            set_param(hHostFile, 'file_name', data_path, 'file_format', 'MAT-file');
        catch
            try set_param(hHostFile, 'FileName', data_path, 'FileFormat', 'MAT-file'); catch, end
        end

        % Wire hardware blocks
        add_line(mdl2, 'V_Sat/1',           'Motor Command/1',           'autorouting', 'smart');
        add_line(mdl2, 'Encoders/1',        'Enc_Cart_to_m/1',           'autorouting', 'smart');
        add_line(mdl2, 'Enc_Cart_to_m/1',   'HW_m_to_cm/1',              'autorouting', 'smart');
        add_line(mdl2, 'HW_m_to_cm/1',     'Sim vs HW Cart [cm]/2',      'autorouting', 'smart');
        add_line(mdl2, 'Enc_Cart_to_m/1',   'ToWS_hw_xc/1',              'autorouting', 'smart');
        add_line(mdl2, 'Encoders/2',        'Enc_Seesaw_to_rad/1',        'autorouting', 'smart');
        add_line(mdl2, 'Enc_Seesaw_to_rad/1', 'HW_rad_to_deg/1',         'autorouting', 'smart');
        add_line(mdl2, 'HW_rad_to_deg/1',  'Sim vs HW Angle [deg]/2',    'autorouting', 'smart');
        add_line(mdl2, 'Enc_Seesaw_to_rad/1', 'ToWS_hw_alpha/1',         'autorouting', 'smart');
        add_line(mdl2, 'm_to_cm/1',        'Sim vs HW Cart [cm]/1',       'autorouting', 'smart');
        add_line(mdl2, 'rad_to_deg/1',     'Sim vs HW Angle [deg]/1',     'autorouting', 'smart');

        fprintf('  QUARC blocks added successfully.\n');
    catch ME
        fprintf('  Warning: Could not add QUARC blocks: %s\n', ME.message);
    end
end

if ~exist('SEESAW_ROOT', 'var')
    [curr_path, ~, ~] = fileparts(mfilename('fullpath'));
    SEESAW_ROOT = fileparts(fileparts(curr_path)); 
end
save_system(mdl2, fullfile(SEESAW_ROOT, 'models', [mdl2 '.slx']));
fprintf('  Phase 2 model saved: models/%s.slx\n', mdl2);

%% =====================================================================
%  PHASE 3: Seesaw Closed-Loop Control  (Full State Feedback)
%  States:  [x_c; x_c_dot; alpha; alpha_dot]   (same as Phase 2)
%
%  Architecture:  u = -K * x  (pole placement, from control_pipeline.m)
%
%  Simulation mode:
%    x_c_ref ──►[Sum_ref] ────────────────────────────────────────►[Mux_err]──► Gain(-K)
%                  ▲(-)                                               ▲  ▲  ▲
%                  │ x_c    x_c_dot    alpha    alpha_dot             │  │  │
%                  └──────────────────────────────────────────────────-┘  │  │
%                        (all 4 states from Plant_SS → Demux)            │  │
%                                                                        ...
%
%  Hardware mode (QUARC at 500 Hz):
%    Encoders ──► [K_ec] ──► x_c   ──► Dirty deriv ──► x_c_dot
%             ──► [K_sw] ──► alpha ──► Dirty deriv ──► alpha_dot
%    [Mux: x_c, x_c_dot, alpha, alpha_dot] ──► Gain(-K) ──► V_Sat ──► Motor
%    Reference x_c_ref enters via (x_c_ref - x_c) replacing x_c in state vec.
%
%  Requires: controller_freq.mat from control_pipeline.m
%  =====================================================================
mdl3 = 'Seesaw_StateFB';
fprintf('\n--- Building Phase 3: %s ---\n', mdl3);

ctrl_path = fullfile(SEESAW_ROOT, 'data', 'controller_freq.mat');
if ~exist(ctrl_path, 'file')
    fprintf('  controller_freq.mat not found — skipping Phase 3.\n');
    fprintf('  Run control_pipeline.m first, then re-run this script.\n');
    phase3_built = false;
else
    phase3_built = true;
    ctrl = load(ctrl_path);

    % Push controller gains K to base workspace
    assignin('base', 'K_fb', ctrl.K);

    % Derivative filter time constant for velocity estimation
    tau_d = 0.01;   % 100 rad/s cutoff (~16 Hz) — safe for 500 Hz sample rate
    assignin('base', 'tau_d', tau_d);

    fprintf('  State feedback gains loaded:\n');
    fprintf('    K = [%.4f, %.4f, %.4f, %.4f]\n', ctrl.K);
    fprintf('         k_xc    k_xcdot  k_alpha  k_alphadot\n');
    fprintf('  Derivative filter: tau_d = %.3f s (cutoff %.0f rad/s)\n', tau_d, 1/tau_d);

    if bdIsLoaded(mdl3), close_system(mdl3, 0); end
    new_system(mdl3);
    open_system(mdl3);

    % --- Solver ---
    if quarc_available
        set_param(mdl3, 'SolverType', 'Fixed-step', ...
            'Solver',    'ode1', ...
            'FixedStep', '0.002', ...
            'StopTime',  'inf');
    else
        set_param(mdl3, 'Solver', 'ode45', ...
            'StopTime', '20', ...
            'MaxStep',  '1e-3', ...
            'RelTol',   '1e-4', ...
            'AbsTol',   '1e-6');
    end

    % ==================================================================
    %  REFERENCE INPUT + ERROR COMPUTATION
    % ==================================================================

    % ---- Cart position reference [m] ----
    % Default: hold at center (x_c_ref = 0).
    % For 5 cm step test: change 'After' to '0.05'.
    add_block('simulink/Sources/Step', [mdl3 '/x_c_ref'], ...
        'Time',   '2', ...
        'Before', '0', ...
        'After',  '0', ...
        'Position', [50 95 90 135]);

    % ---- SUMMING JUNCTION: e_xc = x_c_ref - x_c ----
    % Replaces x_c in the state vector with position error.
    % This ensures the controller drives the cart to the reference.
    add_block('simulink/Math Operations/Sum', [mdl3 '/Sum_ref'], ...
        'Inputs', '+-', ...
        'Position', [160 100 180 120]);

    % ==================================================================
    %  STATE VECTOR MUX + CONTROL LAW: u = -K * [e_xc; xc_dot; alpha; a_dot]
    % ==================================================================

    % ---- Mux: assemble state error vector ----
    %   Port 1: e_xc        (x_c_ref - x_c)
    %   Port 2: x_c_dot     (velocity — zero reference assumed)
    %   Port 3: alpha       (angle — zero reference assumed)
    %   Port 4: alpha_dot   (angular rate)
    add_block('simulink/Signal Routing/Mux', [mdl3 '/Mux_states'], ...
        'Inputs', '4', ...
        'Position', [290 70 295 200]);

    % ---- State feedback gain: u = -K * x_err ----
    % Gain block with -K (1x4 vector), input = 4x1 muxed state error.
    % Simulink Gain block with Multiplication = 'Vector*Matrix(u*K)'
    % where u is 1x4 (transposed internally) and K is 1x4;
    % Actually: input 4x1, Gain 1x4, Multiplication = 'Matrix(K*u)'.
    add_block('simulink/Math Operations/Gain', [mdl3 '/Gain_K'], ...
        'Gain', '-K_fb', ...
        'Multiplication', 'Matrix(K*u)', ...
        'Position', [350 115 440 155]);

    % ---- VOLTAGE SATURATION (±V_sat) ----
    add_block('simulink/Discontinuities/Saturation', [mdl3 '/V_Sat'], ...
        'UpperLimit', 'V_sat', ...
        'LowerLimit', '-V_sat', ...
        'Position', [490 118 540 152]);

    % ==================================================================
    %  DISPLAY: SCOPES + TO-WORKSPACE
    % ==================================================================

    % ---- Unit conversion gains for scopes ----
    add_block('simulink/Math Operations/Gain', [mdl3 '/ref_m_to_cm'], ...
        'Gain', '100', ...
        'Position', [730 52 770 78]);

    add_block('simulink/Math Operations/Gain', [mdl3 '/xc_m_to_cm'], ...
        'Gain', '100', ...
        'Position', [730 92 770 118]);

    add_block('simulink/Math Operations/Gain', [mdl3 '/alpha_to_deg'], ...
        'Gain', '180/pi', ...
        'Position', [730 172 770 198]);

    % ---- Scopes ----
    add_block('simulink/Sinks/Scope', [mdl3 '/Scope_Cart'], ...
        'NumInputPorts', '2', ...
        'Position', [840 60 880 110]);
    set_param([mdl3 '/Scope_Cart'], 'Name', 'Cart [cm]');

    add_block('simulink/Sinks/Scope', [mdl3 '/Scope_Angle'], ...
        'NumInputPorts', '1', ...
        'Position', [840 170 880 200]);
    set_param([mdl3 '/Scope_Angle'], 'Name', 'Angle [deg]');

    add_block('simulink/Sinks/Scope', [mdl3 '/Scope_Vm'], ...
        'NumInputPorts', '1', ...
        'Position', [840 250 880 280]);
    set_param([mdl3 '/Scope_Vm'], 'Name', 'Voltage [V]');

    % ---- To Workspace ----
    add_block('simulink/Sinks/To Workspace', [mdl3 '/ToWS_xc'], ...
        'VariableName', 'ctrl_xc', 'SaveFormat', 'Timeseries', ...
        'Position', [840 310 910 330]);

    add_block('simulink/Sinks/To Workspace', [mdl3 '/ToWS_alpha'], ...
        'VariableName', 'ctrl_alpha', 'SaveFormat', 'Timeseries', ...
        'Position', [840 350 910 370]);

    add_block('simulink/Sinks/To Workspace', [mdl3 '/ToWS_Vm'], ...
        'VariableName', 'ctrl_Vm', 'SaveFormat', 'Timeseries', ...
        'Position', [840 390 910 410]);

    % ==================================================================
    %  WIRING: CONTROLLER CHAIN (forward path)
    % ==================================================================
    % x_c_ref → Sum_ref (port +)
    add_line(mdl3, 'x_c_ref/1',   'Sum_ref/1',    'autorouting', 'smart');
    % Sum_ref → Mux port 1 (e_xc)
    add_line(mdl3, 'Sum_ref/1',   'Mux_states/1', 'autorouting', 'smart');
    % Mux → Gain(-K) → V_Sat
    add_line(mdl3, 'Mux_states/1', 'Gain_K/1',    'autorouting', 'smart');
    add_line(mdl3, 'Gain_K/1',    'V_Sat/1',      'autorouting', 'smart');

    % Voltage → scope + workspace
    add_line(mdl3, 'V_Sat/1', 'Voltage [V]/1', 'autorouting', 'smart');
    add_line(mdl3, 'V_Sat/1', 'ToWS_Vm/1',     'autorouting', 'smart');

    % Reference → Cart scope port 1
    add_line(mdl3, 'x_c_ref/1',     'ref_m_to_cm/1', 'autorouting', 'smart');
    add_line(mdl3, 'ref_m_to_cm/1', 'Cart [cm]/1',   'autorouting', 'smart');

    % ==================================================================
    %  FEEDBACK PATH — hardware vs simulation
    % ==================================================================
    if quarc_available
        % ============ QUARC HARDWARE I/O ============
        fprintf('  Adding QUARC hardware blocks to %s...\n', mdl3);

        try
            % HIL Initialize (configures Q2-USB board)
            add_block('quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize', ...
                [mdl3 '/HIL Initialize'], ...
                'Position', [50 350 134 425]);

            % Motor output: V_Sat → DAC channel 0 → VoltPAQ → motor
            add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Write Analog', ...
                [mdl3 '/Motor Command'], ...
                'Position', [600 118 685 152]);

            % Encoder input: ch0 = cart, ch1 = seesaw
            add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Read Encoder', ...
                [mdl3 '/Encoders'], ...
                'Position', [50 450 135 510]);

            % ---- Encoder → physical units ----
            add_block('simulink/Math Operations/Gain', [mdl3 '/Enc_to_xc'], ...
                'Gain', 'K_ec', ...
                'Position', [200 455 260 485]);

            add_block('simulink/Math Operations/Gain', [mdl3 '/Enc_to_alpha'], ...
                'Gain', 'K_E_SW / K_gs', ...
                'Position', [200 505 260 535]);

            % ---- Dirty derivative filters for velocity estimation ----
            % x_c_dot ≈ s / (tau_d*s + 1) * x_c
            add_block('simulink/Continuous/Transfer Fcn', [mdl3 '/Deriv_xc'], ...
                'Numerator',   '[1, 0]', ...
                'Denominator', '[tau_d, 1]', ...
                'Position', [320 455 420 485]);

            % alpha_dot ≈ s / (tau_d*s + 1) * alpha
            add_block('simulink/Continuous/Transfer Fcn', [mdl3 '/Deriv_alpha'], ...
                'Numerator',   '[1, 0]', ...
                'Denominator', '[tau_d, 1]', ...
                'Position', [320 505 420 535]);

            % ---- Wiring: motor output ----
            add_line(mdl3, 'V_Sat/1', 'Motor Command/1', 'autorouting', 'smart');

            % ---- Wiring: encoders → conversion gains ----
            add_line(mdl3, 'Encoders/1', 'Enc_to_xc/1',    'autorouting', 'smart');
            add_line(mdl3, 'Encoders/2', 'Enc_to_alpha/1',  'autorouting', 'smart');

            % ---- Wiring: positions → dirty derivatives ----
            add_line(mdl3, 'Enc_to_xc/1',    'Deriv_xc/1',    'autorouting', 'smart');
            add_line(mdl3, 'Enc_to_alpha/1',  'Deriv_alpha/1', 'autorouting', 'smart');

            % ---- Wiring: feedback → error / state mux ----
            % x_c → Sum_ref port 2 (negative)
            add_line(mdl3, 'Enc_to_xc/1',   'Sum_ref/2',     'autorouting', 'smart');
            % x_c_dot → Mux port 2
            add_line(mdl3, 'Deriv_xc/1',    'Mux_states/2',  'autorouting', 'smart');
            % alpha → Mux port 3
            add_line(mdl3, 'Enc_to_alpha/1', 'Mux_states/3', 'autorouting', 'smart');
            % alpha_dot → Mux port 4
            add_line(mdl3, 'Deriv_alpha/1',  'Mux_states/4', 'autorouting', 'smart');

            % ---- Wiring: feedback → display scopes ----
            add_line(mdl3, 'Enc_to_xc/1',    'xc_m_to_cm/1',   'autorouting', 'smart');
            add_line(mdl3, 'xc_m_to_cm/1',   'Cart [cm]/2',    'autorouting', 'smart');
            add_line(mdl3, 'Enc_to_alpha/1',  'alpha_to_deg/1', 'autorouting', 'smart');
            add_line(mdl3, 'alpha_to_deg/1',  'Angle [deg]/1',  'autorouting', 'smart');

            % ---- Wiring: feedback → To Workspace ----
            add_line(mdl3, 'Enc_to_xc/1',    'ToWS_xc/1',    'autorouting', 'smart');
            add_line(mdl3, 'Enc_to_alpha/1',  'ToWS_alpha/1', 'autorouting', 'smart');

            fprintf('  QUARC hardware blocks wired successfully.\n');
            fprintf('  Velocity estimation: dirty derivatives (tau=%.3f s)\n', tau_d);
        catch ME
            fprintf('  Warning: Could not add QUARC blocks: %s\n', ME.message);
            fprintf('  Model will NOT work on hardware. Check QUARC installation.\n');
        end
    else
        % ============ SIMULATION ONLY: plant in the loop ============
        fprintf('  No QUARC — adding State-Space plant for closed-loop simulation.\n');

        % State-Space plant (same A_sw, B_sw, C_sw, D_sw as Phase 2)
        % Initial condition: tilted ~4.5° to test disturbance rejection
        add_block('simulink/Continuous/State-Space', [mdl3 '/Plant_SS'], ...
            'A', 'A_sw', 'B', 'B_sw', 'C', 'C_sw', 'D', 'D_sw', ...
            'X0', '[0; 0; 0.08; 0]', ...
            'Position', [600 65 710 215]);

        % Demux: split [x_c; x_c_dot; alpha; alpha_dot]
        add_block('simulink/Signal Routing/Demux', [mdl3 '/Demux_Plant'], ...
            'Outputs', '4', ...
            'Position', [740 65 745 215]);

        % V_Sat → Plant → Demux
        add_line(mdl3, 'V_Sat/1',    'Plant_SS/1',    'autorouting', 'smart');
        add_line(mdl3, 'Plant_SS/1', 'Demux_Plant/1', 'autorouting', 'smart');

        % ---- Feedback: states → error / state mux ----
        % x_c (port 1) → Sum_ref port 2 (negative)
        add_line(mdl3, 'Demux_Plant/1', 'Sum_ref/2',     'autorouting', 'smart');
        % x_c_dot (port 2) → Mux port 2
        add_line(mdl3, 'Demux_Plant/2', 'Mux_states/2', 'autorouting', 'smart');
        % alpha (port 3) → Mux port 3
        add_line(mdl3, 'Demux_Plant/3', 'Mux_states/3', 'autorouting', 'smart');
        % alpha_dot (port 4) → Mux port 4
        add_line(mdl3, 'Demux_Plant/4', 'Mux_states/4', 'autorouting', 'smart');

        % ---- Feedback → display scopes ----
        add_line(mdl3, 'Demux_Plant/1', 'xc_m_to_cm/1',   'autorouting', 'smart');
        add_line(mdl3, 'xc_m_to_cm/1',  'Cart [cm]/2',    'autorouting', 'smart');
        add_line(mdl3, 'Demux_Plant/3', 'alpha_to_deg/1',  'autorouting', 'smart');
        add_line(mdl3, 'alpha_to_deg/1', 'Angle [deg]/1',  'autorouting', 'smart');

        % ---- Feedback → To Workspace ----
        add_line(mdl3, 'Demux_Plant/1', 'ToWS_xc/1',    'autorouting', 'smart');
        add_line(mdl3, 'Demux_Plant/3', 'ToWS_alpha/1', 'autorouting', 'smart');

        fprintf('  Simulation plant wired. IC = [0, 0, 4.5 deg, 0].\n');
    end

    save_system(mdl3, fullfile(SEESAW_ROOT, 'models', [mdl3 '.slx']));
    fprintf('  Phase 3 model saved: models/%s.slx\n', mdl3);
end

%% =====================================================================
%  SUMMARY
%  =====================================================================
fprintf('\n========================================\n');
fprintf(' Models built successfully!\n');
fprintf('========================================\n\n');

fprintf(' Phase 1: %s.slx (Cart on Table — Open Loop)\n', mdl1);
fprintf('   Sine --> Amp --> Sat --> SS(2x2) --> [x_c, x_c_dot]\n\n');

fprintf(' Phase 2: %s.slx (Cart on Seesaw — Open Loop)\n', mdl2);
fprintf('   Sine --> Amp --> Sat --> SS(4x4) --> [x_c, x_c_dot, alpha, alpha_dot]\n\n');

if phase3_built
    fprintf(' Phase 3: %s.slx (Seesaw — Full State Feedback Control)\n', mdl3);
    fprintf('   u = -K * [e_xc, x_c_dot, alpha, alpha_dot]\n');
    fprintf('   K = [%.3f, %.3f, %.3f, %.3f]\n', ctrl.K);
    fprintf('   e_xc = x_c_ref - x_c (position tracking error)\n\n');
else
    fprintf(' Phase 3: SKIPPED (controller_freq.mat not found)\n\n');
end

fprintf(' SIMULATION MODE (Phase 1 & 2):\n');
fprintf('   1. Run seesaw_params.m\n');
fprintf('   2. Open model, set mode to "Normal", click Run\n\n');

if quarc_available
    fprintf(' HARDWARE MODE (QUARC External):\n');
    fprintf('   Phases 1 & 2 (identification):\n');
    fprintf('     1. seesaw_params\n');
    fprintf('     2. Connect Q2-USB, power on VoltPAQ (switch = 1x)\n');
    fprintf('     3. QUARC | Build | Connect | Start\n\n');
    if phase3_built
        fprintf('   Phase 3 (state feedback control):\n');
        fprintf('     1. seesaw_params; control_pipeline;\n');
        fprintf('     2. Connect Q2-USB, power on VoltPAQ (switch = 1x)\n');
        fprintf('     3. QUARC | Build (Ctrl+B)\n');
        fprintf('     4. QUARC | Connect (Ctrl+T)\n');
        fprintf('     5. *** HOLD SEESAW LEVEL BY HAND ***\n');
        fprintf('     6. QUARC | Start — release seesaw gently\n');
        fprintf('     7. If it diverges: QUARC | Stop IMMEDIATELY\n\n');
        fprintf('   SAFETY NOTES:\n');
        fprintf('     - Voltage is hard-limited to +/- %.1f V by Saturation block\n', V_sat);
        fprintf('     - Step block defaults to x_c_ref = 0 (hold at center)\n');
        fprintf('     - Velocities estimated via dirty derivative (tau=%.3f s)\n', tau_d);
        fprintf('     - If cart hits end-stops, STOP and retune poles in control_pipeline\n');
        fprintf('     - Watch Voltage scope: if rail-to-rail, reduce wn_dom\n\n');
    end
else
    fprintf(' TO ADD HARDWARE LATER:\n');
    fprintf('   Re-run build_simulink_models.m with QUARC on the path.\n\n');
end
fprintf('\n');
