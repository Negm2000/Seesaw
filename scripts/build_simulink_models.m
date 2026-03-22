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
    SEESAW_ROOT = fileparts(curr_path); 
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
    SEESAW_ROOT = fileparts(curr_path); 
end
save_system(mdl2, fullfile(SEESAW_ROOT, 'models', [mdl2 '.slx']));
fprintf('  Phase 2 model saved: models/%s.slx\n', mdl2);

%% =====================================================================
%  SUMMARY
%  =====================================================================
fprintf('\n========================================\n');
fprintf(' Models built successfully!\n');
fprintf('========================================\n\n');
fprintf(' Phase 1: %s.slx (Cart on Table)\n', mdl1);
fprintf('   Signal chain:  Sine --> Gain(K_a) --> Sat(V_sat) --> SS(2x2)\n');
fprintf('   States:        [x_c, x_c_dot]  (A_cart, B_cart, C_cart, D_cart)\n');
fprintf('   Outputs:       x_c [m], x_c_dot [m/s]\n\n');
fprintf(' Phase 2: %s.slx (Cart on Seesaw)\n', mdl2);
fprintf('   Signal chain:  Sine --> Gain(K_a) --> Sat(V_sat) --> SS(4x4)\n');
fprintf('   States:        [x_c, x_c_dot, alpha, alpha_dot]  (A_sw, B_sw, C_sw, D_sw)\n');
fprintf('   Outputs:       x_c [m], x_c_dot [m/s], alpha [rad], alpha_dot [rad/s]\n\n');
fprintf(' SIMULATION MODE:\n');
fprintf('   1. Run seesaw_params.m\n');
fprintf('   2. Open model, set mode to "Normal"\n');
fprintf('   3. Click Run (Play button)\n');
fprintf('   4. Check scopes and workspace variables\n\n');
if quarc_available
    fprintf(' HARDWARE VALIDATION MODE (QUARC):\n');
    fprintf('   1. Run seesaw_params.m\n');
    fprintf('   2. Connect Q2-USB, power on VoltPAQ (Gain switch = 1x)\n');
    fprintf('   3. Solver: ode1, Ts=0.002s (500 Hz) -- already set\n');
    fprintf('   4. Set SimulationMode to "External"\n');
    fprintf('   5. QUARC | Build (Ctrl+B)\n');
    fprintf('   6. QUARC | Connect (Ctrl+T)\n');
    fprintf('   7. QUARC | Start\n');
    fprintf('   8. Wait 120 seconds or until finished\n');
    fprintf('   9. QUARC | Stop\n');
    fprintf('  10. Run Sections 5 & 6 of frequency_test.m to compare\n\n');
    fprintf('  QUARC blocks (matching Seesaw_Template.slx):\n');
    fprintf('   - HIL Initialize:  q2_usb board 0\n');
    fprintf('   - Motor Command:   HIL Write Analog ch0 (DAQ --> VoltPAQ --> motor)\n');
    fprintf('   - Encoders:        HIL Read Encoder ch[0,1] (cart + seesaw)\n');
    fprintf('   - To Host File:    data logging\n');
else
    fprintf(' TO ADD HARDWARE LATER (when QUARC is installed):\n');
    fprintf('   Re-run build_simulink_models.m with QUARC on the path.\n');
    fprintf('   QUARC block paths (from Seesaw_Template.slx):\n');
    fprintf('     quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize\n');
    fprintf('     quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Write Analog\n');
    fprintf('     quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Read Encoder\n');
    fprintf('     quarc_library/Sinks/To Host/To Host File\n');
    fprintf('   Settings: ode1 fixed-step solver, Ts=0.002s, SimMode=External\n');
end
fprintf('\n');
