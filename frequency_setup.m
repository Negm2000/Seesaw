%% frequency_setup.m
%  -----------------------------------------------------------------------
%  STEP 1: Analytical Analysis & Simulink Model Builder
%  -----------------------------------------------------------------------
%  Run this script to:
%    1. Analyze the mathematical transfer function of the cart.
%    2. Build the IP02_FreqTest.slx model for hardware testing.
%  -----------------------------------------------------------------------

if ~exist('K_a', 'var')
    seesaw_params;
end

fprintf('\n========================================\n');
fprintf(' Phase 1: Frequency Response Setup\n');
fprintf('========================================\n');

% --- SECTION 1: Analytical Transfer Function ---
alpha_f = (eta_g * K_g * k_t) / (R_m * r_mp);
B_emf = alpha_f * K_g * k_m / r_mp;
B_total = B_eq + B_emf;

fprintf('  B_total = %.2f N*s/m (B_eq + B_emf)\n', B_total);

% Build transfer functions
s = tf('s');
G_xc = K_a * alpha_f * eta_m / (M_c * s^2 + B_total * s);
G_xcdot = K_a * alpha_f * eta_m / (M_c * s + B_total);

% --- SECTION 2: Plot Analytical Bode ---
freq_range = logspace(-1, 1.5, 200);   
[mag, phase] = bode(G_xc, 2*pi*freq_range);
G_xc_dB = 20*log10(squeeze(mag)*100);
G_xc_phase = squeeze(phase);

figure('Name', 'Analytical Bode: Cart on Table', 'Position', [50 50 1000 600]);
subplot(2,1,1); semilogx(freq_range, G_xc_dB, 'b-', 'LineWidth', 2);
grid on; ylabel('Magnitude [dB]'); title('V_cmd -> x_c [cm/V]');
subplot(2,1,2); semilogx(freq_range, G_xc_phase, 'b-', 'LineWidth', 2);
grid on; ylabel('Phase [deg]'); xlabel('Frequency [Hz]');
sgtitle('Analytical Model Prediction');

% --- SECTION 3: Build QUARC Chirp Test Model ---
fprintf('\n--- Building QUARC Chirp Test Model ---\n');

mdl_freq = 'IP02_FreqTest';
chirp_duration = 120;   % seconds
A_chirp = 3.0;          % chirp amplitude [V]
f_chirp_start = 0.1;    % start frequency [Hz]
f_chirp_end = 20.0;     % end frequency [Hz]
Ts_quarc = 0.002;       % QUARC sample time [s]

% Generate logarithmic chirp signal for workspace
t_chirp = (0:Ts_quarc:chirp_duration)';
log_ratio = log(f_chirp_end / f_chirp_start);
phase_chirp = 2*pi * f_chirp_start * chirp_duration / log_ratio ...
    * ((f_chirp_end/f_chirp_start).^(t_chirp/chirp_duration) - 1);
chirp_signal = A_chirp * sin(phase_chirp);

assignin('base', 'chirp_input', timeseries(chirp_signal, t_chirp));
assignin('base', 'chirp_duration', chirp_duration);

% Build Simulink model
if bdIsLoaded(mdl_freq), close_system(mdl_freq, 0); end
new_system(mdl_freq);
open_system(mdl_freq);

% Solver settings
try set_param(mdl_freq, 'Solver', 'ode45', 'StopTime', num2str(chirp_duration)); catch, end

% ---- BLOCKS ----
add_block('simulink/Sources/From Workspace', [mdl_freq '/Chirp_Vcmd'], 'VariableName', 'chirp_input', 'Position', [50 150 150 190]);
add_block('simulink/Math Operations/Gain', [mdl_freq '/Amp_Gain'], 'Gain', 'K_a', 'Position', [180 153 220 187]);
add_block('simulink/Discontinuities/Saturation', [mdl_freq '/V_Sat'], 'UpperLimit', 'V_sat', 'LowerLimit', '-V_sat', 'Position', [240 153 285 187]);
add_block('simulink/Continuous/State-Space', [mdl_freq '/Cart_SS'], 'A', 'A_cart', 'B', 'B_cart', 'C', 'C_cart', 'D', 'D_cart', 'Position', [310 135 430 205]);
add_block('simulink/Signal Routing/Demux', [mdl_freq '/Demux'], 'Outputs', '2', 'Position', [470 145 475 195]);
add_block('simulink/Math Operations/Gain', [mdl_freq '/m_to_cm'], 'Gain', '100', 'Position', [540 115 570 135]);
add_block('simulink/Math Operations/Gain', [mdl_freq '/vel_to_cms'], 'Gain', '100', 'Position', [540 155 570 175]);
add_block('simulink/Sinks/To Workspace', [mdl_freq '/ToWS_sim_xc'], 'VariableName', 'freq_sim_xc', 'SaveFormat', 'Timeseries', 'Position', [620 115 690 135]);
add_block('simulink/Sinks/To Workspace', [mdl_freq '/ToWS_sim_xcdot'], 'VariableName', 'freq_sim_xcdot', 'SaveFormat', 'Timeseries', 'Position', [620 155 690 175]);
add_block('simulink/Sinks/To Workspace', [mdl_freq '/ToWS_Vcmd'], 'VariableName', 'freq_Vcmd', 'SaveFormat', 'Timeseries', 'Position', [250 50 320 70]);

% ---- WIRING ----
add_line(mdl_freq, 'Chirp_Vcmd/1', 'Amp_Gain/1', 'autorouting', 'smart');
add_line(mdl_freq, 'Amp_Gain/1', 'V_Sat/1', 'autorouting', 'smart');
add_line(mdl_freq, 'V_Sat/1', 'Cart_SS/1', 'autorouting', 'smart');
add_line(mdl_freq, 'Cart_SS/1', 'Demux/1', 'autorouting', 'smart');
add_line(mdl_freq, 'Chirp_Vcmd/1', 'ToWS_Vcmd/1', 'autorouting', 'smart');
add_line(mdl_freq, 'Demux/1', 'm_to_cm/1', 'autorouting', 'smart');
add_line(mdl_freq, 'Demux/2', 'vel_to_cms/1', 'autorouting', 'smart');
add_line(mdl_freq, 'm_to_cm/1', 'ToWS_sim_xc/1', 'autorouting', 'smart');
add_line(mdl_freq, 'vel_to_cms/1', 'ToWS_sim_xcdot/1', 'autorouting', 'smart');

% ---- QUARC HARDWARE BLOCKS ----
quarc_available = exist('quarc_library', 'file') == 4;
if quarc_available
    fprintf('  Adding QUARC hardware blocks...\n');
    try set_param(mdl_freq, 'SimulationMode', 'external'); catch, end
    try set_param(mdl_freq, 'SystemTargetFile', 'quarc_win64.tlc'); catch, end
    try set_param(mdl_freq, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', num2str(Ts_quarc)); catch, end
    try set_param(mdl_freq, 'SaveLog', 'on'); set_param(mdl_freq, 'SignalLogging', 'on'); catch, end

    add_block('quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize', [mdl_freq '/HIL Initialize'], 'Position', [50 350 134 425]);
    add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Write Analog', [mdl_freq '/Motor Command'], 'Position', [300 360 385 422]);
    add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Read Encoder', [mdl_freq '/Encoder'], 'Position', [300 450 385 512]);
    add_block('simulink/Math Operations/Gain', [mdl_freq '/Enc_to_m'], 'Gain', 'K_ec * r_pp', 'Position', [430 465 480 495]);
    add_block('simulink/Continuous/Derivative', [mdl_freq '/Deriv_xc'], 'Position', [520 515 560 545]);
    add_block('simulink/Signal Routing/Mux', [mdl_freq '/Mux_Logging'], 'Inputs', '3', 'Position', [700 600 705 660]);
    
    hBlock = add_block('quarc_library/Sinks/To Host/To Host File', [mdl_freq '/To_Host_File'], ...
        'Position', [750 610 830 650]);
    try
        set_param(hBlock, 'file_name', 'data.mat', 'file_format', 'MAT-file');
    catch
        % Fallback for different QUARC versions
        try set_param(hBlock, 'FileName', 'data.mat', 'FileFormat', 'MAT-file'); catch, end
    end

    add_line(mdl_freq, 'V_Sat/1', 'Motor Command/1', 'autorouting', 'smart');
    add_line(mdl_freq, 'Encoder/1', 'Enc_to_m/1', 'autorouting', 'smart');
    add_line(mdl_freq, 'Enc_to_m/1', 'Deriv_xc/1', 'autorouting', 'smart');
    add_line(mdl_freq, 'Chirp_Vcmd/1', 'Mux_Logging/1', 'autorouting', 'smart');
    add_line(mdl_freq, 'Enc_to_m/1', 'Mux_Logging/2', 'autorouting', 'smart');
    add_line(mdl_freq, 'Deriv_xc/1', 'Mux_Logging/3', 'autorouting', 'smart');
    add_line(mdl_freq, 'Mux_Logging/1', 'To_Host_File/1', 'autorouting', 'smart');
end

save_system(mdl_freq);
fprintf('  Model ready: %s.slx\n', mdl_freq);
