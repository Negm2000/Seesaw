%% frequency_test.m
%  -----------------------------------------------------------------------
%  Frequency Response Validation: Model vs Hardware
%  Phase 1: IP02 Cart on Flat Table (open-loop)
%  -----------------------------------------------------------------------
%  PREREQUISITE: Run seesaw_params.m first.
%
%  WORKFLOW:
%    Step 1: Run Sections 1-3 on your PC to see model predictions
%    Step 2: Copy all files to the lab PC (with QUARC)
%    Step 3: Run Section 4 on lab PC to build QUARC chirp model
%    Step 4: Build & deploy via QUARC, collect data
%    Step 5: Run Section 5 on lab PC (or copy data back) to compare
%
%  The script validates the cart model by comparing:
%    - Analytical Bode plot (linearized transfer function)
%    - Nonlinear simulation frequency sweep (ODE at each frequency)
%    - Hardware chirp response (from QUARC test)
%
%  What to tune if model doesn't match hardware:
%    - B_eq:  adjust friction to match gain rolloff
%    - eta_g: adjust if overall gain is off
%    - eta_m: adjust if overall gain is off
%  -----------------------------------------------------------------------

%% ===== Check parameters =====
if ~exist('K_a', 'var')
    error('Run seesaw_params.m first!');
end

%% =====================================================================
%  SECTION 1: Analytical Transfer Function
%  =====================================================================
%  Cart on flat table, reduced motor model (L_m = 0, Good ref Eq. 2.3):
%
%    F_c = alpha_f * (-K_g * k_m * x_c_dot / r_mp + eta_m * V_m)
%
%  where alpha_f = eta_g * K_g * k_t / (R_m * r_mp)
%
%  Cart equation: m_c * x_c_ddot = F_c - B_eq * x_c_dot
%
%  Substituting F_c and rearranging:
%    m_c * s^2 * X_c(s) = alpha_f * eta_m * V_m(s) - (B_eq + B_emf) * s * X_c(s)
%
%  Transfer functions (V_cmd -> outputs, with V_m = K_a * V_cmd):
%    G_xc(s)    = K_a * alpha_f * eta_m / [m_c * s^2 + (B_eq + B_emf) * s]
%    G_xcdot(s) = K_a * alpha_f * eta_m / [m_c * s + (B_eq + B_emf)]
%  =====================================================================

fprintf('\n========================================\n');
fprintf(' Frequency Response Analysis\n');
fprintf('========================================\n');

% Motor force gain factor
alpha_f = (eta_g * K_g * k_t) / (R_m * r_mp);

% Back-EMF damping (embedded in F_c)
B_emf = alpha_f * K_g * k_m / r_mp;

% Total effective damping at cart
B_total = B_eq + B_emf;

fprintf('\nPhase 1: Cart on Flat Table\n');
fprintf('  alpha_f  = %.4f (motor force constant)\n', alpha_f);
fprintf('  B_emf    = %.2f N*s/m (back-EMF damping inside F_c)\n', B_emf);
fprintf('  B_eq     = %.2f N*s/m (mechanical friction)\n', B_eq);
fprintf('  B_total  = %.2f N*s/m (total effective damping)\n', B_total);
fprintf('  m_c      = %.3f kg\n', M_c);

% Key system characteristics
dc_gain_vel = K_a * alpha_f * eta_m / B_total;
bw_vel = B_total / M_c;
tau_vel = M_c / B_total;

fprintf('\nVelocity response (1st order):\n');
fprintf('  DC gain   = %.4f (m/s)/V = %.2f (cm/s)/V\n', dc_gain_vel, dc_gain_vel*100);
fprintf('  Bandwidth = %.2f rad/s = %.2f Hz\n', bw_vel, bw_vel/(2*pi));
fprintf('  Time const = %.4f s\n', tau_vel);

% Build transfer functions using MATLAB Control Toolbox
s = tf('s');

% V_cmd -> cart position [m]
G_xc = K_a * alpha_f * eta_m / (M_c * s^2 + B_total * s);

% V_cmd -> cart velocity [m/s]
G_xcdot = K_a * alpha_f * eta_m / (M_c * s + B_total);

% V_cmd -> motor current [A] (algebraic: i_m = (V_m - k_m*omega_m)/R_m)
% At steady state for sine: this depends on frequency, computed separately

fprintf('\nTransfer functions:\n');
fprintf('  G_xc(s)    = %.4f / (%.4f*s^2 + %.4f*s)  [V_cmd -> x_c]\n', ...
    K_a*alpha_f*eta_m, M_c, B_total);
fprintf('  G_xcdot(s) = %.4f / (%.4f*s + %.4f)      [V_cmd -> x_c_dot]\n', ...
    K_a*alpha_f*eta_m, M_c, B_total);

%% =====================================================================
%  SECTION 2: Analytical Bode Plots
%  =====================================================================

freq_range = logspace(-1, 1.5, 200);   % 0.1 to ~31 Hz
omega_range = 2*pi*freq_range;

% Evaluate frequency response analytically
G_xc_mag = zeros(size(omega_range));
G_xc_phase = zeros(size(omega_range));
G_xcdot_mag = zeros(size(omega_range));
G_xcdot_phase = zeros(size(omega_range));

for k = 1:length(omega_range)
    w = omega_range(k);
    jw = 1j * w;

    % Position: G_xc(jw)
    H_xc = K_a * alpha_f * eta_m / (M_c * (jw)^2 + B_total * jw);
    G_xc_mag(k) = abs(H_xc);
    G_xc_phase(k) = angle(H_xc) * 180/pi;

    % Velocity: G_xcdot(jw)
    H_xcdot = K_a * alpha_f * eta_m / (M_c * jw + B_total);
    G_xcdot_mag(k) = abs(H_xcdot);
    G_xcdot_phase(k) = angle(H_xcdot) * 180/pi;
end

% Convert magnitude to dB (using cm for position, cm/s for velocity)
G_xc_dB = 20*log10(G_xc_mag * 100);         % [cm/V]
G_xcdot_dB = 20*log10(G_xcdot_mag * 100);   % [(cm/s)/V]

figure('Name', 'Analytical Bode: Cart on Table', 'Position', [50 50 1000 700]);

% Position Bode
subplot(2,2,1);
semilogx(freq_range, G_xc_dB, 'b-', 'LineWidth', 2);
xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
title('V_{cmd} \rightarrow x_c [cm/V]');
grid on; xlim([0.1 30]);

subplot(2,2,3);
semilogx(freq_range, G_xc_phase, 'b-', 'LineWidth', 2);
xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
grid on; xlim([0.1 30]); ylim([-270 0]);

% Velocity Bode
subplot(2,2,2);
semilogx(freq_range, G_xcdot_dB, 'b-', 'LineWidth', 2);
xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
title('V_cmd -> x_c_dot [cm-s/V]', 'Interpreter', 'none');
grid on; xlim([0.1 30]);

subplot(2,2,4);
semilogx(freq_range, G_xcdot_phase, 'b-', 'LineWidth', 2);
xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
grid on; xlim([0.1 30]); ylim([-180 0]);

sgtitle('Analytical Bode Plot: Cart on Flat Table (linearized)', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'none');

%% =====================================================================
%  SECTION 3: Nonlinear Simulation Frequency Sweep
%  =====================================================================
%  Run the actual nonlinear ODE at each test frequency to verify the
%  linearized Bode plot. The nonlinear simulation should match the
%  analytical curve for small-amplitude inputs.

fprintf('\n--- Nonlinear Simulation Frequency Sweep ---\n');

% Test frequencies [Hz]
test_freqs = [0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0, 20.0];
A_test = 1.0;   % Input amplitude [V] — keep small for linearity

% Preallocate results
n_freqs = length(test_freqs);
sim_xc_amp = zeros(1, n_freqs);
sim_xc_phase = zeros(1, n_freqs);
sim_xcdot_amp = zeros(1, n_freqs);
sim_xcdot_phase = zeros(1, n_freqs);

% Pack parameters for ODE
p_ode = struct('K_a',K_a, 'V_sat',V_sat, 'R_m',R_m, 'k_t',k_t, 'k_m',k_m, ...
    'eta_m',eta_m, 'eta_g',eta_g, 'K_g',K_g, 'r_mp',r_mp, ...
    'M_c',M_c, 'B_eq',B_eq, 'x_c_max',x_c_max);

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3);

for k = 1:n_freqs
    f = test_freqs(k);
    T = 1/f;

    % Run long enough for transient to decay + 5 steady-state cycles
    n_settle = max(5, ceil(5 * tau_vel / T));  % settle cycles
    n_measure = 5;                              % measurement cycles
    t_end = (n_settle + n_measure) * T;

    V_cmd_fn = @(t) A_test * sin(2*pi*f*t);

    ode_fn = @(t, x) cart_ode_freq(t, x, V_cmd_fn, p_ode);
    [t_sim, x_sim] = ode45(ode_fn, [0 t_end], [0; 0], opts);

    % Extract last n_measure cycles for steady-state analysis
    t_start = n_settle * T;
    idx = t_sim >= t_start;
    t_ss = t_sim(idx);
    xc_ss = x_sim(idx, 1);
    xcdot_ss = x_sim(idx, 2);

    % Extract amplitude and phase via FFT
    [sim_xc_amp(k), sim_xc_phase(k)] = extract_freq_response(t_ss, xc_ss, f);
    [sim_xcdot_amp(k), sim_xcdot_phase(k)] = extract_freq_response(t_ss, xcdot_ss, f);

    % Also get input phase (should be ~0, but extract for consistency)
    V_ss = arrayfun(V_cmd_fn, t_ss);
    [~, input_phase] = extract_freq_response(t_ss, V_ss, f);
    sim_xc_phase(k) = sim_xc_phase(k) - input_phase;
    sim_xcdot_phase(k) = sim_xcdot_phase(k) - input_phase;

    fprintf('  f=%5.1f Hz: |x_c|=%.4f cm, phase=%.1f deg\n', ...
        f, sim_xc_amp(k)*100, sim_xc_phase(k));
end

% Convert to dB (cm units)
sim_xc_dB = 20*log10(sim_xc_amp * 100 / A_test);
sim_xcdot_dB = 20*log10(sim_xcdot_amp * 100 / A_test);

% Overlay on analytical Bode plot
figure('Name', 'Model Bode: Analytical + Simulation', 'Position', [100 80 1000 700]);

% Position
subplot(2,2,1);
semilogx(freq_range, G_xc_dB, 'b-', 'LineWidth', 1.5); hold on;
semilogx(test_freqs, sim_xc_dB, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
title('V_cmd -> x_c [cm/V]', 'Interpreter', 'none');
legend('Analytical (linear)', 'Simulation (nonlinear)', 'Location', 'best');
grid on; xlim([0.1 30]);

subplot(2,2,3);
semilogx(freq_range, G_xc_phase, 'b-', 'LineWidth', 1.5); hold on;
semilogx(test_freqs, sim_xc_phase, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
grid on; xlim([0.1 30]); ylim([-270 0]);

% Velocity
subplot(2,2,2);
semilogx(freq_range, G_xcdot_dB, 'b-', 'LineWidth', 1.5); hold on;
semilogx(test_freqs, sim_xcdot_dB, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
title('V_cmd -> x_c_dot [cm-s/V]', 'Interpreter', 'none');
legend('Analytical (linear)', 'Simulation (nonlinear)', 'Location', 'best');
grid on; xlim([0.1 30]);

subplot(2,2,4);
semilogx(freq_range, G_xcdot_phase, 'b-', 'LineWidth', 1.5); hold on;
semilogx(test_freqs, sim_xcdot_phase, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
grid on; xlim([0.1 30]); ylim([-180 0]);

sgtitle(sprintf('Cart on Table: Model Frequency Response (A_test=%.1fV)', A_test), ...
    'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'none');

% Save simulation results for later comparison
sim_freq_results = struct();
sim_freq_results.freqs = test_freqs;
sim_freq_results.A_test = A_test;
sim_freq_results.xc_amp = sim_xc_amp;
sim_freq_results.xc_phase = sim_xc_phase;
sim_freq_results.xcdot_amp = sim_xcdot_amp;
sim_freq_results.xcdot_phase = sim_xcdot_phase;

fprintf('\nSimulation sweep complete.\n');
fprintf('Linear vs nonlinear should match closely at A=%.1fV.\n', A_test);

%% =====================================================================
%  SECTION 4: Build QUARC Chirp Test Model
%  =====================================================================
%  Creates IP02_FreqTest.slx with:
%    - Logarithmic chirp input (0.1 to 20 Hz over 120 seconds)
%    - Cart plant S-function (model prediction)
%    - QUARC hardware blocks (for real-time validation)
%    - To Workspace logging for both sim and hardware
%
%  NOTE: This model is designed for the LAB PC with QUARC installed.
%        On your home PC, it will build in simulation-only mode.

fprintf('\n--- Building QUARC Chirp Test Model ---\n');

mdl_freq = 'IP02_FreqTest';
chirp_duration = 120;   % seconds
A_chirp = 1.0;          % chirp amplitude [V]
f_chirp_start = 0.1;    % start frequency [Hz]
f_chirp_end = 20.0;     % end frequency [Hz]
Ts_quarc = 0.002;       % QUARC sample time [s]

% Generate logarithmic chirp signal
t_chirp = (0:Ts_quarc:chirp_duration)';
% Instantaneous frequency: f(t) = f_start * (f_end/f_start)^(t/T)
% Phase: phi(t) = 2*pi*f_start*T/ln(f_end/f_start) * ((f_end/f_start)^(t/T) - 1)
log_ratio = log(f_chirp_end / f_chirp_start);
f_inst = f_chirp_start * (f_chirp_end/f_chirp_start).^(t_chirp/chirp_duration);
phase_chirp = 2*pi * f_chirp_start * chirp_duration / log_ratio ...
    * ((f_chirp_end/f_chirp_start).^(t_chirp/chirp_duration) - 1);
chirp_signal = A_chirp * sin(phase_chirp);

% Save chirp to workspace for From Workspace block
chirp_ts = timeseries(chirp_signal, t_chirp);
assignin('base', 'chirp_input', chirp_ts);
assignin('base', 'chirp_duration', chirp_duration);

fprintf('  Chirp: %.1f to %.1f Hz over %.0f s, amplitude = %.1f V\n', ...
    f_chirp_start, f_chirp_end, chirp_duration, A_chirp);

% Also save instantaneous frequency for analysis
assignin('base', 'f_inst_chirp', f_inst);
assignin('base', 't_chirp', t_chirp);

% Build Simulink model
if bdIsLoaded(mdl_freq), close_system(mdl_freq, 0); end
new_system(mdl_freq);
open_system(mdl_freq);

% Solver: use ode45 for simulation; QUARC will override to ode1
set_param(mdl_freq, 'Solver', 'ode45', ...
    'StopTime', num2str(chirp_duration), ...
    'MaxStep', '1e-3', ...
    'RelTol', '1e-4', ...
    'AbsTol', '1e-6');

% ---- CHIRP INPUT ----
add_block('simulink/Sources/From Workspace', [mdl_freq '/Chirp_Vcmd'], ...
    'VariableName', 'chirp_input', ...
    'Position', [50 150 150 190]);

% ---- AMPLIFIER GAIN (K_a) ----
add_block('simulink/Math Operations/Gain', [mdl_freq '/Amp_Gain'], ...
    'Gain', 'K_a', 'Position', [180 153 220 187]);

% ---- VOLTAGE SATURATION (V_sat) ----
add_block('simulink/Discontinuities/Saturation', [mdl_freq '/V_Sat'], ...
    'UpperLimit', 'V_sat', 'LowerLimit', '-V_sat', ...
    'Position', [240 153 285 187]);

% ---- STATE-SPACE PLANT ----
% Uses A_cart, B_cart from seesaw_params.m
add_block('simulink/Continuous/State-Space', [mdl_freq '/Cart_SS'], ...
    'A', 'A_cart', 'B', 'B_cart', 'C', 'C_cart', 'D', 'D_cart', ...
    'Position', [310 135 430 205]);

% ---- DEMUX ----
add_block('simulink/Signal Routing/Demux', [mdl_freq '/Demux'], ...
    'Outputs', '2', 'Position', [470 145 475 195]);

% ---- UNIT CONVERSIONS ----
add_block('simulink/Math Operations/Gain', [mdl_freq '/m_to_cm'], ...
    'Gain', '100', ...
    'Position', [540 115 570 135]);

add_block('simulink/Math Operations/Gain', [mdl_freq '/vel_to_cms'], ...
    'Gain', '100', ...
    'Position', [540 155 570 175]);

% ---- SCOPES ----
add_block('simulink/Sinks/Scope', [mdl_freq '/Scope_Vcmd'], ...
    'NumInputPorts', '1', ...
    'Position', [700 35 740 65]);
set_param([mdl_freq '/Scope_Vcmd'], 'Name', 'V_cmd');

add_block('simulink/Sinks/Scope', [mdl_freq '/Scope_Pos'], ...
    'NumInputPorts', '1', ...
    'Position', [700 110 740 140]);
set_param([mdl_freq '/Scope_Pos'], 'Name', 'Sim Position [cm]');

add_block('simulink/Sinks/Scope', [mdl_freq '/Scope_Vel'], ...
    'NumInputPorts', '1', ...
    'Position', [700 150 740 180]);
set_param([mdl_freq '/Scope_Vel'], 'Name', 'Sim Velocity [cm-s]');

% ---- TO WORKSPACE (simulation data) ----
add_block('simulink/Sinks/To Workspace', [mdl_freq '/ToWS_Vcmd'], ...
    'VariableName', 'freq_Vcmd', 'SaveFormat', 'Timeseries', ...
    'Position', [250 50 320 70]);

add_block('simulink/Sinks/To Workspace', [mdl_freq '/ToWS_sim_xc'], ...
    'VariableName', 'freq_sim_xc', 'SaveFormat', 'Timeseries', ...
    'Position', [620 115 690 135]);

add_block('simulink/Sinks/To Workspace', [mdl_freq '/ToWS_sim_xcdot'], ...
    'VariableName', 'freq_sim_xcdot', 'SaveFormat', 'Timeseries', ...
    'Position', [620 155 690 175]);

% ---- WIRING ----
add_line(mdl_freq, 'Chirp_Vcmd/1', 'Amp_Gain/1', 'autorouting', 'smart');
add_line(mdl_freq, 'Amp_Gain/1', 'V_Sat/1', 'autorouting', 'smart');
add_line(mdl_freq, 'V_Sat/1', 'Cart_SS/1', 'autorouting', 'smart');
add_line(mdl_freq, 'Cart_SS/1', 'Demux/1', 'autorouting', 'smart');

add_line(mdl_freq, 'Chirp_Vcmd/1', 'V_cmd/1', 'autorouting', 'smart');
add_line(mdl_freq, 'Chirp_Vcmd/1', 'ToWS_Vcmd/1', 'autorouting', 'smart');

add_line(mdl_freq, 'Demux/1', 'm_to_cm/1', 'autorouting', 'smart');
add_line(mdl_freq, 'Demux/2', 'vel_to_cms/1', 'autorouting', 'smart');

add_line(mdl_freq, 'm_to_cm/1', 'Sim Position [cm]/1', 'autorouting', 'smart');
add_line(mdl_freq, 'vel_to_cms/1', 'Sim Velocity [cm-s]/1', 'autorouting', 'smart');
add_line(mdl_freq, 'm_to_cm/1', 'ToWS_sim_xc/1', 'autorouting', 'smart');
add_line(mdl_freq, 'vel_to_cms/1', 'ToWS_sim_xcdot/1', 'autorouting', 'smart');

% ---- QUARC HARDWARE BLOCKS ----
quarc_available = exist('quarc_library', 'file') == 4;
if ~quarc_available
    try
        quarc_available = ~isempty(ver('quarc'));
    catch
        quarc_available = false;
    end
end

if quarc_available
    fprintf('  Adding QUARC hardware blocks...\n');

    % --- FORCE EXTERNAL MODE & LOGGING SETTINGS ---
    set_param(mdl_freq, 'SimulationMode', 'external');
    
    % QUARC Target & Solver
    set_param(mdl_freq, 'SystemTargetFile', 'quarc_win64.tlc');
    set_param(mdl_freq, 'SolverType', 'Fixed-step', ...
        'Solver', 'ode1', ...
        'FixedStep', num2str(Ts_quarc), ...
        'StopTime', num2str(chirp_duration));

    % Enable MAT-file logging (This ensures data persists after STOP)
    set_param(mdl_freq, 'RTWLogStorageType', 'RAM');
    set_param(mdl_freq, 'SaveLog', 'on');
    set_param(mdl_freq, 'SignalLogging', 'on');
    set_param(mdl_freq, 'SaveFormat', 'Dataset'); % Modern standard

    try
        % HIL Initialize
        add_block('quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize', ...
            [mdl_freq '/HIL Initialize'], ...
            'Position', [50 350 134 425]);

        % HIL Write Analog (motor command)
        add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Write Analog', ...
            [mdl_freq '/Motor Command'], ...
            'Position', [300 360 385 422]);

        % HIL Read Encoder (cart position)
        add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Read Encoder', ...
            [mdl_freq '/Encoder'], ...
            'Position', [300 450 385 512]);

        % Encoder to cart position [m]
        add_block('simulink/Math Operations/Gain', [mdl_freq '/Enc_to_m'], ...
            'Gain', 'K_ec * r_pp', ...
            'Position', [430 465 480 495]);

        % Hardware position to cm
        add_block('simulink/Math Operations/Gain', [mdl_freq '/HW_to_cm'], ...
            'Gain', '100', ...
            'Position', [520 465 550 495]);

        % Numerical derivative for velocity (1st-order backward diff)
        add_block('simulink/Continuous/Derivative', [mdl_freq '/Deriv_xc'], ...
            'Position', [520 515 560 545]);

        add_block('simulink/Math Operations/Gain', [mdl_freq '/HW_vel_to_cms'], ...
            'Gain', '100', ...
            'Position', [600 515 630 545]);

        % Comparison scopes
        add_block('simulink/Sinks/Scope', [mdl_freq '/Scope_Cmp_Pos'], ...
            'NumInputPorts', '2', ...
            'Position', [700 440 740 500]);
        set_param([mdl_freq '/Scope_Cmp_Pos'], 'Name', 'Sim vs HW Position [cm]');

        add_block('simulink/Sinks/Scope', [mdl_freq '/Scope_Cmp_Vel'], ...
            'NumInputPorts', '2', ...
            'Position', [700 510 740 570]);
        set_param([mdl_freq '/Scope_Cmp_Vel'], 'Name', 'Sim vs HW Velocity [cm-s]');

        % To Workspace (hardware data)
        add_block('simulink/Sinks/To Workspace', [mdl_freq '/ToWS_hw_xc'], ...
            'VariableName', 'freq_hw_xc', 'SaveFormat', 'Timeseries', ...
            'Position', [600 460 670 480]);

        add_block('simulink/Sinks/To Workspace', [mdl_freq '/ToWS_hw_xcdot'], ...
            'VariableName', 'freq_hw_xcdot', 'SaveFormat', 'Timeseries', ...
            'Position', [670 515 740 535]);

        % Wire QUARC blocks
        add_line(mdl_freq, 'V_Sat/1', 'Motor Command/1', 'autorouting', 'smart');
        add_line(mdl_freq, 'Encoder/1', 'Enc_to_m/1', 'autorouting', 'smart');
        add_line(mdl_freq, 'Enc_to_m/1', 'HW_to_cm/1', 'autorouting', 'smart');
        add_line(mdl_freq, 'Enc_to_m/1', 'Deriv_xc/1', 'autorouting', 'smart');
        add_line(mdl_freq, 'Deriv_xc/1', 'HW_vel_to_cms/1', 'autorouting', 'smart');
        add_line(mdl_freq, 'HW_to_cm/1', 'Sim vs HW Position [cm]/2', 'autorouting', 'smart');
        add_line(mdl_freq, 'HW_vel_to_cms/1', 'Sim vs HW Velocity [cm-s]/2', 'autorouting', 'smart');
        add_line(mdl_freq, 'm_to_cm/1', 'Sim vs HW Position [cm]/1', 'autorouting', 'smart');
        add_line(mdl_freq, 'vel_to_cms/1', 'Sim vs HW Velocity [cm-s]/1', 'autorouting', 'smart');
        add_line(mdl_freq, 'HW_to_cm/1', 'ToWS_hw_xc/1', 'autorouting', 'smart');
        add_line(mdl_freq, 'HW_vel_to_cms/1', 'ToWS_hw_xcdot/1', 'autorouting', 'smart');

        fprintf('  QUARC blocks added.\n');
    catch ME
        fprintf('  Warning: QUARC block error: %s\n', ME.message);
    end
else
    fprintf('  QUARC not detected (simulation-only mode).\n');
    fprintf('  Re-run on lab PC to add hardware blocks.\n');
end

save_system(mdl_freq, fullfile(pwd, [mdl_freq '.slx']));
fprintf('  Model saved: %s.slx\n', mdl_freq);

% Print hardware test instructions
fprintf('\n========================================\n');
fprintf(' HARDWARE TEST INSTRUCTIONS\n');
fprintf('========================================\n');
fprintf(' On the lab PC (with QUARC):\n');
fprintf('   1. Run seesaw_params.m\n');
fprintf('   2. Run frequency_test.m (builds the model)\n');
fprintf('   3. Open IP02_FreqTest.slx\n');
fprintf('   4. QUARC > Build (Ctrl+B)\n');
fprintf('   5. Set SimulationMode = External\n');
fprintf('   6. QUARC > Connect (Ctrl+T)\n');
fprintf('   7. QUARC > Start\n');
fprintf('   8. Wait %.0f seconds for chirp to complete\n', chirp_duration);
fprintf('   9. QUARC > Stop\n');
fprintf('  10. Run Section 5 of this script to compare\n');
fprintf('\n  SAFETY: Chirp amplitude = %.1f V (cart stays within limits)\n', A_chirp);
fprintf('  Total test duration: %.0f seconds\n', chirp_duration);

%% =====================================================================
%  SECTION 5: Process Hardware Data & Compare
%  =====================================================================
%  Run this AFTER collecting hardware data via QUARC.
%  Expects workspace variables: freq_Vcmd, freq_hw_xc, freq_hw_xcdot
%  (saved by To Workspace blocks during QUARC run)

fprintf('\n--- Processing Hardware Data ---\n');

% Check if hardware data exists (try Workspace first, then Dataset)
if ~exist('freq_hw_xc', 'var')
    % Check if it's buried in 'logsout' (Standard for modern Simulink logging)
    if exist('logsout', 'var')
        try
            freq_hw_xc = logsout.get('freq_hw_xc').Values;
            if exist('freq_hw_xcdot', 'var') == 0
                try freq_hw_xcdot = logsout.get('freq_hw_xcdot').Values; catch, end
            end
            if exist('freq_Vcmd', 'var') == 0
                try freq_Vcmd = logsout.get('freq_Vcmd').Values; catch, end
            end
            fprintf('  Data successfully extracted from "logsout" dataset.\n');
        catch
            fprintf('  Could not find signals in "logsout".\n');
        end
    end
end

if ~exist('freq_hw_xc', 'var')
    fprintf('  No hardware data found (freq_hw_xc not in workspace or logsout).\n');
    fprintf('  Run the QUARC chirp test first, then re-run this section.\n');
    fprintf('  Showing simulation chirp response instead...\n');

    % Run simulation chirp response for preview
    fprintf('  Running simulation with chirp input...\n');
    sim_chirp_fn = @(t) A_chirp * sin(2*pi * f_chirp_start * chirp_duration / log_ratio ...
        * ((f_chirp_end/f_chirp_start).^(t/chirp_duration) - 1));

    ode_fn = @(t, x) cart_ode_freq(t, x, sim_chirp_fn, p_ode);
    [t_chirp_sim, x_chirp_sim] = ode45(ode_fn, [0 chirp_duration], [0; 0], opts);

    % Compute frequency response from chirp via FFT
    % Resample to uniform time grid
    t_uniform = (0:Ts_quarc:chirp_duration)';
    xc_uniform = interp1(t_chirp_sim, x_chirp_sim(:,1), t_uniform);
    xcdot_uniform = interp1(t_chirp_sim, x_chirp_sim(:,2), t_uniform);
    Vcmd_uniform = arrayfun(sim_chirp_fn, t_uniform);

    [sim_chirp_freq, sim_chirp_xc_H, sim_chirp_xcdot_H] = ...
        compute_frf(t_uniform, Vcmd_uniform, xc_uniform, xcdot_uniform, Ts_quarc);

    % Plot chirp Bode overlay
    figure('Name', 'Bode Comparison: Analytical + Sim Sweep + Sim Chirp', ...
        'Position', [150 100 1000 700]);

    % Position magnitude
    subplot(2,2,1);
    semilogx(freq_range, G_xc_dB, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(test_freqs, sim_xc_dB, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    semilogx(sim_chirp_freq, 20*log10(abs(sim_chirp_xc_H)*100), 'g-', 'LineWidth', 1);
    xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
    title('V_{cmd} \rightarrow x_c [cm/V]');
    legend('Analytical', 'Sim (sine)', 'Sim (chirp)', 'Location', 'best');
    grid on; xlim([0.1 30]);

    % Position phase
    subplot(2,2,3);
    semilogx(freq_range, G_xc_phase, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(test_freqs, sim_xc_phase, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    semilogx(sim_chirp_freq, angle(sim_chirp_xc_H)*180/pi, 'g-', 'LineWidth', 1);
    xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
    grid on; xlim([0.1 30]); ylim([-270 0]);

    % Velocity magnitude
    subplot(2,2,2);
    semilogx(freq_range, G_xcdot_dB, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(test_freqs, sim_xcdot_dB, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    semilogx(sim_chirp_freq, 20*log10(abs(sim_chirp_xcdot_H)*100), 'g-', 'LineWidth', 1);
    xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
    title('V_{cmd} \rightarrow \dot{x}_c [(cm/s)/V]');
    legend('Analytical', 'Sim (sine)', 'Sim (chirp)', 'Location', 'best');
    grid on; xlim([0.1 30]);

    % Velocity phase
    subplot(2,2,4);
    semilogx(freq_range, G_xcdot_phase, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(test_freqs, sim_xcdot_phase, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    semilogx(sim_chirp_freq, angle(sim_chirp_xcdot_H)*180/pi, 'g-', 'LineWidth', 1);
    xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
    grid on; xlim([0.1 30]); ylim([-180 0]);

    sgtitle('Model Frequency Response (no hardware data yet)', ...
        'FontSize', 14, 'FontWeight', 'bold');

else
    fprintf('  Hardware data found! Processing...\n');

    % Extract timeseries data
    t_hw = freq_hw_xc.Time;
    xc_hw = freq_hw_xc.Data / 100;        % cm -> m
    Vcmd_hw = freq_Vcmd.Data;

    % Velocity: use hardware derivative if available, else compute
    if exist('freq_hw_xcdot', 'var')
        xcdot_hw = freq_hw_xcdot.Data / 100;  % cm/s -> m/s
    else
        xcdot_hw = gradient(xc_hw, t_hw);
    end

    % Also get sim data from same run
    t_sim_hw = freq_sim_xc.Time;
    xc_sim_hw = freq_sim_xc.Data / 100;    % cm -> m
    xcdot_sim_hw = freq_sim_xcdot.Data / 100;

    % Compute FRF for both
    dt_hw = mean(diff(t_hw));
    [hw_freq, hw_xc_H, hw_xcdot_H] = compute_frf(t_hw, Vcmd_hw, xc_hw, xcdot_hw, dt_hw);

    dt_sim = mean(diff(t_sim_hw));
    [sim_hw_freq, sim_hw_xc_H, sim_hw_xcdot_H] = ...
        compute_frf(t_sim_hw, Vcmd_hw, xc_sim_hw, xcdot_sim_hw, dt_sim);

    % Plot: Analytical + Sim sweep + Hardware
    figure('Name', 'VALIDATION: Model vs Hardware Bode', ...
        'Position', [100 50 1100 800]);

    % Position magnitude
    subplot(2,2,1);
    semilogx(freq_range, G_xc_dB, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(test_freqs, sim_xc_dB, 'bo', 'MarkerSize', 6);
    semilogx(hw_freq, 20*log10(abs(hw_xc_H)*100), 'r-', 'LineWidth', 1.5);
    semilogx(sim_hw_freq, 20*log10(abs(sim_hw_xc_H)*100), 'g--', 'LineWidth', 1);
    xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
    title('V_{cmd} \rightarrow x_c [cm/V]');
    legend('Analytical', 'Sim (sine sweep)', 'HARDWARE', 'Sim (chirp)', 'Location', 'best');
    grid on; xlim([0.1 25]);

    % Position phase
    subplot(2,2,3);
    semilogx(freq_range, G_xc_phase, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(test_freqs, sim_xc_phase, 'bo', 'MarkerSize', 6);
    semilogx(hw_freq, angle(hw_xc_H)*180/pi, 'r-', 'LineWidth', 1.5);
    semilogx(sim_hw_freq, angle(sim_hw_xc_H)*180/pi, 'g--', 'LineWidth', 1);
    xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
    grid on; xlim([0.1 25]); ylim([-270 0]);

    % Velocity magnitude
    subplot(2,2,2);
    semilogx(freq_range, G_xcdot_dB, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(test_freqs, sim_xcdot_dB, 'bo', 'MarkerSize', 6);
    semilogx(hw_freq, 20*log10(abs(hw_xcdot_H)*100), 'r-', 'LineWidth', 1.5);
    semilogx(sim_hw_freq, 20*log10(abs(sim_hw_xcdot_H)*100), 'g--', 'LineWidth', 1);
    xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
    title('V_{cmd} \rightarrow \dot{x}_c [(cm/s)/V]');
    legend('Analytical', 'Sim (sine sweep)', 'HARDWARE', 'Sim (chirp)', 'Location', 'best');
    grid on; xlim([0.1 25]);

    % Velocity phase
    subplot(2,2,4);
    semilogx(freq_range, G_xcdot_phase, 'b-', 'LineWidth', 1.5); hold on;
    semilogx(test_freqs, sim_xcdot_phase, 'bo', 'MarkerSize', 6);
    semilogx(hw_freq, angle(hw_xcdot_H)*180/pi, 'r-', 'LineWidth', 1.5);
    semilogx(sim_hw_freq, angle(sim_hw_xcdot_H)*180/pi, 'g--', 'LineWidth', 1);
    xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
    grid on; xlim([0.1 25]); ylim([-180 0]);

    sgtitle('VALIDATION: Model vs Hardware Frequency Response', ...
        'FontSize', 14, 'FontWeight', 'bold');

    % Time-domain comparison plot
    figure('Name', 'Chirp Time Response: Sim vs HW', 'Position', [150 100 1000 500]);

    subplot(3,1,1);
    plot(t_hw, Vcmd_hw, 'b-', 'LineWidth', 0.8);
    ylabel('V_{cmd} [V]'); title('Chirp Input'); grid on;

    subplot(3,1,2);
    plot(t_sim_hw, xc_sim_hw*100, 'b-', 'LineWidth', 1); hold on;
    plot(t_hw, xc_hw*100, 'r-', 'LineWidth', 1);
    ylabel('x_c [cm]'); title('Cart Position');
    legend('Model', 'Hardware'); grid on;

    subplot(3,1,3);
    plot(t_sim_hw, xcdot_sim_hw*100, 'b-', 'LineWidth', 1); hold on;
    plot(t_hw, xcdot_hw*100, 'r-', 'LineWidth', 1);
    ylabel('\dot{x}_c [cm/s]'); xlabel('Time [s]');
    title('Cart Velocity'); legend('Model', 'Hardware'); grid on;

    sgtitle('Chirp Test: Time Domain Comparison', 'FontSize', 14, 'FontWeight', 'bold');

    % Quantify model error
    % Resample to common time vector
    t_common = t_hw;
    xc_sim_interp = interp1(t_sim_hw, xc_sim_hw, t_common, 'linear', 0);
    err = xc_hw - xc_sim_interp;
    rmse = sqrt(mean(err.^2)) * 100;  % cm
    nrmse = rmse / (max(abs(xc_hw))*100) * 100;  % % of peak

    fprintf('\n========================================\n');
    fprintf(' VALIDATION RESULTS\n');
    fprintf('========================================\n');
    fprintf('  Position RMSE:     %.3f cm\n', rmse);
    fprintf('  Normalized RMSE:   %.1f%% of peak\n', nrmse);
    fprintf('\n');
    fprintf('  If model gain is too HIGH: increase B_eq (more friction)\n');
    fprintf('  If model gain is too LOW:  decrease B_eq (less friction)\n');
    fprintf('  If phase is wrong:         check eta_g, eta_m values\n');
    fprintf('  Current B_eq = %.2f N*s/m\n', B_eq);
end

%% =====================================================================
%  SECTION 6: Auto-Tune B_eq (and optionally eta_g) from Hardware Data
%  =====================================================================
%  Uses fminsearch to find the B_eq that minimizes time-domain RMSE
%  between model and hardware cart position.
%
%  The model is LINEAR (L_m = 0), so lsim is used for speed (~1 second).
%
%  Tunable parameters:
%    B_eq  : mechanical friction at cart [N*s/m]  (always tuned)
%    eta_g : gearbox efficiency [-]               (optional, set flag below)
%
%  After tuning, the script:
%    - Shows before/after time-domain comparison
%    - Shows before/after Bode comparison
%    - Prints the optimal values
%    - Offers to update seesaw_params.m
%  =====================================================================

% --- Configuration ---
tune_eta_g = false;   % Set true to also tune gearbox efficiency
t_skip = 2.0;         % Skip first N seconds (transient settling) [s]

fprintf('\n========================================\n');
fprintf(' AUTO-TUNE: Fitting Model to Hardware\n');
fprintf('========================================\n');

if ~exist('freq_hw_xc', 'var')
    fprintf('  No hardware data (freq_hw_xc) in workspace.\n');
    fprintf('  Run the QUARC chirp test first.\n');
    fprintf('  Skipping auto-tune.\n');
else
    % Extract hardware data
    t_hw_raw = freq_hw_xc.Time;
    xc_hw_raw = freq_hw_xc.Data / 100;   % cm -> m
    Vcmd_hw_raw = freq_Vcmd.Data;

    % Ensure uniform time step (should be from QUARC fixed-step)
    dt_tune = mean(diff(t_hw_raw));
    t_tune = (0:dt_tune:t_hw_raw(end))';
    xc_hw_tune = interp1(t_hw_raw, xc_hw_raw, t_tune, 'linear', 0);
    Vcmd_tune = interp1(t_hw_raw, Vcmd_hw_raw, t_tune, 'linear', 0);

    % Saturate input (same as model does)
    V_m_tune = max(-V_sat, min(V_sat, K_a * Vcmd_tune));

    % Skip initial transient
    idx_fit = t_tune >= t_skip;
    t_fit = t_tune(idx_fit);
    xc_hw_fit = xc_hw_tune(idx_fit);

    % Fixed parameters for cost function
    fp = struct('K_a',K_a, 'V_sat',V_sat, 'R_m',R_m, 'k_t',k_t, 'k_m',k_m, ...
        'eta_m',eta_m, 'eta_g',eta_g, 'K_g',K_g, 'r_mp',r_mp, 'M_c',M_c);

    % --- Cost function (defined as local function at bottom of file) ---
    % tune_cost(params_vec, V_m_in, t_in, xc_hw_in, idx_fit_in, fp_in, flag)

    % --- Initial guess ---
    if tune_eta_g
        x0_tune = [B_eq, eta_g];
        param_names = {'B_eq', 'eta_g'};
        fprintf('  Tuning: B_eq and eta_g\n');
    else
        x0_tune = B_eq;
        param_names = {'B_eq'};
        fprintf('  Tuning: B_eq only (eta_g = %.2f fixed)\n', eta_g);
    end

    % --- Before-tune RMSE ---
    cost_before = tune_cost(x0_tune, V_m_tune, t_tune, xc_hw_fit, idx_fit, fp, tune_eta_g);
    fprintf('  Initial B_eq = %.2f N*s/m\n', B_eq);
    fprintf('  Initial RMSE = %.4f cm\n', cost_before * 100);

    % --- Run fminsearch ---
    fms_opts = optimset('Display', 'iter', 'TolX', 1e-4, 'TolFun', 1e-6, 'MaxIter', 200);
    cost_fn = @(p) tune_cost(p, V_m_tune, t_tune, xc_hw_fit, idx_fit, fp, tune_eta_g);

    fprintf('\n  Running fminsearch...\n');
    [x_opt, cost_opt] = fminsearch(cost_fn, x0_tune, fms_opts);

    B_eq_opt = x_opt(1);
    if tune_eta_g
        eta_g_opt = x_opt(2);
    else
        eta_g_opt = eta_g;
    end

    % --- After-tune results ---
    fprintf('\n========================================\n');
    fprintf(' AUTO-TUNE RESULTS\n');
    fprintf('========================================\n');
    fprintf('  B_eq:   %.2f -> %.4f N*s/m\n', B_eq, B_eq_opt);
    if tune_eta_g
        fprintf('  eta_g:  %.2f -> %.4f\n', eta_g, eta_g_opt);
    end
    fprintf('  RMSE:   %.4f -> %.4f cm  (%.1f%% improvement)\n', ...
        cost_before*100, cost_opt*100, (1 - cost_opt/cost_before)*100);

    % --- Simulate before and after for plotting ---
    % Before (original params)
    af_before = alpha_f;
    B_total_before = B_eq + B_emf;
    sys_before = ss([0,1;0,-B_total_before/M_c], [0; af_before*eta_m/M_c], [1,0], 0);
    xc_sim_before = lsim(sys_before, V_m_tune, t_tune);

    % After (optimized params)
    af_after = (eta_g_opt * K_g * k_t) / (R_m * r_mp);
    B_emf_after = af_after * K_g * k_m / r_mp;
    B_total_after = B_eq_opt + B_emf_after;
    sys_after = ss([0,1;0,-B_total_after/M_c], [0; af_after*eta_m/M_c], [1,0], 0);
    xc_sim_after = lsim(sys_after, V_m_tune, t_tune);

    % --- Time-domain comparison: Before/After ---
    figure('Name', 'Auto-Tune: Time Domain', 'Position', [100 80 1000 600]);

    subplot(3,1,1);
    plot(t_tune, Vcmd_tune, 'b-', 'LineWidth', 0.8);
    ylabel('V_{cmd} [V]'); title('Chirp Input'); grid on;
    xline(t_skip, 'k--', 'Fit start');

    subplot(3,1,2);
    plot(t_tune, xc_hw_tune*100, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_tune, xc_sim_before*100, 'b--', 'LineWidth', 1);
    plot(t_tune, xc_sim_after*100, 'r-', 'LineWidth', 1);
    ylabel('x_c [cm]'); title('Cart Position');
    legend('Hardware', sprintf('Before (B_{eq}=%.1f)', B_eq), ...
        sprintf('After (B_{eq}=%.2f)', B_eq_opt), 'Location', 'best');
    grid on; xline(t_skip, 'k--');

    subplot(3,1,3);
    plot(t_tune, (xc_hw_tune - xc_sim_before)*100, 'b--', 'LineWidth', 1); hold on;
    plot(t_tune, (xc_hw_tune - xc_sim_after)*100, 'r-', 'LineWidth', 1);
    ylabel('Error [cm]'); xlabel('Time [s]');
    title('Model Error (Hardware - Model)');
    legend(sprintf('Before: RMSE=%.3fcm', cost_before*100), ...
        sprintf('After: RMSE=%.3fcm', cost_opt*100), 'Location', 'best');
    grid on; xline(t_skip, 'k--');

    sgtitle('Auto-Tune: Time Domain Fit', 'FontSize', 14, 'FontWeight', 'bold');

    % --- Bode comparison: Before/After vs Hardware ---
    figure('Name', 'Auto-Tune: Bode Comparison', 'Position', [150 60 1000 700]);

    % Analytical Bode for BEFORE params
    G_xc_dB_before = zeros(size(omega_range));
    G_xc_phase_before = zeros(size(omega_range));
    for kk = 1:length(omega_range)
        jw = 1j * omega_range(kk);
        H = af_before * eta_m / (M_c * jw^2 + B_total_before * jw);
        G_xc_dB_before(kk) = 20*log10(abs(H)*100);
        G_xc_phase_before(kk) = angle(H) * 180/pi;
    end

    % Analytical Bode for AFTER params
    G_xc_dB_after = zeros(size(omega_range));
    G_xc_phase_after = zeros(size(omega_range));
    G_xcdot_dB_after = zeros(size(omega_range));
    G_xcdot_phase_after = zeros(size(omega_range));
    for kk = 1:length(omega_range)
        jw = 1j * omega_range(kk);
        H = af_after * eta_m / (M_c * jw^2 + B_total_after * jw);
        G_xc_dB_after(kk) = 20*log10(abs(H)*100);
        G_xc_phase_after(kk) = angle(H) * 180/pi;
        Hv = af_after * eta_m / (M_c * jw + B_total_after);
        G_xcdot_dB_after(kk) = 20*log10(abs(Hv)*100);
        G_xcdot_phase_after(kk) = angle(Hv) * 180/pi;
    end

    % Hardware FRF (recompute if not already done)
    if ~exist('hw_freq', 'var')
        xcdot_hw_raw = gradient(xc_hw_raw, t_hw_raw);
        [hw_freq, hw_xc_H, hw_xcdot_H] = ...
            compute_frf(t_hw_raw, Vcmd_hw_raw, xc_hw_raw, xcdot_hw_raw, dt_tune);
    end

    % Position Bode
    subplot(2,2,1);
    semilogx(freq_range, G_xc_dB_before, 'b--', 'LineWidth', 1); hold on;
    semilogx(freq_range, G_xc_dB_after, 'r-', 'LineWidth', 2);
    semilogx(hw_freq, 20*log10(abs(hw_xc_H)*100), 'k-', 'LineWidth', 1.2);
    xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
    title('V_{cmd} \rightarrow x_c [cm/V]');
    legend(sprintf('Before (B_{eq}=%.1f)', B_eq), ...
        sprintf('After (B_{eq}=%.2f)', B_eq_opt), 'Hardware', 'Location', 'best');
    grid on; xlim([0.1 25]);

    subplot(2,2,3);
    semilogx(freq_range, G_xc_phase_before, 'b--', 'LineWidth', 1); hold on;
    semilogx(freq_range, G_xc_phase_after, 'r-', 'LineWidth', 2);
    semilogx(hw_freq, angle(hw_xc_H)*180/pi, 'k-', 'LineWidth', 1.2);
    xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
    grid on; xlim([0.1 25]); ylim([-270 0]);

    % Velocity Bode
    subplot(2,2,2);
    semilogx(freq_range, G_xcdot_dB, 'b--', 'LineWidth', 1); hold on;
    semilogx(freq_range, G_xcdot_dB_after, 'r-', 'LineWidth', 2);
    semilogx(hw_freq, 20*log10(abs(hw_xcdot_H)*100), 'k-', 'LineWidth', 1.2);
    xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]');
    title('V_{cmd} \rightarrow \dot{x}_c [(cm/s)/V]');
    legend(sprintf('Before (B_{eq}=%.1f)', B_eq), ...
        sprintf('After (B_{eq}=%.2f)', B_eq_opt), 'Hardware', 'Location', 'best');
    grid on; xlim([0.1 25]);

    subplot(2,2,4);
    semilogx(freq_range, G_xcdot_phase, 'b--', 'LineWidth', 1); hold on;
    semilogx(freq_range, G_xcdot_phase_after, 'r-', 'LineWidth', 2);
    semilogx(hw_freq, angle(hw_xcdot_H)*180/pi, 'k-', 'LineWidth', 1.2);
    xlabel('Frequency [Hz]'); ylabel('Phase [deg]');
    grid on; xlim([0.1 25]); ylim([-180 0]);

    sgtitle('Auto-Tune: Bode Before/After vs Hardware', ...
        'FontSize', 14, 'FontWeight', 'bold');

    % --- Save tuned values to workspace ---
    assignin('base', 'B_eq_tuned', B_eq_opt);
    if tune_eta_g
        assignin('base', 'eta_g_tuned', eta_g_opt);
    end

    % --- Print update instructions ---
    fprintf('\n  To apply the tuned values, update seesaw_params.m:\n');
    fprintf('    B_eq = %.4f;     %% Auto-tuned from chirp test\n', B_eq_opt);
    if tune_eta_g
        fprintf('    eta_g = %.4f;    %% Auto-tuned from chirp test\n', eta_g_opt);
    end
    fprintf('\n  Values saved to workspace: B_eq_tuned');
    if tune_eta_g
        fprintf(', eta_g_tuned');
    end
    fprintf('\n');

    % --- Sanity checks ---
    if B_eq_opt < 0
        fprintf('\n  WARNING: B_eq < 0 is unphysical! Check hardware data quality.\n');
    elseif B_eq_opt > 50
        fprintf('\n  WARNING: B_eq > 50 is unusually high. Check for mechanical issues.\n');
    end
    if tune_eta_g && (eta_g_opt < 0.5 || eta_g_opt > 1.0)
        fprintf('  WARNING: eta_g = %.2f is outside expected range [0.5, 1.0].\n', eta_g_opt);
    end
end

%% =====================================================================
%  NOTE: Phase 2 (seesaw) is open-loop UNSTABLE — cannot frequency test
%  without a controller. Validate Phase 1 first, then design LQR
%  (Good ref Section 2.2) before testing the seesaw on hardware.
%  =====================================================================

fprintf('\nFrequency test script complete.\n');

%% =====================================================================
%  LOCAL FUNCTIONS
%  =====================================================================

function dxdt = cart_ode_freq(t, x, V_cmd_fn, p)
    x_c = x(1); x_c_dot = x(2);
    V_cmd = V_cmd_fn(t);
    V_m = max(-p.V_sat, min(p.V_sat, p.K_a * V_cmd));
    F_c = (p.eta_g * p.K_g * p.k_t) / (p.R_m * p.r_mp) ...
        * (-p.K_g * p.k_m * x_c_dot / p.r_mp + p.eta_m * V_m);
    x_c_ddot = (F_c - p.B_eq * x_c_dot) / p.M_c;
    if (x_c >= p.x_c_max && x_c_dot > 0)
        x_c_ddot = -5000*(x_c - p.x_c_max) - 50*x_c_dot;
    elseif (x_c <= -p.x_c_max && x_c_dot < 0)
        x_c_ddot = -5000*(x_c + p.x_c_max) - 50*x_c_dot;
    end
    dxdt = [x_c_dot; x_c_ddot];
end

function [amp, phase_deg] = extract_freq_response(t, y, f)
    % Extract amplitude and phase of signal y at frequency f using FFT
    N = length(y);
    dt = (t(end) - t(1)) / (N-1);
    Y = fft(y - mean(y));  % remove DC offset
    freqs = (0:N-1) / (N * dt);

    % Find bin closest to test frequency
    [~, k] = min(abs(freqs - f));

    amp = 2 * abs(Y(k)) / N;
    phase_deg = angle(Y(k)) * 180/pi;
end

function [freq_out, H_xc, H_xcdot] = compute_frf(t, u, xc, xcdot, dt)
    % Compute frequency response function from time-domain chirp data
    % Uses cross-spectral density method: H(f) = Syu(f) / Suu(f)

    N = length(t);
    Fs = 1/dt;

    % Window parameters for Welch method (manual implementation)
    n_seg = 2048;                     % segment length
    n_overlap = round(n_seg * 0.5);   % 50% overlap
    win = hanning(n_seg);

    % Number of segments
    n_step = n_seg - n_overlap;
    n_segs = floor((N - n_overlap) / n_step);

    % Frequency vector
    freq_fft = (0:n_seg/2) * Fs / n_seg;

    % Accumulate power spectral densities
    Suu = zeros(n_seg/2+1, 1);
    Syu_xc = zeros(n_seg/2+1, 1);
    Syu_xcdot = zeros(n_seg/2+1, 1);

    for k = 0:n_segs-1
        idx = k*n_step + (1:n_seg);
        if max(idx) > N, break; end

        u_seg = (u(idx) - mean(u(idx))) .* win;
        xc_seg = (xc(idx) - mean(xc(idx))) .* win;
        xcdot_seg = (xcdot(idx) - mean(xcdot(idx))) .* win;

        U = fft(u_seg);
        Xc = fft(xc_seg);
        Xcdot = fft(xcdot_seg);

        U_half = U(1:n_seg/2+1);
        Xc_half = Xc(1:n_seg/2+1);
        Xcdot_half = Xcdot(1:n_seg/2+1);

        Suu = Suu + abs(U_half).^2;
        Syu_xc = Syu_xc + Xc_half .* conj(U_half);
        Syu_xcdot = Syu_xcdot + Xcdot_half .* conj(U_half);
    end

    % Transfer function estimates
    H_xc_full = Syu_xc ./ Suu;
    H_xcdot_full = Syu_xcdot ./ Suu;

    % Only keep frequencies in the chirp range (0.1 to 20 Hz)
    valid = freq_fft >= 0.08 & freq_fft <= 25;
    freq_out = freq_fft(valid)';
    H_xc = H_xc_full(valid);
    H_xcdot = H_xcdot_full(valid);
end

function s = ternary_str(cond, yes, no)
    if cond, s = yes; else, s = no; end
end

function cost = tune_cost(params_vec, V_m_in, t_in, xc_hw_in, idx_fit_in, fp_in, tune_eta_g_flag)
    % Builds a linear SS model and returns RMSE vs hardware cart position.
    % Used by fminsearch in Section 6 (auto-tune B_eq / eta_g).
    B_eq_try = params_vec(1);
    if tune_eta_g_flag
        eta_g_try = params_vec(2);
    else
        eta_g_try = fp_in.eta_g;
    end

    % Recompute motor force constant and total damping
    af = (eta_g_try * fp_in.K_g * fp_in.k_t) / (fp_in.R_m * fp_in.r_mp);
    B_emf_try = af * fp_in.K_g * fp_in.k_m / fp_in.r_mp;
    B_total_try = B_eq_try + B_emf_try;

    % State-space model: V_m -> x_c
    A_ss = [0, 1; 0, -B_total_try / fp_in.M_c];
    B_ss = [0; af * fp_in.eta_m / fp_in.M_c];
    C_ss = [1, 0];
    D_ss = 0;

    sys_try = ss(A_ss, B_ss, C_ss, D_ss);
    xc_sim_try = lsim(sys_try, V_m_in, t_in);

    % RMSE on fitting window only
    cost = sqrt(mean((xc_sim_try(idx_fit_in) - xc_hw_in).^2));
end
