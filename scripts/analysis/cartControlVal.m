%% Closed-Loop Inner Controller Validation
% This script loads recorded hardware data for both step and multi-sine 
% responses, compares them against the theoretical closed-loop model (T_in), 
% and generates time-domain and frequency-domain (Bode) validation plots.

clear all; close all; clc;

% Configure global plot settings for publication quality
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. FILE MANAGEMENT & THEORETICAL MODEL LOADING

% Define project root dynamically
if ~exist('SEESAW_ROOT', 'var')
    SEESAW_ROOT = fileparts(mfilename('fullpath')); 
    SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); 
end

data_dir = fullfile(SEESAW_ROOT, 'data', 'cartControl');

% --- Load Theoretical Closed-Loop Model ---
tf_file = fullfile(SEESAW_ROOT, 'data', 'controller_inner_pid.mat');
if exist(tf_file, 'file')
    load(tf_file, 'T_in'); % Loads the theoretical closed-loop system T_in(s)
else
    error('Theoretical model (controller_inner_pid.mat) not found. Run PID design first.');
end

T_in_discrete = c2d(T_in, 0.002, 'tustin'); % Discretize for accurate bode comparison

%% ========================================================================
%  PART 1: TIME-DOMAIN VALIDATION (STEP RESPONSE)
%  ========================================================================
disp('--- Validating Time-Domain (Step Response) ---');

step_file = fullfile(data_dir, 'step_output.mat');
if ~exist(step_file, 'file')
    warning('Step output data not found. Skipping Time-Domain validation.');
else
    % Load Data
    loaded_step = load(step_file);
    vars = fieldnames(loaded_step);
    if ismember('ip02_freq_data', vars), raw_step = loaded_step.ip02_freq_data;
    elseif ismember('data', vars), raw_step = loaded_step.data;
    else, error('Expected variable in step_output.mat not found.'); end

    % Extract columns: [time; x_ref; x_hw; V_cmd]
    t_step      = raw_step(1, :)';
    xref_step   = raw_step(2, :)';
    xhw_step    = raw_step(3, :)';
    vcmd_step   = raw_step(4, :)';

    % Simulate Theoretical Response
    [xsim_step, ~] = lsim(T_in, xref_step, t_step);

    % Metrics Calculation
    rmse_step = sqrt(mean((xhw_step - xsim_step).^2));
    peak_v_step = max(abs(vcmd_step));
    
    fprintf('Step RMSE (Hardware vs Model): %.4f cm\n', rmse_step * 100);
    fprintf('Peak Command Voltage used:   %.2f V\n', peak_v_step);

    % Visualization
    figure('Name', 'Time-Domain Validation', 'Position', [100 100 1000 600]);
    subplot(2,1,1);
    plot(t_step, xref_step*100, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Reference ($x_{ref}$)'); hold on;
    plot(t_step, xsim_step*100, 'g--', 'LineWidth', 2, 'DisplayName', 'Theoretical Model');
    plot(t_step, xhw_step*100, 'r-', 'LineWidth', 1.2, 'DisplayName', 'Hardware Response');
    ylabel('Position [cm]'); title('Closed-Loop Step Response Validation');
    legend('Location', 'southeast'); grid on;

    subplot(2,1,2);
    plot(t_step, vcmd_step, 'k-', 'LineWidth', 1);
    ylabel('Voltage [V]'); xlabel('Time [s]'); title('Hardware Command Effort ($V_{cmd}$)');
    grid on;
end

%% ========================================================================
%  PART 2: FREQUENCY-DOMAIN VALIDATION (MULTI-SINE)
%  ========================================================================
disp('--- Validating Frequency-Domain (Multi-Sine FRF) ---');

ms_file = fullfile(data_dir, 'multisine_output.mat');
params_file = fullfile(data_dir, 'multisine_params.mat');

if ~exist(ms_file, 'file') || ~exist(params_file, 'file')
    warning('Multi-sine data or params not found. Skipping Frequency validation.');
else
    % Load Parameters and Data
    load(params_file, 'w_vals', 'N_freq');
    loaded_ms = load(ms_file);
    vars = fieldnames(loaded_ms);
    if ismember('ip02_freq_data', vars), raw_ms = loaded_ms.ip02_freq_data;
    elseif ismember('data', vars), raw_ms = loaded_ms.data;
    else, error('Expected variable in multisine_output.mat not found.'); end

    % Extract columns: [time; x_ref; x_hw; V_cmd]
    t_hw     = raw_ms(1, :)';
    xref_hw  = raw_ms(2, :)'; % Input to the closed-loop system
    xhw_ms   = raw_ms(3, :)'; % Output of the closed-loop system

    % 1. Apply Zero-Phase Hardware Noise Filter (EXACT PROTOCOL)
    dt = mean(diff(t_hw));
    Fs = 1/dt;
    cutoff = 50; % [Hz]
    [b, a] = butter(2, cutoff / (Fs/2));
    
    ref_clean = filtfilt(b, a, xref_hw);
    x_clean   = filtfilt(b, a, xhw_ms);

    % 2. Take Derivative (EXACT PROTOCOL)
    % Note: T(s) = (s*X_hw)/(s*X_ref) = X_hw/X_ref. Using velocities cleanly 
    % removes any DC sensor offsets without changing the transfer function.
    v_ref_clean = gradient(ref_clean) / dt;
    v_hw_clean  = gradient(x_clean) / dt;

    % 3. Trim startup transient and end (EXACT PROTOCOL)
    T_test = max(20, 3 * (2*pi/w_vals(1)));
    T_settle = 3.0; 
    valid_idx = find(t_hw >= T_settle & t_hw < T_test);
    
    v_ref_ss = detrend(v_ref_clean(valid_idx));
    v_hw_ss  = detrend(v_hw_clean(valid_idx));

    % 4. tfestimate via Welch's Method (EXACT PROTOCOL)
    f_target = w_vals / (2*pi);
    window = hanning(length(v_ref_ss));
    [Txy_CL, ~] = tfestimate(v_ref_ss, v_hw_ss, window, 0, f_target, Fs);

    % 5. Extract Magnitude and Phase
    % Because we mapped velocity to velocity, Txy_CL is already exactly T_in.
    % No need to divide by (jw) like we did for the open-loop Gx.
    mag_exp = 20 * log10(abs(Txy_CL));
    phase_exp = unwrap(angle(Txy_CL)) * 180 / pi;

    % Align phase to standard Bode convention
    if phase_exp(1) > 0
        phase_exp = phase_exp - 360;
    end

    % --- Generate Theoretical Bode Lines ---
    w_theo = logspace(log10(min(w_vals)*0.5), log10(max(w_vals)*1.5), 1000);
    [mag_theo, phase_theo, ~] = bode(T_in_discrete, w_theo);
    mag_theo = squeeze(mag_theo);
    phase_theo = squeeze(phase_theo);
    mag_theo_dB = 20 * log10(mag_theo);

    % --- Plotting ---
    figure('Name', 'Closed-Loop Frequency Validation', 'Position', [150 150 900 700]);

    % Magnitude Plot
    subplot(2,1,1);
    semilogx(w_theo, mag_theo_dB, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Theoretical Model ($T_{in}$)');
    hold on;
    semilogx(w_vals, mag_exp, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Hardware FRF');
    grid on;
    ylabel('Magnitude [dB]');
    title('Bode Diagram: Closed-Loop Inner Controller ($x_{c} / x_{ref}$)');
    legend('Location', 'southwest');
    xlim([min(w_theo) max(w_theo)]);

    % Phase Plot
    subplot(2,1,2);
    semilogx(w_theo, phase_theo, 'b-', 'LineWidth', 1.5);
    hold on;
    semilogx(w_vals, phase_exp, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    grid on;
    ylabel('Phase [deg]');
    xlabel('Frequency [rad/s]');
    xlim([min(w_theo) max(w_theo)]);
    
    disp('>>> Multi-Sine spectral analysis complete.');
end
disp('========================================================================');