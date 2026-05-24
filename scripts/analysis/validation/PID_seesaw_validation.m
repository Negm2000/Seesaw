%% Closed-Loop Cascade Controller Validation (Seesaw Outer Loop)
% This script loads recorded hardware data for both step and multi-sine 
% responses, compares them against the theoretical outer closed-loop model (T_theta), 
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

data_dir = fullfile(SEESAW_ROOT, 'data', 'seesawControl');

% --- Load Theoretical Closed-Loop Model ---
tf_file = fullfile(SEESAW_ROOT, 'data', 'controller_outer_pid.mat');
if exist(tf_file, 'file')
    load(tf_file, 'T_theta'); % Loads the theoretical cascade system T_theta(s)
else
    error('Theoretical model (controller_outer_pid.mat) not found. Run Outer PID design first.');
end

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

    % Extract columns (Adjust indices if your Simulink routing differs!)
    % Assuming: [time; theta_ref(rad); theta_hw(rad); x_cart(m); V_cmd]
    t_step        = raw_step(1, :)';
    th_ref_step   = raw_step(2, :)';
    th_hw_step    = raw_step(3, :)';
    
    if size(raw_step, 1) >= 4
        x_hw_step = raw_step(4, :)';
    else
        x_hw_step = zeros(size(t_step)); % Fallback
    end

    % Simulate Theoretical Response
    [th_sim_step, ~] = lsim(T_theta, th_ref_step, t_step);

    % Metrics Calculation (in degrees for readability)
    rmse_step_deg = sqrt(mean((rad2deg(th_hw_step) - rad2deg(th_sim_step)).^2));
    fprintf('Step RMSE (Hardware vs Model): %.4f degrees\n', rmse_step_deg);

    % Visualization
    figure('Name', 'Time-Domain Validation (Outer Loop)', 'Position', [100 100 1000 600]);
    
    subplot(2,1,1);
    plot(t_step, rad2deg(th_ref_step), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Reference ($\theta_{ref}$)'); hold on;
    plot(t_step, rad2deg(th_sim_step), 'g--', 'LineWidth', 2, 'DisplayName', 'Theoretical Model');
    plot(t_step, rad2deg(th_hw_step), 'r-', 'LineWidth', 1.2, 'DisplayName', 'Hardware Response');
    ylabel('Angle [deg]'); title('Cascade Control Step Response Validation');
    legend('Location', 'northeast'); grid on;

    subplot(2,1,2);
    plot(t_step, x_hw_step * 100, 'k-', 'LineWidth', 1);
    ylabel('Cart Position [cm]'); xlabel('Time [s]'); title('Cart Kinematic Trajectory ($x_c$)');
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

    % Extract columns
    t_hw        = raw_ms(1, :)';
    th_ref_hw   = raw_ms(2, :)'; % Input to the closed-loop system
    th_hw_ms    = raw_ms(3, :)'; % Output of the closed-loop system

    % 1. Apply Zero-Phase Hardware Noise Filter
    dt = mean(diff(t_hw));
    Fs = 1/dt;
    cutoff = 50; % [Hz]
    [b, a] = butter(2, cutoff / (Fs/2));
    
    ref_clean = filtfilt(b, a, th_ref_hw);
    th_clean  = filtfilt(b, a, th_hw_ms);

    % 2. Detrend instead of Derivative
    % Taking the derivative of a 3-tick signal creates infinite noise spikes.
    % Instead, we simply remove the static DC offset so tfestimate is centered.
    v_ref_clean = detrend(ref_clean, 'constant');
    v_hw_clean  = detrend(th_clean, 'constant');

    % 3. Trim startup transient
    T_test = max(20, 3 * (2*pi/w_vals(1)));
    T_settle = 3.0; 
    valid_idx = find(t_hw >= T_settle & t_hw < T_test);
    
    v_ref_ss = detrend(v_ref_clean(valid_idx));
    v_hw_ss  = detrend(v_hw_clean(valid_idx));

    % 4. tfestimate via Welch's Method
    f_target = w_vals / (2*pi);
    window = hanning(length(v_ref_ss));
    [Txy_CL, ~] = tfestimate(v_ref_ss, v_hw_ss, window, 0, f_target, Fs);

    % 5. Extract Magnitude and Phase
    mag_exp = 20 * log10(abs(Txy_CL));
    phase_exp = unwrap(angle(Txy_CL)) * 180 / pi;

    % --- Generate Theoretical Bode Lines ---
    w_theo = logspace(log10(min(w_vals)*0.5), log10(max(w_vals)*1.5), 1000);
    [mag_theo, phase_theo, ~] = bode(T_theta, w_theo);
    mag_theo = squeeze(mag_theo);
    phase_theo = squeeze(phase_theo);
    mag_theo_dB = 20 * log10(mag_theo);

    % --- DYNAMIC PHASE ALIGNMENT ---
    % Find the closest 360-degree difference between the start of both plots
    % and shift the experimental data up/down to match the theoretical branch.
    offset_laps = round((phase_theo(1) - phase_exp(1)) / 360);
    phase_exp = phase_exp + (offset_laps * 360);

    % --- Plotting ---
    figure('Name', 'Cascade Frequency Validation', 'Position', [150 150 900 700]);

    % Magnitude Plot
    subplot(2,1,1);
    semilogx(w_theo, mag_theo_dB, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Theoretical Model ($T_{\theta}$)');
    hold on;
    semilogx(w_vals, mag_exp, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Hardware FRF');
    grid on;
    ylabel('Magnitude [dB]');
    title('Bode Diagram: Closed-Loop Cascade Controller ($\theta / \theta_{ref}$)');
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