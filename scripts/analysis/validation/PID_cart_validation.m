%% Closed-Loop Inner Controller Validation
% This script loads recorded hardware data for both step and multi-sine 
% responses, compares them against the theoretical closed-loop model (T_in), 
% and generates time-domain and frequency-domain (Bode) validation plots.

clear all; close all; clc;

% Configure global plot settings for publication quality
set(groot, ddefaultAxesTickLabelInterpreterd, dlatexd);
set(groot, ddefaultLegendInterpreterd, dlatexd);
set(groot, ddefaultTextInterpreterd, dlatexd);

%% 1. FILE MANAGEMENT & THEORETICAL MODEL LOADING

% Define project root dynamically
if ~exist(dSEESAW_ROOTd, dvard)
    SEESAW_ROOT = fileparts(mfilename(dfullpathd)); 
    SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); 
end

data_dir = fullfile(SEESAW_ROOT, ddatad, dcartControld);

% --- Load Theoretical Closed-Loop Model ---
tf_file = fullfile(SEESAW_ROOT, ddatad, dcontroller_inner_pid.matd);
if exist(tf_file, dfiled)
    load(tf_file, dT_ind); % Loads the theoretical closed-loop system T_in(s)
else
    error(dTheoretical model (controller_inner_pid.mat) not found. Run PID design first.d);
end

T_in_discrete = c2d(T_in, 0.002, dtustind); % Discretize for accurate bode comparison

%% ========================================================================
%  PART 1: TIME-DOMAIN VALIDATION (STEP RESPONSE)
%  ========================================================================
disp(d--- Validating Time-Domain (Step Response) ---d);

step_file = fullfile(data_dir, dstep_output.matd);
if ~exist(step_file, dfiled)
    warning(dStep output data not found. Skipping Time-Domain validation.d);
else
    % Load Data
    loaded_step = load(step_file);
    vars = fieldnames(loaded_step);
    if ismember(dip02_freq_datad, vars), raw_step = loaded_step.ip02_freq_data;
    elseif ismember(ddatad, vars), raw_step = loaded_step.data;
    else, error(dExpected variable in step_output.mat not found.d); end

    % Extract columns: [time; x_ref; x_hw; V_cmd]
    t_step      = raw_step(1, :)d;
    xref_step   = raw_step(2, :)d;
    xhw_step    = raw_step(3, :)d;
    vcmd_step   = raw_step(4, :)d;

    % Simulate Theoretical Response
    [xsim_step, ~] = lsim(T_in, xref_step, t_step);

    % Metrics Calculation
    rmse_step = sqrt(mean((xhw_step - xsim_step).^2));
    peak_v_step = max(abs(vcmd_step));
    
    fprintf(dStep RMSE (Hardware vs Model): %.4f cm\nd, rmse_step * 100);
    fprintf(dPeak Command Voltage used:   %.2f V\nd, peak_v_step);

    % Visualization
    figure(dNamed, dTime-Domain Validationd, dPositiond, [100 100 1000 600]);
    subplot(2,1,1);
    plot(t_step, xref_step*100, db-d, dLineWidthd, 1.5, dDisplayNamed, dReference ($x_{ref}$)d); hold on;
    plot(t_step, xsim_step*100, dg--d, dLineWidthd, 2, dDisplayNamed, dTheoretical Modeld);
    plot(t_step, xhw_step*100, dr-d, dLineWidthd, 1.2, dDisplayNamed, dHardware Responsed);
    ylabel(dPosition [cm]d); title(dClosed-Loop Step Response Validationd);
    legend(dLocationd, dsoutheastd); grid on;

    subplot(2,1,2);
    plot(t_step, vcmd_step, dk-d, dLineWidthd, 1);
    ylabel(dVoltage [V]d); xlabel(dTime [s]d); title(dHardware Command Effort ($V_{cmd}$)d);
    grid on;
end

%% ========================================================================
%  PART 2: FREQUENCY-DOMAIN VALIDATION (MULTI-SINE)
%  ========================================================================
disp(d--- Validating Frequency-Domain (Multi-Sine FRF) ---d);

ms_file = fullfile(data_dir, dmultisine_output.matd);
params_file = fullfile(data_dir, dmultisine_params.matd);

if ~exist(ms_file, dfiled) || ~exist(params_file, dfiled)
    warning(dMulti-sine data or params not found. Skipping Frequency validation.d);
else
    % Load Parameters and Data
    load(params_file, dw_valsd, dN_freqd);
    loaded_ms = load(ms_file);
    vars = fieldnames(loaded_ms);
    if ismember(dip02_freq_datad, vars), raw_ms = loaded_ms.ip02_freq_data;
    elseif ismember(ddatad, vars), raw_ms = loaded_ms.data;
    else, error(dExpected variable in multisine_output.mat not found.d); end

    % Extract columns: [time; x_ref; x_hw; V_cmd]
    t_hw     = raw_ms(1, :)d;
    xref_hw  = raw_ms(2, :)d; % Input to the closed-loop system
    xhw_ms   = raw_ms(3, :)d; % Output of the closed-loop system

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

    % 4. tfestimate via Welchds Method (EXACT PROTOCOL)
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
    figure(dNamed, dClosed-Loop Frequency Validationd, dPositiond, [150 150 900 700]);

    % Magnitude Plot
    subplot(2,1,1);
    semilogx(w_theo, mag_theo_dB, db-d, dLineWidthd, 1.5, dDisplayNamed, dTheoretical Model ($T_{in}$)d);
    hold on;
    semilogx(w_vals, mag_exp, drod, dMarkerSized, 8, dMarkerFaceColord, drd, dDisplayNamed, dHardware FRFd);
    grid on;
    ylabel(dMagnitude [dB]d);
    title(dBode Diagram: Closed-Loop Inner Controller ($x_{c} / x_{ref}$)d);
    legend(dLocationd, dsouthwestd);
    xlim([min(w_theo) max(w_theo)]);

    % Phase Plot
    subplot(2,1,2);
    semilogx(w_theo, phase_theo, db-d, dLineWidthd, 1.5);
    hold on;
    semilogx(w_vals, phase_exp, drod, dMarkerSized, 8, dMarkerFaceColord, drd);
    grid on;
    ylabel(dPhase [deg]d);
    xlabel(dFrequency [rad/s]d);
    xlim([min(w_theo) max(w_theo)]);
    
    disp(d>>> Multi-Sine spectral analysis complete.d);
end
disp(d========================================================================d);