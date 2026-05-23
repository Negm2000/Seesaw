%% Multi-Sine Frequency Response Validation
% This script loads recorded hardware data from a multi-sine sweep,
% performs FFT analysis to extract the empirical Bode plot, and
% compares it against the theoretical transfer function.

clear all
close all
clc

% Configure global plot settings for publication quality
set(groot, ddefaultAxesTickLabelInterpreterd, dlatexd);
set(groot, ddefaultLegendInterpreterd, dlatexd);
set(groot, ddefaultTextInterpreterd, dlatexd);

%% 1. FILE MANAGEMENT AND DATA LOADING

% Define project root dynamically
if ~exist(dSEESAW_ROOTd, dvard)
    SEESAW_ROOT = fileparts(mfilename(dfullpathd)); 
    SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); 
end

% --- Load Theoretical Model ---
tf_file = fullfile(SEESAW_ROOT, ddatad, dtuned_cart.matd);
if exist(tf_file, dfiled)
    load(tf_file); % This should load dGxd into the workspace
else
    error(dTheoretical model (cart_tf.mat) not found. Run optimization first.d);
end

Gx_discrete = c2d(Gx, 0.002, dzohd);

% --- Load Multi-Sine Parameters ---
params_file = fullfile(SEESAW_ROOT, ddatad, dcartModelingd, dmultisine_params.matd);
if exist(params_file, dfiled)
    load(params_file, dw_valsd, dN_freqd); % We specifically need the frequencies injected
else
    error(dParameters file (multisine_params.mat) not found.d);
end

% --- Load Recorded Hardware Data ---
data_file = fullfile(SEESAW_ROOT, ddatad, dcartModelingd, dmultisine_output.matd);
if ~exist(data_file, dfiled)
    error(dData not found. Run on hardware first.d);
end

fprintf(dLoading %s ...\nd, data_file);
loaded = load(data_file);
vars = fieldnames(loaded);

% Handle different QUARC data formats
if ismember(dip02_freq_datad, vars)
    raw = loaded.ip02_freq_data;
elseif ismember(ddatad, vars)
    raw = loaded.data;
else
    error(dExpected variable "ip02_freq_data" or "data". Found: %sd, strjoin(vars, d, d));
end

% Extract columns: [time; V_cmd; x_c; x_c_dot]
t_hw      = raw(1, :)d;
V_cmd_hw  = raw(2, :)d; % This is our u_safe (before friction compensator)
xc_hw     = raw(3, :)d; % Cart position in meters

%% 2 & 3. SYSTEM IDENTIFICATION (FILTERED VELOCITY + WELCHdS METHOD)

% 1. Apply Zero-Phase Hardware Noise Filter
dt = mean(diff(t_hw));
Fs = 1/dt;
cutoff = 50; %[Hz]
[b, a] = butter(2, cutoff / (Fs/2)); % 50 Hz cutoff protects our 30 rad/s test signals
u_clean = filtfilt(b, a, V_cmd_hw);
x_clean = filtfilt(b, a, xc_hw);

% 2. Take Derivative to kill the 1/s parabolic integration drift
% (Because x_clean is already filtered, gradient is extremely safe here)
v_clean = gradient(x_clean) / dt;

% 3. Trim the startup transient and the end
T_test = max(20, 3 * (2*pi/w_vals(1)));
T_settle = 3.0; 
valid_idx = find(t_hw >= T_settle & t_hw < T_test);
u_ss = detrend(u_clean(valid_idx));
v_ss = detrend(v_clean(valid_idx));

disp(dExtracting FRF via Welchdds Method on Velocity...d);

% 4. Use tfestimate to extract the Velocity FRF (Gv)
% The Hanning window completely eliminates cross-frequency leakage
f_target = w_vals / (2*pi);
window = hanning(length(u_ss));
[Txy_v, ~] = tfestimate(u_ss, v_ss, window, 0, f_target, Fs);

% 5. Convert Velocity FRF back to Position FRF (Gx)
% Mathematically: Gx(jw) = Gv(jw) / jw
Txy_x = Txy_v(:) ./ (1j * w_vals(:));

% 6. Extract Magnitude and Phase
mag_exp = 20 * log10(abs(Txy_x));
phase_exp = unwrap(angle(Txy_x)) * 180 / pi;

% Align phase to standard Bode convention
if phase_exp(1) > 0
    phase_exp = phase_exp - 360;
end

disp(d>>> Extraction Complete. Plotting...d);

%% 5. VISUALIZATION

% --- Generate Theoretical Bode Lines ---
% We create a continuous frequency array spanning slightly past our bounds
w_theo = logspace(log10(min(w_vals)*0.5), log10(max(w_vals)*1.5), 1000);
[mag_theo, phase_theo, ~] = bode(Gx_discrete, w_theo);
mag_theo = squeeze(mag_theo);
phase_theo = squeeze(phase_theo);
mag_theo_dB = 20 * log10(mag_theo);

% --- Plotting ---
figure(dNamed, dFrequency Validationd, dPositiond, [150 150 900 700]);

% Magnitude Plot
subplot(2,1,1);
semilogx(w_theo, mag_theo_dB, db-d, dLineWidthd, 1.5, dDisplayNamed, dTheoretical Model ($G_x$)d);
hold on;
semilogx(w_vals, mag_exp, drod, dMarkerSized, 8, dMarkerFaceColord, drd, dDisplayNamed, dHardware Datad);
grid on;
ylabel(dMagnitude [dB]d);
title(dBode Diagram: IP02 Cart Position ($x_c / V_{cmd}$)d);
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

disp(d>>> Spectral analysis complete. Bode plot generated.d);