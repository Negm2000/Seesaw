%% Multi-Sine Frequency Response Validation
% This script loads recorded hardware data from a multi-sine sweep,
% performs FFT analysis to extract the empirical Bode plot, and
% compares it against the theoretical transfer function.

clear all
close all
clc

% Configure global plot settings for publication quality
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. FILE MANAGEMENT AND DATA LOADING

% Define project root dynamically
if ~exist('SEESAW_ROOT', 'var')
    SEESAW_ROOT = fileparts(mfilename('fullpath')); 
    SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); 
end

% --- Load Theoretical Model ---
tf_file = fullfile(SEESAW_ROOT, 'data', 'tuned_cart.mat');
if exist(tf_file, 'file')
    load(tf_file); % This should load 'Gx' into the workspace
else
    error('Theoretical model (cart_tf.mat) not found. Run optimization first.');
end

Gx_discrete = c2d(Gx, 0.002, 'zoh');

% --- Load Multi-Sine Parameters ---
params_file = fullfile(SEESAW_ROOT, 'data', 'cartModeling', 'multisine_params.mat');
if exist(params_file, 'file')
    load(params_file, 'w_vals', 'N_freq'); % We specifically need the frequencies injected
else
    error('Parameters file (multisine_params.mat) not found.');
end

% --- Load Recorded Hardware Data ---
data_file = fullfile(SEESAW_ROOT, 'data', 'cartModeling', 'multisine_output2.mat');
if ~exist(data_file, 'file')
    error('Data not found. Run on hardware first.');
end

fprintf('Loading %s ...\n', data_file);
loaded = load(data_file);
vars = fieldnames(loaded);

% Handle different QUARC data formats
if ismember('ip02_freq_data', vars)
    raw = loaded.ip02_freq_data;
elseif ismember('data', vars)
    raw = loaded.data;
else
    error('Expected variable "ip02_freq_data" or "data". Found: %s', strjoin(vars, ', '));
end

% Extract columns: [time; V_cmd; x_c; x_c_dot]
t_hw      = raw(1, :)';
V_cmd_hw  = raw(2, :)'; % This is our u_safe (before friction compensator)
xc_hw     = raw(3, :)'; % Cart position in meters

%% 2 & 3. SYSTEM IDENTIFICATION (FILTERED VELOCITY + WELCH'S METHOD)

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

disp('Extracting FRF via Welch''s Method on Velocity...');

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

disp('>>> Extraction Complete. Plotting...');

%% 5. VISUALIZATION

% --- Generate Theoretical Bode Lines ---
% We create a continuous frequency array spanning slightly past our bounds
w_theo = logspace(log10(min(w_vals)*0.5), log10(max(w_vals)*1.5), 1000);
[mag_theo, phase_theo, ~] = bode(Gx_discrete, w_theo);
mag_theo = squeeze(mag_theo);
phase_theo = squeeze(phase_theo);
mag_theo_dB = 20 * log10(mag_theo);

% --- Plotting ---
figure('Name', 'Frequency Validation', 'Position', [150 150 900 700]);

% Magnitude Plot
subplot(2,1,1);
semilogx(w_theo, mag_theo_dB, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Theoretical Model ($G_x$)');
hold on;
semilogx(w_vals, mag_exp, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Hardware Data');
grid on;
ylabel('Magnitude [dB]');
title('Bode Diagram: IP02 Cart Position ($x_c / V_{cmd}$)');
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

disp('>>> Spectral analysis complete. Bode plot generated.');