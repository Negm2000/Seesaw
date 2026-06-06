%% Time Evolution Comparison of Multiple Controllers
% This script loads recorded hardware data for four specific test files
% and generates time-domain plots comparing their performance side-by-side.

clear all; close all; clc;

% Configure global plot settings for publication quality
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. FILE MANAGEMENT
% Define project root dynamically
if ~exist('SEESAW_ROOT', 'var')
    SEESAW_ROOT = fileparts(mfilename('fullpath')); 
    SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT));
end

data_dir = fullfile(SEESAW_ROOT, 'data', 'seesawControl');

% Define the target files and their plot labels
target_file = 'pid_stable.mat'

%% 2. TIME-DOMAIN VISUALIZATION & ANALYSIS
disp('--- Generating Time Evolution Comparison ---');

figure('Name', 'Controller Time Evolution Comparison', 'Position', [100 100 1000 800]);

file_path = fullfile(data_dir, target_file);
if ~exist(file_path, 'file')
    warning('File %s not found. Skipping...', target_file);
end
    
% Load Data
loaded_data = load(file_path);
vars = fieldnames(loaded_data);
    
% Handle variable naming differences
if ismember('ip02_freq_data', vars)
    raw_data = loaded_data.ip02_freq_data;
elseif ismember('data', vars)
    raw_data = loaded_data.data;
else
    warning('Expected variable in %s not found.', target_file);
end

% Extract Rows: [time; theta_ref(rad); theta_hw(rad); xc_hw(m); vm_hw]
t      = raw_data(1, :)';
th_ref = raw_data(2, :)';
th_hw  = raw_data(3, :)' - deg2rad(11.66);
x_hw   = raw_data(4, :)';
v_hw   = raw_data(5, :)';

% --- Determine ON / OFF / Steady State Regions ---
% 1. Find Switch On (First massive voltage spike)
idx_on = find(abs(v_hw) >= 5.5, 1, 'first');
t_on = t(idx_on);

% 2. Find Switch Off (Last point where voltage is actively doing work)
% Using 0.05V threshold to account for potential sensor/ADC noise
idx_off = find(abs(v_hw) > 0.05, 1, 'last');
t_off = t(idx_off);

% 3. Isolate the Steady State Data
idx_ss = idx_on:idx_off;
t_ss = t(idx_ss);
th_hw_ss_deg = rad2deg(th_hw(idx_ss));

% --- Computations (No Plotting for these) ---
mean_angle = mean(th_hw_ss_deg);
min_angle = min(th_hw_ss_deg);
max_angle = max(th_hw_ss_deg);

% Calculate Time Cycle using Peak-to-Peak distance
[~, peak_locs] = findpeaks(-th_hw_ss_deg);
if length(peak_locs) > 1
    time_cycle = mean(diff(t_ss(peak_locs)));
else
    time_cycle = NaN;
end

% Print the computed metrics
fprintf('\n--- Steady State Analysis ---\n');
fprintf('Switch ON Time:   %.2f s\n', t_on);
fprintf('Switch OFF Time:  %.2f s\n', t_off);
fprintf('Mean Angle:       %.2f deg\n', mean_angle);
fprintf('Min Angle:        %.2f deg\n', min_angle);
fprintf('Max Angle:        %.2f deg\n', max_angle);
fprintf('Time Cycle:       %.2f s\n', time_cycle);
disp('-----------------------------');

% --- Plotting --- 
% 1. Angle Tracking 
subplot(3,1,1); hold on;
plot(t, rad2deg(th_ref), 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');
plot(t, rad2deg(th_hw), 'b-', 'LineWidth', 1.5);
xline(t_on, 'k--', 'Switch ON', 'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
xline(t_off, 'k--', 'Switch OFF', 'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
yline(mean_angle, 'g--', sprintf('Mean: %.2f°', mean_angle), 'HandleVisibility', 'off');
ylabel('Angle [$^\circ$]'); title('Pendulum Angle ($\theta_{hw}$) vs Reference ($\theta_{ref}$)');
ylim([-15 5]); % Lock y-limits for consistency
grid on;
    
% 2. Cart Position
subplot(3,1,2); hold on;
plot(t, x_hw, 'b-', 'LineWidth', 1.5);
xline(t_on, 'k--', 'HandleVisibility', 'off');
xline(t_off, 'k--', 'HandleVisibility', 'off');
ylabel('Position [m]'); title('Cart Position ($x_c$)');
grid on;
    
% 3. Motor Voltage (Control Effort)
subplot(3,1,3); hold on;
plot(t, v_hw, 'b-', 'LineWidth', 1.5);
xline(t_on, 'k--', 'HandleVisibility', 'off');
xline(t_off, 'k--', 'HandleVisibility', 'off');
ylabel('Voltage [V]'); xlabel('Time [s]'); title('Motor Control Effort ($V_m$)');
grid on;

disp('>>> evolution plot generated successfully.');