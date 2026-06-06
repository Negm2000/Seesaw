%% Comprehensive Hardware Data Analysis 
% Generates both a macro-overview of the physical run (ON/OFF, steady-state)
% and a micro-analysis of specific transient step events.

clear all; close all; clc;

% Configure global plot settings for publication quality
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. FILE MANAGEMENT & LOADING
if ~exist('SEESAW_ROOT', 'var')
    SEESAW_ROOT = fileparts(mfilename('fullpath')); 
    SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT));
end

data_dir = fullfile(SEESAW_ROOT, 'data', 'seesawControl');
target_file = 'PID_STEP.mat';

disp('--- Loading Hardware Data ---');
file_path = fullfile(data_dir, target_file);
if ~exist(file_path, 'file')
    error('File %s not found. Please check the path.', target_file);
end
    
loaded_data = load(file_path);
vars = fieldnames(loaded_data);
    
if ismember('ip02_freq_data', vars)
    raw_data = loaded_data.ip02_freq_data;
elseif ismember('data', vars)
    raw_data = loaded_data.data;
else
    error('Expected variable in %s not found.', target_file);
end

% Extract Rows: [time; theta_ref(rad); theta_hw(rad); xc_hw(m); vm_hw]
t      = raw_data(1, :)';
th_ref = raw_data(2, :)';
th_hw  = raw_data(3, :)' - deg2rad(11.66); % Sensor offset correction
x_hw   = raw_data(4, :)';
v_hw   = raw_data(5, :)';

%% 2. MACRO ANALYSIS: GLOBAL STEADY STATE
% --- Determine ON / OFF / Steady State Regions ---
idx_on = find(abs(v_hw) >= 5.5, 1, 'first');
t_on = t(idx_on);

idx_off = find(abs(v_hw) > 0.05, 1, 'last');
t_off = t(idx_off);

idx_ss = idx_on:idx_off;
t_ss = t(idx_ss);
th_hw_ss_deg = rad2deg(th_hw(idx_ss));

% --- Computations ---
mean_angle = mean(th_hw_ss_deg);
min_angle = min(th_hw_ss_deg);
max_angle = max(th_hw_ss_deg);

[~, peak_locs] = findpeaks(-th_hw_ss_deg);
if length(peak_locs) > 1
    time_cycle = mean(diff(t_ss(peak_locs)));
else
    time_cycle = NaN;
end

fprintf('\n=== PART 1: GLOBAL STEADY STATE ===\n');
fprintf('Switch ON Time:   %.2f s\n', t_on);
fprintf('Switch OFF Time:  %.2f s\n', t_off);
fprintf('Mean Angle:       %.2f deg\n', mean_angle);
fprintf('Min/Max Angle:    [%.2f, %.2f] deg\n', min_angle, max_angle);
fprintf('Time Cycle:       %.2f s\n', time_cycle);

%% 3. MICRO ANALYSIS: TRANSIENT STEP (t ~ 85s)
% --- Isolate the Step Event ---
step_search_window = (t > 80) & (t < 90);
d_ref = diff(th_ref);

idx_step = find(abs(d_ref) > 1e-3 & step_search_window(1:end-1), 1, 'first');
t_step = t(idx_step);

idx_start = find(t >= t_step - 2, 1, 'first');
idx_end   = find(t >= t_step + 20, 1, 'first');

t_win      = t(idx_start:idx_end);
th_ref_win = th_ref(idx_start:idx_end);
th_hw_win  = th_hw(idx_start:idx_end);

y_ref = rad2deg(th_ref_win);
y_hw  = rad2deg(th_hw_win);
y0_target = y_ref(1);   
yf_target = y_ref(end); 
step_size = yf_target - y0_target;

% --- Transient Metrics ---
[peak_val, idx_peak] = max(y_hw);
t_peak = t_win(idx_peak);
overshoot_pct = (peak_val - yf_target) / step_size * 100;

idx_ss_step = find(t_win >= t_win(end) - 2);
ss_val_step = mean(y_hw(idx_ss_step));
ss_error = yf_target - ss_val_step;

tolerance = 0.1 * step_size;
upper_bound = yf_target + tolerance;
lower_bound = yf_target - tolerance;

out_of_bounds = (y_hw > upper_bound) | (y_hw < lower_bound);
idx_last_out = find(out_of_bounds & (t_win > t_step), 1, 'last');

if isempty(idx_last_out) || idx_last_out == length(t_win)
    t_settle = NaN; 
else
    t_settle = t_win(idx_last_out + 1) - t_step;
end

fprintf('\n=== PART 2: STEP RESPONSE (t=%.2fs) ===\n', t_step);
fprintf('Step Size:             %.2f deg\n', step_size);
fprintf('Peak Value:            %.2f deg (at t=%.2fs)\n', peak_val, t_peak);
fprintf('Overshoot:             %.1f %%\n', overshoot_pct);
fprintf('Settling Time (10%%):  %.2f s\n', t_settle);
fprintf('Steady-State Error:    %.3f deg\n', ss_error);
disp('=======================================');

%% 4. VISUALIZATION - FIGURE 1: GENERAL OVERVIEW
figure('Name', 'Fig 1: General Time Evolution', 'Position', [100 100 1000 800]);

% Angle
subplot(3,1,1); hold on;
plot(t, rad2deg(th_ref), 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');
plot(t, rad2deg(th_hw), 'b-', 'LineWidth', 1.5);
xline(t_on, 'k--', 'Switch ON', 'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
xline(t_off, 'k--', 'Switch OFF', 'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
ylabel('Angle [$^\circ$]'); title('Pendulum Angle ($\theta_{hw}$) vs Reference ($\theta_{ref}$)');
ylim([-15 5]); grid on;

% Position
subplot(3,1,2); hold on;
plot(t, x_hw, 'b-', 'LineWidth', 1.5);
xline(t_on, 'k--', 'HandleVisibility', 'off');
xline(t_off, 'k--', 'HandleVisibility', 'off');
ylabel('Position [m]'); title('Cart Position ($x_c$)');
grid on;

% Voltage
subplot(3,1,3); hold on;
plot(t, v_hw, 'b-', 'LineWidth', 1.5);
xline(t_on, 'k--', 'HandleVisibility', 'off');
xline(t_off, 'k--', 'HandleVisibility', 'off');
ylabel('Voltage [V]'); xlabel('Time [s]'); title('Motor Control Effort ($V_m$)');
grid on;

%% 5. VISUALIZATION - FIGURE 2: ZOOMED STEP RESPONSE
figure('Name', 'Fig 2: Isolated Step Analysis', 'Position', [200 200 900 600]);

plot(t_win, y_hw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Hardware Response ($\theta_{hw}$)'); hold on;
plot(t_win, y_ref, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference ($\theta_{ref}$)');

xline(t_step, 'k:', 'Step Initiated', 'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
plot(t_peak, peak_val, 'r*', 'MarkerSize', 8, 'DisplayName', 'Peak Overshoot');

yline(upper_bound, 'g:', 'HandleVisibility', 'off');
yline(lower_bound, 'g:', '10% Tolerance Band', 'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');

if ~isnan(t_settle)
    xline(t_step + t_settle, 'm--', 'Settled', 'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
    plot(t_step + t_settle, y_hw(idx_last_out + 1), 'mo', 'MarkerSize', 6, 'DisplayName', 'Settling Point');
end

title(sprintf('Step Response Detail (Zoomed around t = %.1fs)', t_step));
xlabel('Time [s]'); ylabel('Pendulum Angle [$^\circ$]');
legend('Location', 'southeast'); grid on; xlim([t_win(1) t_win(end)]);

disp('>>> Analysis complete. Both figures generated successfully.');