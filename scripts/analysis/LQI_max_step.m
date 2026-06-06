%% LQR Comprehensive Hardware Data Analysis 
% Analyzes a sequence of incremental steps (1 to 10 deg)
% Generates a macro-overview of the staircase sequence and a 
% micro-analysis of the maximum step event.

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

data_dir = fullfile(SEESAW_ROOT, 'data', 'lqr');
target_file = 'LQR_max_step.mat';

disp('--- Loading LQR Hardware Data ---');
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

%% 2. DATA EXTRACTION (UPDATED INDEXING)
% Row 1: Time
% Rows 2-5: References [0; theta_ref; 0; 0]
% Rows 6-9: States [xc_hw; theta_hw; xcdot_hw; thetadot_hw]
% Rows 10-13: Observer [xc_hat; theta_hat; xcdot_hat; thetadot_hat]
% Row 14 (Optional): Index
% Row end: Voltage v_cmd

t      = raw_data(1, :)';
th_ref = raw_data(3, :)'; % Row 3 corresponds to theta_ref
x_hw   = raw_data(6, :)'; % Row 6 corresponds to xc_hw
th_hw  = raw_data(7, :)'; % Row 7 is theta_hw
v_hw   = raw_data(end, :)'; % last row is input voltage

% Convert angles to degrees for easier interpretation
y_ref = rad2deg(th_ref);
y_hw  = rad2deg(th_hw);

%% 3. MACRO ANALYSIS: TARGET STEP DETECTION
% --- Determine Step Occurrences ---
dy_ref = [0; diff(y_ref)];
% Lower threshold slightly to safely catch transitions
raw_step_idx = find(abs(dy_ref) > 0.1); 

% Filter out contiguous indices to find the exact start of each step
step_idx = [];
for i = 1:length(raw_step_idx)
    if i == 1 || (t(raw_step_idx(i)) - t(raw_step_idx(i-1)) > 0.5)
        step_idx = [step_idx; raw_step_idx(i)];
    end
end

fprintf('\n=== PART 1: INCREMENTAL SEQUENCE OVERVIEW ===\n');
fprintf('Detected %d distinct steps in the reference signal.\n', length(step_idx));

% --- Find the Step that reaches the MAXIMUM reference (e.g., 10 degrees) ---
max_ref_overall = max(abs(y_ref));
target_step_idx = step_idx(1); % Default fallback

for i = 1:length(step_idx)
    idx = step_idx(i);
    
    % Look at a stable window before and after the step to avoid 1-sample noise
    idx_prev = max(1, idx - 50);
    idx_next = min(length(y_ref), idx + 50);
    
    val_before = mean(y_ref(idx_prev:idx-5));
    val_after  = mean(y_ref(idx+5:idx_next));
    
    current_step_size = abs(val_after - val_before);
    
    fprintf('Step %d: t = %5.2f s | Transition: %5.1f deg -> %5.1f deg (Mag: %4.1f)\n', ...
        i, t(idx), val_before, val_after, current_step_size);
        
    % We want the step that *lands* on the maximum reference plateau
    % (Using 0.95 to account for any slight noise in the reference generation)
    if abs(val_after) >= 0.95 * max_ref_overall
        target_step_idx = idx;
    end
end

% Override max_step_idx to specifically use the peak amplitude step
max_step_idx = target_step_idx;

%% 4. MICRO ANALYSIS: MAXIMUM TRANSIENT STEP
% --- Isolate the Maximum Step Event ---
t_step = t(max_step_idx);
y0_target = y_ref(max_step_idx - 1);   
yf_target = y_ref(max_step_idx); 
step_size = yf_target - y0_target;

% --- Determine Dynamic Evaluation Window ---
step_order = find(step_idx == max_step_idx);

if step_order < length(step_idx)
    % Find the time of the next step and add a 2-second buffer for the plot
    t_next_step = t(step_idx(step_order + 1));
    idx_end = find(t >= t_next_step + 2, 1, 'first');
    if isempty(idx_end)
        idx_end = length(t);
    end
else
    % Fallback if it happens to be the very last step in the entire file
    t_next_step = t(end);
    idx_end = length(t);
end

% Start window 2 seconds before the step
idx_start = find(t >= t_step - 2, 1, 'first');

t_win      = t(idx_start:idx_end);
y_ref_win  = y_ref(idx_start:idx_end);
y_hw_win   = y_hw(idx_start:idx_end);

% --- Transient Metrics ---
% Check direction of step for proper peak calculation
if step_size > 0
    [peak_val, idx_peak] = max(y_hw_win);
else
    [peak_val, idx_peak] = min(y_hw_win);
end
t_peak = t_win(idx_peak);
overshoot_pct = abs((peak_val - yf_target) / step_size) * 100;

% Steady-state is evaluated over the 2 seconds immediately PRIOR to the next step
idx_ss_start = find(t_win >= t_next_step - 2, 1, 'first');
idx_ss_end   = find(t_win >= t_next_step, 1, 'first');
ss_val_step  = mean(y_hw_win(idx_ss_start:idx_ss_end));
ss_error     = yf_target - ss_val_step;

tolerance = 0.5; 
upper_bound = yf_target + tolerance;
lower_bound = yf_target - tolerance;

% Settling time calculation (strictly before the next step occurs)
out_of_bounds = (y_hw_win > upper_bound) | (y_hw_win < lower_bound);
idx_last_out = find(out_of_bounds & (t_win > t_step) & (t_win < t_next_step), 1, 'last');

if isempty(idx_last_out) || idx_last_out == length(t_win)
    t_settle = NaN; 
else
    t_settle = t_win(idx_last_out + 1) - t_step;
end

fprintf('\n=== PART 2: MAX STEP RESPONSE (t=%.2fs) ===\n', t_step);
fprintf('Step Size:             %.2f deg\n', step_size);
fprintf('Peak Value:            %.2f deg (at t=%.2fs)\n', peak_val, t_peak);
fprintf('Overshoot:             %.1f %%\n', overshoot_pct);
fprintf('Settling Time (0.5deg): %.2f s\n', t_settle);
fprintf('Steady-State Error:    %.3f deg\n', ss_error);
disp('=============================================');

%% 5. VISUALIZATION - FIGURE 1: MACRO OVERVIEW
figure('Name', 'Fig 1: LQR Staircase Sequence', 'Position', [100 100 1000 800]);

% Angle
subplot(3,1,1); hold on;
plot(t, y_ref, 'r--', 'LineWidth', 1, 'DisplayName', 'Reference ($\theta_{ref}$)');
plot(t, y_hw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Hardware Response ($\theta_{hw}$)');
ylabel('Angle [$^\circ$]'); title('Pendulum Angle vs Staircase Reference');
legend('Location', 'northwest');
grid on;

% Position
subplot(3,1,2); hold on;
plot(t, x_hw, 'b-', 'LineWidth', 1.5);
ylabel('Position [m]'); title('Cart Position ($x_c$)');
grid on;

% Voltage
subplot(3,1,3); hold on;
plot(t, v_hw, 'b-', 'LineWidth', 1.5);
ylabel('Voltage [V]'); xlabel('Time [s]'); title('LQR Control Effort ($v_{cmd}$)');
grid on;

%% 6. VISUALIZATION - FIGURE 2: ZOOMED MAX STEP RESPONSE
figure('Name', 'Fig 2: LQR Max Step Analysis', 'Position', [200 200 900 600]);

plot(t_win, y_hw_win, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Hardware Response ($\theta_{hw}$)'); hold on;
plot(t_win, y_ref_win, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference ($\theta_{ref}$)');

xline(t_step, 'k:', 'Step Initiated', 'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
plot(t_peak, peak_val, 'r*', 'MarkerSize', 8, 'DisplayName', 'Peak Overshoot');

yline(upper_bound, 'g:', 'HandleVisibility', 'off');
yline(lower_bound, 'g:', '10\% Tolerance Band', 'LabelHorizontalAlignment', 'right', 'HandleVisibility', 'off');

if ~isnan(t_settle)
    xline(t_step + t_settle, 'm--', 'Settled', 'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
    plot(t_step + t_settle, y_hw_win(idx_last_out + 1), 'mo', 'MarkerSize', 6, 'DisplayName', 'Settling Point');
end

title(sprintf('LQR Maximum Step Response Detail (Zoomed around t = %.1fs)', t_step));
xlabel('Time [s]'); ylabel('Pendulum Angle [$^\circ$]');
legend('Location', 'southeast'); grid on; xlim([t_win(1) t_win(end)]);

disp('>>> Analysis complete. Both LQR figures generated successfully.');