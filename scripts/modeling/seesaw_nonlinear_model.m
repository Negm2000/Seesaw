%% seesaw_nonlinear_model.m
%  -----------------------------------------------------------------------
%  Full Nonlinear Simulation of Quanser SEESAW-E + IP02 System
%  -----------------------------------------------------------------------
%  PREREQUISITE: Run seesaw_params.m first to load all parameters.
%
%  Equations exactly match the Quanser Seesaw Laboratory Guide ("Good ref"):
%    - 1st Lagrange eq (cart):   page 6, top
%    - 2nd Lagrange eq (seesaw): page 6, bottom
%    - Motor force F_c:          Eq. 2.3 (reduced model, L_m = 0)
%
%  State vector:  x = [x_c; x_c_dot; alpha; alpha_dot]
%    x(1) = x_c       : cart position [m] (0 = centered on seesaw)
%    x(2) = x_c_dot   : cart velocity [m/s]
%    x(3) = alpha      : seesaw tilt angle [rad] (0 = level)
%    x(4) = alpha_dot  : seesaw angular velocity [rad/s]
%
%  Input: V_cmd [V] = voltage command from DAQ
%  -----------------------------------------------------------------------
% setting default parameter with LaTeX interpreter
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');
%% ===== Check that parameters are loaded =====
if ~exist('K_a', 'var')
    error('Run seesaw_params.m first to load system parameters!');
end

seesaw_params;

%% 4. LOAD & INSPECT HARDWARE DATA
%  Load the frequency sweep data collected from QUARC.
%  Plot raw time traces to sanity-check before analysis.

if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
data_file = fullfile(SEESAW_ROOT, 'data', 'cartModeling', 'non_linear.mat');

if ~exist(data_file, 'file')
    error('data not found. Run on hardware first.');
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
    error('Expected variable "ip02_freq_data" or "data" in data.mat. Found: %s', strjoin(vars, ', '));
end

% Extract columns: [time; V_cmd; x_c; x_c_dot]
t_hw      = raw(1, :)';
V_cmd_hw  = raw(2, :)';
xc_hw     = raw(3, :)';      % [m] — with corrected encoder gain
dt_hw     = mean(diff(t_hw));
Fs_hw     = 1 / dt_hw;

% Compute the velocity in post-processing to avoid phase-lag
cutoff_freq = B_total/M_e * 2;
[b, a] = butter(2, cutoff_freq / (Fs_hw/2));
xc_hw_clean = filtfilt(b, a, xc_hw);

% Because of the differential we "lose" the last data point
tdot_hw = t_hw(1:end-1);
xcdot_hw = diff(xc_hw_clean)/dt_hw;

v_thresh = 0.002;   % m/s

move_mask = abs(xcdot_hw) > v_thresh;
pos_move_mask = xcdot_hw > v_thresh;
neg_move_mask = xcdot_hw < -v_thresh;

move_sustain = false(size(move_mask));
pos_move_sustain = false(size(move_mask));
neg_move_sustain = false(size(move_mask));

for k = 1:length(move_mask)
    if all(move_mask(k))
        move_sustain(k) = true;
    end
    if all(pos_move_mask(k))
        pos_move_sustain(k) = true;
    end
    if all(neg_move_mask(k))
        neg_move_sustain(k) = true;
    end
end

%% 4. Estimate deadzone by voltage side
% This matches the ud_pos / ud_neg convention used by the identification model.
V_diff = V_cmd_hw(1:end-1);
pos_u_idx = find(move_sustain & V_diff > 0);
neg_u_idx = find(move_sustain & V_diff < 0);

if isempty(pos_u_idx)
    warning('No sustained motion under positive voltage detected. ud_pos estimate unavailable.');
    ud_pos = NaN;
else
    ud_pos = min(abs(V_diff(pos_u_idx)));
end

if isempty(neg_u_idx)
    warning('No sustained motion under negative voltage detected. ud_neg estimate unavailable.');
    ud_neg = NaN;
else
    ud_neg = min(abs(V_diff(neg_u_idx)));
end

ud_sym = mean([ud_pos, ud_neg], 'omitnan');

%% 5. Estimate deadzone by motion direction
% These values answer the question of positive/negative cart motion directly.
pos_motion_idx = find(pos_move_sustain);
neg_motion_idx = find(neg_move_sustain);

if isempty(pos_motion_idx)
    warning('No sustained positive motion detected. ud_pos_motion estimate unavailable.');
    ud_pos_motion = NaN;
else
    ud_pos_motion = min(abs(V_diff(pos_motion_idx)));
end

if isempty(neg_motion_idx)
    warning('No sustained negative motion detected. ud_neg_motion estimate unavailable.');
    ud_neg_motion = NaN;
else
    ud_neg_motion = min(abs(V_diff(neg_motion_idx)));
end

%% 6. Check sign consistency between voltage and motion direction
pos_motion_with_pos_u = nnz(pos_move_sustain & V_diff > 0);
pos_motion_with_neg_u = nnz(pos_move_sustain & V_diff < 0);
neg_motion_with_neg_u = nnz(neg_move_sustain & V_diff < 0);
neg_motion_with_pos_u = nnz(neg_move_sustain & V_diff > 0);

%% 7. Print results
fprintf('Estimated deadzone values:\n');
fprintf('By voltage side (same convention as identification):\n');
fprintf('ud_pos = %.6f V\n', ud_pos);
fprintf('ud_neg = %.6f V\n', ud_neg);
fprintf('ud_sym = %.6f V\n\n', ud_sym);

fprintf('By motion direction:\n');
fprintf('ud_pos_motion = %.6f V\n', ud_pos_motion);
fprintf('ud_neg_motion = %.6f V\n\n', ud_neg_motion);

fprintf('Motion/voltage sign consistency counts:\n');
fprintf('positive motion with positive voltage: %d\n', pos_motion_with_pos_u);
fprintf('positive motion with negative voltage: %d\n', pos_motion_with_neg_u);
fprintf('negative motion with negative voltage: %d\n', neg_motion_with_neg_u);
fprintf('negative motion with positive voltage: %d\n', neg_motion_with_pos_u);

%% 8. Plot for visual inspection
figure('Name', 'Deadzone Estimation', 'NumberTitle', 'off')

subplot(3,1,1)
plot(tdot_hw, V_diff, 'b', 'LineWidth', 1.0)
grid on
hold on
yline(ud_pos, 'r--', 'u_{d,pos}')
yline(-ud_neg, 'm--', '-u_{d,neg}')
xlabel('Time (s)')
ylabel('Input u (V)')
title('Input Signal with Estimated Deadzone Levels')

subplot(3,1,2)
plot(t_hw, xc_hw, 'k', 'LineWidth', 1.0)
grid on
xlabel('Time (s)')
ylabel('Position x (m)')
title('Measured Position')

subplot(3,1,3)
plot(tdot_hw, xcdot_hw, 'g', 'LineWidth', 1.0)
hold on
plot(tdot_hw(pos_move_sustain), xcdot_hw(pos_move_sustain), 'r.', 'MarkerSize', 8)
plot(tdot_hw(neg_move_sustain), xcdot_hw(neg_move_sustain), 'm.', 'MarkerSize', 8)
grid on
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Estimated Velocity and Sustained-Motion Points')
legend('$\dot{x}_c$', 'Positive motion', 'Negative motion', 'Location', 'best')

%% SAVE DATA
if ~exist('SEESAW_ROOT', 'var'), SEESAW_ROOT = fileparts(mfilename('fullpath')); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
save_file = fullfile(SEESAW_ROOT, 'data', 'param_nonlinear.mat');
save(save_file, 'ud_neg', 'ud_pos', 'ud_sym');
fprintf('\n  Tuned parameters saved to: data/tuned_cart.mat\n');
fprintf('============================================================\n');
