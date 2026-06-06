%% Closed-Loop PID Validation: Step & Multi-Sine Response
% This script loads recorded hardware data for both a step response and
% a multi-sine sweep. It plots the time-domain tracking performance, 
% control effort, and extracts the empirical closed-loop Bode plot T(s).

% The execution must be done right after the pid_cart.m script.

% Configure global plot settings for publication quality
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% 1. FILE MANAGEMENT & THEORETICAL MODEL SETUP

% Define project root dynamically
if ~exist('SEESAW_ROOT', 'var')
    SEESAW_ROOT = fileparts(mfilename('fullpath')); 
    SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); 
end

% --- Load Multi-Sine Parameters ---
params_file = fullfile(SEESAW_ROOT, 'data', 'cartControl', 'multisine_params.mat');
if exist(params_file, 'file')
    load(params_file, 'w_vals'); 
else
    error('Parameters file (multisine_params.mat) not found.');
end

%% 2. STEP RESPONSE ANALYSIS (TIME DOMAIN)

step_file = fullfile(SEESAW_ROOT, 'data', 'cartControl', 'step_output.mat');
if exist(step_file, 'file')
    fprintf('Loading %s ...\n', step_file);
    step_loaded = load(step_file);
    
    if isfield(step_loaded, 'data')
        raw_step = step_loaded.data;
    else
        vars = fieldnames(step_loaded);
        raw_step = step_loaded.(vars{1});
    end
    
    % Extract columns: [time; x_ref; x_hw; v_input; v_real]
    t_step     = raw_step(1, :)';
    xref_step  = raw_step(2, :)';
    xhw_step   = raw_step(3, :)';
    v_input    = raw_step(4, :)';
    v_real     = raw_step(5, :)';
    
    % Simulate theoretical response to the actual reference signal
    x_sim = lsim(T_in, xref_step, t_step);
    
    % --- Plot Step Response (2 Subplots) ---
    figure('Name', 'Closed-Loop Step Response', 'Position', [100 100 800 700]);
    
    % Subplot 1: Voltage
    subplot(2,1,1);
    plot(t_step, v_input, 'b-', 'LineWidth', 1.5, 'DisplayName', '$v_{input}$ (Commanded)');
    hold on;
    plot(t_step, v_real, 'r--', 'LineWidth', 1.5, 'DisplayName', '$v_{real}$ (Actual)');
    grid on;
    ylabel('Voltage [V]');
    title('Control Effort');
    legend('Location', 'northeast');
    
    % Subplot 2: Position
    subplot(2,1,2);
    plot(t_step, xhw_step, 'k-', 'LineWidth', 1.5, 'DisplayName', '$x_{hw}$ (Hardware)');
    hold on;
    plot(t_step, xref_step, 'r-', 'LineWidth', 1.5, 'DisplayName', '$x_{ref}$ (Reference)');
    plot(t_step, x_sim, 'b--', 'LineWidth', 1.5, 'DisplayName', '$x_{sim}$ (Model)');
    grid on;
    xlabel('Time [s]');
    ylabel('Position $x_c$ [m]');
    title('Tracking Performance');
    legend('Location', 'southeast');
else
    warning('Step response data not found. Skipping time-domain plot.');
end

%% 3. MULTI-SINE ANALYSIS (FREQUENCY DOMAIN)
multisine_file = fullfile(SEESAW_ROOT, 'data', 'cartControl', 'multisine_output.mat');
if ~exist(multisine_file, 'file')
    error('Multi-sine data not found. Check file path.');
end

fprintf('Loading %s ...\n', multisine_file);
ms_loaded = load(multisine_file);

if isfield(ms_loaded, 'data')
    raw_ms = ms_loaded.data;
else
    vars = fieldnames(ms_loaded);
    raw_ms = ms_loaded.(vars{1});
end

% Extract columns for frequency analysis
t_ms    = raw_ms(1, :)';
xref_ms = raw_ms(2, :)'; 
xhw_ms  = raw_ms(3, :)';
v_input = raw_ms(4, :)';
v_real  = raw_ms(5, :)';

% Visualization
figure('Name', 'Closed-Loop Multi-Sine Profile', 'Position', [150 150 900 600]);
subplot(2,1,1);
plot(t_ms, xref_ms, 'b', 'LineWidth', 1.2); hold on;
plot(t_ms, xhw_ms, 'r--', 'LineWidth', 1);
ylabel('Position [m]'); title('Command vs Hardware Position');
legend('Reference ($x_{ref}$)', 'Output ($x_c$)'); grid on;

subplot(2,1,2);
% Plot the compensated "real" hardware command (in red)
plot(t_ms, v_real, 'r--', 'LineWidth', 1, 'DisplayName', 'Hardware Command ($u_{real}$)');
hold on;
% Plot the ideal linear command (in blue)
plot(t_ms, v_input, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Linear Command ($u_{safe}$)');

% Add threshold lines
yline(V_sat, 'k-.', 'Hardware Peak Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(-V_sat, 'k-.', 'HandleVisibility', 'off');
yline(V_nom, 'k--', 'Max Safe Limit', 'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
yline(-V_nom, 'k--', 'HandleVisibility', 'off');

xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Simulated Voltage [V]', 'Interpreter', 'latex');
title('Schroeder-Phased Multi-Sine Input Vector', 'Interpreter', 'latex');
grid on;
legend('Location', 'northeast', 'Interpreter', 'latex');
ylim([-V_sat*1.2, V_sat*1.2]);

%% 4. MULTI-SINE BODE (FREQUENCY DOMAIN)

% 1. System Parameters
dt = mean(diff(t_ms));
Fs = 1/dt;

% 2. Trim the startup transient
T_settle = max(3.0, 2 * (2*pi/w_vals(1))); 
T_end = 37.7;
valid_idx = (t_ms >= T_settle & t_ms <= T_end);

% Detrend the data to remove DC offsets
xref_ss = detrend(xref_ms(valid_idx));
xhw_ss  = detrend(xhw_ms(valid_idx));

v_input = v_input(valid_idx);
v_real  = v_real(valid_idx);

disp('Extracting FRF via Welch''s Method on Position Data...');

% 3. Extract FRF (T)
f_target = w_vals / (2*pi);
window = hanning(length(xref_ss));
[Txy, ~] = tfestimate(xref_ss, xhw_ss, window, 0, f_target, Fs);

% 4. Extract Magnitude and Phase
mag_exp = 20 * log10(abs(Txy));
phase_exp = unwrap(angle(Txy)) * 180 / pi;

% Align phase to standard closed-loop Bode convention (start near 0 deg)
while phase_exp(1) > 180
    phase_exp = phase_exp - 360;
end
while phase_exp(1) < -180
    phase_exp = phase_exp + 360;
end

disp('>>> Extraction Complete. Plotting Bode...');

%% 4. VISUALIZATION: BODE PLOT

figure('Name', 'Closed-Loop Frequency Validation', 'Position', [150 150 900 700]);

% --- Generate Theoretical Bode Lines ---
w_theo = logspace(log10(min(w_vals)*0.5), log10(max(w_vals)*1.5), 1000);
[mag_theo, phase_theo, ~] = bode(T_in, w_theo);
mag_theo_dB = 20 * log10(squeeze(mag_theo));
phase_theo = squeeze(phase_theo);

% Magnitude Plot
subplot(2,1,1);
semilogx(w_theo, mag_theo_dB, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Theoretical ($T(s)$)');
hold on;
semilogx(w_vals, mag_exp, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Hardware Data');
grid on;
ylabel('Magnitude [dB]');
title('Closed-Loop Bode Diagram: $T(s) = X_{hw}(s) / X_{ref}(s)$');
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

disp('>>> Script execution complete.');