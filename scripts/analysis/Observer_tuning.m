%% --- PREREQUISITES ---
% Load your system parameters so A_sw, B_sw, C_sw, D_sw are available!
% Uncomment the following line if your matrices are saved in a .mat file:
% load('data/tuned_seesaw.mat'); 

% Load the KF data
kf_file = fullfile(SEESAW_ROOT, 'data', 'lqr', 'kf.mat');
load('kf.mat', 'data');

% Extract the data rows as specified
t_hw         = data(1, :);
dt_hw = mean(diff(t_hw));
u_hw         = data(2, :);

theta_hw = data(3, :);
xc_hw = data(5, :);
xc_dot_hw = gradient(xc_hw);
theta_dot_hw = gradient(theta_hw);

Fs_hw     = 1 / dt_hw;
cutoff_freq = 50;
[b, a] = butter(2, cutoff_freq / (Fs_hw/2));
xc_hw_clean = filtfilt(b, a, xc_hw);
theta_hw_clean = filtfilt(b, a, theta_hw);
xc_dot_hw_clean = filtfilt(b, a, xc_hw);
theta_dot_hw_clean = filtfilt(b, a, theta_hw);


x_hw    = [xc_hw;theta_hw;xc_dot_hw_clean; theta_dot_hw_clean];  % [xc; theta; xc_dot; theta_dot] (Filtered velocities)
x_hat = data(7:10, :); % [xc_hat; theta_hat; xc_dot_hat; theta_dot_hat] (Current KF)

% Keep only data after 10 seconds
idx_valid = (t_hw >= 10);

t_hw              = t_hw(idx_valid);
u_hw              = u_hw(idx_valid);
xc_hw        = xc_hw(idx_valid);
theta_hw     = theta_hw(idx_valid);
xc_dot_hw    = xc_dot_hw(idx_valid);
theta_dot_hw = theta_dot_hw(idx_valid);
x_hw = x_hw(:,idx_valid);
x_hat = x_hat(:, idx_valid);
%% =========================================================================
% STEP 1: PLOTTING CURRENT MEASUREMENTS
% =========================================================================
disp('--- Step 1: Plotting Current Data ---');

figure('Name', '1) Current Measurements', 'Position', [100, 100, 800, 800]);

% 1) Voltage
subplot(5,1,1);
plot(t_hw, u_hw, 'm', 'LineWidth', 1.2);
ylabel('Voltage [V]', 'Interpreter', 'latex'); 
title('Control Input', 'Interpreter', 'latex'); 
grid on;

% 2) States Comparison
labels = {'x_c \text{ [m]}', '\theta \text{ [rad]}', '\dot{x}_c \text{ [m/s]}', '\dot{\theta} \text{ [rad/s]}'};
for i = 1:4
    subplot(5,1,i+1);
    plot(t_hw, x_hw(i,:), 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtered (Target)');
    hold on;
    plot(t_hw, x_hat(i,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Current KF');
    ylabel(['$', labels{i}, '$'], 'Interpreter', 'latex');
    grid on; 
    legend('Location', 'best', 'Interpreter', 'latex');
end
xlabel('Time [s]', 'Interpreter', 'latex');


%% =========================================================================
% STEP 2: TUNING Rn AND Qn
% =========================================================================
disp('--- Step 2: Tuning Qn and Rn ---');

% Define Measurements for the Filter
% Assuming standard inverted pendulum: sensors measure xc and theta.
% We extract rows 1 and 2 of the filtered data (which are just the raw positions)
y_meas = x_hw(1:2, :); 

% Initial guess for Qn (4x1 diag) and Rn (2x1 diag)
% We optimize log10 of the values so fminsearch doesn't crash on negative variances
p0 = log10([1e-4; 1e-4; 1e-2; 1e-2;  % Initial Qn diagonal
            1e-3; 1e-3]);            % Initial Rn diagonal

% Optimization settings
options = optimset('Display', 'iter', 'MaxIter', 500, 'TolFun', 1e-3, 'TolX', 1e-3);

disp('Running optimization (minimizing innovation errors)...');
p_opt = fminsearch(@(p) kf_cost(p, Ad, Bd, Cd, Dd, u_hw, y_meas), p0, options);

% Reconstruct the optimized matrices from logarithmic scale
Qn_opt = diag(p_opt(1:4));
Rn_opt = diag(p_opt(5:6));

fprintf('\n--- Optimized Tuning Matrices ---\n');
disp('Optimized Qn:'); disp(Qn_opt);
disp('Optimized Rn:'); disp(Rn_opt);


%% =========================================================================
%% STEP 3: PLOTTING THE IMPROVEMENT
%% =========================================================================
disp('--- Step 3: Plotting the Improvement ---');

% Simulate the Kalman Filter using the optimized Qn and Rn
[~, x_hat_opt] = kf_cost(p_opt, Ad, Bd, Cd, Dd, u_hw, y_meas);

figure('Name', '3) KF Tuning Improvement', 'Position', [150, 150, 800, 800]);

for i = 1:4
    subplot(4,1,i);
    % Plot the target filtered signal
    plot(t_hw, x_hw(i,:), 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtered (Target)');
    hold on;
    % Plot the old/current Kalman Filter performance
    plot(t_hw, x_hat(i,:), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Current KF');
    % Plot the new Optimized Kalman Filter performance
    plot(t_hw, x_hat_opt(i,:), 'g-.', 'LineWidth', 2.0, 'DisplayName', 'Optimized KF');
    
    ylabel(['$', labels{i}, '$'], 'Interpreter', 'latex');
    grid on;
    
    if i == 1
        title('Kalman Filter Optimization Results', 'Interpreter', 'latex');
    end
    legend('Location', 'best', 'Interpreter', 'latex');
end
xlabel('Time [s]', 'Interpreter', 'latex');


%% =========================================================================
%% HELPER FUNCTION: KALMAN FILTER COST EVALUATION
%% =========================================================================
function [cost, x_hat_sim] = kf_cost(p, Ad, Bd, Cd, Dd, u, y_meas)
    % Reconstruct Qn and Rn from the log10 search parameters
    Qn = diag(10.^p(1:4));
    Rn = diag(10.^p(5:6));

    nx = size(Ad, 1);
    N = length(u);

    % Solve Discrete Algebraic Riccati Equation for Steady-State Kalman Gain
    try
        [~, ~, L] = dare(Ad', Cd', Qn, Rn);
        L = L'; % The steady-state Kalman Gain
    catch
        % If Qn/Rn are a bad combination and DARE fails, return a high penalty
        cost = 1e8; 
        x_hat_sim = zeros(nx, N);
        return;
    end

    x_hat_sim = zeros(nx, N);
    x_hat = zeros(nx, 1); % Assume starting state is zero (adjust if necessary)
    cost = 0;

    for k = 1:N
        % 1. Predict the measurement
        y_hat = Cd * x_hat + Dd * u(k);

        % 2. Calculate the Innovation (Error between measurement and prediction)
        inn = y_meas(:, k) - y_hat;

        % 3. Correct the state estimate using the Kalman Gain
        x_hat = x_hat + L * inn;

        % Store the posterior state estimate
        x_hat_sim(:, k) = x_hat;

        % 4. Time Update (Predict next state)
        x_hat = Ad * x_hat + Bd * u(k);

        % 5. Accumulate cost (Sum of Squared Innovation Errors)
        cost = cost + sum(inn.^2);
    end
end