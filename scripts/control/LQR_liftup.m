% IP02 / SEESAW-E Swing-Up Trajectory Generator
% 3-Phase Splined Minimum Jerk Trajectory

%% 1. Define Spatial Waypoints (Meters)
x_start = -0.407;     % Hard stop resting position
x_stage =  0.050;     % Staging point (just before lift-off at 0.0568)
x_peak  =  0.0827;     % Overshoot point (pumps kinetic energy into seesaw)

% The lift-off point is x = 0.0568m.
% To ensure acceleration is exactly 0 when we cross 0.0568m on the way back, 
% 0.0568m MUST be the exact midpoint of the 3rd spline.
% Midpoint = (x_peak + x_end) / 2  -->  x_end = 2*(Midpoint) - x_peak
x_end = 2 * (0.0568) - x_peak; % Evaluates to -0.0364 m

%% 2. Define Temporal Durations (Seconds)
% Tuned to ensure peak velocity (1.875 * D / T) <= 0.2 m/s
T1 = 5.0; % Phase 1: Safe approach (D = 0.457m, vmax = 0.171 m/s)
T2 = 0.303; % Phase 2: The Pump (D = 0.100m, vmax = 0.187 m/s)
T3 = 0.867; % Phase 3: The Retract (D = 0.186m, vmax = 0.174 m/s)

%% 3. Generate the 3 Splines
[t1, pos1, vel1, acc1] = generate_min_jerk(x_start, x_stage, T1, Ts);
[t2, pos2, vel2, acc2] = generate_min_jerk(x_stage, x_peak, T2, Ts);
[t3, pos3, vel3, acc3] = generate_min_jerk(x_peak, x_end, T3, Ts);

%% 4. Concatenate the Trajectories
% Shift the time vectors to be continuous
t2_shifted = t2(2:end) + t1(end);
t3_shifted = t3(2:end) + t2_shifted(end);

time_total = [t1, t2_shifted, t3_shifted];
pos_total  = [pos1, pos2(2:end), pos3(2:end)];
vel_total  = [vel1, vel2(2:end), vel3(2:end)];
acc_total  = [acc1, acc2(2:end), acc3(2:end)];

%% 5. Verify the Intercept Point (Handoff to LQR)
% Find the exact index where the cart crosses 0.0568m during Phase 3
handoff_idx = find(time_total > (T1 + T2) & pos_total <= 0.0568, 1, 'first');

fprintf('--- Handoff State at Switch --- \n');
fprintf('Time:         %.3f s\n', time_total(handoff_idx));
fprintf('Cart Pos:     %.4f m\n', pos_total(handoff_idx));
fprintf('Cart Vel:     %.4f m/s\n', vel_total(handoff_idx));
fprintf('Cart Accel:   %.4f m/s^2\n', acc_total(handoff_idx));

%% 6. Plotting to Verify Constraints
figure('Name', 'Swing-Up Kinematics', 'Position', [100, 100, 800, 800]);

subplot(3,1,1);
plot(time_total, pos_total, 'b', 'LineWidth', 1.5); hold on;
yline(0.0568, 'r--', 'Lift-off Threshold (0.0568m)');
plot(time_total(handoff_idx), pos_total(handoff_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
ylabel('Position (m)'); title('Cart Position (x_c)'); grid on;

subplot(3,1,2);
plot(time_total, vel_total, 'g', 'LineWidth', 1.5); hold on;
yline(0.2, 'k--', 'Max Velocity Limit');
yline(-0.2, 'k--');
plot(time_total(handoff_idx), vel_total(handoff_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
ylabel('Velocity (m/s)'); title('Cart Velocity'); grid on;

subplot(3,1,3);
plot(time_total, acc_total, 'm', 'LineWidth', 1.5); hold on;
plot(time_total(handoff_idx), acc_total(handoff_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
ylabel('Acceleration (m/s^2)'); xlabel('Time (s)'); title('Cart Acceleration'); grid on;

%% Helper Function: 5th-Order Minimum Jerk
function [t, pos, vel, acc] = generate_min_jerk(x0, xf, T, dt)
    t = 0:dt:T;
    tau = t / T; % Normalized time (0 to 1)
    
    % Min jerk polynomials
    pos = x0 + (xf - x0) * (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    vel = ((xf - x0) / T) * (30*tau.^2 - 60*tau.^3 + 30*tau.^4);
    acc = ((xf - x0) / T^2) * (60*tau - 180*tau.^2 + 120*tau.^3);
end