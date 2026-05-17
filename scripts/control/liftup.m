%% Two-Segment Minimum Jerk Lift-up Trajectory
clc; close all;

% =========================================================================
% 1. TIMING PARAMETERS (Your Tuning Knobs)
% =========================================================================
% Based on your sim, total time needs to be ~2.4s. We split it:
T_launch = 4;  % Time to reach the lift-off trip-wire
T_catch  = 1;  % Time from lift-off until the 5-degree trigger
T_total  = T_launch + T_catch;

fprintf('Total Lift-up Time: %.2f seconds\n', T_total);

% =========================================================================
% 2. SEGMENT 1: THE LAUNCH (0 to T_launch)
% =========================================================================
x0 = -0.407;  v0 = 0.0;   a0 = 0.0; % Start on table
x1 = 0.07;   v1 = 0.0;   a1 = 0.0; % Overshoot 0.057 slightly to guarantee tip

t1 = (0:Ts:T_launch)';
M1 = [ 0,              0,              0,            0,          0,  1;
       0,              0,              0,            0,          1,  0;
       0,              0,              0,            2,          0,  0;
       T_launch^5,     T_launch^4,     T_launch^3,   T_launch^2, T_launch, 1;
       5*T_launch^4,   4*T_launch^3,   3*T_launch^2, 2*T_launch, 1,  0;
       20*T_launch^3,  12*T_launch^2,  6*T_launch,   2,          0,  0 ];
b1 = [x0; v0; a0; x1; v1; a1];
coeffs1 = M1 \ b1;

xc1      = polyval(coeffs1, t1);
xc_dot1  = polyval(polyder(coeffs1), t1);
xc_ddot1 = polyval(polyder(polyder(coeffs1)), t1);

% =========================================================================
% 3. SEGMENT 2: THE CATCH PREP (T_launch to T_total)
% =========================================================================
% States at the moment of the switch
theta_switch = deg2rad(5);      
theta_dot_switch = -0.15;       % Tune this based on what you see in the sim
theta_ddot_switch = 9 * theta_switch; % Approx angular accel using pole p=3

% 1. Ideal Catch Position (Zero Proportional Error)
xc_ideal = -0.05; %(Kp_out * theta_switch + Kd_out * theta_dot_switch);

% 2. Ideal Catch Velocity (Zero Velocity Error)
xc_dot_ideal = -0.1; %(Kp_out * theta_dot_switch + Kd_out * theta_ddot_switch);

% Target states based on our Cascade PID calculation
x2 = xc_ideal;   v2 = xc_dot_ideal;  a2 = 0.0;

t2 = (0:Ts:T_catch)';
M2 = [ 0,             0,             0,           0,         0,  1;
       0,             0,             0,           0,         1,  0;
       0,             0,             0,           2,         0,  0;
       T_catch^5,     T_catch^4,     T_catch^3,   T_catch^2, T_catch, 1;
       5*T_catch^4,   4*T_catch^3,   3*T_catch^2, 2*T_catch, 1,  0;
       20*T_catch^3,  12*T_catch^2,  6*T_catch,   2,         0,  0 ];
b2 = [x1; v1; a1; x2; v2; a2]; % Note: Starts where Segment 1 ended!
coeffs2 = M2 \ b2;

xc2      = polyval(coeffs2, t2);
xc_dot2  = polyval(polyder(coeffs2), t2);
xc_ddot2 = polyval(polyder(polyder(coeffs2)), t2);

% =========================================================================
% 4. STITCH THE TRAJECTORIES TOGETHER
% =========================================================================
% Remove the first point of segment 2 to avoid duplicate time steps
t2_shifted = t2(2:end) + T_launch; 
xc2        = xc2(2:end);
xc_dot2    = xc_dot2(2:end);
xc_ddot2   = xc_ddot2(2:end);

t_total_arr = [t1; t2_shifted];
xc_total    = [xc1; xc2];
xc_dot_total= [xc_dot1; xc_dot2];
xc_ddot_total=[xc_ddot1; xc_ddot2];

% Add a brief buffer at the end so the cart continues moving at v2 if the 
% switch triggers slightly late
t_buffer = (t_total_arr(end)+Ts : Ts : t_total_arr(end)+0.5)';
xc_buffer = x2 + v2 * (t_buffer - t_total_arr(end));
t_final = [t_total_arr; t_buffer];
xc_final = [xc_total; xc_buffer];

% Package for Simulink
sim_xc_ref = [t_final, xc_final];

% =========================================================================
% 5. VISUALIZATION
% =========================================================================
figure('Name', 'Two-Segment Catch Trajectory');
subplot(2,1,1);
plot(t_final, xc_final, 'b', 'LineWidth', 2); hold on;
yline(0.057, 'k--', 'Lift-off threshold (0.057m)');
plot(T_launch, x1, 'ro', 'MarkerSize', 8, 'DisplayName', 'Lift-off Point');
plot(T_total, x2, 'g*', 'MarkerSize', 10, 'DisplayName', 'Ideal Catch Point');
ylabel('Position (m)'); grid on; title('Cart Position'); legend;

subplot(2,1,2);
plot(t_total_arr, xc_dot_total, 'b', 'LineWidth', 2); hold on;
yline(0, 'k-');
plot(T_total, v2, 'g*', 'MarkerSize', 10, 'DisplayName', 'Ideal Catch Velocity');
yline(0.15, 'r--', 'Speed Limit (+0.15 m/s)');
yline(-0.15, 'r--', 'Speed Limit (-0.15 m/s)');
ylabel('Velocity (m/s)'); xlabel('Time (s)'); grid on; title('Cart Velocity');