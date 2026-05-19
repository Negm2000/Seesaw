% --- Piece-Wise Trajectory Generation ---
% Run this AFTER your CasADi optimization has solved

% 1. Phase 1 Parameters (Minimum Jerk)
x0 = -0.407;     v0 = 0;      a0 = 0;
xf = 0.057;      vf = v_required; af = 0;
T1 = 3.0;        % Duration of Phase 1 [s] (Tune to adjust acceleration)
t1 = 0:Ts:T1;

% 2. Solve for 5th-Order Coefficients
% Setting up the matrix equation M * C = B
M = [ T1^3,    T1^4,     T1^5;
     3*T1^2,  4*T1^3,   5*T1^4;
     6*T1,   12*T1^2,  20*T1^3];
 
B = [xf - x0; 
     vf; 
     af];
 
C = M \ B; % Solve for [c3; c4; c5]
c3 = C(1); c4 = C(2); c5 = C(3);

% 3. Generate Phase 1 Trajectories
xc_phase1 = x0 + c3*t1.^3 + c4*t1.^4 + c5*t1.^5;
vc_phase1 = 3*c3*t1.^2 + 4*c4*t1.^3 + 5*c5*t1.^4;
ac_phase1 = 6*c3*t1 + 12*c4*t1.^2 + 20*c5*t1.^3;          % Acceleration
th_phase1 = deg2rad(11.66) * ones(size(t1)); % Stuck on table
th_dot_phase1 = zeros(size(t1));
u_phase1 = (M_e * ac_phase1 + B_total * vc_phase1) / alpha_f;

% 4. Extract Phase 2 from CasADi (Assuming x_opt and t_opt exist)
% We shift CasADi's time vector to start exactly when Phase 1 ends
t2 = t_opt + T1 + Ts; 
xc_phase2 = x_opt(1, :);
vc_phase2 = x_opt(3, :);
th_phase2 = x_opt(2, :);
th_dot_phase2 = x_opt(4, :);
u_phase2 = [u_opt, u_opt(end)];

% 5. Stitching them together (The Piece-Wise Trajectory)
t_total = [t1, t2];
xc_total = [xc_phase1, xc_phase2];
vc_total = [vc_phase1, vc_phase2];
th_total = [th_phase1, th_phase2];
th_dot_total = [th_dot_phase1, th_dot_phase2];
u_ff_total = [u_phase1, u_phase2];

% 6. Export to Simulink (as timeseries)
ts_cart_pos = timeseries(xc_total', t_total');
ts_cart_vel = timeseries(vc_total', t_total');
ts_seesaw_angle = timeseries(th_total', t_total');
ts_u_ff = timeseries(u_ff_total', t_total');

% 7. Plotting the Full Mission Profile
figure('Name', 'Full Piece-Wise Trajectory');
subplot(2,2,1);
plot(t_total, xc_total, 'b', 'LineWidth', 1.5); hold on;
xline(T1, 'r--', 'Lift-Off ($0.057$m)', 'Interpreter', 'latex');
ylabel('Cart Position [m]'); title('Cart Dynamics'); grid on;

subplot(2,2,3);
plot(t_total, vc_total, 'g', 'LineWidth', 1.5); hold on;
xline(T1, 'r--', 'Interpreter', 'latex'); xlabel('Time [s]'); 
ylabel('Cart Velocity [m/s]'); grid on;

subplot(2,2,2);
plot(t_total, rad2deg(th_total), 'k', 'LineWidth', 1.5); hold on;
xline(T1, 'r--', 'Interpreter', 'latex');
ylabel('Seesaw Angle [deg]');
title('Seesaw Dynamics'); grid on;

subplot(2,2,4);
plot(t_total, rad2deg(th_dot_total), 'k', 'LineWidth', 1.5); hold on;
xline(T1, 'r--', 'Interpreter', 'latex');
ylabel('Seesaw Angular Speed [deg/s]'); xlabel('Time [s]'); grid on;