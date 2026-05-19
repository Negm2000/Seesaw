%% Autonomous 2-Phase Minimum Jerk Lift-up Trajectory (Corrected Physics)
clc; close all;

% =========================================================================
% 1. INITIAL PARAMETERS & WORKSPACE EXTRACTION
% =========================================================================
               
T_launch = 2;             % Time to reach lift-off point (constrained on table)
T_sim_max = 3;            % Max time to simulate the fall AFTER lift-off

% Boundary states
x0 = -0.407;                % Initial cart position (m)
x1 = 0.57;                 % Lift-off position overshoot (m)
theta_0 = deg2rad(11.66);   % Initial resting angle (rad)
theta_target = deg2rad(5);  % Catch trigger angle (rad)

% Extract Plant Dynamics from Workspace 
if ~exist('Gt', 'var')
    error('Transfer function Gt(s) not found in workspace.');
end

% Convert Gt(s) to State-Space 
sys_t = ss(Gt);
A_t = sys_t.A; B_t = sys_t.B; C_t = sys_t.C; D_t = sys_t.D;

% Initialize pendulum perfectly at rest at 11.66 deg
x_init = pinv([C_t; C_t*A_t]) * [theta_0; 0]; 

% Extract Controller Gains 
if exist('Kp_out', 'var') && exist('Kd_out', 'var')
    Kp = Kp_out; 
    Kd = Kd_out;
else
    % Fallback: Approximate from PI-Lead C_out(s) if individual gains aren't in workspace
    [num, den] = tfdata(C_out, 'v');
    Kp = -num(end-1)/den(end-1); 
    Kd = -num(1)/den(1);         
end

% =========================================================================
% 2. PHASE 1: THE LAUNCH TRAJECTORY (Mechanically Constrained)
% =========================================================================
t1 = (0:Ts:T_launch)';
M1 = [ 0,             0,             0,           0,         0,  1;
       0,             0,             0,           0,         1,  0;
       0,             0,             0,           2,         0,  0;
       T_launch^5,    T_launch^4,    T_launch^3,  T_launch^2,T_launch, 1;
       5*T_launch^4,  4*T_launch^3,  3*T_launch^2,2*T_launch,1,  0;
       20*T_launch^3, 12*T_launch^2, 6*T_launch,  2,         0,  0 ];
b1 = [x0; 0; 0; x1; 0; 0];
coeffs1 = M1 \ b1;

xc1 = polyval(coeffs1, t1);
xc_dot1 = polyval(polyder(coeffs1), t1);
xc_ddot1 = polyval(polyder(polyder(coeffs1)), t1);

% =========================================================================
% 3. ITERATIVE SOLVER (Simulate ONLY the Fall)
% =========================================================================
fprintf('--- Autonomous Catch Solver ---\n');
max_iters = 10;

% Time vector strictly for the falling phase
t2_sim = (0:Ts:T_sim_max)'; 
T_catch_duration = 1.0; % Initial guess

for iter = 1:max_iters
    
    % --- A. Build Phase 2 Cart Trajectory ---
    if iter == 1
        % Iteration 1: Assume cart just sits still at x1 after lift-off
        xc2_sim = x1 * ones(size(t2_sim));
    else
        % Iteration 2+: Generate Minimum Jerk curve to the ideal catch states
        M2 = M1; % Re-use matrix structure
        M2(4:6, 1:5) = [T_catch_duration^5,    T_catch_duration^4,    T_catch_duration^3,  T_catch_duration^2,T_catch_duration;
                        5*T_catch_duration^4,  4*T_catch_duration^3,  3*T_catch_duration^2,2*T_catch_duration,1;
                        20*T_catch_duration^3, 12*T_catch_duration^2, 6*T_catch_duration,  2,                 0];
        b2 = [x1; 0; 0; x_ideal; v_ideal; 0];
        coeffs2 = M2 \ b2;
        
        xc2_curve = polyval(coeffs2, (0:Ts:T_catch_duration)');
        % Pad the rest of the simulation with coasting velocity to see the crossing
        xc2_coast = x_ideal + v_ideal * ((T_catch_duration+Ts : Ts : T_sim_max)' - T_catch_duration);
        xc2_sim = [xc2_curve; xc2_coast];
    end
    
    % --- B. Simulate Plant Gt(s) Response ---
    % Simulate ONLY the time after lift-off!
    [theta_sim, ~, x_state_sim] = lsim(sys_t, xc2_sim, t2_sim, x_init);
    
    % Find exact crossing of 5 degrees
    idx_catch = find(theta_sim <= theta_target, 1, 'first');
    
    if isempty(idx_catch)
        error('Seesaw never reached 5 degrees AFTER lift-off. Check initial angle or Gt sign conventions.');
    end
    
    % Update the catch duration for the next trajectory generation
    T_catch_duration = t2_sim(idx_catch);
    
    % Extract exact kinematics at the 5-degree crossing
    x_state_catch = x_state_sim(idx_catch, :)';
    theta_catch = C_t * x_state_catch;
    theta_dot_catch = C_t * A_t * x_state_catch + C_t * B_t * xc2_sim(idx_catch);
    theta_ddot_catch = C_t * A_t^2 * x_state_catch + C_t * A_t * B_t * xc2_sim(idx_catch);
    
    % --- C. Calculate Ideal PID Targets ---
    x_ideal = -(Kp * theta_catch + Kd * theta_dot_catch);
    v_ideal = -(Kp * theta_dot_catch + Kd * theta_ddot_catch);
    
    fprintf('Iter %d: Fall Time = %.3fs | theta_dot = %.3f rad/s | Target [x: %.3fm, v: %.3fm/s]\n', ...
            iter, T_catch_duration, theta_dot_catch, x_ideal, v_ideal);
end

% =========================================================================
% 4. FINALIZE AND EXPORT TRAJECTORY
% =========================================================================
t2 = (0:Ts:T_catch_duration)';
xc2 = polyval(coeffs2, t2);
xc_dot2 = polyval(polyder(coeffs2), t2);

% Stitch arrays together perfectly
t2_global = t2(2:end) + T_launch;
t_total = [t1; t2_global];
xc_total = [xc1; xc2(2:end)];
xc_dot_total = [xc_dot1; xc_dot2(2:end)];

% Add buffer so the cart continues coasting if the hardware switch triggers late
t_buffer = (t_total(end)+Ts : Ts : t_total(end)+0.5)';
xc_buffer = x_ideal + v_ideal * (t_buffer - t_total(end));
t_final = [t_total; t_buffer];
xc_final = [xc_total; xc_buffer];

sim_xc_ref = [t_final, xc_final];
assignin('base', 'sim_xc_ref', sim_xc_ref);

fprintf('\nSuccess! Trajectory saved to Workspace as "sim_xc_ref".\n');