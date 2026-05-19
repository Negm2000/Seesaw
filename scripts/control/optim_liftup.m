% IP02 / SEESAW-E Swing-Up Trajectory Optimizer
% Uses fmincon to find the fastest splined trajectory that lands in the RoA

%% 0. Define Physical Parameters (Passed to local functions via struct)
% Using nominal values from Quanser manual where applicable
params.g       = 9.81;    % Gravity [m/s^2]
params.M_SW    = M_SW;  % Mass of Seesaw [kg]
params.M_total = M_total;  % Mass of Cart + Weight [kg]
params.D_T     = D_T;   % Distance from pivot to track [m]
params.D_C     = D_C;   % Distance from pivot to CG [m]
params.B_SW    = B_SW;     % Viscous damping [N-m-s/rad]
params.J_pivot = J_pivot;   % Moment of Inertia [kg-m^2]

%% 1. Optimization Setup
% Decision Variables: z = [T2, T3, x_peak]
% Initial Guess (based on our previous manual estimates)
z0 = [1.0, 2.0, 0.15]; 

% Lower Bounds: [Min T2, Min T3, Min x_peak (must be > 0.0568)]
lb = [0.2, 0.2, 0.08]; 

% Upper Bounds: [Max T2, Max T3, Max x_peak (track limit)]
ub = [3.0, 5.0, 0.35];

% Optimizer Options
options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'iter-detailed', ...
    'StepTolerance', 1e-4, ...
    'ConstraintTolerance', 1e-4, ...
    'MaxFunctionEvaluations', 1000);

%% 2. Run fmincon
fprintf('Starting Trajectory Optimization...\n');

% NOTE: We use an anonymous function @(z) to pass 'params' into the constraints
[z_opt, fval, exitflag, output] = fmincon(@objective_func, z0, [], [], [], [], lb, ub, @(z) nonlinear_constraints(z, params), options);

%% 3. Results
if exitflag > 0
    fprintf('\n--- OPTIMIZATION SUCCESSFUL ---\n');
    fprintf('Optimal T2 (Pump Time):      %.3f s\n', z_opt(1));
    fprintf('Optimal T3 (Retract Time):   %.3f s\n', z_opt(2));
    fprintf('Optimal x_peak (Overshoot):  %.4f m\n', z_opt(3));
    fprintf('Total Maneuver Time:         %.3f s\n', fval);
else
    fprintf('\n--- OPTIMIZATION FAILED ---\n');
    disp(output.message);
end

%% =========================================================================
%  LOCAL FUNCTIONS
%  =========================================================================

%% Objective Function: Minimize Total Time
function J = objective_func(z)
    T2 = z(1);
    T3 = z(2);
    J = T2 + T3; % We want the fastest possible catch
end

%% Nonlinear Constraints: The Physical Reality Check
function [c, ceq] = nonlinear_constraints(z, params)
    T2 = z(1);
    T3 = z(2);
    x_peak = z(3);
    
    % Known Fixed Parameters
    x_stage = 0.050; 
    lift_off_point = 0.0568;
    x_end = 2 * lift_off_point - x_peak; % Guarantees acc=0 at 0.0568m
    
    % --- Constraint 1 & 2: Kinematic Velocity Limits ---
    % v_max = 1.875 * (Distance / Time)
    v_max_limit = 0.15; % m/s
    v_peak_T2 = 1.875 * abs(x_peak - x_stage) / T2;
    v_peak_T3 = 1.875 * abs(x_end - x_peak) / T3;
    
    % --- Constraint 3: Region of Attraction (LQR Catch) ---
    % 1. Simulate the nonlinear dynamics forward in time
    % Initial state of seesaw at x_stage
    % State vector for ODE: X = [theta; theta_dot]
    X0 = [11.66 * (pi/180); 0]; 
    
    % Time span for the dynamic portion (T2 + T3)
    tspan = [0, T2 + T3];
    
    % Run ODE45 (Pass params down again)
    [t_sim, X_sim] = ode45(@(t, X) seesaw_dynamics(t, X, z, x_stage, x_end, params), tspan, X0);
    
    % 2. Extract the Final State at the exact moment of handoff
    theta_final = X_sim(end, 1);
    theta_dot_final = X_sim(end, 2);
    
    % Calculate cart state at the end of T3 (we know this analytically)
    xc_final = 0.0568; 
    xc_dot_final = -1.875 * abs(x_end - x_peak) / T3; % Peak negative velocity
    
    % Assemble the 4D state vector for the LQR evaluation
    % Ensure this matches the state vector order of your P matrix!
    x_sys = [xc_final; theta_final; xc_dot_final; theta_dot_final];
    
    % --- INSERT YOUR P MATRIX HERE ---
    P = [ 100,  10,   5,   1; 
           10, 500,  20,   5; 
            5,  20,  50,   2; 
            1,   5,   2,  10]; % REPLACE WITH YOUR REAL P MATRIX
            
    % Calculate Lyapunov Energy
    energy = x_sys' * P * x_sys;
    
    % Inequality constraints: c(x) <= 0
    c(1) = v_peak_T2 - v_max_limit; % Must be <= 0
    c(2) = v_peak_T3 - v_max_limit; % Must be <= 0
    c(3) = energy - 0.05;           % RoA Catch Constraint
    
    ceq = []; % No equality constraints
end

%% Nonlinear Equations of Motion (ODE)
function dXdt = seesaw_dynamics(t, X, z, x_stage, x_end, params)
    T2 = z(1);
    T3 = z(2);
    x_peak = z(3);
    
    theta = X(1);
    theta_dot = X(2);
    
    % Unpack Parameters
    M_total = params.M_total;
    M_SW    = params.M_SW;
    D_T     = params.D_T;
    D_C     = params.D_C;
    B_SW    = params.B_SW;
    J_pivot = params.J_pivot;
    g       = params.g;
    
    % Determine current cart kinematics based on time
    if t <= T2
        % Phase 2 (The Pump)
        tau = t / T2;
        D = x_peak - x_stage;
        xc      = x_stage + D * (10*tau^3 - 15*tau^4 + 6*tau^5);
        xc_dot  = (D / T2) * (30*tau^2 - 60*tau^3 + 30*tau^4);       % Added for Coriolis term
        xc_ddot = (D / T2^2) * (60*tau - 180*tau^2 + 120*tau^3);
    else
        % Phase 3 (The Retract)
        t_local = t - T2;
        tau = t_local / T3;
        D = x_end - x_peak;
        xc      = x_peak + D * (10*tau^3 - 15*tau^4 + 6*tau^5);
        xc_dot  = (D / T3) * (30*tau^2 - 60*tau^3 + 30*tau^4);       % Added for Coriolis term
        xc_ddot = (D / T3^2) * (60*tau - 180*tau^2 + 120*tau^3);
    end
    
    % User's specific Euler-Lagrange Equation of Motion
    theta_ddot = (M_total * D_T * xc_ddot - g * M_total * xc * cos(theta) - ...
                  2 * M_total * xc_dot * theta_dot + ...
                  g * (M_total * D_T + M_SW * D_C) * sin(theta) - ...
                  B_SW * theta_dot) / (J_pivot + M_total * D_T^2 + M_total * xc^2);
                  
    % Return state derivatives
    dXdt = [theta_dot; theta_ddot];
end