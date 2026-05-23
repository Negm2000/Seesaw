%% 4D Region of Attraction Point-Cloud Mapping
clc;

% Configure global plot settings
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

% Load parameters identically to Setup 1
p.M_e = M_e; p.M_total = M_total; p.D_T = D_T; p.J_pivot = J_pivot;
p.M_SW = M_SW; p.D_C = D_C; p.B_total = B_total; p.B_SW = B_SW;
p.alpha_f = alpha_f; p.g = g; 

p.X_max = 0.407; p.Theta_max = 11.66 * (pi/180);
p.V_max = 6*sqrt(3); p.Slew_max = 150.0;
p.w_c = 30;

num_in = [30.15, 304.7, 335.9]; den_in = [0.01, 1, 0];
[p.A_in, p.B_in, p.C_in, p.D_in] = tf2ss(num_in, den_in);
num_out = [-5.198, -14.66, -3.341]; den_out = [1, 25, 0];
[p.A_out, p.B_out, p.C_out, p.D_out] = tf2ss(num_out, den_out);

% --- Define 4D Evaluation Sample Size ---
num_samples = 5000; 

% Define broad ranges for velocities to stress-test momentum boundaries
max_xc_dot = 0.0;       % Max initial test cart velocity (m/s)
max_theta_dot = 0 * (pi/180); % Max initial test tilt speed (rad/s)

% Generate 4D uniform random points
rng(42); % For reproducibility
raw_samples = zeros(num_samples, 4);

for i = 1:num_samples
    % 1. Sample a random cart position across the full rail length
    xc_val = (2*rand() - 1) * p.X_max;
    
    % 2. Look at your 2D plot: the nominal safe angle line is roughly: theta_deg = 40 * xc
    theta_center_rad = (40 * xc_val) * (pi/180);
    
    % 3. Sample a tight window (+/- 2.5 degrees) around that stable diagonal center line
    theta_spread_rad = (2*rand() - 1) * (2.5 * pi/180);
    theta_val = theta_center_rad + theta_spread_rad;
    
    % 4. Bound it to the absolute table limit
    theta_val = max(-p.Theta_max, min(p.Theta_max, theta_val));
    
    % 5. Store the 4D initial states
    raw_samples(i, :) = [
        xc_val, ...
        theta_val, ...
        (2*rand() - 1) * max_xc_dot, ...
        (2*rand() - 1) * max_theta_dot
    ];
end
raw_samples(1, :) = [0, 0, 0, 0]; % Enforce origin check

% Diagnostic Counters
count_total = 0;
count_crashed = 0;
count_failed_xc_settle = 0;
count_failed_theta_settle = 0;
count_gear_grind = 0;
safe_4D_points = [];

tspan = [0, 20]; % Extended window to give integrators room to settle
options = odeset('Events', @(t,y) system_limits_event(t,y,p), 'RelTol', 1e-3);

fprintf('Evaluating %d positions with Diagnostic Tracking...\n', num_samples);
for i = 1:num_samples
    y0 = [raw_samples(i,1); raw_samples(i,2); raw_samples(i,3); raw_samples(i,4); 0;0;0;0;0];
    
    % Using ode15s to prevent any chattering/freezing issues
    [t, y, te, ~, ~] = ode45(@(t,y) nonlinear_cascade_dynamics(t,y,p), tspan, y0, options);
    
    if ~isempty(te)
        count_crashed = count_crashed + 1;
        continue;
    end
    
    % Check settling conditions separately for diagnostics
    if abs(y(end,1)) >= 0.01
        count_failed_xc_settle = count_failed_xc_settle + 1;
        continue;
    end
    
    if abs(y(end,2)) >= (0.5 * pi/180)
        count_failed_theta_settle = count_failed_theta_settle + 1;
        continue;
    end
    
    % Check gear grinding constraints (skipping the very first step t=0 to ignore step-input artifacts)
    grind = false;
    for k = 2:length(t) 
        [~, v_m_dot] = nonlinear_cascade_dynamics(t(k), y(k,:)', p);
        if abs(v_m_dot) > p.Slew_max
            grind = true; 
            break; 
        end
    end
    
    if grind
        count_gear_grind = count_gear_grind + 1;
        continue;
    end
    
    % If it survives all gates, save it
    safe_4D_points = [safe_4D_points; raw_samples(i,:)];
end

%% --- Display Diagnostic Report ---
fprintf('\n================ DIAGNOSTIC REPORT ================ Layout\n');
fprintf('Total Samples Processed:      %d\n', num_samples);
fprintf('1. Hit Track/Table Limits:    %d\n', count_crashed);
fprintf('2. Failed Cart Settle (<0.02): %d\n', count_failed_xc_settle);
fprintf('3. Failed Tilt Settle (<0.5°):%d\n', count_failed_theta_settle);
fprintf('4. Disqualified by Gear Slew: %d\n', count_gear_grind);
fprintf('---------------------------------------------------\n');
fprintf('SUCCESSFUL SAFE POINTS FOUND: %d\n', size(safe_4D_points,1));
fprintf('===================================================\n\n');
% =========================================================================
% 5. VISUALIZATION
% =========================================================================
if isempty(safe_4D_points)
    error(['No safe points were found! The system is 100%% unstable under ' ...
           'these parameters or initial conditions. Graph cannot be plotted.']);
else
    figure('Color', [1 1 1]);
    scatter3(safe_4D_points(:,1), safe_4D_points(:,2)*180/pi, safe_4D_points(:,4)*180/pi, ...
             30, safe_4D_points(:,3), 'filled');
    xlabel('Cart Position $x_c$ (m)'); 
    ylabel('Seesaw Angle $\theta$ (deg)'); 
    zlabel('Angular Rate $\dot{\theta}$ (deg/s)');
    title('3D Representation Slice of the 4D Safe RoA Dataset');
    cb = colorbar; ylabel(cb, 'Cart Initial Velocity $\dot{x}_c$ (m/s)');
    grid on; view(3);

    % Save dataset for software safety bounds verification
    save(fullfile(SEESAW_ROOT, 'data', 'analysis', 'Safe_4D_RoA_Dataset.mat'), 'safe_4D_points');
end

%% HELPERS
% =========================================================================
% CORE FUNCTIONS
% =========================================================================

function [dydt, vm_dot] = nonlinear_cascade_dynamics(~, y, p)
    % Extract current states
    x_c       = y(1);
    theta     = y(2);
    x_c_dot   = y(3);
    theta_dot = y(4);
    z_out     = [y(5); y(6)];
    z_in      = [y(7); y(8)];
    vm_f      = y(9);
    
    % 1. Outer Loop Controller (Input tracking error e_\theta = 0 - \theta)
    e_theta = -theta;
    dz_out = p.A_out * z_out + p.B_out * e_theta;
    x_c_ref = p.C_out * z_out + p.D_out * e_theta;
    
    % 2. Inner Loop Controller (Input tracking error e_xc = x_c_ref - x_c)
    e_xc = x_c_ref - x_c;
    dz_in = p.A_in * z_in + p.B_in * e_xc;
    vm_raw = p.C_in * z_in + p.D_in * e_xc;
    
    % Enforce voltage saturation constraints
    vm_sat = max(-p.V_max, min(p.V_max, vm_raw));
    

    
    % 3. Calculate Exact Slew Rate (Voltage Derivative)
    if abs(vm_raw) >= p.V_max
        vm_dot = 0; % Voltage is flat against the rail; no physical grinding
    else
        vm_dot = p.w_c * (vm_sat - vm_f);
    end
    
    % 4. Nonlinear Equation Matrix Reconstruction
    % Mass Matrix: M(q)
    M = [p.M_e, -p.M_total * p.D_T;
        -p.M_total * p.D_T, p.J_pivot + p.M_total * (p.D_T^2) + p.M_total * (x_c^2)];
        
    % Nonlinear Dynamics forces and frictions vector: n(q, q_dot)
    n = [-p.M_total * x_c * (theta_dot^2) + p.g * p.M_total * sin(theta) + p.B_total * x_c_dot;
          p.g * p.M_total * x_c * cos(theta) + 2 * p.M_total * x_c_dot * theta_dot ...
          - p.g * (p.M_total * p.D_T + p.M_SW * p.D_C) * sin(theta) + p.B_SW * theta_dot];
          
    % Motor Force vector Input: F
    F = [p.alpha_f * vm_f; 0];
    
    % Invert mass matrix to find accelerations [x_c_ddot; theta_ddot]
    accel = M \ (F - n);
    
    % Pack derivative outputs
    dydt = [x_c_dot; theta_dot; accel(1); accel(2); dz_out(1); dz_out(2); dz_in(1); dz_in(2); vm_dot];
end

function [value, isterminal, direction] = system_limits_event(~, y, p)
    x_c   = y(1);
    theta = y(2);
    
    % Triggers event termination if boundaries are breached
    % Event condition functions cross zero when hitting maximum constraints
    value = [p.X_max - abs(x_c); p.Theta_max - abs(theta)]; 
    isterminal = [1; 1];   % Terminate simulation step instantly
    direction  = [-1; -1]; % Trigger only when leaving safe region bounds
end