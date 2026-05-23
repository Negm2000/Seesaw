%% 2D Region of Attraction (RoA) Map Generator
clc; close all;

set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

% =========================================================================
% 1. PHYSICAL PARAMETERS (Replace placeholder values with your known constants)
% =========================================================================
p.M_e      = M_e;       % Effective mass of the cart (kg)
p.M_total  = M_total;   % Combined mass of cart and seesaw (kg)
p.D_T      = D_T;       % Distance parameter (m)
p.J_pivot  = J_pivot;   % Seesaw inertia about the pivot (kg*m^2)
p.M_SW     = M_SW;      % Mass of the seesaw alone (kg)
p.D_C      = D_C;       % Distance to seesaw center of mass (m)
p.B_total  = B_total;   % Friction coefficient for the cart
p.B_SW     = B_SW;      % Friction coefficient for the pivot
p.alpha_f  = alpha_f;   % Motor torque/force scaling factor
p.g        = g;         % Acceleration due to gravity (m/s^2)

% --- Constraints ---
p.X_max     = 0.407;               % Hard track limits (m)
p.Theta_max = 11.66 * (pi / 180);  % Table limit converted to radians
p.V_max     = 6;         % Peak allowable motor voltage (V)
p.Slew_max  = 60.0;                % Max safe voltage derivative (V/s) to avoid grinding

p.w_c      = 30;

% =========================================================================
% 2. CONTROLLER STATE-SPACE CONVERSION
% =========================================================================
% Inner Loop: C_in(s) = (30.15*s^2 + 304.7*s + 335.9) / (0.01*s^2 + s)
num_in = [30.15, 304.7, 335.9]; den_in = [0.01, 1, 0];
[p.A_in, p.B_in, p.C_in, p.D_in] = tf2ss(num_in, den_in);

% Outer Loop: C_out(s) = (-5.198*s^2 - 14.66*s - 3.341) / (s^2 + 25*s)
num_out = [-5.198, -14.66, -3.341]; den_out = [1, 25, 0];
[p.A_out, p.B_out, p.C_out, p.D_out] = tf2ss(num_out, den_out);

% =========================================================================
% 3. GRID GENERATION (Zero initial velocities assumed)
% =========================================================================
grid_res  = 75; % Increase to 100+ for higher resolution resolution
xc_vec    = linspace(-p.X_max, p.X_max, grid_res);
theta_vec = linspace(-p.Theta_max, p.Theta_max, grid_res);
[Xc_grid, Theta_grid] = meshgrid(xc_vec, theta_vec);
RoA_matrix = zeros(size(Xc_grid));

% =========================================================================
% 4. NUMERICAL GRID EVALUATION
% =========================================================================
tspan = [0, 10]; % 6 second evaluation window
options = odeset('Events', @(t,y) system_limits_event(t,y,p), 'RelTol', 1e-4);

fprintf('Mapping 2D Region of Attraction... Grid size: %dx%d\n', grid_res, grid_res);
tic;
for i = 1:numel(Xc_grid)
    % Initial State: [xc, theta, x_dot, theta_dot, z_out1, z_out2, z_in1, z_in2]
    y0 = [Xc_grid(i); Theta_grid(i); 0; 0; 0; 0; 0; 0; 0];
    
    [t, y, te, ~, ~] = ode45(@(t,y) nonlinear_cascade_dynamics(t,y,p), tspan, y0, options);
    
    if ~isempty(te)
        RoA_matrix(i) = 0; % Crashed: Hit track limit or table surface
    else
        % Check steady state target window convergence
        if abs(y(end,1)) < 0.01 && abs(y(end,2)) < (0.2 * pi/180)
            % Check if the motor voltage derivative stripped gears along the path
            gear_grind_detected = false;
            for k = 1:length(t)
                [~, v_m_dot] = nonlinear_cascade_dynamics(t(k), y(k,:)', p);
                if abs(v_m_dot) > p.Slew_max
                    gear_grind_detected = true;
                    break;
                end
            end
            
            if gear_grind_detected
                RoA_matrix(i) = 1; % Stable but hazardous to hardware
            else
                RoA_matrix(i) = 2; % Safely Bounded RoA
            end
        else
            RoA_matrix(i) = 0; % Did not crash, but failed to stabilize
        end
    end
end
toc;

%% =========================================================================
% 5. VISUALIZATION
% =========================================================================
figure('Color', [1 1 1]);
custom_cmap = [1.0, 0.4, 0.4;  % Red: Failed
               1.0, 0.8, 0.2;  % Yellow: Gear Wear Hazard
               0.3, 0.8, 0.3]; % Green: Safe RoA
           
contourf(Xc_grid, Theta_grid * (180/pi), RoA_matrix, [0 1 2]);
colormap(custom_cmap);
cb = colorbar('Ticks', [0.33, 1.0, 1.66], 'TickLabels', {'Crashed/Unstable', 'Gear Slew Hazard', 'Safe Bounded RoA'});
xlabel('Initial Cart Position $x_c$ (meters)');
ylabel('Initial Seesaw Angle $\theta$ (degrees)');
title('Seesaw-Cart 2D Region of Attraction Slice ($\dot{x}_c(0)=0$, $\dot{\theta}(0)=0$)');
grid on;

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