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

%% ===== Check that parameters are loaded =====
if ~exist('K_a', 'var')
    error('Run seesaw_params.m first to load system parameters!');
end

%% ===== Simulation Settings =====
t_final = 10;              % Simulation duration [s]
dt_plot = 0.001;            % Output time resolution [s]

% --- Input Signal ---
% Modify these to match your actual experiment
input_type = 'sine';        % 'sine', 'step', or 'chirp'
A_input    = 3.0;           % Amplitude of V_cmd [V] (match your hardware test)
f_input    = 0.5;           % Frequency for sine [Hz]
step_time  = 1.0;           % Step time for step input [s]

%% ===== Input Signal Function =====
switch input_type
    case 'sine'
        V_cmd_fn = @(t) A_input * sin(2*pi*f_input*t);
    case 'step'
        V_cmd_fn = @(t) A_input * (t >= step_time);
    case 'chirp'
        f_start = 0.1;  f_end = 5.0;
        V_cmd_fn = @(t) A_input * sin(2*pi*(f_start + (f_end-f_start)*t/(2*t_final))*t);
    otherwise
        error('Unknown input_type: %s', input_type);
end

%% ===== Nonlinear ODE Function =====
% Exactly matches the Good ref (Quanser Seesaw Laboratory Guide).

function dxdt = seesaw_ode(t, x, V_cmd_fn, params)
    % Unpack state
    x_c       = x(1);   % cart position [m]
    x_c_dot   = x(2);   % cart velocity [m/s]
    alpha     = x(3);   % seesaw angle [rad]
    alpha_dot = x(4);   % seesaw angular rate [rad/s]

    p = params;

    % ----- AMPLIFIER: Voltage command -> Motor voltage -----
    V_cmd = V_cmd_fn(t);
    V_m = p.K_a * V_cmd;

    % Saturation: amplifier output clipping
    V_m = max(-p.V_sat, min(p.V_sat, V_m));

    % ----- CART FORCE: Good ref Eq. 2.3 (L_m = 0) -----
    %   F_c = (eta_g * K_g * k_t) / (R_m * r_mp) * (-K_g * k_m * x_c_dot / r_mp + eta_m * V_m)
    F_c = (p.eta_g * p.K_g * p.k_t) / (p.R_m * p.r_mp) ...
        * (-p.K_g * p.k_m * x_c_dot / p.r_mp + p.eta_m * V_m);

    % ----- COUPLED EQUATIONS OF MOTION (Good ref, page 6) -----
    %
    % 1st Lagrange equation (cart, q1 = x_c):
    %   m_c*x_c'' - m_c*D_t*alpha'' - m_c*x_c*alpha_dot^2 + g*m_c*sin(alpha) = F_c - B_eq*x_c_dot
    %
    % 2nd Lagrange equation (seesaw, q2 = alpha):
    %   m_c*alpha''*x_c^2 + 2*m_c*x_c_dot*x_c*alpha_dot + g*m_c*cos(alpha)*x_c - m_c*D_t*x_c''
    %   + (J_sw + m_c*D_t^2)*alpha'' + g*(-m_c*D_t - m_sw*D_c)*sin(alpha) = -B_sw*alpha_dot
    %
    % Rearranged into mass matrix form [M][q''] = [RHS]:
    %   [m_c,       -m_c*D_t ] [x_c'' ]   [RHS_cart ]
    %   [-m_c*D_t,   J_total ] [alpha''] = [RHS_alpha]

    J_total = p.J_pivot + p.M_c * (x_c^2 + p.D_T^2);

    % RHS of cart equation
    RHS_cart = F_c - p.B_eq * x_c_dot ...
             + p.M_c * x_c * alpha_dot^2 ...     % centrifugal
             - p.M_c * p.g * sin(alpha);          % gravity along beam

    % RHS of seesaw equation
    RHS_alpha = -2 * p.M_c * x_c * x_c_dot * alpha_dot ...  % Coriolis
              - p.M_c * p.g * cos(alpha) * x_c ...           % cart weight restoring
              + (p.M_c * p.D_T + p.M_SW * p.D_C) * p.g * sin(alpha) ... % CoG destabilizing
              - p.B_SW * alpha_dot;                           % pivot friction

    % Solve coupled 2x2 system
    M11 = p.M_c;
    M12 = -p.M_c * p.D_T;
    M21 = -p.M_c * p.D_T;
    M22 = J_total;

    det_M = M11 * M22 - M12 * M21;
    x_c_ddot   = ( M22 * RHS_cart  - M12 * RHS_alpha) / det_M;
    alpha_ddot = (-M21 * RHS_cart  + M11 * RHS_alpha) / det_M;

    % ----- PHYSICAL LIMITS (soft stops) -----
    % Cart travel limits
    if (x_c >= p.x_c_max && x_c_dot > 0)
        x_c_ddot = -5000 * (x_c - p.x_c_max) - 50 * x_c_dot;
    elseif (x_c <= -p.x_c_max && x_c_dot < 0)
        x_c_ddot = -5000 * (x_c + p.x_c_max) - 50 * x_c_dot;
    end

    % Seesaw angle limits
    if (alpha >= p.alpha_max && alpha_dot > 0)
        alpha_ddot = -5000 * (alpha - p.alpha_max) - 50 * alpha_dot;
    elseif (alpha <= -p.alpha_max && alpha_dot < 0)
        alpha_ddot = -5000 * (alpha + p.alpha_max) - 50 * alpha_dot;
    end

    % ----- ASSEMBLE STATE DERIVATIVE -----
    dxdt = [x_c_dot; x_c_ddot; alpha_dot; alpha_ddot];
end

%% ===== Pack Parameters into Struct =====
params = struct();
params.K_a = K_a;
params.V_sat = V_sat;
params.R_m = R_m;
params.k_t = k_t;
params.k_m = k_m;
params.eta_m = eta_m;
params.eta_g = eta_g;
params.K_g = K_g;
params.r_mp = r_mp;
params.M_c = M_c;
params.M_SW = M_SW;
params.g = g;
params.D_T = D_T;
params.D_C = D_C;
params.J_pivot = J_pivot;
params.B_SW = B_SW;
params.B_eq = B_eq;
params.x_c_max = x_c_max;
params.alpha_max = alpha_max;

%% ===== Run Simulation =====
fprintf('\nRunning nonlinear simulation...\n');
fprintf('  Input: %s, amplitude = %.1f V', input_type, A_input);
if strcmp(input_type, 'sine')
    fprintf(', frequency = %.2f Hz', f_input);
end
fprintf('\n  Duration: %.1f s\n', t_final);

% Initial conditions: everything at rest, centered
x0 = [0; 0; 0; 0];  % [x_c; x_c_dot; alpha; alpha_dot]

% Solve ODE (ode45 works fine with L_m = 0, no stiff electrical dynamics)
ode_fn = @(t, x) seesaw_ode(t, x, V_cmd_fn, params);
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3);
[t_sim, x_sim] = ode45(ode_fn, [0 t_final], x0, opts);

fprintf('  Simulation complete. %d time steps.\n', length(t_sim));

%% ===== Extract Outputs =====
x_c_sim       = x_sim(:, 1);          % Cart position [m]
x_c_dot_sim   = x_sim(:, 2);          % Cart velocity [m/s]
alpha_sim     = x_sim(:, 3);          % Seesaw angle [rad]
alpha_dot_sim = x_sim(:, 4);          % Seesaw angular rate [rad/s]
alpha_deg_sim = alpha_sim * 180/pi;   % Seesaw angle [deg]

% Reconstruct input signal and algebraic current
V_cmd_sim = arrayfun(V_cmd_fn, t_sim);
V_m_sim = min(max(V_cmd_sim * K_a, -V_sat), V_sat);
omega_m_sim = K_g * x_c_dot_sim / r_mp;
i_m_sim = (V_m_sim - k_m * omega_m_sim) / R_m;  % algebraic (L_m = 0)

%% ===== Plot Results =====
figure('Name', 'Seesaw Nonlinear Simulation', 'Position', [100 100 900 800]);

subplot(4,1,1);
plot(t_sim, V_cmd_sim, 'b-', 'LineWidth', 1.2);
hold on;
plot(t_sim, V_m_sim, 'r--', 'LineWidth', 1);
ylabel('Voltage [V]');
title('Input Command & Saturated Motor Voltage');
legend('V_{cmd}', 'V_m (saturated)', 'Location', 'best');
grid on;

subplot(4,1,2);
plot(t_sim, alpha_deg_sim, 'b-', 'LineWidth', 1.2);
hold on;
yline(11.5, 'r--', '+11.5\circ limit');
yline(-11.5, 'r--', '-11.5\circ limit');
ylabel('Angle [deg]');
title('Seesaw Tilt Angle');
grid on;

subplot(4,1,3);
plot(t_sim, x_c_sim * 100, 'b-', 'LineWidth', 1.2);  % Convert to cm
ylabel('Position [cm]');
title('Cart Position');
grid on;

subplot(4,1,4);
plot(t_sim, i_m_sim, 'b-', 'LineWidth', 1.2);
hold on;
yline(I_max, 'r--', 'I_{max}');
yline(-I_max, 'r--', '-I_{max}');
ylabel('Current [A]');
xlabel('Time [s]');
title('Motor Current (algebraic, L_m = 0)');
grid on;

sgtitle(sprintf('Seesaw Nonlinear Model: %s input, A=%.1fV, f=%.2fHz', ...
    input_type, A_input, f_input), 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n=== Simulation Summary ===\n');
fprintf('  Max seesaw angle: %.2f deg\n', max(abs(alpha_deg_sim)));
fprintf('  Max cart position: %.2f cm\n', max(abs(x_c_sim))*100);
fprintf('  Max motor current: %.3f A (algebraic)\n', max(abs(i_m_sim)));
fprintf('  Voltage saturated: %s\n', ...
    ternary(any(abs(V_cmd_sim * K_a) > V_sat), 'YES - peaks will be clipped', 'No'));

%% ===== Helper =====
function result = ternary(cond, yes, no)
    if cond
        result = yes;
    else
        result = no;
    end
end
