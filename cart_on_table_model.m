%% cart_on_table_model.m
%  -----------------------------------------------------------------------
%  Phase 1: IP02 Cart on Flat Table (No Seesaw)
%  -----------------------------------------------------------------------
%  PREREQUISITE: Run seesaw_params.m first to load all parameters.
%
%  This models the cart-and-rack system sitting on a flat, fixed table.
%  The seesaw is NOT involved: alpha = 0, alpha_dot = 0 at all times.
%  Use this to validate the motor + cart dynamics before adding the seesaw.
%
%  Reduced motor model (L_m = 0): F_c from Good ref Eq. 2.3.
%
%  State vector:  x = [x_c; x_c_dot]
%    x(1) = x_c       : cart position [m] (0 = centered on track)
%    x(2) = x_c_dot   : cart velocity [m/s]
%
%  Input: V_cmd [V] = voltage command from DAQ
%  -----------------------------------------------------------------------

%% ===== Check that parameters are loaded =====
if ~exist('K_a', 'var')
    error('Run seesaw_params.m first to load system parameters!');
end

%% ===== Simulation Settings =====
t_final = 10;              % Simulation duration [s]

% --- Input Signal ---
input_type = 'sine';        % 'sine', 'step', or 'chirp'
A_input    = 3.0;           % Amplitude of V_cmd [V]
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

%% ===== Nonlinear ODE: Cart on Flat Table =====

function dxdt = cart_table_ode(t, x, V_cmd_fn, params)
    % Unpack state
    x_c     = x(1);   % cart position [m]
    x_c_dot = x(2);   % cart velocity [m/s]

    p = params;

    % ----- AMPLIFIER -----
    V_cmd = V_cmd_fn(t);
    V_m = p.K_a * V_cmd;
    V_m = max(-p.V_sat, min(p.V_sat, V_m));

    % ----- CART FORCE: Good ref Eq. 2.3 (L_m = 0) -----
    %   F_c = (eta_g * K_g * k_t) / (R_m * r_mp) * (-K_g * k_m * x_c_dot / r_mp + eta_m * V_m)
    F_c = (p.eta_g * p.K_g * p.k_t) / (p.R_m * p.r_mp) ...
        * (-p.K_g * p.k_m * x_c_dot / p.r_mp + p.eta_m * V_m);

    % ----- CART EQUATION OF MOTION (flat table, alpha = 0) -----
    % Good ref 1st Lagrange eq with alpha=0:
    %   m_c * x_c_ddot = F_c - B_eq * x_c_dot
    x_c_ddot = (F_c - p.B_eq * x_c_dot) / p.M_c;

    % ----- CART TRAVEL LIMITS (soft stops) -----
    if (x_c >= p.x_c_max && x_c_dot > 0)
        x_c_ddot = -5000 * (x_c - p.x_c_max) - 50 * x_c_dot;
    elseif (x_c <= -p.x_c_max && x_c_dot < 0)
        x_c_ddot = -5000 * (x_c + p.x_c_max) - 50 * x_c_dot;
    end

    % ----- ASSEMBLE STATE DERIVATIVE -----
    dxdt = [x_c_dot; x_c_ddot];
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
params.B_eq = B_eq;
params.x_c_max = x_c_max;

%% ===== Run Simulation =====
fprintf('\n--- Phase 1: Cart on Flat Table ---\n');
fprintf('  Input: %s, amplitude = %.1f V', input_type, A_input);
if strcmp(input_type, 'sine')
    fprintf(', frequency = %.2f Hz', f_input);
end
fprintf('\n  Duration: %.1f s\n', t_final);

% Initial conditions: everything at rest, centered
x0 = [0; 0];  % [x_c; x_c_dot]

% Solve ODE
ode_fn = @(t, x) cart_table_ode(t, x, V_cmd_fn, params);
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3);
[t_sim, x_sim] = ode45(ode_fn, [0 t_final], x0, opts);

fprintf('  Simulation complete. %d time steps.\n', length(t_sim));

%% ===== Extract Outputs =====
x_c_sim     = x_sim(:, 1);          % Cart position [m]
x_c_dot_sim = x_sim(:, 2);          % Cart velocity [m/s]

% Reconstruct input signal and algebraic current
V_cmd_sim = arrayfun(V_cmd_fn, t_sim);
omega_m_sim = K_g * x_c_dot_sim / r_mp;
V_m_sim = min(max(V_cmd_sim * K_a, -V_sat), V_sat);
i_m_sim = (V_m_sim - k_m * omega_m_sim) / R_m;  % algebraic (L_m = 0)

%% ===== Plot Results =====
figure('Name', 'Phase 1: Cart on Flat Table', 'Position', [100 100 900 700]);

subplot(4,1,1);
plot(t_sim, V_cmd_sim, 'b-', 'LineWidth', 1.2);
hold on;
plot(t_sim, V_m_sim, 'r--', 'LineWidth', 1);
ylabel('Voltage [V]');
title('Input Command & Saturated Motor Voltage');
legend('V_{cmd}', 'V_m (saturated)', 'Location', 'best');
grid on;

subplot(4,1,2);
plot(t_sim, x_c_sim * 100, 'b-', 'LineWidth', 1.2);  % cm
hold on;
yline(x_c_max * 100, 'r--', 'Travel limit');
yline(-x_c_max * 100, 'r--');
ylabel('Position [cm]');
title('Cart Position');
grid on;

subplot(4,1,3);
plot(t_sim, x_c_dot_sim * 100, 'b-', 'LineWidth', 1.2);  % cm/s
ylabel('Velocity [cm/s]');
title('Cart Velocity');
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

sgtitle(sprintf('Phase 1: Cart on Flat Table | %s input, A=%.1fV, f=%.2fHz', ...
    input_type, A_input, f_input), 'FontSize', 14, 'FontWeight', 'bold');

%% ===== Summary =====
fprintf('\n=== Phase 1 Summary ===\n');
fprintf('  Max cart position: %.2f cm\n', max(abs(x_c_sim))*100);
fprintf('  Max cart velocity: %.2f cm/s\n', max(abs(x_c_dot_sim))*100);
fprintf('  Max motor current: %.3f A (algebraic)\n', max(abs(i_m_sim)));
fprintf('  Hit travel limit:  %s\n', ...
    ternary(max(abs(x_c_sim)) > 0.95*x_c_max, 'YES', 'No'));

%% ===== Helper =====
function result = ternary(cond, yes, no)
    if cond
        result = yes;
    else
        result = no;
    end
end
