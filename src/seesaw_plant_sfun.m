function seesaw_plant_sfun(block)
%SEESAW_PLANT_SFUN Level-2 MATLAB S-Function for cart on seesaw.
%  States: [x_c, x_c_dot, alpha, alpha_dot]
%  Input:  V_cmd (voltage command)
%  Output: [x_c, x_c_dot, alpha, alpha_dot, i_m, V_m, F_c]
%
%  Full nonlinear coupled Lagrangian model, exactly matching the
%  Quanser Seesaw Laboratory Guide ("Good ref") equations.
%  Reduced motor model (L_m = 0): F_c from Good ref Eq. 2.3.
%  Requires seesaw_params.m to be run first (parameters in base workspace).

    setup(block);
end

%% ========== SETUP ==========
function setup(block)
    block.NumInputPorts  = 1;   % V_cmd
    block.NumOutputPorts = 1;   % [x_c, x_c_dot, alpha, alpha_dot, i_m, V_m, F_c]

    block.InputPort(1).Dimensions        = 1;
    block.InputPort(1).DirectFeedthrough = true;  % F_c depends directly on V_cmd
    block.InputPort(1).SamplingMode      = 'Sample';

    block.OutputPort(1).Dimensions   = 7;
    block.OutputPort(1).SamplingMode = 'Sample';

    block.NumContStates = 4;    % [x_c, x_c_dot, alpha, alpha_dot] -- no electrical state
    block.SampleTimes   = [0 0]; % Continuous

    block.SetAccelRunOnTLC(false);
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Output);
    block.RegBlockMethod('Derivatives',          @Derivatives);
end

%% ========== INITIAL CONDITIONS ==========
function InitConditions(block)
    block.ContStates.Data = zeros(4, 1);
end

%% ========== OUTPUTS ==========
function Output(block)
    x = block.ContStates.Data;
    p = getParams();

    x_c_dot = x(2);

    V_cmd = block.InputPort(1).Data;
    V_m   = max(-p.V_sat, min(p.V_sat, p.K_a * V_cmd));

    % Cart force: Good ref Eq. 2.3
    F_c = (p.eta_g * p.K_g * p.k_t) / (p.R_m * p.r_mp) ...
        * (-p.K_g * p.k_m * x_c_dot / p.r_mp + p.eta_m * V_m);

    % Algebraic motor current (for monitoring only)
    omega_m = p.K_g * x_c_dot / p.r_mp;
    i_m = (V_m - p.k_m * omega_m) / p.R_m;

    block.OutputPort(1).Data = [x(1); x(2); x(3); x(4); i_m; V_m; F_c];
end

%% ========== DERIVATIVES ==========
function Derivatives(block)
    x = block.ContStates.Data;
    V_cmd = block.InputPort(1).Data;
    p = getParams();

    x_c       = x(1);
    x_c_dot   = x(2);
    alpha     = x(3);
    alpha_dot = x(4);

    % ---- Amplifier + saturation ----
    V_m = max(-p.V_sat, min(p.V_sat, p.K_a * V_cmd));

    % ---- Cart force: Good ref Eq. 2.3 (reduced model, L_m = 0) ----
    %   F_c = (eta_g * K_g * k_t) / (R_m * r_mp) * (-K_g * k_m * x_c_dot / r_mp + eta_m * V_m)
    F_c = (p.eta_g * p.K_g * p.k_t) / (p.R_m * p.r_mp) ...
        * (-p.K_g * p.k_m * x_c_dot / p.r_mp + p.eta_m * V_m);

    % ---- Coupled equations of motion (Good ref, page 6) ----
    %
    % 1st Lagrange equation (cart, q1 = x_c):
    %   m_c*x_c'' - m_c*D_t*alpha'' - m_c*x_c*alpha_dot^2 + g*m_c*sin(alpha) = F_c - B_eq*x_c_dot
    %
    % 2nd Lagrange equation (seesaw, q2 = alpha):
    %   m_c*alpha''*x_c^2 + 2*m_c*x_c_dot*alpha_dot*x_c + g*m_c*cos(alpha)*x_c - m_c*D_t*x_c''
    %   + (J_sw + m_c*D_t^2)*alpha'' + g*(-m_c*D_t*sin(alpha) - m_sw*D_c*sin(alpha)) = -B_sw*alpha_dot
    %
    % Rearranged into mass matrix form [M][q''] = [RHS]:
    %   [m_c,       -m_c*D_t ] [x_c'' ]   [RHS_cart ]
    %   [-m_c*D_t,   J_total ] [alpha''] = [RHS_alpha]

    J_total = p.J_pivot + p.M_c * (x_c^2 + p.D_T^2);

    % RHS of cart equation (everything except m_c*x_c'' and -m_c*D_t*alpha'')
    RHS_cart = F_c - p.B_eq * x_c_dot ...
             + p.M_c * x_c * alpha_dot^2 ...     % centrifugal
             - p.M_c * p.g * sin(alpha);          % gravity along beam

    % RHS of seesaw equation (everything except J_total*alpha'' and -m_c*D_t*x_c'')
    RHS_alpha = -2 * p.M_c * x_c * x_c_dot * alpha_dot ...  % Coriolis
              - p.M_c * p.g * cos(alpha) * x_c ...           % cart weight restoring
              + (p.M_c * p.D_T + p.M_SW * p.D_C) * p.g * sin(alpha) ... % CoG destabilizing
              - p.B_SW * alpha_dot;                           % pivot friction

    % Solve coupled 2x2 system
    M11 = p.M_c;
    M12 = -p.M_c * p.D_T;
    M21 = -p.M_c * p.D_T;
    M22 = J_total;

    det_M      = M11 * M22 - M12 * M21;
    x_c_ddot   = ( M22 * RHS_cart  - M12 * RHS_alpha) / det_M;
    alpha_ddot = (-M21 * RHS_cart  + M11 * RHS_alpha) / det_M;

    % ---- Physical limits (smooth soft stops) ----
    k_stop = 500;   b_stop = 20;

    if x_c > p.x_c_max
        x_c_ddot = x_c_ddot - k_stop * (x_c - p.x_c_max) - b_stop * max(x_c_dot, 0);
    elseif x_c < -p.x_c_max
        x_c_ddot = x_c_ddot - k_stop * (x_c + p.x_c_max) - b_stop * min(x_c_dot, 0);
    end

    if alpha > p.alpha_max
        alpha_ddot = alpha_ddot - k_stop * (alpha - p.alpha_max) - b_stop * max(alpha_dot, 0);
    elseif alpha < -p.alpha_max
        alpha_ddot = alpha_ddot - k_stop * (alpha + p.alpha_max) - b_stop * min(alpha_dot, 0);
    end

    block.Derivatives.Data = [x_c_dot; x_c_ddot; alpha_dot; alpha_ddot];
end

%% ========== LOAD PARAMETERS FROM BASE WORKSPACE (cached) ==========
function p = getParams()
    persistent p_cached;
    if isempty(p_cached)
        p_cached.K_a     = evalin('base', 'K_a');
        p_cached.V_sat   = evalin('base', 'V_sat');
        p_cached.R_m     = evalin('base', 'R_m');
        p_cached.k_t     = evalin('base', 'k_t');
        p_cached.k_m     = evalin('base', 'k_m');
        p_cached.eta_m   = evalin('base', 'eta_m');
        p_cached.eta_g   = evalin('base', 'eta_g');
        p_cached.K_g     = evalin('base', 'K_g');
        p_cached.r_mp    = evalin('base', 'r_mp');
        p_cached.M_c     = evalin('base', 'M_c');
        p_cached.M_SW    = evalin('base', 'M_SW');
        p_cached.g       = evalin('base', 'g');
        p_cached.D_T     = evalin('base', 'D_T');
        p_cached.D_C     = evalin('base', 'D_C');
        p_cached.J_pivot = evalin('base', 'J_pivot');
        p_cached.B_SW    = evalin('base', 'B_SW');
        p_cached.B_eq    = evalin('base', 'B_eq');
        p_cached.x_c_max = evalin('base', 'x_c_max');
        p_cached.alpha_max = evalin('base', 'alpha_max');
    end
    p = p_cached;
end
