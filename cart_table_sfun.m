function cart_table_sfun(block)
%CART_TABLE_SFUN Level-2 MATLAB S-Function for IP02 cart on flat table.
%  States: [x_c, x_c_dot]
%  Input:  V_cmd (voltage command)
%  Output: [x_c, x_c_dot, i_m, V_m, F_c]
%
%  Reduced motor model (L_m = 0): F_c from Good ref Eq. 2.3.
%  Requires seesaw_params.m to be run first (parameters in base workspace).

    setup(block);
end

%% ========== SETUP ==========
function setup(block)
    block.NumInputPorts  = 1;   % V_cmd
    block.NumOutputPorts = 1;   % [x_c, x_c_dot, i_m, V_m, F_c]

    block.InputPort(1).Dimensions        = 1;
    block.InputPort(1).DirectFeedthrough = true;  % F_c depends directly on V_cmd
    block.InputPort(1).SamplingMode      = 'Sample';

    block.OutputPort(1).Dimensions   = 5;
    block.OutputPort(1).SamplingMode = 'Sample';

    block.NumContStates = 2;    % [x_c, x_c_dot] -- no electrical state
    block.SampleTimes   = [0 0]; % Continuous

    block.SetAccelRunOnTLC(false);
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Output);
    block.RegBlockMethod('Derivatives',          @Derivatives);
end

%% ========== INITIAL CONDITIONS ==========
function InitConditions(block)
    block.ContStates.Data = zeros(2, 1); % [x_c=0, x_c_dot=0]
end

%% ========== OUTPUTS ==========
function Output(block)
    x = block.ContStates.Data;
    p = getParams();

    x_c_dot = x(2);

    % Amplifier + saturation
    V_cmd = block.InputPort(1).Data;
    V_m   = max(-p.V_sat, min(p.V_sat, p.K_a * V_cmd));

    % Cart force: Good ref Eq. 2.3 (reduced model, L_m = 0)
    %   F_c = (eta_g * K_g * k_t) / (R_m * r_mp) * (-K_g * k_m * x_c_dot / r_mp + eta_m * V_m)
    F_c = (p.eta_g * p.K_g * p.k_t) / (p.R_m * p.r_mp) ...
        * (-p.K_g * p.k_m * x_c_dot / p.r_mp + p.eta_m * V_m);

    % Algebraic motor current (for monitoring only): i_m = (V_m - k_m*omega_m) / R_m
    omega_m = p.K_g * x_c_dot / p.r_mp;
    i_m = (V_m - p.k_m * omega_m) / p.R_m;

    block.OutputPort(1).Data = [x(1); x(2); i_m; V_m; F_c];
end

%% ========== DERIVATIVES ==========
function Derivatives(block)
    x = block.ContStates.Data;
    V_cmd = block.InputPort(1).Data;
    p = getParams();

    x_c     = x(1);
    x_c_dot = x(2);

    % Amplifier + saturation
    V_m = max(-p.V_sat, min(p.V_sat, p.K_a * V_cmd));

    % Cart force: Good ref Eq. 2.3
    F_c = (p.eta_g * p.K_g * p.k_t) / (p.R_m * p.r_mp) ...
        * (-p.K_g * p.k_m * x_c_dot / p.r_mp + p.eta_m * V_m);

    % Cart dynamics (flat table: alpha = 0, no seesaw coupling)
    % Good ref 1st Lagrange eq with alpha=0, alpha_dot=0:
    %   m_c * x_c_ddot = F_c - B_eq * x_c_dot
    x_c_ddot = (F_c - p.B_eq * x_c_dot) / p.M_c;

    % Cart travel limits (smooth soft stops)
    k_stop = 500;  b_stop = 20;
    if x_c > p.x_c_max
        x_c_ddot = x_c_ddot - k_stop*(x_c - p.x_c_max) - b_stop*max(x_c_dot,0);
    elseif x_c < -p.x_c_max
        x_c_ddot = x_c_ddot - k_stop*(x_c + p.x_c_max) - b_stop*min(x_c_dot,0);
    end

    block.Derivatives.Data = [x_c_dot; x_c_ddot];
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
        p_cached.B_eq    = evalin('base', 'B_eq');
        p_cached.x_c_max = evalin('base', 'x_c_max');
    end
    p = p_cached;
end
