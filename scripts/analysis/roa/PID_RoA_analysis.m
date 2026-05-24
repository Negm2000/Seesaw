%% Explicit RoA Estimation for Cascade PID (Un-scrambled States)
%% 1. Physical Parameters & System Constants
% --- Physical Plant Parameters ---

a = num_t(1); 
b = num_t(3); 
c = den_t(1); 
d = den_t(3); 

% Auxiliary Plant Parameters (from G_out partial fraction expansion)
gamma = (a * d - b * c) / (c^2);
beta  = d / c;

% Effective proportional/high-frequency gains
k_p_out_eff = Kp_out + Kd_out * N_out;
k_p_in_eff  = Kp_in  + Kd_in  * N_in;

% --- Physical Constraints ---
limit_pos  = 0.407; % [m]
limit_volt = 6.0;   % [V]


%% 2. Explicit Closed-Loop Algebraic Coefficients
% To keep the code clean, we pre-calculate how errors map to states.
% Error Outer: e_theta = -(a/c)*x1 - gamma*x3
% Error Inner: e_xc = c_x1*x1 + c_x3*x3 + Ki_out*z_out + x_d_out
c_x1 = -(1 + k_p_out_eff * (a / c));
c_x3 = -k_p_out_eff * gamma;


%% 3. Construct the Un-Scrambled A_cl Matrix (8x8)
% State Vector ordering:
% x = [ x1: position     (xc)
%       x2: velocity     (d_xc)
%       x3: angle_state1 (internal G_out)
%       x4: angle_state2 (internal G_out)
%       z_out: outer loop integrator
%       x_d_out: outer loop derivative filter
%       z_in: inner loop integrator
%       x_d_in: inner loop derivative filter ]

A_cl = zeros(8,8);

% Row 1: d(x1) = x2
A_cl(1, 2) = 1;

% Row 2: d(x2) = -(M_e/B_total)*x2 + (alpha/B_total)*vm
A_cl(2, 2) = -M_e / B_total;
A_cl(2, 1) = (alpha_f / B_total) * (k_p_in_eff * c_x1);
A_cl(2, 3) = (alpha_f / B_total) * (k_p_in_eff * c_x3);
A_cl(2, 5) = (alpha_f / B_total) * (k_p_in_eff * Ki_out);
A_cl(2, 6) = (alpha_f / B_total) * (k_p_in_eff);
A_cl(2, 7) = (alpha_f / B_total) * (Ki_in);
A_cl(2, 8) = (alpha_f / B_total) * 1;

% Row 3: d(x3) = x4
A_cl(3, 4) = 1;

% Row 4: d(x4) = beta*x3 + x1
A_cl(4, 1) = 1;
A_cl(4, 3) = beta;

% Row 5: d(z_out) = e_theta = -(a/c)*x1 - gamma*x3
A_cl(5, 1) = -a / c;
A_cl(5, 3) = -gamma;

% Row 6: d(x_d_out) = -N_out*x_d_out - Kd_out*N_out^2*e_theta
A_cl(6, 1) = -Kd_out * (N_out^2) * (-a / c);
A_cl(6, 3) = -Kd_out * (N_out^2) * (-gamma);
A_cl(6, 6) = -N_out;

% Row 7: d(z_in) = e_xc
A_cl(7, 1) = c_x1;
A_cl(7, 3) = c_x3;
A_cl(7, 5) = Ki_out;
A_cl(7, 6) = 1;

% Row 8: d(x_d_in) = -N_in*x_d_in - Kd_in*N_in^2*e_xc
A_cl(8, 1) = -Kd_in * (N_in^2) * c_x1;
A_cl(8, 3) = -Kd_in * (N_in^2) * c_x3;
A_cl(8, 5) = -Kd_in * (N_in^2) * Ki_out;
A_cl(8, 6) = -Kd_in * (N_in^2);
A_cl(8, 8) = -N_in;


%% 4. Define Physical Constraint Mapping Vectors (h)
% Position is directly state x1
h_pos = [1; 0; 0; 0; 0; 0; 0; 0];

% Voltage is the full algebraic combination feeding the plant input
h_volt = [ k_p_in_eff * c_x1; ...
           0; ...
           k_p_in_eff * c_x3; ...
           0; ...
           k_p_in_eff * Ki_out; ...
           k_p_in_eff; ...
           Ki_in; ...
           1 ];


%% 5. Optimize Lyapunov Matrix via LMI Lab
fprintf('Optimizing Region of Attraction via native LMI Lab...\n');
setlmis([]);

[W, n, sW] = lmivar(1, [8, 1]); % 8x8 symmetric matrix W = P^-1

% LMI 1: Stability (A_cl*W + W*A_cl' < 0)
lmi_stab = newlmi;
lmiterm([lmi_stab, 1, 1, W], A_cl, 1, 's');

% LMI 2: Position Boundary (h_pos'*W*h_pos <= limit_pos^2)
lmi_pos = newlmi;
lmiterm([lmi_pos, 1, 1, W], h_pos', h_pos);
lmiterm([lmi_pos, 1, 1, 0], -limit_pos^2);

% LMI 3: Voltage Boundary (h_volt'*W*h_volt <= limit_volt^2)
lmi_volt = newlmi;
lmiterm([lmi_volt, 1, 1, W], h_volt', h_volt);
lmiterm([lmi_volt, 1, 1, 0], -limit_volt^2);

% LMI 4: Positive Definiteness (W >= 1e-12 * I)
lmi_posdef = newlmi;
lmiterm([lmi_posdef, 1, 1, W], -1, 1);
lmiterm([lmi_posdef, 1, 1, 0], 1e-12 * eye(8));

% Compile and optimize
lmis = getlmis;
n_dec = decnbr(lmis); 
c_obj = zeros(n_dec, 1);
for j = 1:n_dec
    c_obj(j) = -trace(defcx(lmis, j, W));
end

[copt, xopt] = mincx(lmis, c_obj, [1e-4, 200, 0, 0, 1]);

if isempty(xopt)
    error('LMI optimization failed. Verify your closed-loop poles: %s', num2str(eig(A_cl)'));
end

W_opt = dec2mat(lmis, xopt, W);


%% 6. Extract Physical Boundaries
max_states = sqrt(diag(W_opt));
state_labels = {
    'Position (xc) [m]'
    'Velocity (d_xc) [m/s]'
    'G_out Internal State 1'
    'G_out Internal State 2'
    'Outer Integrator (z_out)'
    'Outer Deriv Filter (x_d_out)'
    'Inner Integrator (z_in)'
    'Inner Deriv Filter (x_d_in)'
};


%% 7. Print Real-World Safe Envelopes
fprintf('\n======================================================\n');
fprintf('        MANUAL EXPLICIT CASCADE PID RoA RESULTS       \n');
fprintf('======================================================\n');
fprintf('Max allowable physical envelope at c* = 1.0:\n\n');
for i = 1:8
    fprintf('  State %d -> %-28s: Max Absolute Value = %.4f\n', ...
            i, state_labels{i}, max_states(i));
end
fprintf('======================================================\n');