% =========================================================================
% LQR Region of Attraction (RoA) Calculator under Constraints
% =========================================================================

%% 1. Define your System Dynamics (Example: Inverted Pendulum-like system)

% Compute LQR gain K and Lyapunov Matrix P
%[K4, P4, ~] = lqr(A_sw, B_sw, Q4, R4);

%% 2. Define your Constraints
%V_nom = 6;          % Input voltage limit: |-Kx| < V_nom
xc_lim = 0.407;        % State 1 limit: |x(1)| < xc_lim
theta_lim = deg2rad(11.66);     % State 2 limit: |x(2)| < theta_lim
xcdot_max = 1.5;     % State 3 limit: |x(3)| < xcdot_max
thetadot_max = 0.5;  % State 4 limit: |x(4)| < thetadot_max

%% 3. Formulate Constraints in the form |a_i' * x| <= b_i
% Each row of A_mat represents an a_i vector
A_mat = [
    K4;               % Input constraint: |-Kx| <= V_nom -> |Kx| <= V_nom
    1, 0, 0, 0;      % State 1 constraint
    0, 1, 0, 0;      % State 2 constraint
    0, 0, 1, 0;      % State 3 constraint
    0, 0, 0, 1       % State 4 constraint
];

b_mat = [V_nom; xc_lim; theta_lim; xcdot_max; thetadot_max];

%% 4. Calculate Critical c Values
num_constraints = length(b_mat);
c_values = zeros(num_constraints, 1);
P_inv = inv(P4); % Invert P once to save computational time

for i = 1:num_constraints
    a_i = A_mat(i, :)'; % Extract a_i as a column vector
    b_i = b_mat(i);
    
    % Apply the ellipsoid-bounding formula
    c_values(i) = (b_i^2) / (a_i' * P_inv * a_i);
end

% The true RoA boundary is dictated by the most restrictive constraint
[c_max, limiting_idx] = min(c_values);

%% 5. Display Results
constraint_names = {'Input (-Kx)', 'State x(1)', 'State x(2)', 'State x(3)', 'State x(4)'};

fprintf('--- Constraint Analysis ---\n');
for i = 1:num_constraints
    fprintf('%s limit allows max c = %10.4f\n', pad(constraint_names{i}, 15), c_values(i));
end
fprintf('---------------------------\n');
fprintf('The limiting constraint is: %s\n', constraint_names{limiting_idx});
fprintf('Largest Ellipsoid Scaling Parameter (c*): %f\n', c_max);
fprintf('Your estimated Region of Attraction is: { x | x'' * P * x <= %f }\n', c_max);