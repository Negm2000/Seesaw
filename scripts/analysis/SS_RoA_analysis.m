% =========================================================================
% LQR Region of Attraction (RoA) Calculator under Constraints
% =========================================================================

%% 1. Define your System Dynamics (Example: Inverted Pendulum-like system)

% Compute LQR gain K and Lyapunov Matrix P
%[K4, P4, ~] = lqr(A_sw, B_sw, Q4, R4);

%% 2. Define your Constraints
V_nom = 6;          % Input voltage limit: |-Kx| < V_nom
xc_lim = 0.407;        % State 1 limit: |x(1)| < xc_lim
theta_lim = deg2rad(11.66);     % State 2 limit: |x(2)| < theta_lim
xcdot_max = 1.5;     % State 3 limit: |x(3)| < xcdot_max
thetadot_max = inf;  % State 4 limit: |x(4)| < thetadot_max

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

%% 6. VISUALIZE 2D REGION OF ATTRACTION (Cross-section at velocities = 0)
% Extract the 2x2 submatrix corresponding to position states (xc and theta)
% Assuming P4 is the Lyapunov matrix from the LQR design
P_2D = P4(1:2, 1:2);

% Define points for a unit circle
alpha_pts = linspace(0, 2*pi, 200);
circle_pts = [cos(alpha_pts); sin(alpha_pts)];

% Transform the circle into the ellipse defined by x_2D' * P_2D * x_2D = c_max
% Using the eigenvalue decomposition: P_2D = V * D * V'
[V, D] = eig(P_2D);
% The semi-axes lengths are sqrt(c_max ./ diag(D))
ellipse_pts = V * diag(sqrt(c_max ./ diag(D))) * circle_pts;

% -------------------------------------------------------------------------
% Generate the Plot
% -------------------------------------------------------------------------
figure('Name', 'RoA 2D Cross-Section');
hold on; grid on;

% 1. Plot the RoA Ellipse
fill(ellipse_pts(1,:), ellipse_pts(2,:), [0.2 0.6 1], 'FaceAlpha', 0.4, ...
    'EdgeColor', 'b', 'LineWidth', 1.5, 'DisplayName', 'RoA Estimate ($x^T P x \leq c_{\max}$)');

% 2. Plot the state constraints bounding box (|xc| < xc_lim, |theta| < theta_lim)
plot([-xc_lim, xc_lim, xc_lim, -xc_lim, -xc_lim], ...
     [-theta_lim, -theta_lim, theta_lim, theta_lim, -theta_lim], ...
     'r--', 'LineWidth', 1.5, 'DisplayName', 'State Constraints');

% 3. Plot the input constraints (|Kx| < V_nom) projected onto 2D plane
% Assuming x3 = 0, x4 = 0, the input constraint is |K4(1)*xc + K4(2)*theta| <= V_nom
% Solving for theta: theta = (+/- V_nom - K4(1)*xc) / K4(2)
xc_range = linspace(-xc_lim*1.5, xc_lim*1.5, 100);
theta_u_pos = (V_nom - K4(1)*xc_range) / K4(2);
theta_u_neg = (-V_nom - K4(1)*xc_range) / K4(2);

plot(xc_range, theta_u_pos, 'k-.', 'LineWidth', 1.2, 'DisplayName', 'Input Saturation ($+V_{nom}$)');
plot(xc_range, theta_u_neg, 'k-.', 'LineWidth', 1.2, 'DisplayName', 'Input Saturation ($-V_{nom}$)');

% -------------------------------------------------------------------------
% Formatting
% -------------------------------------------------------------------------
xlabel('Cart Position $x_c$ [m]', 'Interpreter', 'latex');
ylabel('Pendulum Angle $\theta$ [rad]', 'Interpreter', 'latex');
title('2D Cross-Section of Region of Attraction ($\dot{x}_c = 0, \dot{\theta} = 0$)', 'Interpreter', 'latex');

% Set axis limits dynamically based on constraints to ensure a good view
xlim([-xc_lim*1.2, xc_lim*1.2]);
ylim([-theta_lim*1.2, theta_lim*1.2]);

% Clean up Legend
legend('Interpreter', 'latex', 'Location', 'best');

%% 7. LQI REGION OF ATTRACTION ANALYSIS
% The LQI system has 5 states: x_c, theta, x_c_dot, theta_dot, and theta_i (integral)

thetai_max = inf; % Unconstrained integral state

% Formulate Constraints in the form |a_i' * x| <= b_i for 5 states
A_mat_5 = [
    K5;               % Input constraint: |-K5x| <= V_nom
    1, 0, 0, 0, 0;    % State 1: xc
    0, 1, 0, 0, 0;    % State 2: theta
    0, 0, 1, 0, 0;    % State 3: xcdot
    0, 0, 0, 1, 0;    % State 4: thetadot
    0, 0, 0, 0, 1     % State 5: theta_integral
];

b_mat_5 = [V_nom; xc_lim; theta_lim; xcdot_max; thetadot_max; thetai_max];

% Calculate Critical c Values for LQI
num_constraints_5 = length(b_mat_5);
c_values_5 = zeros(num_constraints_5, 1);
P5_inv = inv(P5); 

for i = 1:num_constraints_5
    a_i = A_mat_5(i, :)';
    b_i = b_mat_5(i);
    
    % For infinite limits, c_value is naturally infinite (MATLAB handles Inf^2 / x = Inf)
    c_values_5(i) = (b_i^2) / (a_i' * P5_inv * a_i);
end

[c_max_5, limiting_idx_5] = min(c_values_5);

% Display LQI Results
constraint_names_5 = {'Input (-K5x)', 'State x(1)', 'State x(2)', 'State x(3)', 'State x(4)', 'State x(5) (int)'};

fprintf('\n--- LQI Constraint Analysis ---\n');
for i = 1:num_constraints_5
    fprintf('%s limit allows max c = %10.4f\n', pad(constraint_names_5{i}, 15), c_values_5(i));
end
fprintf('---------------------------\n');
fprintf('The limiting constraint is: %s\n', constraint_names_5{limiting_idx_5});
fprintf('Largest Ellipsoid Scaling Parameter (c5*): %f\n', c_max_5);


%% 8. VISUALIZATIONS (Separated Figures)

% Extract 2D Submatrices for both LQR and LQI
P4_2D = P4(1:2, 1:2);
P5_2D = P5(1:2, 1:2);

% Define points for a unit circle
alpha_pts = linspace(0, 2*pi, 200);
circle_pts = [cos(alpha_pts); sin(alpha_pts)];

% Calculate LQR Ellipse
[V4, D4] = eig(P4_2D);
ellipse_pts_4 = V4 * diag(sqrt(c_max ./ diag(D4))) * circle_pts;

% Calculate LQI Ellipse
[V5, D5] = eig(P5_2D);
ellipse_pts_5 = V5 * diag(sqrt(c_max_5 ./ diag(D5))) * circle_pts;

% X-axis range for input saturation lines
xc_range = linspace(-xc_lim*1.5, xc_lim*1.5, 100);

% LQR Input Saturation Lines
theta_u_pos_4 = (V_nom - K4(1)*xc_range) / K4(2);
theta_u_neg_4 = (-V_nom - K4(1)*xc_range) / K4(2);

% LQI Input Saturation Lines
theta_u_pos_5 = (V_nom - K5(1)*xc_range) / K5(2);
theta_u_neg_5 = (-V_nom - K5(1)*xc_range) / K5(2);

% -------------------------------------------------------------------------
% FIGURE 1: LQR Region of Attraction (Isolated)
% -------------------------------------------------------------------------
figure('Name', 'LQR RoA 2D Cross-Section');
hold on; grid on;

fill(ellipse_pts_4(1,:), ellipse_pts_4(2,:), [0.2 0.6 1], 'FaceAlpha', 0.4, ...
    'EdgeColor', 'b', 'LineWidth', 1.5, 'DisplayName', 'LQR RoA ($c_4^*$)');

plot([-xc_lim, xc_lim, xc_lim, -xc_lim, -xc_lim], ...
     [-theta_lim, -theta_lim, theta_lim, theta_lim, -theta_lim], ...
     'k--', 'LineWidth', 1.5, 'DisplayName', 'State Constraints');

plot(xc_range, theta_u_pos_4, 'b-.', 'LineWidth', 1.2, 'DisplayName', 'Input Saturation ($+V_{nom}$)');
plot(xc_range, theta_u_neg_4, 'b-.', 'LineWidth', 1.2, 'HandleVisibility','off');

xlabel('Cart Position $x_c$ [m]', 'Interpreter', 'latex');
ylabel('Pendulum Angle $\theta$ [rad]', 'Interpreter', 'latex');
title('LQR Region of Attraction ($\dot{x}_c = 0, \dot{\theta} = 0$)', 'Interpreter', 'latex');
xlim([-xc_lim*1.2, xc_lim*1.2]);
ylim([-theta_lim*1.5, theta_lim*1.5]);
legend('Interpreter', 'latex', 'Location', 'best');

% -------------------------------------------------------------------------
% FIGURE 2: LQI Region of Attraction (Isolated)
% -------------------------------------------------------------------------
figure('Name', 'LQI RoA 2D Cross-Section');
hold on; grid on;

fill(ellipse_pts_5(1,:), ellipse_pts_5(2,:), [1 0.4 0.4], 'FaceAlpha', 0.4, ...
    'EdgeColor', 'r', 'LineWidth', 1.5, 'DisplayName', 'LQI RoA ($c_5^*$)');

plot([-xc_lim, xc_lim, xc_lim, -xc_lim, -xc_lim], ...
     [-theta_lim, -theta_lim, theta_lim, theta_lim, -theta_lim], ...
     'k--', 'LineWidth', 1.5, 'DisplayName', 'State Constraints');

plot(xc_range, theta_u_pos_5, 'r-.', 'LineWidth', 1.2, 'DisplayName', 'Input Saturation ($+V_{nom}$)');
plot(xc_range, theta_u_neg_5, 'r-.', 'LineWidth', 1.2, 'HandleVisibility','off');

xlabel('Cart Position $x_c$ [m]', 'Interpreter', 'latex');
ylabel('Pendulum Angle $\theta$ [rad]', 'Interpreter', 'latex');
title('LQI Region of Attraction ($\dot{x}_c = 0, \dot{\theta} = 0, \int\theta = 0$)', 'Interpreter', 'latex');
xlim([-xc_lim*1.2, xc_lim*1.2]);
ylim([-theta_lim*1.5, theta_lim*1.5]);
legend('Interpreter', 'latex', 'Location', 'best');

% -------------------------------------------------------------------------
% FIGURE 3: Comparative Plot (Overlay)
% -------------------------------------------------------------------------
figure('Name', 'Comparative RoA 2D Cross-Section');
hold on; grid on;

fill(ellipse_pts_4(1,:), ellipse_pts_4(2,:), [0.2 0.6 1], 'FaceAlpha', 0.2, ...
    'EdgeColor', 'b', 'LineWidth', 1.5, 'DisplayName', 'LQR RoA');

fill(ellipse_pts_5(1,:), ellipse_pts_5(2,:), [1 0.4 0.4], 'FaceAlpha', 0.4, ...
    'EdgeColor', 'r', 'LineWidth', 1.5, 'DisplayName', 'LQI RoA');

plot([-xc_lim, xc_lim, xc_lim, -xc_lim, -xc_lim], ...
     [-theta_lim, -theta_lim, theta_lim, theta_lim, -theta_lim], ...
     'k--', 'LineWidth', 1.5, 'DisplayName', 'State Constraints');

plot(xc_range, theta_u_pos_4, 'b-.', 'LineWidth', 1.2, 'DisplayName', 'LQR Saturation');
plot(xc_range, theta_u_neg_4, 'b-.', 'LineWidth', 1.2, 'HandleVisibility','off');

plot(xc_range, theta_u_pos_5, 'r-.', 'LineWidth', 1.2, 'DisplayName', 'LQI Saturation');
plot(xc_range, theta_u_neg_5, 'r-.', 'LineWidth', 1.2, 'HandleVisibility','off');

xlabel('Cart Position $x_c$ [m]', 'Interpreter', 'latex');
ylabel('Pendulum Angle $\theta$ [rad]', 'Interpreter', 'latex');
title('Comparative 2D RoA ($\dot{x}_c = 0, \dot{\theta} = 0, \int\theta = 0$)', 'Interpreter', 'latex');
xlim([-xc_lim*1.2, xc_lim*1.2]);
ylim([-theta_lim*1.5, theta_lim*1.5]);
legend('Interpreter', 'latex', 'Location', 'best');