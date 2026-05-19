% --- CasADi Trajectory Optimization for Seesaw Lift-Up ---
import casadi.*

% 1. Create optimization environment
opti = casadi.Opti();


theta_max = deg2rad(11.66);
xc_max = 0.407;

% 3. Time and Grid Setup
N = 100;                      % Number of control intervals
T = opti.variable();          % Total time is a free variable to optimize
dt = T/N;                     % Time step

% 4. State and Control Variables
X = opti.variable(4, N+1);    % States: [xc; theta; xc_dot; theta_dot]
U = opti.variable(1, N);      % Control: vm

% Initial velocity is a FREE variable we want CasADi to find
v_init = opti.variable();     

% 5. Continuous Dynamics Function (Symbolic)
x  = MX.sym('x', 4);
u  = MX.sym('u', 1);
xc = x(1); th = x(2); xc_dot = x(3); th_dot = x(4);

J_eq = J_pivot + M_total*D_T^2 + M_total*xc^2;
detM = M_e * J_eq - (M_total*D_T)^2;

F1 = alpha_f*u + M_total*xc*th_dot^2 - g*M_total*sin(th) - B_total*xc_dot;
F2 = -g*M_total*xc*cos(th) - 2*M_total*xc_dot*th_dot + g*(M_total*D_T + M_SW*D_C)*sin(th) - B_SW*th_dot;

xc_ddot = (J_eq*F1 + M_total*D_T*F2) / detM;
th_ddot = (M_total*D_T*F1 + M_e*F2) / detM;

f = Function('f', {x, u}, {[xc_dot; th_dot; xc_ddot; th_ddot]});

% 6. Multiple Shooting Loop (RK4 Integration)
for k = 1:N
    k1 = f(X(:,k),         U(:,k));
    k2 = f(X(:,k)+dt/2*k1, U(:,k));
    k3 = f(X(:,k)+dt/2*k2, U(:,k));
    k4 = f(X(:,k)+dt*k3,   U(:,k));
    x_next = X(:,k) + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    
    % Close the gap (Dynamics constraint)
    opti.subject_to(X(:,k+1) == x_next);
end

% 7. Boundary Conditions
% Initial State (At the lift-off point)
opti.subject_to(X(1,1) == 0.057);      % xc
opti.subject_to(X(2,1) == theta_max);  % theta
opti.subject_to(X(3,1) == v_init);     % xc_dot (Our optimized entry speed)
opti.subject_to(X(4,1) == 0);          % theta_dot

% Final State (Inside the RoA - target origin)
opti.subject_to(abs(X(1,end)) < 0.0198);
opti.subject_to(abs(X(2,end)) < 0.0176);
opti.subject_to(abs(X(3,end)) < 0.2454);
opti.subject_to(abs(X(4,end)) < 0.1119);

% 8. Physical and Control Constraints
opti.subject_to(-xc_max <= X(1,:) <= xc_max);
opti.subject_to(-theta_max <= X(2,:) <= theta_max); 
opti.subject_to(-6 <= U <= 6);                  % Control limit
opti.subject_to(0.5 <= T <= 3.0);               % Time bounds
opti.subject_to(v_init >= 0);                   % Must be moving forward

% Slew rate constraint limit (< 40 V/s)
for k = 1:N-1
    opti.subject_to(-40*dt <= U(k+1) - U(k) <= 40*dt);
end

% 9. Objective Function (Cost)
% Minimize control effort, penalize long times, and penalize high entry velocity
cost = sumsqr(U)*dt + 10*T + 75*v_init^2;
opti.minimize(cost);

% 10. Solver Setup and Initial Guesses
opti.solver('ipopt');
opti.set_initial(T, 1.5);
opti.set_initial(X(1,:), linspace(0.057, 0, N+1));
opti.set_initial(X(2,:), linspace(theta_max, 0, N+1));

% 11. Solve!
sol = opti.solve();

% Extract results
t_opt = linspace(0, sol.value(T), N+1);
x_opt = sol.value(X);
u_opt = sol.value(U);
v_required = sol.value(v_init);

fprintf('Optimal Lift-Off Velocity required: %.4f m/s\n', v_required);