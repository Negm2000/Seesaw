%% ========================================================================
% FULL SEESAW LIFT-UP PIPELINE: TRAJECTORY OPTIMIZATION & TV-LQR (LQI READY)
% ========================================================================
clc;
import casadi.*

%% ─── 0. Initialization ─────────────────────────────────
theta_max = deg2rad(11.66);
xc_max    = 0.432;
u_start   = 3;
u_max     = 6;
slew_max  = 30;
max_T1    = 5;
max_T2    = 2;

weight_T1 = 10; 
weight_T2 = 10;
weight_U1 = 5;
weight_U2 = 0.1;
weight_smooth = 50; 

% Tuning for the backward gain computation:

% Depending on the final controller : 
% keep the P4 or take the submatrix of P5
P_end = P5(1:4, 1:4);
K_end = K5(1:4);

P_inv = inv(P_end);

% a) Input constraint: |-Kx| < 6 V
H_u = K_end;
c_u = u_max^2 / (H_u * P_inv * H_u');

% b) Cart position constraint: |xc| < 0.432 m
H_xc = [1, 0, 0, 0];
c_xc = xc_max^2 / (H_xc * P_inv * H_xc');

% c) Angle constraint: |theta| < 11.66 deg
H_theta = [0, 1, 0, 0];
c_theta = theta_max^2 / (H_theta * P_inv * H_theta');

% d) Slew rate constraint: |-K(A-BK)x| < 30 V/s
H_sr = K_end * (A_sw - B_sw * K_end);
c_sr = slew_max^2 / (H_sr * P_inv * H_sr');

% The Region of Attraction limit c_star is the most restrictive (minimum) c
c_array = [c_u, c_xc, c_theta, c_sr];
c_names = {'Voltage Input', 'Cart Position', 'Pendulum Angle', 'Slew Rate'};
[c_star, min_idx] = min(c_array);

fprintf('--- Region of Attraction Bounds ---\n');
fprintf('c for Voltage Input : %10.4f\n', c_u);
fprintf('c for Cart Position : %10.4f\n', c_xc);
fprintf('c for Pendulum Angle: %10.4f\n', c_theta);
fprintf('c for Slew Rate     : %10.4f\n\n', c_sr);

fprintf('Selected c_star: %.4f\n', c_star);
fprintf('The limiting constraint is the **%s**.\n\n', c_names{min_idx});

% Compute Maximal Values of Each Variable within the ROA
% The maximum value of x_i subject to x'*P*x <= c_star is:
% max_x_i = sqrt( c_star * (P^-1)_ii )

% Extract the diagonal of the inverse P matrix
P_inv_diag = diag(P_inv);

% Compute maximums
max_bounds = sqrt(c_star * P_inv_diag);

fprintf('--- Maximum State Values inside V(x) <= c* ---\n');
fprintf('Max Cart Position     (x_c)      : %.4f m\n', max_bounds(1));
fprintf('Max Pendulum Angle    (theta)    : %.4f rad (%.2f deg)\n', max_bounds(2), rad2deg(max_bounds(2)));
fprintf('Max Cart Velocity     (x_c_dot)  : %.4f m/s\n', max_bounds(3));
fprintf('Max Angular Velocity  (theta_dot): %.4f rad/s\n', max_bounds(4));

% Standard LQR Weights for TV-LQR (Phase 2)
Q_tv = diag([1/(0.1^2), 1/(deg2rad(2)^2), 1/(0.15^2), 1/(deg2rad(5)^2)]); 
R_tv = 1/(3^2);

% Phase 1 LQI tuning [xc, xc_dot, int_xc]
Q_lqi = diag([1/(0.05^2), 1/(0.15^2), 1/((0.5*0.1)^2)]); 
%% ========================================================================
% PART 1: CASADI SIMULTANEOUS TRAJECTORY OPTIMIZATION
% ========================================================================
fprintf('=== PART 1: Running CasADi Trajectory Optimization ===\n');

N1 = 250;   
N2 = 250;   
opti = casadi.Opti();

% --- Phase 1 Variables & Dynamics (Cart crawls on table) ---
T1  = opti.variable();            
dt1 = T1 / N1;
S1  = opti.variable(2, N1+1);    
U1  = opti.variable(1, N1);      

s_sym = MX.sym('s', 2);
u_sym = MX.sym('u', 1);
xc_r     = s_sym(1);
xc_dot_r = s_sym(2);

xc_ddot_r = (alpha_f * u_sym - M_total*g*sin(theta_max) - B_total * xc_dot_r) / M_e;
f1 = Function('f1', {s_sym, u_sym}, {[xc_dot_r; xc_ddot_r]});

for k = 1:N1
    k1 = f1(S1(:,k),           U1(:,k));
    k2 = f1(S1(:,k)+dt1/2*k1,  U1(:,k));
    k3 = f1(S1(:,k)+dt1/2*k2,  U1(:,k));
    k4 = f1(S1(:,k)+dt1*k3,    U1(:,k));
    opti.subject_to( S1(:,k+1) == S1(:,k) + dt1/6*(k1 + 2*k2 + 2*k3 + k4) );
end

% --- Phase 2 Variables & Dynamics (Swing-up) ---
T2  = opti.variable();
dt2 = T2 / N2;
X2  = opti.variable(4, N2+1);   
U2  = opti.variable(1, N2);

x_sym  = MX.sym('x', 4);
u_sym2 = MX.sym('u', 1);
xc2    = x_sym(1);  th2    = x_sym(2);
xc_d2  = x_sym(3);  th_d2  = x_sym(4);

J_eq2  = J_pivot + M_total*D_T^2 + M_total*xc2^2;
detM2  = M_e * J_eq2 - (M_total*D_T)^2;

F1_2 = alpha_f*u_sym2 + M_total*xc2*th_d2^2 - g*M_total*sin(th2) - B_total*xc_d2;
F2_2 = -g*M_total*xc2*cos(th2) - 2*M_total*xc_d2*th_d2 ...
       + g*(M_total*D_T + M_SW*D_C)*sin(th2) - B_SW*th_d2;

xc_ddot2 = (J_eq2*F1_2 + M_total*D_T*F2_2) / detM2;
th_ddot2 = (M_total*D_T*F1_2 + M_e*F2_2)   / detM2;
f2 = Function('f2', {x_sym, u_sym2}, {[xc_d2; th_d2; xc_ddot2; th_ddot2]});

for k = 1:N2
    k1 = f2(X2(:,k),           U2(:,k));
    k2 = f2(X2(:,k)+dt2/2*k1,  U2(:,k));
    k3 = f2(X2(:,k)+dt2/2*k2,  U2(:,k));
    k4 = f2(X2(:,k)+dt2*k3,    U2(:,k));
    opti.subject_to( X2(:,k+1) == X2(:,k) + dt2/6*(k1 + 2*k2 + 2*k3 + k4) );
end

% --- Boundary & Junction Constraints ---
opti.subject_to( S1(1,1) == -xc_max );   
opti.subject_to( S1(2,1) == 0 );         
opti.subject_to( abs(X2(1,end)) < 0.75*max_bounds(1) );
opti.subject_to( abs(X2(2,end)) < 0.75*max_bounds(2) );
opti.subject_to( abs(X2(3,end)) < 0.75*max_bounds(3) );
opti.subject_to( abs(X2(4,end)) < 0.75*max_bounds(4) );

% The Junction
opti.subject_to( X2(1,1) == S1(1,end) );   
opti.subject_to( X2(2,1) == theta_max );   
opti.subject_to( X2(3,1) == S1(2,end) );   
opti.subject_to( X2(4,1) == 0 );           
opti.subject_to( U2(1)   == U1(end) );     

% C2 Continuity (Smooth Lift-off)
f2_junc = f2(X2(:,1), U2(1));
opti.subject_to( f2_junc(4) == 0 ); 

% Path Constraints
opti.subject_to( -xc_max <= S1(1,:) <= xc_max );
opti.subject_to( -u_max <= U1 <= u_max );  
opti.subject_to( U1(1) == u_start);  
opti.subject_to( 0.1 <= T1 <= max_T1 );

opti.subject_to( -xc_max <= X2(1,:) <= xc_max );
opti.subject_to( -theta_max <= X2(2,:) <= theta_max );
opti.subject_to( -u_max <= U2 <= u_max );
opti.subject_to( 0.1 <= T2 <= max_T2 );

for k = 1:N1-1
    opti.subject_to( -slew_max*dt1 <= U1(k+1) - U1(k) <= slew_max*dt1 );
end
for k = 1:N2-1
    opti.subject_to( -slew_max*dt2 <= U2(k+1) - U2(k) <= slew_max*dt2 );
end

dU1 = U1(2:end) - U1(1:end-1);
dU2 = U2(2:end) - U2(1:end-1);
cost = weight_U1*sumsqr(U1)*dt1 + weight_U2*sumsqr(U2)*dt2 ...
        + weight_T1*T1 + weight_T2*T2 ...
        + weight_smooth * (sumsqr(dU1)/dt1 + sumsqr(dU2)/dt2);
opti.minimize( cost );

% Solve
opti.solver('ipopt', struct('print_time', false), struct('print_level', 0));
opti.set_initial(T1, 4.0);
opti.set_initial(S1(1,:), linspace(-xc_max, 0.0568, N1+1));
opti.set_initial(S1(2,:), 0);
opti.set_initial(U1, 0);
opti.set_initial(T2, 1.5);
opti.set_initial(X2(1,:), linspace(0.0568, 0, N2+1));
opti.set_initial(X2(2,:), linspace(theta_max, 0, N2+1));
opti.set_initial(X2(3,:), linspace(0, 0, N2+1)); 
opti.set_initial(X2(4,:), 0);
opti.set_initial(U2, 0);

disp('Solving NLP...');
sol = opti.solve();

% Extract
T1_val = sol.value(T1);
T2_val = sol.value(T2);
s1_opt = sol.value(S1);
x2_opt = sol.value(X2);
u1_opt = sol.value(U1);
u2_opt = sol.value(U2);
u_junction = u1_opt(end);

fprintf('Optimization complete! T1 = %.2f s, T2 = %.2f s\n', T1_val, T2_val);

%% ─── Stitch & Export Trajectory Arrays ───────────────────────────────────
t_p1 = linspace(0,          T1_val,           N1+1);
x_p1  = [s1_opt(1,:); theta_max*ones(1,N1+1); s1_opt(2,:); zeros(1,N1+1)]';
u_p1  = [u1_opt, u1_opt(end)]'; % Padded for length

t_p2 = linspace(T1_val,     T1_val + T2_val,  N2+1);
x_p2  = [x2_opt(1,:); x2_opt(2,:); x2_opt(3,:); x2_opt(4,:)]';
u_p2  = [u2_opt, u2_opt(end)]'; % Padded for length

t_total      = [t_p1,       t_p2(2:end)      ];
xc_total     = [s1_opt(1,:),  x2_opt(1,2:end)    ];
vc_total     = [s1_opt(2,:),  x2_opt(3,2:end)    ];
th_total     = [theta_max*ones(1,N1+1), x2_opt(2,2:end)  ];
th_dot_total = [zeros(1,N1+1),          x2_opt(4,2:end)  ];

x_ref_full = [xc_total; th_total; vc_total; th_dot_total];

t_u_1  = t_p1(1:end-1);          
t_u_2  = t_p2(1:end-1);          
t_u    = [t_u_1, t_u_2, t_total(end)];   
u_ff   = [u1_opt, u2_opt, u2_opt(end)];  

fprintf('\n=== WORKSPACE ARRAYS READY ===\n');
fprintf('Phase 1: t_p1, x_p1, u_p1, K_p1\n');
fprintf('Phase 2: t_p2, x_p2, u_p2, K_p2\n');

%% ========================================================================
% PART 2: TWO-PHASE TV-LQR COMPUTATION (HYBRID METHOD)
% ========================================================================
fprintf('\n=== PART 2: Computing Hybrid TV-LQR Gains ===\n');

fprintf('1. Performing Symbolic Linearization for Phase 2...\n');
syms sym_xc sym_th sym_xc_d sym_th_d sym_u real
sym_x = [sym_xc; sym_th; sym_xc_d; sym_th_d];

J_eq2_sym = J_pivot + M_total*D_T^2 + M_total*sym_xc^2;
detM2_sym = M_e * J_eq2_sym - (M_total*D_T)^2;
F1_2_sym = alpha_f*sym_u + M_total*sym_xc*sym_th_d^2 - g*M_total*sin(sym_th) - B_total*sym_xc_d;
F2_2_sym = -g*M_total*sym_xc*cos(sym_th) - 2*M_total*sym_xc_d*sym_th_d ...
           + g*(M_total*D_T + M_SW*D_C)*sin(sym_th) - B_SW*sym_th_d;
sym_xc_ddot2 = (J_eq2_sym*F1_2_sym + M_total*D_T*F2_2_sym) / detM2_sym;
sym_th_ddot2 = (M_total*D_T*F1_2_sym + M_e*F2_2_sym)   / detM2_sym;

f_sym2 = [sym_xc_d; sym_th_d; sym_xc_ddot2; sym_th_ddot2];
A_func2 = matlabFunction(jacobian(f_sym2, sym_x), 'Vars', {sym_x, sym_u});
B_func2 = matlabFunction(jacobian(f_sym2, sym_u), 'Vars', {sym_x, sym_u});

fprintf('2. Solving DRE Backward for Phase 2 (From final SS controller)...\n');

options = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);
DRE_ode2 = @(t, P_vec) dre_dynamics(t, P_vec, t_total, x_ref_full, u_ff, A_func2, B_func2, Q_tv, R_tv);
tspan2 = [t_total(end), T1_val];
[t_dre2, P_raw2] = ode45(DRE_ode2, tspan2, P_end(:), options);

t_K2 = flipud(t_dre2);
K_tv2 = zeros(4, length(t_K2));
for i = 1:length(t_K2)
    P_current = reshape(P_raw2(end-i+1, :), 4, 4);
    x_curr = interp1(t_total, x_ref_full', t_K2(i))';
    u_curr = interp1(t_u, u_ff, t_K2(i))'; 
    B_curr = B_func2(x_curr, u_curr);
    K_tv2(:, i) = (1/R_tv) * B_curr' * P_current;
end

fprintf('3. Computing Augmented LQI for Phase 1 (On Table)...\n');
% State vector: [xc_dot; xc_ddot; integral(xc)]
A_carti = [0, 1, 0; 
         0, -B_total/M_e, 0; 
         1, 0, 0];
B_carti = [0; alpha_f/M_e; 0];

[K_phase1, ~] = lqr(A_carti, B_carti, Q_lqi, R_tv);

% Separate the Proportional/Derivative and Integral gains
K_phase1_pd = [K_phase1(1), 0, K_phase1(2), 0]; 
K_phase1_int = K_phase1(3); 

K_p1  = repmat(K_phase1_pd, length(t_p1), 1); % Constant LQR gains

fprintf('4. Formatting variables for Simulink lookup tables...\n');
t_simulink = (0 : Ts : t_total(end))';

xref_simulink = interp1(t_total, x_ref_full', t_simulink);
uff_simulink  = interp1(t_u, u_ff, t_simulink, 'previous'); 

K_phase2 = zeros(length(t_simulink), 4);
for i = 1:length(t_simulink)
    if t_simulink(i) < T1_val
        K_phase2(i, :) = K_phase1_pd;
    else
        K_phase2(i, 1) = interp1(t_K2, K_tv2(1,:), t_simulink(i), 'linear', 'extrap');
        K_phase2(i, 2) = interp1(t_K2, K_tv2(2,:), t_simulink(i), 'linear', 'extrap');
        K_phase2(i, 3) = interp1(t_K2, K_tv2(3,:), t_simulink(i), 'linear', 'extrap');
        K_phase2(i, 4) = interp1(t_K2, K_tv2(4,:), t_simulink(i), 'linear', 'extrap');
    end
end

K_p2 = zeros(length(t_p2), 4);
for i = 1:4
    % t_K2 starts at T1, so we shift it to 0 for interpolation
    K_p2(:,i) = interp1(t_K2 - T1_val, K_tv2(i,:), t_p2, 'linear', 'extrap');
end


%% ─── PLOTS ───────────────────────────────────────────────────────────────
figure('Name', 'Joint Optimised Trajectory & Gains', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

% Trajectory Subplots
subplot(4,2,1); plot(t_total, xc_total, 'b', 'LineWidth', 1.5); hold on; xline(T1_val, 'r--'); ylabel('$x_c$ [m]'); title('Cart Position'); grid on;
subplot(4,2,3); plot(t_total, vc_total, 'g', 'LineWidth', 1.5); hold on; xline(T1_val, 'r--'); ylabel('$\dot{x}_c$ [m/s]'); title('Cart Velocity'); grid on;
subplot(4,2,5); plot(t_total, gradient(vc_total, t_total), 'm', 'LineWidth', 1.5); hold on; xline(T1_val, 'r--'); ylabel('$\ddot{x}_c$ [m/s²]'); title('Cart Accel (FD)'); grid on;
subplot(4,2,2); plot(t_total, rad2deg(th_total), 'k', 'LineWidth', 1.5); hold on; xline(T1_val, 'r--'); ylabel('$\theta$ [deg]'); title('Seesaw Angle'); grid on;
subplot(4,2,4); plot(t_total, rad2deg(th_dot_total), 'k', 'LineWidth', 1.5); hold on; xline(T1_val, 'r--'); ylabel('$\dot{\theta}$ [deg/s]'); title('Seesaw Rate'); grid on;
subplot(4,2,6); stairs(t_u, u_ff, 'r', 'LineWidth', 1.5); hold on; xline(T1_val, 'r--'); ylabel('$u$ [V]'); title('Feed-Forward'); grid on;

% Gain Schedule Subplot
subplot(4,2,[7 8]);
plot(t_simulink, K_phase2, 'LineWidth', 1.5); hold on;
xline(T1_val, 'k--', 'Lift-Off', 'LabelVerticalAlignment','bottom');
title('Hybrid TV-LQR Gain Schedule');
xlabel('Time [s]'); ylabel('Gain Magnitude');
legend('$K_{x_c}$', '$K_{\theta}$', '$K_{\dot{x}_c}$', '$K_{\dot{\theta}}$', 'Location', 'best');
grid on;

%% ─── FINAL OUTPUT INSTRUCTIONS ───────────────────────────────────────────
fprintf('\n=== PIPELINE COMPLETE ===\n');
fprintf('Lift-off Time (T1): %.3f s\n', T1_val);
fprintf('Total Maneuver Time: %.3f s\n', t_total(end));
fprintf('\n--- Phase 1 LQI Integrator Gain ---\n');
fprintf('Use this constant in your Simulink model for Phase 1:\n');
fprintf('K_int_phase1 = %.4f\n', K_phase1_int);

%% --- HELPER FUNCTION: DRE DYNAMICS ---
function dPdt_vec = dre_dynamics(t, P_vec, t_ref, x_ref, u_ref, A_func, B_func, Q, R)
    P = reshape(P_vec, 4, 4);
    x_t = interp1(t_ref, x_ref', t)';
    u_t = interp1(t_ref, u_ref, t)'; 
    A = A_func(x_t, u_t);
    B = B_func(x_t, u_t);
    dPdt = -(A'*P + P*A - P*B*(1/R)*B'*P + Q);
    dPdt = (dPdt + dPdt') / 2; 
    dPdt_vec = dPdt(:);
end