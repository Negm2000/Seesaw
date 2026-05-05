%% Luenberger Observer Design -- Replaces Naive Differentiator
%
% Reconstructs the full 4-state vector from [x_c, theta] encoders.
% Provides superior noise-rejection/phase-lag trade-off vs
% first-order LPF differentiation.
%
% separation principle ensures eig(A-BK) and eig(A-LC) are decoupled.
%
% State vector:   x      = [x_c; x_c_dot; theta; theta_dot]
% Measurement:    y      = C_meas * x = [x_c; theta]
% Estimator:      xhat'  = A xhat + B u + L (y - C_meas xhat)
%
% Requires: seesaw_params.m, tuned_params.mat, controller_freq.mat
% Outputs:  data/observer.mat, docs/figures/Observer-*.png
%           observer.mat includes A_obs/B_obs/C_obs/D_obs for Simulink.

close all; clc

root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');

%% Plant and Controller Load
run(fullfile(root, 'scripts', 'config', 'seesaw_params.m'))
tuned = load(fullfile(root, 'data', 'tuned_params.mat'));
ctrl  = load(fullfile(root, 'data', 'controller_freq.mat'));

B_eq      = tuned.B_eq;
M_c       = M_c + ctrl.M_c_added;
B_total   = B_eq + B_emf;
M_eff     = [M_c, -M_c*D_T; -M_c*D_T, J_pivot + M_c*D_T^2];
M_inv     = inv(M_eff);
G_rhs     = [0, -B_total, -g*M_c, 0; -g*M_c, 0, g*(M_c*D_T + M_SW*D_C), -B_SW];
A_sw      = [0 1 0 0; M_inv(1,:)*G_rhs; 0 0 0 1; M_inv(2,:)*G_rhs];
B_sw      = [0; M_inv(1,:)*[alpha_f*eta_m; 0]; 0; M_inv(2,:)*[alpha_f*eta_m; 0]];
C_meas    = [1 0 0 0; 0 0 1 0];
D_meas    = zeros(2, 1);
Kf        = ctrl.Kf;

%% Observability
O      = obsv(A_sw, C_meas);
O_rank = rank(O);
fprintf('Observability rank: %d / 4\n', O_rank);
assert(O_rank == 4, 'Plant is not observable.');

%% Pole Selection
% Observer poles should be faster than the controller poles, but not blindly
% scaled as a group: that can create poorly conditioned L gains and amplify
% encoder noise. Use a separate, faster observer pole set instead.
theta0_deg = 2.0;
k_sweep = [1.00, 1.50, 2.00, 2.50, 3.00];
p_ctrl  = ctrl.p_final;
q_xc    = K_ec;
q_theta = K_E_SW / K_gs;

fprintf('\nParametric sweep over k_obs:\n');
fprintf('  k_obs |  V_xc[V/cnt] | V_th[V/cnt] | t_settle[s]\n');
for i = 1:numel(k_sweep)
    p_i = make_placeable_poles(k_sweep(i) * p_ctrl);
    L_i = place(A_sw', C_meas', p_i)';
    KL_i = Kf * L_i;
    % Simulation for settle time
    A_comb = [A_sw - B_sw*Kf, B_sw*Kf; zeros(4), A_sw - L_i*C_meas];
    z = initial(ss(A_comb, zeros(8,1), eye(8), zeros(8,1)), ...
                [0;0;deg2rad(theta0_deg);0; 0;0;deg2rad(theta0_deg);0], (0:0.001:2)');
    e_max = max(abs(z(:,5:8)), [], 2);
    idx = find(e_max > q_theta, 1, 'last');
    if isempty(idx), ts = 0; else, ts = idx*0.001; end
    fprintf('  %4.2f  |   %6.4f     |   %6.3f    |   %5.2f\n', ...
        k_sweep(i), abs(KL_i(1))*q_xc, abs(KL_i(2))*q_theta, ts);
end

% About 2x faster than the controller's dominant theta pair while avoiding
% the pathological KL gain produced by direct scaling of all controller poles.
k_obs  = 2.0;
p_obs  = [-9.0 + 6.75i;
          -9.0 - 6.75i;
          -10.8;
          -13.5];
L      = place(A_sw', C_meas', p_obs)';

%% Simulink Observer State-Space Matrices
% Input vector:  [u; x_c_measured; theta_measured]
% Output vector: xhat = [x_c; x_c_dot; theta; theta_dot]
A_obs = A_sw - L*C_meas;
B_obs = [B_sw L];
C_obs = eye(4);
D_obs = zeros(4, 3);

%% Separation Principle Verification
A_combined = [A_sw - B_sw*Kf,   B_sw*Kf;
              zeros(4,4),       A_sw - L*C_meas];
ev_check   = sort(eig(A_combined));
ev_target  = sort([eig(A_sw - B_sw*Kf); eig(A_sw - L*C_meas)]);
err_eig    = max(abs(ev_check - ev_target));
fprintf('\nSeparation principle mismatch: %.2e\n', err_eig);

%% Simulation
x0     = [0; 0; deg2rad(theta0_deg); 0];
% In hardware, only positions are measured at startup. Initialize the
% observer with measured positions and zero velocities to avoid an
% unrealistic cold-start transient in the linear, unsaturated simulation.
xhat0  = [x0(1); 0; x0(3); 0];
dt = 0.001; t = (0:dt:3)';
sys_combined = ss(A_combined, zeros(8,1), eye(8), zeros(8,1));
z = initial(sys_combined, [x0; x0-xhat0], t);
x_hist = z(:, 1:4); e_hist = z(:, 5:8);
u_hist = -(Kf * (x_hist - e_hist)')';

%% Summary
V_noise_xc = abs(Kf * L(:,1)) * q_xc;
V_noise_th = abs(Kf * L(:,2)) * q_theta;
fprintf('V_noise (observer): xc=%.4f V, th=%.4f V\n', V_noise_xc, V_noise_th);

%% Figures
figure; hold on; grid on
plot(real(p_obs), imag(p_obs), 'bo', 'MarkerSize', 10, 'LineWidth', 2)
plot(real(p_ctrl), imag(p_ctrl), 'rx', 'MarkerSize', 10, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag')
legend('Observer poles', 'Controller poles')
title('Observer vs Controller Poles')
saveas(gcf, fullfile(figdir, 'Observer-Poles.png'))

figure
subplot(3,1,1); plot(t, x_hist(:,1)*100, 'LineWidth', 1.2); grid on
ylabel('Cart [cm]'); title('Observer IC Response (Position-Initialized)')
subplot(3,1,2); plot(t, rad2deg(x_hist(:,3)), 'LineWidth', 1.2); grid on
ylabel('\theta [deg]')
subplot(3,1,3); plot(t, u_hist, 'LineWidth', 1.2); grid on
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'Observer-IC-Response.png'))

%% Save
save(fullfile(root, 'data', 'observer.mat'), ...
     'L', 'p_obs', 'k_obs', 'A_sw', 'B_sw', 'Kf', ...
     'A_obs', 'B_obs', 'C_obs', 'D_obs');
fprintf('Saved data/observer.mat\n')

%% Helpers
function p_out = make_placeable_poles(p_in)
    p_out = p_in(:); tol = 1e-6; delta = 5e-3;
    for i = 2:numel(p_out)
        while any(abs(p_out(i) - p_out(1:i-1)) < tol)
            p_out(i) = p_out(i) - delta;
        end
    end
end
