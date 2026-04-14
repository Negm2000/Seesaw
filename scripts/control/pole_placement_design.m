%% Pole Placement Controller Design for Quanser SEESAW-E + IP02
%
% State vector:  x = [x_c; x_c_dot; theta; theta_dot]
%   x_c       — cart position along rail [m]
%   x_c_dot   — cart velocity [m/s]
%   theta     — seesaw tilt from horizontal [rad]
%   theta_dot — seesaw angular velocity [rad/s]
% Requires: seesaw_params.m (plant matrices), tuned_params.mat (B_eq)
% Outputs:  controller_freq.mat, all report figures

close all; clc

root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');

%% Load tuned plant
% seesaw_params.m builds A_sw, B_sw using the default cart friction
% B_eq = 5.0 N*s/m.  The frequency-sweep experiment gave B_eq = 6.7217,
% so we patch the three quantities that depend on it:
%   B_total  = B_eq + B_emf        (total cart damping)
%   G_rhs    = [... -B_total ...]   (damping column in the EOM RHS)
%   A_sw     = f(M_inv, G_rhs)      (state matrix rebuilt)
% B_sw is unchanged because the input gain alpha_f*eta_m has no B_eq term.
run(fullfile(root, 'scripts', 'config', 'seesaw_params.m'))

tuned      = load(fullfile(root, 'data', 'tuned_params.mat'));
B_eq       = tuned.B_eq;
B_total    = B_eq + B_emf;
G_rhs(1,2) = -B_total;
A_sw       = [0 1 0 0; M_inv(1,:)*G_rhs; 0 0 0 1; M_inv(2,:)*G_rhs];

poles_ol = sort(eig(A_sw));
fprintf('Tuned B_eq = %.4f\n', B_eq)
fprintf('Open-loop poles: '); fprintf('%.4f  ', poles_ol); fprintf('\n')
fprintf('Controllability rank: %d/4\n\n', rank(ctrb(A_sw, B_sw)))

%% Open-loop pole map
figure; hold on; grid on
plot(real(poles_ol), imag(poles_ol), 'rx', 'MarkerSize', 12, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag'); title('Open-Loop Poles')
saveas(gcf, fullfile(figdir, 'OL-Poles.png'))

%% Simulation setup
theta_0 = deg2rad(4.5);                     % initial tilt disturbance
x0      = [0; 0; theta_0; 0];               % cart at origin, tilted, at rest
t       = 0:0.001:10;

%% ---- Attempt 1: mirror the unstable pole, keep the rest ----
% Simplest stabilising controller: reflect each OL pole into the LHP.
% All poles stay at their natural speeds — only the sign of the
% unstable one (+2.61 → -2.61) changes.
p1 = sort(-abs(poles_ol), 'descend');
[K1, pcl1, y1, u1, m1] = sim_regulator(A_sw, B_sw, p1, x0, t);

figure; hold on; grid on
plot(real(pcl1), imag(pcl1), 'rx', 'MarkerSize', 12, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag'); title('CL Poles -- Attempt 1')
saveas(gcf, fullfile(figdir, 'CL-Poles-Att1.png'))

plot_3panel(t, y1, u1, 'IC Response -- Attempt 1');
saveas(gcf, fullfile(figdir, 'IC-Response-Att1.png'))

%% ---- Final design: dominant-pole placement from Ts and zeta ----
% The baseline is too slow (>10 s settling).  Size a dominant complex pair
% using the 2% settling-time formula:  Ts ≈ 4/(zeta*wn).
% With Ts = 2 s and zeta = 0.7 → wn ≈ 2.86, rounded to 3.0 rad/s.
%
% The two already-stable OL poles (-2.25 and -36.50) are left unchanged —
% they are fast enough not to limit the transient and moving them would
% spend control effort for no benefit.
Ts_des = 2.0;
zeta   = 0.7;
wn     = 3.0;                                % rounded from 4/(0.7*2) = 2.86

p_dom   = -zeta*wn + 1j*wn*sqrt(1-zeta^2);  % dominant complex pair
p_keep  = poles_ol(poles_ol < -1);            % the two fast stable OL poles
p_final = [p_dom; conj(p_dom); p_keep];

[Kf, pcl_f, yf, uf, mf] = sim_regulator(A_sw, B_sw, p_final, x0, t);

% Frequency-domain margins (SISO loop transfer: break at plant input)
L = tf(ss(A_sw, B_sw, Kf, 0));
[Gm, Pm, ~, wgc] = margin(L);
Gm_dB = 20*log10(Gm);

%% ---- Constant bias torque test (no integral action) ----
% A 100 g mass placed off-centre on the seesaw produces a constant
% gravitational torque  tau_bias = m*g*D_T  about the pivot.
% (D_T = pivot-to-rail distance, from seesaw_params.m)
% This enters the EOM as an external torque on the seesaw DOF.
% Without integral action, the controller reaches a non-zero theta_ss.
[t_bias, x_bias, u_bias, mb] = sim_bias_load( ...
    A_sw - B_sw*Kf, Kf, M_inv, D_T);

%% ---- Integral action: augment state with xi_dot = theta ----
% To drive theta_ss → 0 under a constant disturbance, add an integrator
% on theta.  The augmented state is x_a = [x_c; x_c_dot; theta;
% theta_dot; xi], with xi_dot = theta.
%
% One extra pole is placed at -1.0 rad/s — slower than the dominant
% pair (-2.10), so the transient shape is preserved.
C_theta = [0 0 1 0];                         % picks theta from x
A_aug   = [A_sw, zeros(4,1); C_theta, 0];
B_aug   = [B_sw; 0];

p_int = -1.0;
p_aug = [p_final; p_int];

x0_aug = [x0; 0];                            % integrator starts at zero
[K_aug, ~, yi, ui, mi] = sim_regulator(A_aug, B_aug, p_aug, x0_aug, t);

[t_bi, x_bi, u_bi, mbi] = sim_bias_load_aug( ...
    A_aug - B_aug*K_aug, K_aug, M_inv, D_T);

%% Print summary
fprintf('%-18s %12s %12s\n', '', 'Attempt 1', 'Final')
fprintf('%-18s %10.2f V  %10.2f V\n',   'Peak voltage', m1.peak_v, mf.peak_v)
fprintf('%-18s %10.2f deg %10.2f deg\n', 'Peak theta',   m1.peak_th, mf.peak_th)
fprintf('%-18s %10.2f cm  %10.2f cm\n',  'Peak cart',    m1.peak_xc, mf.peak_xc)
fprintf('%-18s %10.2f s   %10.2f s\n',   'Settling',     m1.Ts,      mf.Ts)

fprintf('\nDominant pair: Ts=%.1f s, zeta=%.1f => wn=%.2f (using %.1f)\n', ...
    Ts_des, zeta, 4/(zeta*Ts_des), wn)
fprintf('Kept OL poles: %.2f and %.2f\n', p_keep(2), p_keep(1))
fprintf('\nKf = [%.2f  %.2f  %.2f  %.2f]\n', Kf)
fprintf('Margins: PM=%.1f deg, GM=%.1f dB, wgc=%.2f rad/s\n', Pm, Gm_dB, wgc)
fprintf('Bias (100 g): theta_ss=%.2f deg, cart_ss=%.2f cm, V_ss=%.2f V\n', ...
    mb.theta_ss, mb.cart_ss, mb.v_ss)

fprintf('\nK_aug = [%.2f  %.2f  %.2f  %.2f  %.2f]\n', K_aug)
fprintf('Integrator pole: %.1f rad/s\n', p_int)
fprintf('Bias w/ integral: theta_ss=%.4f deg, cart_ss=%.2f cm, V_ss=%.2f V\n', ...
    mbi.theta_ss, mbi.cart_ss, mbi.v_ss)

%% ---- Generate all figures ----

% Attempt 1 vs Final comparison
figure
subplot(3,1,1); hold on; grid on
plot(t, y1(:,1)*100, t, yf(:,1)*100, 'LineWidth', 1.2)
ylabel('Cart [cm]'); title('Pole Placement Evolution -- 4.5 deg Initial Tilt')
legend('Attempt 1: baseline stabilising', 'Final: dominant-pole design', 'Location', 'best')
subplot(3,1,2); hold on; grid on
plot(t, rad2deg(y1(:,3)), t, rad2deg(yf(:,3)), 'LineWidth', 1.2)
ylabel('\theta [deg]'); yline([-11.5 11.5], 'k--')
subplot(3,1,3); hold on; grid on
plot(t, u1, t, uf, 'LineWidth', 1.2)
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'Pole-Comparison.png'))

% Final design CL poles + IC response
figure; hold on; grid on
plot(real(pcl_f), imag(pcl_f), 'rx', 'MarkerSize', 12, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag'); title('CL Poles -- Final')
saveas(gcf, fullfile(figdir, 'CL-Poles-Final.png'))

plot_3panel(t, yf, uf, 'IC Response -- Final Design');
saveas(gcf, fullfile(figdir, 'IC-Response-Final.png'))

% Bias torque tests
plot_3panel(t_bias, x_bias, u_bias, 'Constant Bias Torque (100 g)');
saveas(gcf, fullfile(figdir, 'repeated_disturbance.png'))

plot_3panel(t_bi, x_bi(:,1:4), u_bi, 'Constant Bias Torque (100 g) -- With Integral');
saveas(gcf, fullfile(figdir, 'bias_with_integral.png'))

% Frequency-domain analysis
plot_loop(L, A_sw - B_sw*Kf, B_sw, C_sw, D_sw, Pm, Gm_dB);
saveas(gcf, fullfile(figdir, 'loop_analysis.png'))

%% Save controller data
save(fullfile(root, 'data', 'controller_freq.mat'), ...
    'Kf', 'p_final', 'K_aug', 'p_aug', 'p_int', 'A_aug', 'B_aug', ...
    'A_sw', 'B_sw', 'C_sw', 'D_sw')
fprintf('\nSaved to data/controller_freq.mat\n')


%% ===== Helpers =====

function [K, pcl, y, u, info] = sim_regulator(A, B, p_des, x0, t)
% Place closed-loop poles at p_des, simulate initial-condition response.
% Works for any state dimension (4-state or 5-state augmented).
    n   = size(A, 1);
    K   = place(A, B, p_des);
    Acl = A - B*K;
    pcl = eig(Acl);

    y = initial(ss(Acl, zeros(n,1), eye(n), zeros(n,1)), x0, t);
    u = (-K * y')';

    % 2% settling time on theta (state 3)
    theta = abs(y(:,3));
    idx   = find(theta > 0.02*abs(x0(3)), 1, 'last');

    info.peak_th = rad2deg(max(theta));
    info.peak_xc = max(abs(y(:,1))) * 100;       % [cm]
    info.peak_v  = max(abs(u));                   % [V]
    info.Ts      = t(max(idx, 1));                % [s]
end

function [t, x, u, info] = sim_bias_load(Acl, K, M_inv, D_T)
% Simulate a constant gravitational torque from a 100 g offset mass.
% The torque enters as  tau = m*g*D_T  on the seesaw DOF.
%
% Disturbance input vector B_tau maps the scalar torque into the
% 4-state dynamics via the inverse inertia matrix:
%   [x_c_ddot; theta_ddot] = M_inv * [0; tau]
    dt = 0.001;  t = (0:dt:15)';

    m_bias   = 0.100;                            % 100 g added mass [kg]
    tau_bias = m_bias * 9.81 * D_T;              % constant torque [N*m]

    % Torque enters only the seesaw EOM (second row of M_eff * q_ddot = ...)
    B_tau = [0; M_inv(1,:)*[0;1]; 0; M_inv(2,:)*[0;1]];

    x = lsim(ss(Acl, B_tau, eye(4), zeros(4,1)), tau_bias*ones(size(t)), t);
    u = (-K * x')';

    info.theta_ss = rad2deg(x(end,3));
    info.cart_ss  = x(end,1) * 100;              % [cm]
    info.v_ss     = u(end);
end

function [t, x, u, info] = sim_bias_load_aug(Acl, K, M_inv, D_T)
% Same as sim_bias_load but for the 5-state augmented system.
% The extra row in B_tau is zero (torque does not directly feed xi).
    dt = 0.001;  t = (0:dt:15)';

    m_bias   = 0.100;
    tau_bias = m_bias * 9.81 * D_T;

    B_tau = [0; M_inv(1,:)*[0;1]; 0; M_inv(2,:)*[0;1]; 0];

    x = lsim(ss(Acl, B_tau, eye(5), zeros(5,1)), tau_bias*ones(size(t)), t);
    u = (-K * x')';

    info.theta_ss = rad2deg(x(end,3));
    info.cart_ss  = x(end,1) * 100;
    info.v_ss     = u(end);
end

function plot_3panel(t, y, u, ttl)
    figure
    subplot(3,1,1); plot(t, y(:,1)*100, 'LineWidth', 1.2)
    ylabel('Cart [cm]'); title(ttl); grid on
    subplot(3,1,2); plot(t, rad2deg(y(:,3)), 'LineWidth', 1.2)
    ylabel('\theta [deg]'); yline([-11.5 11.5], 'r--'); grid on
    subplot(3,1,3); plot(t, u, 'LineWidth', 1.2)
    ylabel('V_m [V]'); xlabel('Time [s]'); grid on
end

function plot_loop(L, Acl, B, C, D, Pm, Gm_dB)
    figure('Position', [50 50 1200 800])
    subplot(2,2,1); margin(L)
    title(sprintf('Bode: PM=%.1f deg, GM=%.1f dB', Pm, Gm_dB)); grid on
    subplot(2,2,2); nyquist(L, logspace(-1, 2.5, 500))
    hold on; plot(-1, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2)
    title('Nyquist'); grid on; xlim([-3 1]); ylim([-3 3])
    subplot(2,2,3); rlocus(L)
    title('Root Locus'); xline(0, 'k--'); sgrid(0.5, []); grid on
    subplot(2,2,4); pzmap(ss(Acl, B, C, D))
    title('CL Pole-Zero Map'); sgrid; grid on
end
