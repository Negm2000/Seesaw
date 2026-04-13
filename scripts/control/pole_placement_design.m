%% Pole Placement Controller Design
% Simple story: baseline stabilizing design -> dominant-pole redesign.
% Produces all figures and saves the final controller.

close all; clc

root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');

%% Load tuned plant
% seesaw_params.m builds A_sw/B_sw with default B_eq = 5.0.
% Patch in the tuned B_eq from frequency sweep.
run(fullfile(root, 'scripts', 'config', 'seesaw_params.m'))

tuned = load(fullfile(root, 'data', 'tuned_params.mat'));
B_eq       = tuned.B_eq;
B_total    = B_eq + B_emf;
G_rhs(1,2) = -B_total;
A_sw = [0 1 0 0; M_inv(1,:)*G_rhs; 0 0 0 1; M_inv(2,:)*G_rhs];
% B_sw unchanged (G_inp doesn't depend on B_eq)

poles_ol = sort(eig(A_sw));
fprintf('Tuned B_eq = %.4f\n', B_eq)
fprintf('Open-loop poles: '); fprintf('%.4f  ', poles_ol); fprintf('\n')
fprintf('Controllability: %d/4\n\n', rank(ctrb(A_sw, B_sw)))

%% Open-loop pole map
figure; hold on; grid on
plot(real(poles_ol), imag(poles_ol), 'rx', 'MarkerSize', 12, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag'); title('Open-Loop Poles')
saveas(gcf, fullfile(figdir, 'OL-Poles.png'))

%% Simulation setup
t  = 0:0.001:10;
x0 = [0; 0; deg2rad(4.5); 0];   % 4.5 deg initial tilt

%% Attempt 1 -- mirror unstable pole, keep the rest
p1 = sort(-abs(poles_ol), 'descend');
[K1, pcl1, y1, u1, m1] = run_regulator(A_sw, B_sw, p1, x0, t);

figure; hold on; grid on
plot(real(pcl1), imag(pcl1), 'rx', 'MarkerSize', 12, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag'); title('CL Poles -- Attempt 1')
saveas(gcf, fullfile(figdir, 'CL-Poles-Att1.png'))

plot_3panel(t, y1, u1, 'IC Response -- Attempt 1');
saveas(gcf, fullfile(figdir, 'IC-Response-Att1.png'))

%% Final design -- choose the dominant pair from the desired transient
Ts_target = 2.0;
zeta      = 0.7;
wn_target = 4/(zeta*Ts_target);
wn        = 3.0;   % round 2.86 to a simple value

p_dom  = -zeta*wn + 1j*wn*sqrt(1-zeta^2);
p_keep = poles_ol(1:2);  % keep the already-fast stable poles
p_final = [p_dom; conj(p_dom); p_keep];

[Kf, pcl_f, yf, uf, mf] = run_regulator(A_sw, B_sw, p_final, x0, t);

L = tf(ss(A_sw, B_sw, Kf, 0));
[Gm, Pm, ~, wgc] = margin(L);
Gm_dB = 20*log10(Gm);

[t_bias, x_bias, u_bias, mb] = run_bias_load(A_sw - B_sw*Kf, Kf, M_inv, D_T);

%% Integral action on theta -- add one slow pole for the integrator state
C_theta = [0 0 1 0];
A_aug = [A_sw, zeros(4,1);
         C_theta, 0];
B_aug = [B_sw; 0];

p_int = -1.0;
p_aug = [p_final; p_int];
[K_aug, yi, ui, mi] = run_integral_regulator(A_aug, B_aug, p_aug, t);
[t_bias_i, x_bias_i, u_bias_i, mbi] = run_bias_load_integral(A_aug - B_aug*K_aug, K_aug, M_inv, D_T);

%% Print summary
fprintf('%-18s %12s %12s\n', '', 'Attempt 1', 'Final')
fprintf('%-18s %10.2f V  %10.2f V\n',   'Peak voltage', m1.peak_v, mf.peak_v)
fprintf('%-18s %10.2f deg %10.2f deg\n','Peak theta',   m1.peak_theta, mf.peak_theta)
fprintf('%-18s %10.2f cm  %10.2f cm\n', 'Peak cart',    m1.peak_cart, mf.peak_cart)
fprintf('%-18s %10.2f s   %10.2f s\n',  'Settling',     m1.settle, mf.settle)
fprintf('\nDominant pair from Ts ~= %.1f s and zeta = %.1f => wn ~= %.2f rad/s (using %.1f).\n', ...
    Ts_target, zeta, wn_target, wn)
fprintf('Remaining poles kept at open-loop values: %.2f and %.2f.\n', p_keep(2), p_keep(1))
fprintf('\nKf = [%.2f  %.2f  %.2f  %.2f]\n', Kf)
fprintf('Margins: PM = %.1f deg, GM = %.1f dB, wgc = %.2f rad/s\n', Pm, Gm_dB, wgc)
fprintf('Bias load (100 g): theta_ss = %.2f deg, cart_ss = %.2f cm, V_ss = %.2f V\n', ...
    mb.theta_ss, mb.cart_ss, mb.v_ss)
fprintf('\nK_aug = [%.2f  %.2f  %.2f  %.2f  %.2f]\n', K_aug)
fprintf('Integral pole: %.1f rad/s\n', p_int)
fprintf('With integral action (100 g bias): theta_ss = %.4f deg, cart_ss = %.2f cm, V_ss = %.2f V\n', ...
    mbi.theta_ss, mbi.cart_ss, mbi.v_ss)

%% Comparison figure
figure
subplot(3,1,1); hold on; grid on
plot(t, y1(:,1)*100, t, yf(:,1)*100, 'LineWidth', 1.2)
ylabel('Cart [cm]'); title('Pole Placement Evolution -- 4.5 deg Initial Tilt')
legend('Attempt 1: baseline stabilizing', 'Final: dominant-pole design', 'Location', 'best')

subplot(3,1,2); hold on; grid on
plot(t, rad2deg(y1(:,3)), t, rad2deg(yf(:,3)), 'LineWidth', 1.2)
ylabel('\theta [deg]'); yline([-11.5 11.5], 'k--')

subplot(3,1,3); hold on; grid on
plot(t, u1, t, uf, 'LineWidth', 1.2)
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'Pole-Comparison.png'))

%% Final design figures
figure; hold on; grid on
plot(real(pcl_f), imag(pcl_f), 'rx', 'MarkerSize', 12, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag'); title('CL Poles -- Final')
saveas(gcf, fullfile(figdir, 'CL-Poles-Final.png'))

plot_3panel(t, yf, uf, 'IC Response -- Final Design');
saveas(gcf, fullfile(figdir, 'IC-Response-Final.png'))

plot_3panel(t_bias, x_bias, u_bias, 'Constant Bias Torque Test');
saveas(gcf, fullfile(figdir, 'repeated_disturbance.png'))

plot_3panel(t_bias_i, x_bias_i(:,1:4), u_bias_i, 'Constant Bias Torque Test -- With Integral Action');
saveas(gcf, fullfile(figdir, 'bias_with_integral.png'))

plot_loop(L, A_sw - B_sw*Kf, B_sw, C_sw, D_sw, Pm, Gm_dB);
saveas(gcf, fullfile(figdir, 'loop_analysis.png'))

%% Save controller
save(fullfile(root, 'data', 'controller_freq.mat'), ...
    'Kf', 'p_final', 'K_aug', 'p_aug', 'p_int', 'A_aug', 'B_aug', ...
    'A_sw', 'B_sw', 'C_sw', 'D_sw')
fprintf('\nSaved to data/controller_freq.mat\n')


%% ===== Helpers =====

function [K, pcl, y, u, info] = run_regulator(A, B, p_des, x0, t)
    K   = place(A, B, p_des);
    Acl = A - B*K;
    pcl = eig(Acl);

    sys = ss(Acl, zeros(size(B)), eye(4), zeros(4,1));
    y   = initial(sys, x0, t);
    u   = (-K * y')';

    theta = abs(y(:,3));
    idx   = find(theta > 0.02*abs(x0(3)), 1, 'last');
    if isempty(idx), ts = 0; else, ts = t(idx); end

    info.peak_theta = rad2deg(max(theta));
    info.peak_cart  = max(abs(y(:,1))) * 100;
    info.peak_v     = max(abs(u));
    info.settle     = ts;
end

function [t, x, u, info] = run_bias_load(Acl, K, M_inv, D_T)
    dt = 0.001;  t = (0:dt:15)';

    m_bias = 0.100;                 % 100 g equivalent added mass
    tau_bias = m_bias * 9.81 * D_T; % constant seesaw torque bias [N*m]
    B_tau = [0; M_inv(1,:)*[0;1]; 0; M_inv(2,:)*[0;1]];

    sys = ss(Acl, B_tau, eye(4), zeros(4,1));
    x   = lsim(sys, tau_bias*ones(size(t)), t);
    u   = (-K * x')';

    info.theta_ss = rad2deg(x(end,3));
    info.cart_ss  = x(end,1) * 100;
    info.v_ss     = u(end);
end

function [K_aug, y, u, info] = run_integral_regulator(A_aug, B_aug, p_aug, t)
    x0_aug = [0; 0; deg2rad(4.5); 0; 0];
    K_aug = place(A_aug, B_aug, p_aug);
    Acl = A_aug - B_aug*K_aug;

    sys = ss(Acl, zeros(size(B_aug)), eye(5), zeros(5,1));
    y   = initial(sys, x0_aug, t);
    u   = (-K_aug * y')';

    theta = abs(y(:,3));
    idx   = find(theta > 0.02*abs(x0_aug(3)), 1, 'last');
    if isempty(idx), ts = 0; else, ts = t(idx); end

    info.peak_theta = rad2deg(max(theta));
    info.peak_cart  = max(abs(y(:,1))) * 100;
    info.peak_v     = max(abs(u));
    info.settle     = ts;
end

function [t, x, u, info] = run_bias_load_integral(Acl_aug, K_aug, M_inv, D_T)
    dt = 0.001;  t = (0:dt:15)';

    m_bias = 0.100;
    tau_bias = m_bias * 9.81 * D_T;
    B_tau_aug = [0; M_inv(1,:)*[0;1]; 0; M_inv(2,:)*[0;1]; 0];

    sys = ss(Acl_aug, B_tau_aug, eye(5), zeros(5,1));
    x   = lsim(sys, tau_bias*ones(size(t)), t);
    u   = (-K_aug * x')';

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
