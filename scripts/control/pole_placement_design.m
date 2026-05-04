%% Pole Placement Design -- Bounded-Rocking Objective
%
% The linearised model says theta -> 0 asymptotically. The hardware
% does not get there: cart stiction, theta encoder quantum
% q_th = K_E_SW/K_gs = 5e-4 rad, gear backlash, and DAC resolution
% form a deadband at theta ~ 0 the controller cannot push through.
% Steady state is a limit cycle, not a fixed point.
%
% Goal: minimise the limit-cycle (rocking) amplitude. We place all
% four poles independently; each pole has a distinct physical role
% and its own upper/lower bound, justified inline below.
%
% State vector: x = [x_c; x_c_dot; theta; theta_dot]
% Requires:     seesaw_params.m (plant), tuned_params.mat (B_eq)
% Outputs:      controller_freq.mat, figures

close all; clc

root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');

%% Build plant for current configuration
% Patch B_eq to the value identified by the frequency sweep, then add
% the 370 g cart payload that is currently mounted.
run(fullfile(root, 'scripts', 'config', 'seesaw_params.m'))
tuned     = load(fullfile(root, 'data', 'tuned_params.mat'));
B_eq      = tuned.B_eq;
M_c_base  = M_c;
M_c_added = 0.370;
M_c       = M_c_base + M_c_added;

B_total = B_eq + B_emf;
M_eff   = [M_c, -M_c*D_T; -M_c*D_T, J_pivot + M_c*D_T^2];
M_inv   = inv(M_eff);
G_rhs   = [0,      -B_total, -g*M_c,                       0;
           -g*M_c,  0,        g*(M_c*D_T + M_SW*D_C),  -B_SW];
A_sw    = [0 1 0 0; M_inv(1,:)*G_rhs; 0 0 0 1; M_inv(2,:)*G_rhs];
B_sw    = [0; M_inv(1,:)*[alpha_f*eta_m; 0]; 0; M_inv(2,:)*[alpha_f*eta_m; 0]];

poles_ol   = sort(eig(A_sw));
p_unstable = max(real(poles_ol));     % seesaw fall rate -- design driver

fprintf('Tuned B_eq = %.4f, M_c = %.3f kg (%.3f base + %.3f added)\n', ...
    B_eq, M_c, M_c_base, M_c_added)
fprintf('Open-loop poles: '); fprintf('%.3f  ', poles_ol); fprintf('\n')
fprintf('Unstable mode: p_OL = %+.3f rad/s (seesaw fall rate)\n', p_unstable)
fprintf('Controllability rank: %d/4\n\n', rank(ctrb(A_sw, B_sw)))

figure; hold on; grid on
plot(real(poles_ol), imag(poles_ol), 'rx', 'MarkerSize', 12, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag')
title('Open-Loop Poles -- one in RHP')
saveas(gcf, fullfile(figdir, 'OL-Poles.png'))

%% Pole selection -- four independent choices
%
% Plant references used as bounds below:
%   p_OL  = +3.03 rad/s   open-loop seesaw fall rate (post-tuning, +370 g)
%   q_th  = K_E_SW/K_gs   theta encoder quantum (5e-4 rad)
%   V_sat = 6 V           voltage saturation
%   T_c/2 = 0.407 m       cart rail
%
% --- p1, p2: complex pair, theta-loop dynamics ---
% sigma_th (= -Re of pair) governs how fast a theta error is corrected.
%   Lower bound: > p_OL = 3.03 to outrun divergence; >= 2*p_OL ~ 6
%       gives bandwidth margin against modelling error.
%   Upper bound: K_theta scales ~ sigma_th^2 for this plant. Set the
%       soft limit by the per-count voltage step
%           V_chatter = K_theta * q_th
%       and demand V_chatter < ~0.1 V (~2% of V_sat). Empirically
%       sigma_th ~ 7-9 satisfies this; the verification print below
%       reports the actual value -- bump sigma_th down if it overshoots.
% zeta_th sets transient overshoot. A theta-loop overshoot IS a rock:
%   the cart correction sends theta past zero to the opposite side once
%   before settling.
%       0.7 -> 5%   overshoot
%       0.8 -> 1.5% overshoot
%       1.0 -> 0%   but slowest decay (real double pole, zero margin)
%   Pick 0.8 as the compromise.
sigma_th = 7.0;
zeta_th  = 0.8;

% --- p3: real pole, cart-velocity damping ---
% Once the cart is moving to fight a theta error, p3 governs how fast
% the cart velocity itself decays.
%   Too slow (|p3| < sigma_th): cart overshoots its correction and
%       drives theta back the other way -> wider rocking band.
%   Too fast (|p3| >> sigma_th): K_xdot grows. q_xdot from naive
%       differentiation of position is large (~ K_ec/dt = 2.3e-2 m/s
%       per count at 1 ms), so K_xdot * q_xdot adds to chatter.
% Place slightly outside the dominant pair: -1.5 * sigma_th.
p3 = -1.5 * sigma_th;

% --- p4: real pole, cart-position regulation (slowest) ---
% Sets the cart return-to-centre time constant ~ 1/|p4|.
%   Lower bound: under bias the cart settles offset (see bias test
%       below). A 100 g offset gives ~ tens of cm of cart drift; rail
%       is +-40.7 cm; allow ~3 s to recover -> |p4| > ~0.3.
%   Upper bound: faster than ~3 rad/s puts cart-position regulation on
%       the same timescale as theta, so the cart fights the limit cycle
%       instead of just supporting it.
% Pick -1.0 (1 s time constant).
p4 = -1.0;

wn_th = sigma_th / zeta_th;
p_dom = -sigma_th + 1j*wn_th*sqrt(1-zeta_th^2);
p_des = make_placeable_poles([p_dom; conj(p_dom); p3; p4]);

% Realistic IC: hardware is hand-placed near horizontal at startup;
% operator alignment is good to ~1-2 deg. We use 2 deg as a representative
% startup excursion (the seesaw rocks in a limit cycle anyway, so a perfect
% start is neither possible nor required). Plant is linear so other ICs
% scale proportionally; the largest disturbance the controller can absorb
% without saturating sets an upper bound on the operating envelope.
theta0_deg = 2.0;
[Kf, pcl_f, yf, uf, mf] = sim_regulator(A_sw, B_sw, p_des, ...
    [0; 0; deg2rad(theta0_deg); 0], 0:0.001:10);

% Headroom: how large an IC can the controller absorb before peak |V_m|
% hits V_sat? Linear plant -> peak V scales linearly with IC.
theta_max_deg = theta0_deg * V_sat / mf.peak_v;

% Verify the noise-into-voltage check used to bound sigma_th.
q_th         = K_E_SW / K_gs;
V_chatter_th = abs(Kf(3)) * q_th;

% SISO loop transfer (break at plant input) for margin reporting.
L = tf(ss(A_sw, B_sw, Kf, 0));
[Gm, Pm, ~, wgc] = margin(L);
Gm_dB = 20*log10(Gm);

%% IC response sanity check -- 4.5 deg disturbance recovery
% This is NOT a settling-time test (the hardware never settles). It
% verifies the controller can absorb a one-off disturbance (operator
% bumps the seesaw) without:
%   - voltage saturating at +/- V_sat (= 6 V)
%   - the cart hitting the rail end-stops (+/- T_c/2 = 40.7 cm)
t = 0:0.001:10;

%% Constant bias-torque test -- shifts the rocking centre
% A 100 g mass sitting off-centre on the seesaw produces a constant
% gravitational torque tau = m*g*D_T about the pivot. Without integral
% action, the closed loop converges to a non-zero theta_ss. On hardware
% this means the limit cycle is offset (e.g. -0.5 deg to +1.5 deg
% instead of +/- 1 deg) -- the rocking band shifts, it does not widen.
[t_bias, x_bias, u_bias, mb] = sim_bias_load( ...
    A_sw - B_sw*Kf, Kf, M_inv, D_T);

%% Integral action -- recenters the rocking band
% Augment with xi_dot = theta. Under bias, the integrator drives
% mean(theta) -> 0, so the limit cycle becomes symmetric about the
% equilibrium. The integrator does NOT close the rocking band itself;
% the unmodelled deadband still produces a limit cycle of similar
% amplitude, just now centred on zero.
%
% p_int placed slower than p4 (cart position) so the integrator cannot
% dominate cart dynamics; ~3 s recentering time constant is acceptable.
C_theta = [0 0 1 0];
A_aug   = [A_sw, zeros(4,1); C_theta, 0];
B_aug   = [B_sw; 0];
p_int   = -0.3;
p_aug   = make_placeable_poles([p_des; p_int]);

[K_aug, ~, ~, ~, ~] = sim_regulator(A_aug, B_aug, p_aug, ...
    [0; 0; deg2rad(4.5); 0; 0], t);

[t_bi, x_bi, u_bi, mbi] = sim_bias_load_aug( ...
    A_aug - B_aug*K_aug, K_aug, M_inv, D_T);

%% Print summary
fprintf('Pole selection (vs |p_OL| = %.2f rad/s):\n', abs(p_unstable))
fprintf('  p1,2 (theta pair):   sigma_th=%.2f, zeta_th=%.2f -> %.2f +/- %.2fj\n', ...
    sigma_th, zeta_th, real(p_dom), imag(p_dom))
fprintf('  p3   (cart vel):     %.2f rad/s  (= -1.5 * sigma_th)\n', p3)
fprintf('  p4   (cart pos):     %.2f rad/s  (~ 1 s time constant)\n', p4)
fprintf('\nKf = [%.2f  %.2f  %.2f  %.2f]\n', Kf)
fprintf('Margins: PM=%.1f deg, GM=%.1f dB, wgc=%.2f rad/s\n', Pm, Gm_dB, wgc)
fprintf('Per-count theta noise -> %.3f V (target < ~0.1; lower = less chatter)\n', ...
    V_chatter_th)
fprintf('\nIC sanity check (%.1f deg disturbance):\n', theta0_deg)
fprintf('  Peak voltage = %.2f V (sat = +/- %.1f V)\n', mf.peak_v, V_sat)
fprintf('  Peak theta   = %.2f deg\n', mf.peak_th)
fprintf('  Peak cart    = %.2f cm  (rail = +/- %.1f cm)\n', ...
    mf.peak_xc, x_c_max*100)
fprintf('  Headroom    : controller absorbs up to %.2f deg before V_sat\n', ...
    theta_max_deg)
fprintf('\nBias 100 g, no integral:    theta_ss=%+.2f deg, cart_ss=%+.2f cm, V_ss=%+.2f V\n', ...
    mb.theta_ss, mb.cart_ss, mb.v_ss)
fprintf('Bias 100 g, with integral:  theta_ss=%+.4f deg, cart_ss=%+.2f cm, V_ss=%+.2f V\n', ...
    mbi.theta_ss, mbi.cart_ss, mbi.v_ss)
fprintf('\nK_aug = [%.2f  %.2f  %.2f  %.2f  %.2f]   (integrator pole %.2f rad/s)\n', ...
    K_aug, p_int)

%% Figures
figure; hold on; grid on
plot(real(pcl_f), imag(pcl_f), 'rx', 'MarkerSize', 12, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag')
title(sprintf('Closed-Loop Poles (\\sigma_{th}=%.1f, \\zeta_{th}=%.2f, p_3=%.1f, p_4=%.1f)', ...
    sigma_th, zeta_th, p3, p4))
saveas(gcf, fullfile(figdir, 'CL-Poles-Final.png'))

plot_3panel(t, yf, uf, sprintf('IC Response -- %.1f deg Startup Excursion', theta0_deg));
saveas(gcf, fullfile(figdir, 'IC-Response-Final.png'))

plot_3panel(t_bias, x_bias, u_bias, 'Bias Torque (100 g) -- No Integral');
saveas(gcf, fullfile(figdir, 'repeated_disturbance.png'))

plot_3panel(t_bi, x_bi(:,1:4), u_bi, 'Bias Torque (100 g) -- With Integral');
saveas(gcf, fullfile(figdir, 'bias_with_integral.png'))

plot_loop(L, A_sw - B_sw*Kf, B_sw, C_sw, D_sw, Pm, Gm_dB);
saveas(gcf, fullfile(figdir, 'loop_analysis.png'))

%% Save controller
% Variable names preserved so build_simulink_models.m (which reads
% ctrl.K) and any other downstream consumer keeps working unchanged.
K         = Kf;
p_final   = p_des;
p_desired = p_des;
wn_dom    = wn_th;
zeta_dom  = zeta_th;
save(fullfile(root, 'data', 'controller_freq.mat'), ...
    'Kf', 'p_final', 'K_aug', 'p_aug', 'p_int', 'A_aug', 'B_aug', ...
    'A_sw', 'B_sw', 'C_sw', 'D_sw', 'K', 'p_desired', 'wn_dom', ...
    'zeta_dom', 'p_unstable', 'sigma_th', 'zeta_th', 'p3', 'p4', ...
    'V_chatter_th', 'M_c_base', 'M_c_added', 'M_c')
fprintf('\nSaved to data/controller_freq.mat\n')


%% ===== Helpers =====

function [K, pcl, y, u, info] = sim_regulator(A, B, p_des, x0, t)
% Place poles at p_des and run an initial-condition response. Works
% for the 4-state plant or the 5-state augmented system.
    n   = size(A, 1);
    K   = place(A, B, p_des);
    Acl = A - B*K;
    pcl = eig(Acl);

    y = initial(ss(Acl, zeros(n,1), eye(n), zeros(n,1)), x0, t);
    u = (-K * y')';

    info.peak_th = rad2deg(max(abs(y(:,3))));
    info.peak_xc = max(abs(y(:,1))) * 100;       % [cm]
    info.peak_v  = max(abs(u));                   % [V]
end

function [t, x, u, info] = sim_bias_load(Acl, K, M_inv, D_T)
% 100 g offset mass -> constant tau = m*g*D_T into the seesaw EOM.
% Disturbance enters via M_inv applied to [0; tau].
    dt = 0.001;  t = (0:dt:15)';
    tau_bias = 0.100 * 9.81 * D_T;
    B_tau    = [0; M_inv(1,:)*[0;1]; 0; M_inv(2,:)*[0;1]];

    x = lsim(ss(Acl, B_tau, eye(4), zeros(4,1)), tau_bias*ones(size(t)), t);
    u = (-K * x')';

    info.theta_ss = rad2deg(x(end,3));
    info.cart_ss  = x(end,1) * 100;              % [cm]
    info.v_ss     = u(end);
end

function [t, x, u, info] = sim_bias_load_aug(Acl, K, M_inv, D_T)
% Same disturbance, augmented 5-state system (xi row in B_tau is 0).
    dt = 0.001;  t = (0:dt:15)';
    tau_bias = 0.100 * 9.81 * D_T;
    B_tau    = [0; M_inv(1,:)*[0;1]; 0; M_inv(2,:)*[0;1]; 0];

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

function p_out = make_placeable_poles(p_in)
% Nudge near-coincident poles so place() stays well posed for SISO B.
    p_out = p_in(:);
    tol   = 1e-6;
    delta = 5e-3;
    for i = 2:numel(p_out)
        while any(abs(p_out(i) - p_out(1:i-1)) < tol)
            p_out(i) = p_out(i) - delta;
        end
    end
end
