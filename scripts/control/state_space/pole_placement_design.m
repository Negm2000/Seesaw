%% Pole Placement Design -- Limit-Cycle Minimization Objective
%
% Nonlinearities (Coulomb friction, encoder quantization, backlash) 
% produce a steady-state limit cycle. The design goal is to minimize 
% the oscillation amplitude by independent placement of the four 
% closed-loop poles.
%
% State vector: x = [x_c; x_c_dot; theta; theta_dot]
% Requires:     seesaw_params.m (plant), tuned_params.mat (B_eq)
% Outputs:      controller_freq.mat, figures

close all; clc

root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');

%% Build plant for current configuration
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

%% Pole selection
%
% Constraints:
%   p_OL  ~ +2.24 rad/s   (unstable pole)
%   q_th  ~ 5e-4 rad      (encoder resolution)
%   V_sat = 6 V           (saturation)
%   T_c/2 = 0.407 m       (rail limit)
%
% Design rule: The complex conjugate pair governing theta dynamics is the 
% dominant pair. Cart poles are real and placed at higher frequencies to 
% maintain a ~2:1 bandwidth window.

% --- p1, p2: complex pair, dominant theta-loop mode ---
% sigma_th (= -Re of pair) governs error correction rate.
%   Lower bound: > p_OL to outrun divergence; 2*p_OL recommended.
%   zeta_th = 0.8: balances settling time and stability margins.
sigma_th = 5.5;
zeta_th  = 0.8;

% --- p3: real pole, cart-velocity damping (non-dominant) ---
% Placed at |p3| > sigma_th to ensure velocity settles faster than 
% the balancing mode.
p3 = -1.5 * sigma_th;

% --- p4: real pole, cart-position regulation (non-dominant) ---
% Enforced |Re(p4)| > sigma_th to maintain p1,2 dominance. 
% Sets the cart return-to-center time constant.
p4 = -1.2 * sigma_th;

wn_th = sigma_th / zeta_th;
p_dom = -sigma_th + 1j*wn_th*sqrt(1-zeta_th^2);
p_des = make_placeable_poles([p_dom; conj(p_dom); p3; p4]);

theta0_deg = 2.0;
[Kf, pcl_f, yf, uf, mf] = sim_regulator(A_sw, B_sw, p_des, ...
    [0; 0; deg2rad(theta0_deg); 0], 0:0.001:10);

% Max IC before saturation
theta_max_deg = theta0_deg * V_sat / mf.peak_v;

% Noise check (mapping measurement resolution to actuator voltage)
q_th = K_E_SW / K_gs;
V_noise_th = abs(Kf(3)) * q_th;

% Stability margins
L = tf(ss(A_sw, B_sw, Kf, 0));
[Gm, Pm, ~, wgc] = margin(L);
Gm_dB = 20*log10(Gm);

%% IC response sanity check
t = 0:0.001:10;

%% Constant bias-torque test -- shifts the oscillation center
% 100g mass off-center. Without integral action, the oscillation 
% band shifts from zero.
[t_bias, x_bias, u_bias, mb] = sim_bias_load( ...
    A_sw - B_sw*Kf, Kf, M_inv, D_T);

%% Integral action
% Augment with xi_dot = theta to eliminate steady-state error.
C_theta = [0 0 1 0];
A_aug   = [A_sw, zeros(4,1); C_theta, 0];
B_aug   = [B_sw; 0];
p_int   = -0.5;
p_aug   = make_placeable_poles([p_des; p_int]);

[K_aug, ~, ~, ~, ~] = sim_regulator(A_aug, B_aug, p_aug, ...
    [0; 0; deg2rad(4.5); 0; 0], t);

[t_bi, x_bi, u_bi, mbi] = sim_bias_load_aug( ...
    A_aug - B_aug*K_aug, K_aug, M_inv, D_T);

%% Print summary
fprintf('Pole selection (vs |p_OL| = %.2f rad/s):\n', abs(p_unstable))
fprintf('  p1,2 (theta pair):   sigma_th=%.2f, zeta_th=%.2f -> %.2f +/- %.2fj\n', ...
    sigma_th, zeta_th, real(p_dom), imag(p_dom))
fprintf('  p3 (cart velocity):  %.2f\n', p3)
fprintf('  p4 (cart position):  %.2f\n\n', p4)

fprintf('Sim results (theta0 = %.1f deg):\n', theta0_deg)
fprintf('  Peak voltage:   %.2f V  (limit 6V)\n', mf.peak_v)
fprintf('  Peak theta:     %.2f deg\n', rad2deg(mf.peak_y(3)))
fprintf('  Peak cart:      %.2f cm\n', mf.peak_y(1)*100)
fprintf('  Max IC (linear):%.2f deg\n', theta_max_deg)
fprintf('  V_noise_th:     %.3f V\n\n', V_noise_th)

fprintf('Stability margins:\n')
fprintf('  Gain margin:    %.2f dB\n', Gm_dB)
fprintf('  Phase margin:   %.2f deg\n', Pm)
fprintf('  Crossover:      %.2f rad/s\n', wgc)

%% Figures
figure
subplot(3,1,1); plot(yf.t, yf.x(:,1)*100, 'LineWidth', 1.2); grid on
ylabel('Cart [cm]')
title(sprintf('Regulator IC Response (%.1f deg)', theta0_deg))
subplot(3,1,2); plot(yf.t, rad2deg(yf.x(:,3)), 'LineWidth', 1.2); grid on
ylabel('\theta [deg]')
subplot(3,1,3); plot(yf.t, uf.v, 'LineWidth', 1.2); grid on
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'IC-Response-Final.png'))

figure; hold on; grid on
plot(real(pcl_f), imag(pcl_f), 'bo', 'MarkerSize', 10, 'LineWidth', 2)
plot(real(poles_ol), imag(poles_ol), 'rx', 'MarkerSize', 10, 'LineWidth', 2)
xline(0, 'k--'); xlabel('Real'); ylabel('Imag')
legend('Closed-loop poles', 'Open-loop poles')
title('Pole Constellation - Dominant Theta Pair')
saveas(gcf, fullfile(figdir, 'CL-Poles-Final.png'))

figure; 
subplot(2,1,1); plot(t_bias, rad2deg(x_bias(:,3)), 'LineWidth', 1.2); grid on
ylabel('\theta [deg]'); title('Bias load response WITHOUT integral action')
subplot(2,1,2); plot(t_bias, u_bias, 'LineWidth', 1.2); grid on
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'repeated_disturbance.png'))

figure; 
subplot(2,1,1); plot(t_bi, rad2deg(x_bi(:,3)), 'LineWidth', 1.2); grid on
ylabel('\theta [deg]'); title('Bias load response WITH integral action (xi\_dot = theta)')
subplot(2,1,2); plot(t_bi, u_bi, 'LineWidth', 1.2); grid on
ylabel('V_m [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'bias_with_integral.png'))

figure
[~] = loop_analysis(A_sw, B_sw, Kf, figdir);

%% Save
p_final = p_des;
save(fullfile(root, 'data', 'controller_freq.mat'), ...
     'Kf', 'p_final', 'sigma_th', 'zeta_th', 'p3', 'p4', 'M_c_added', 'V_noise_th');
fprintf('Saved data/controller_freq.mat\n')

%% Helpers
function [K, poles_cl, x, v, m] = sim_regulator(A, B, p, x0, t)
    K = place(A, B, p);
    Acl = A - B*K;
    poles_cl = eig(Acl);
    sys = ss(Acl, zeros(size(A,1), 1), eye(size(A,1)), zeros(size(A,1), 1));
    [y, t] = initial(sys, x0, t);
    voltage = -K * y';
    m.peak_v = max(abs(voltage));
    m.peak_y = max(abs(y));
    x.t = t; x.x = y;
    v.t = t; v.v = voltage';
end

function [t, x, v, m] = sim_bias_load(Acl, K, M_inv, D_T)
    t = 0:0.001:5;
    % Bias torque constant 0.123 Nm maps to accelerations
    % [x_ddot; theta_ddot] = M_inv * [0; tau_bias]
    accel_bias = M_inv * [0; 0.123];
    B_bias = [0; accel_bias(1); 0; accel_bias(2)];
    sys = ss(Acl, B_bias, eye(4), zeros(4,1));
    [x, t] = step(sys, t);
    v = -K * x';
    v = v';
    m = 0;
end

function [t, x, v, m] = sim_bias_load_aug(Acl, K, M_inv, D_T)
    t = 0:0.001:5;
    accel_bias = M_inv * [0; 0.123];
    B_bias = [0; accel_bias(1); 0; accel_bias(2); 0];
    sys = ss(Acl, B_bias, eye(5), zeros(5,1));
    [x, t] = step(sys, t);
    v = -K * x';
    v = v';
    m = 0;
end

function p_out = make_placeable_poles(p_in)
    p_out = p_in(:);
    tol = 1e-6; delta = 5e-3;
    for i = 2:numel(p_out)
        while any(abs(p_out(i) - p_out(1:i-1)) < tol)
            p_out(i) = p_out(i) - delta;
        end
    end
end

function margins = loop_analysis(A, B, K, figdir)
    L = tf(ss(A, B, K, 0));
    [Gm, Pm, ~, wgc] = margin(L);
    margins.gain_margin_db = 20*log10(Gm);
    margins.phase_margin_deg = Pm;
    margins.crossover_rad_s = wgc;

    margin(L); grid on
    title('Loop Transfer Function Margin Analysis')
    saveas(gcf, fullfile(figdir, 'loop_analysis.png'))
end
