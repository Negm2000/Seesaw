%% SMC vs. Pole Placement -- Control-Deadzone Limit Cycle
%
% The hardware rocks because the cart motor has an asymmetric voltage
% deadzone (static friction + directional bias) in the actuator path.
%
% Simple approach: linear plant + deadzone + Coulomb friction.
% Euler integration at Ts -- no RK4, no nonlinear Mq inversion.

clear; close all; clc

root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

%% 1. Load everything
run(fullfile(root, 'scripts', 'config', 'seesaw_params.m'))
tuned = load(fullfile(root, 'data', 'tuned', 'tuned_params.mat'));
mpc   = load(fullfile(root, 'data', 'controllers', 'controller_mpc.mat'));
smcd  = load(fullfile(root, 'data', 'controllers', 'controller_smc.mat'));
ctrl  = load(fullfile(root, 'data', 'controllers', 'controller_freq.mat'));

% Plant
A = tuned.A_sw;
B = tuned.B_sw;
V_dz = 0.12;                              % symmetric deadzone
F_c = 0.18;                               % tuned Coulomb to match ~3 deg PP
G_pos = 1.0;  G_neg = 0.5;               % directional gain bias (2:1)

% Controllers
Kf = ctrl.Kf;
SB = smcd.S * B;
SB_smc = SB * G_neg;  % use weaker gain so SMC never under-actuates

% Total cart mass matches controller design (cart + clipped weight from seesaw_params)
M_c_total = M_total;  % 0.38 + 0.37 = 0.75 kg

fprintf('Deadzone: +/- %.2f V (symmetric)\n', V_dz)
fprintf('Directional gain: G_pos=%.1f / G_neg=%.1f (%.0f%% bias)\n', G_pos, G_neg, (G_pos/G_neg-1)*100)
fprintf('Coulomb: %.2f N\n', F_c)
fprintf('PP:  Kf = [%+.1f %+.1f %+.1f %+.1f]\n', Kf)
fprintf('SMC: S*B = %+.4f (safe SB = %+.4f), k1=%.2f, k2=%.2f\n\n', SB, SB_smc, smcd.k1, smcd.k2)

%% 2. Simulation
Ts = 0.002;  T = 20;  N = round(T/Ts)+1;
Qx = K_ec;  Qt = K_E_SW / K_gs;
x0 = [0; 0; deg2rad(2.0); 0];

% Coulomb force mapped to equivalent acceleration via B(2) and B(4)
% F_coulomb acts on cart -> add to x_dot(2) as disturbance
% Simplified: lump into single matched disturbance d entering via B
d_coulomb = F_c / (M_c_total);  % cart accel from Coulomb friction

%% 3. Run PP
fprintf('Running PP...\n'); tic
Xpp = zeros(N,4); Upp = zeros(N,1);
x = x0(:); u = 0;
for k = 1:N
    xc_q = round(x(1)/Qx)*Qx;  th_q = round(x(3)/Qt)*Qt;
    x_ctrl = [xc_q; x(2); th_q; x(4)];
    u = -Kf * x_ctrl;
    u = max(min(u, V_sat), -V_sat);
    u_eff = deadzone(u, V_dz);
    u_eff = u_eff * (G_pos * (u_eff >= 0) + G_neg * (u_eff < 0));
    f_c = -F_c * sign(x(2));
    x = x + Ts * (A*x + B*u_eff + [0; f_c/M_c_total; 0; 0]);
    x(1) = max(min(x(1), 0.407), -0.407);  % rail clamp
    Xpp(k,:) = x';  Upp(k) = u;
end
fprintf('  done (%.1fs)\n', toc)

%% 4. Run SMC
fprintf('Running SMC...\n'); tic
Xsmc = zeros(N,4); Usmc = zeros(N,1);
x = x0(:); u = 0; v = 0;
vsat = V_sat * abs(SB_smc);
for k = 1:N
    xc_q = round(x(1)/Qx)*Qx;  th_q = round(x(3)/Qt)*Qt;
    x_ctrl = [xc_q; x(2); th_q; x(4)];
    s = smcd.S * x_ctrl;
    sig = max(min(s/smcd.phi_bl, 1), -1);
    v = v - smcd.k2*sig*Ts;
    v = max(min(v, vsat), -vsat);
    u = -smcd.K_eq*x_ctrl + (-smcd.k1*sqrt(abs(s))*sig + v) / SB_smc;
    u = max(min(u, V_sat), -V_sat);
    u_eff = deadzone(u, V_dz);
    u_eff = u_eff * (G_pos * (u_eff >= 0) + G_neg * (u_eff < 0));
    f_c = -F_c * sign(x(2));
    x = x + Ts * (A*x + B*u_eff + [0; f_c/M_c_total; 0; 0]);
    x(1) = max(min(x(1), 0.407), -0.407);
    Xsmc(k,:) = x';  Usmc(k) = u;
end
fprintf('  done (%.1fs)\n', toc)

%% 5. Metrics
t = (0:N-1)' * Ts;
win = t > (T - 5);
amp = @(y) (max(y(win)) - min(y(win)));
p2p_smc_th = rad2deg(amp(Xsmc(:,3)));
p2p_pp_th  = rad2deg(amp(Xpp(:,3)));
p2p_smc_xc = amp(Xsmc(:,1)) * 1000;
p2p_pp_xc  = amp(Xpp(:,1))  * 1000;
tv_smc = sum(abs(diff(Usmc))) / T;
tv_pp  = sum(abs(diff(Upp)))  / T;

fprintf('--------------------------------------------------------------\n')
fprintf('                         SMC (super-twist)   PP (placement)\n')
fprintf('Rocking theta [deg p2p]  %14.2f        %14.2f\n', p2p_smc_th, p2p_pp_th)
fprintf('Rocking cart  [mm  p2p]  %14.1f        %14.1f\n', p2p_smc_xc, p2p_pp_xc)
fprintf('Control variation [V/s]  %14.2f        %14.2f\n', tv_smc, tv_pp)
fprintf('Peak voltage      [V]    %14.2f        %14.2f\n', max(abs(Usmc)), max(abs(Upp)))
fprintf('--------------------------------------------------------------\n')

%% 6. Figures
cPP  = [0.85 0.33 0.10];
cSMC = [0    0.45 0.74];
expo = @(f) exportgraphics(f, fullfile(figdir, get(f,'FileName')), 'Resolution', 150);

f1 = figure('Name', 'SMC vs PP', 'Position', [80 80 980 780], 'Color', 'w');
f1.FileName = 'SMC-vs-PP-Nonlinear.png';
subplot(3,1,1); hold on; grid on; box on
plot(t, rad2deg(Xpp(:,3)),  'Color', cPP,  'LineWidth', 1.0)
plot(t, rad2deg(Xsmc(:,3)), 'Color', cSMC, 'LineWidth', 1.3)
ylabel('\theta [deg]'); xlim([0 T])
legend('Pole placement', 'Super-twisting SMC', 'Location', 'northeast')
title('Seesaw Rocking -- Control Deadzone (asymmetric, actuator path)')
subplot(3,1,2); hold on; grid on; box on
plot(t, Xpp(:,1)*1000,  'Color', cPP,  'LineWidth', 1.0)
plot(t, Xsmc(:,1)*1000, 'Color', cSMC, 'LineWidth', 1.3)
ylabel('Cart x_c [mm]'); xlim([0 T])
subplot(3,1,3); hold on; grid on; box on
plot(t, Upp,  'Color', cPP,  'LineWidth', 1.0)
plot(t, Usmc, 'Color', cSMC, 'LineWidth', 1.3)
yline( V_sat, 'k--'); yline(-V_sat, 'k--')
yline( V_dz, 'k:'); yline(-V_dz, 'k:')
ylabel('V_m [V]'); xlabel('Time [s]'); xlim([0 T]); ylim([-1.2 1.2]*V_sat)
expo(f1)

z0 = T - 8;
f2 = figure('Name', 'SMC vs PP -- zoom', 'Position', [100 100 980 600], 'Color', 'w');
f2.FileName = 'SMC-vs-PP-LimitCycle.png';
subplot(2,1,1); hold on; grid on; box on
plot(t, rad2deg(Xpp(:,3)),  'Color', cPP,  'LineWidth', 1.3)
plot(t, rad2deg(Xsmc(:,3)), 'Color', cSMC, 'LineWidth', 1.5)
xlim([z0 T]); ylabel('\theta [deg]')
legend('Pole placement', 'Super-twisting SMC', 'Location', 'northeast')
title(sprintf('Steady-State -- PP %.2f^\\circ p2p  vs  SMC %.2f^\\circ p2p', p2p_pp_th, p2p_smc_th))
subplot(2,1,2); hold on; grid on; box on
plot(t, Upp,  'Color', cPP,  'LineWidth', 1.3)
plot(t, Usmc, 'Color', cSMC, 'LineWidth', 1.5)
xlim([z0 T]); ylabel('V_m [V]'); xlabel('Time [s]')
yline( V_dz, 'k:'); yline(-V_dz, 'k:')
expo(f2)

f3 = figure('Name', 'SMC vs PP -- phase', 'Position', [120 120 740 600], 'Color', 'w');
f3.FileName = 'SMC-vs-PP-PhasePortrait.png';
hold on; grid on; box on
plot(rad2deg(Xpp(win,3)),  rad2deg(Xpp(win,4)),  'Color', cPP,  'LineWidth', 1.2)
plot(rad2deg(Xsmc(win,3)), rad2deg(Xsmc(win,4)), 'Color', cSMC, 'LineWidth', 1.5)
xlabel('\theta [deg]'); ylabel('d\theta/dt [deg/s]')
legend(sprintf('PP -- %.2f^\\circ p2p', p2p_pp_th), sprintf('SMC -- %.2f^\\circ p2p', p2p_smc_th), 'Location', 'best')
title('Phase Portrait -- Steady-State (last 5 s)')
expo(f3)

fprintf('\nSaved: SMC-vs-PP-Nonlinear / -LimitCycle / -PhasePortrait\n')

%% Helpers
function u_eff = deadzone(u, Vz)
    if abs(u) < Vz
        u_eff = 0;
    elseif u > 0
        u_eff = u - Vz;
    else
        u_eff = u + Vz;
    end
end
