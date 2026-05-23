%% SMC -- Before/After STA Comparison Design Script
%
% Builds two state-feedback sliding mode controllers that share the same
% sliding surface but differ in their reaching law:
%
%   (a) Classical SMC (BEFORE STA):
%         u  = -K_eq*x  -  eta * sat( s/phi )
%       One first-order (relay) switching term.  Reaches s=0 in finite
%       time but the switching action injects high-frequency chattering
%       into the voltage signal -- the term that motivated the move to
%       super-twisting in the first place.
%
%   (b) Super-Twisting SMC (AFTER STA):
%         u  = -K_eq*x  +  ( -k1*sqrt(|s|)*sat(s/phi) + v ) / (S*B)
%         v_dot = -k2 * sat(s/phi)
%       Second-order sliding mode.  Drives s and s_dot to zero in finite
%       time with a CONTINUOUS commanded voltage -- no sign() in the
%       deployed output path, no chattering.
%
% Both controllers use a boundary-layer sat(s/phi) instead of sign(s) to
% absorb encoder-quantization jitter that would otherwise flip the switch
% every sample.  This is the standard hardware-friendly modification.
%
% Hardware-aware design choices derived from the pole-placement lift-off
% data of Experience 3:
%   - V_sat = 6 V (the design saturation; the HW VoltPAQ goes to 7 V but
%     the pinion gear starts slipping above ~6 V, so we design 1 V of
%     mechanical margin into the synthesis).
%   - Surface bandwidth sigma_s = 4 rad/s -- about 2x the open-loop
%     unstable mode at +2.6 rad/s, the minimum that "outruns" the seesaw
%     fall.  Faster surfaces demand more voltage at the resting tilt.
%
% Outputs:
%   data/controller_smc.mat      (overwritten)   classical + STA designs
%   docs/figures/SMC-BeforeAfter-Response.png
%   docs/figures/SMC-BeforeAfter-Chattering.png
%   docs/figures/SMC-BeforeAfter-PhasePlane.png
%
% After this script:
%   scripts/control/build_smc_hw_models.m  -- builds two Simulink HW models
%   models/controllers/smc/SMC_Classical_HW.slx
%   models/controllers/smc/SMC_STA_HW.slx

clear; close all; clc
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

%% 1. Plant ----------------------------------------------------------------
run(fullfile(root, 'scripts', 'config', 'seesaw_params.m'))
tuned = load(fullfile(root, 'data', 'tuned_params.mat'));
A = tuned.A_sw;
B = tuned.B_sw;
n = size(A, 1);

p_unstable = max(real(eig(A)));
fprintf('Plant: unstable mode at %+.3f rad/s, controllability rank %d/%d\n', ...
    p_unstable, rank(ctrb(A,B)), n);

%% 2. Sliding surface (shared by both controllers) ------------------------
%
% Regular form transform z = T*x with T*B = [0;0;0;bbar].  In the new
% coordinates the dynamics split into a 3-state sliding subsystem driven
% by z(end) as a virtual input.  Choose F such that A11 - A12*F has the
% desired sliding poles, then S = [F 1]*T.
N_b  = null(B');                            % 4x3 orthonormal basis of null(B')
Tr   = [N_b'; (B/norm(B))'];                % orthonormal regular-form matrix
Abar = Tr * A * Tr';
A11  = Abar(1:n-1, 1:n-1);
A12  = Abar(1:n-1, n);

sigma_s = 4.0;                  % dominant decay rate [rad/s] -- 2x p_OL
zeta_s  = 0.80;
wn_s    = sigma_s / zeta_s;
p_real  = -1.6 * sigma_s;
p_slide = [ -sigma_s + 1j*wn_s*sqrt(1-zeta_s^2);
            -sigma_s - 1j*wn_s*sqrt(1-zeta_s^2);
             p_real ];

F   = place(A11, A12, p_slide);
S   = [F, 1] * Tr;
SB  = S * B;
K_eq = (SB) \ (S * A);          % equivalent control gain

fprintf('Sliding-surface poles: ');
for ii = 1:numel(p_slide), fprintf('%+.3f%+.3fj ', real(p_slide(ii)), imag(p_slide(ii))); end
fprintf('\nS   = [%+.4f %+.4f %+.4f %+.4f]\n', S);
fprintf('S*B = %+.4f\n\n', SB);

%% 3. Boundary layer (encoder quantization band on s) ---------------------
q_xc     = K_ec;
q_alpha  = K_E_SW / K_gs;
ds_quant = abs(S(1))*q_xc + abs(S(3))*q_alpha;
phi_bl   = 2.5 * ds_quant;
fprintf('Boundary layer phi = %.4f (2.5x quantization band)\n\n', phi_bl);

%% 4. Reaching-law gains ---------------------------------------------------
%
% Classical relay SMC: u_sw = -eta*sat(s/phi).  eta must exceed |s_dot|
% along trajectories OFF the surface, plus a margin to overcome the
% matched-disturbance amplitude D in s-coordinates: eta > D + margin.
% A reasonable choice is eta = 3*D (3x margin) which on the actuator
% becomes eta/(S*B) volts.
D_match = 1.5;                  % matched-disturbance amplitude in s-coords [rad/s^2-equiv]
eta_c   = 3.0 * D_match;        % classical reaching gain
fprintf('Classical reaching gain  eta = %.3f   (Vmax contribution = %.2f V)\n', ...
    eta_c, eta_c/abs(SB));

% Super-twisting gains: finite-time convergence requires
%   k1 >= 1.5*sqrt(L),  k2 >= 1.1*L
% where L bounds the time-derivative of the matched perturbation.
L_dist = 18;
k1 = 1.5 * sqrt(L_dist);
k2 = 1.1 * L_dist;
fprintf('STA gains: L = %.1f -> k1 = %.3f, k2 = %.3f\n\n', L_dist, k1, k2);

%% 5. Closed-loop simulation -- both designs on the same plant ------------
Ts = 0.002;                     % QUARC sample time
t  = (0:Ts:6)';
x0 = [0; 0; deg2rad(2.5); 0];   % small initial tilt

% Matched disturbance to exercise both reaching laws fairly.  Constant
% bias + a slow sinusoid mimics off-centre mass + drifting friction.
d_fun = @(tt) 0.6 + 0.4 * sin(0.8 * tt);

cls = sim_smc(A, B, S, K_eq, t, x0, V_sat, q_xc, q_alpha, d_fun, ...
              'classical', eta_c, [], [], phi_bl);
sta = sim_smc(A, B, S, K_eq, t, x0, V_sat, q_xc, q_alpha, d_fun, ...
              'sta',       [],    k1, k2,    phi_bl);

% Chattering metric: total variation of the voltage per second.
tv_cls = sum(abs(diff(cls.u))) / (t(end) - t(1));
tv_sta = sum(abs(diff(sta.u))) / (t(end) - t(1));

fprintf('================ Simulation comparison ================\n');
fprintf('Settling |alpha|<0.2 deg:    classical = %.2f s    STA = %.2f s\n', ...
    settle_time(t, cls.x(:,3), deg2rad(0.2)), ...
    settle_time(t, sta.x(:,3), deg2rad(0.2)));
fprintf('Peak |V|:                    classical = %.2f V    STA = %.2f V\n', ...
    max(abs(cls.u)), max(abs(sta.u)));
fprintf('Voltage total variation:     classical = %.1f V/s  STA = %.1f V/s\n', ...
    tv_cls, tv_sta);
fprintf('Chattering reduction with STA: %.1fx smoother voltage\n\n', tv_cls / max(tv_sta, eps));

%% 6. Figures --------------------------------------------------------------
fig1 = figure('Name', 'SMC -- before/after STA response', 'Color', 'w');
subplot(3,1,1); hold on; grid on
plot(t, rad2deg(cls.x(:,3)), 'LineWidth', 1.1)
plot(t, rad2deg(sta.x(:,3)), 'LineWidth', 1.3)
ylabel('$\alpha$ [deg]'); title(sprintf('IC response from $\\alpha_0 = %.1f^\\circ$', rad2deg(x0(3))))
legend('Classical SMC (before STA)', 'Super-Twisting SMC (after STA)', 'Location', 'best')
subplot(3,1,2); hold on; grid on
plot(t, 100*cls.x(:,1), 'LineWidth', 1.1)
plot(t, 100*sta.x(:,1), 'LineWidth', 1.3)
ylabel('Cart [cm]')
subplot(3,1,3); hold on; grid on
plot(t, cls.u, 'LineWidth', 1.0)
plot(t, sta.u, 'LineWidth', 1.3)
yline( V_sat, 'r--'); yline(-V_sat, 'r--')
ylabel('$V_m$ [V]'); xlabel('Time [s]')
saveas(fig1, fullfile(figdir, 'SMC-BeforeAfter-Response.png'));

fig2 = figure('Name', 'SMC -- chattering comparison', 'Color', 'w');
subplot(2,1,1); hold on; grid on
plot(t, cls.u, 'LineWidth', 0.9)
plot(t, sta.u, 'LineWidth', 1.2)
ylabel('$V_m$ [V]'); title('Commanded voltage')
legend('Classical SMC', 'Super-Twisting SMC', 'Location', 'best')
subplot(2,1,2); hold on; grid on
plot(t, cls.s, 'LineWidth', 0.9)
plot(t, sta.s, 'LineWidth', 1.2)
yline( phi_bl, 'k--'); yline(-phi_bl, 'k--')
ylabel('Sliding variable $s$'); xlabel('Time [s]')
legend('Classical', 'STA', 'Location', 'best')
saveas(fig2, fullfile(figdir, 'SMC-BeforeAfter-Chattering.png'));

fig3 = figure('Name', 'SMC -- phase plane', 'Color', 'w');
subplot(1,2,1); plot(cls.s, cls.s_dot, 'LineWidth', 1.1); grid on
xlabel('$s$'); ylabel('$\dot s$'); title('Classical SMC reaching')
subplot(1,2,2); plot(sta.s, sta.s_dot, 'LineWidth', 1.3); grid on
xlabel('$s$'); ylabel('$\dot s$'); title('Super-Twisting SMC reaching')
saveas(fig3, fullfile(figdir, 'SMC-BeforeAfter-PhasePlane.png'));

%% 7. Save -----------------------------------------------------------------
% Overwrites data/controller_smc.mat with BOTH designs so the build
% scripts can pick the variant they need.
classical.S      = S;
classical.K_eq   = K_eq;
classical.eta    = eta_c;
classical.phi_bl = phi_bl;

sta_design.S      = S;
sta_design.K_eq   = K_eq;
sta_design.k1     = k1;
sta_design.k2     = k2;
sta_design.L_dist = L_dist;
sta_design.phi_bl = phi_bl;

surface.S      = S;
surface.K_eq   = K_eq;
surface.SB     = SB;
surface.poles  = p_slide;

save(fullfile(root, 'data', 'controller_smc.mat'), ...
     'classical', 'sta_design', 'surface', ...
     'Ts', 'V_sat', 'phi_bl', ...
     'sigma_s', 'zeta_s', 'p_real', 'L_dist');
fprintf('Saved data/controller_smc.mat\n');
fprintf('  classical:  u = -K_eq*x - eta*sat(s/phi)\n');
fprintf('  sta:        u = -K_eq*x + (-k1*sqrt(|s|)*sat(s/phi) + v)/(S*B),  v_dot = -k2*sat(s/phi)\n\n');
fprintf('Next:  scripts/control/build_smc_hw_models.m\n');

%% Helpers -----------------------------------------------------------------
function out = sim_smc(A, B, S, K_eq, t, x0, Vsat, q_xc, q_al, d_fun, ...
                       law, eta, k1, k2, phi)
    N  = numel(t);
    h  = t(2) - t(1);
    SB = S * B;
    x  = x0(:);
    v  = 0;
    X  = zeros(N,4); U = zeros(N,1); Sg = zeros(N,1); Sd = zeros(N,1);

    f = @(xx, uu, dd) A*xx + B*(uu + dd);

    for k = 1:N
        xm = x;
        xm(1) = round(x(1)/q_xc) * q_xc;
        xm(3) = round(x(3)/q_al) * q_al;
        s   = S * xm;
        sig = max(min(s/phi, 1), -1);

        switch law
            case 'classical'
                u_n = -eta * sig;
            case 'sta'
                u_n = (-k1*sqrt(max(abs(s), phi))*sig + v) / SB;
                v   = v - k2*sig*h;
        end
        u = -K_eq*xm + u_n;
        u = max(min(u, Vsat), -Vsat);

        d = d_fun(t(k));
        X(k,:) = x';  U(k) = u;  Sg(k) = s;
        Sd(k)  = S * f(x, u, d);

        % RK4 plant step
        d2 = d_fun(t(k) + h/2);
        k1x = f(x,            u, d);
        k2x = f(x + h/2*k1x,  u, d2);
        k3x = f(x + h/2*k2x,  u, d2);
        k4x = f(x + h*k3x,    u, d_fun(t(k)+h));
        x   = x + h/6*(k1x + 2*k2x + 2*k3x + k4x);
    end
    out.x = X; out.u = U; out.s = Sg; out.s_dot = Sd;
end

function ts = settle_time(t, y, tol)
    idx = find(abs(y) > tol, 1, 'last');
    if isempty(idx) || idx == numel(t)
        ts = NaN;
    else
        ts = t(idx);
    end
end
