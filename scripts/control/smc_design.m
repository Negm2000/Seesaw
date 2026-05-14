%% Sliding Mode Control Design -- Super-Twisting (Minimal Chattering)
%
% The seesaw is open-loop unstable and the plant carries unmodelled
% matched perturbations (Coulomb friction, encoder quantization, mass
% offset, linearisation error). A first-order (relay) sliding mode would
% reject all of this but at the cost of a switching control voltage that
% the VoltPAQ/motor cannot follow cleanly -- chattering.
%
% This script designs a second-order sliding mode using the super-twisting
% algorithm (STA). The STA acts on the sliding variable s while keeping the
% commanded voltage CONTINUOUS (no sign() in the output path), so the
% actuator sees a smooth signal -- chattering is minimised -- yet finite-
% time convergence and matched-disturbance rejection are retained.
%
% State vector: x = [x_c; x_c_dot; alpha; alpha_dot]
% Requires:     seesaw_params.m (plant), tuned_params.mat (B_eq)
% Outputs:      controller_smc.mat, figures
%
% Control law:
%   s    = S * x                                  (sliding variable)
%   u_eq = -(S*B)\(S*A) * x                        (equivalent control)
%   u_st = (1/(S*B)) * ( -k1*sqrt(|s|)*sign(s) + v )
%   v_dot = -k2 * sign(s)                          (STA integrator state)
%   u    = sat( u_eq + u_st , +/-V_sat )

clear; close all; clc

set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');

%% 1. Build plant for current configuration
run(fullfile(root, 'scripts', 'config', 'seesaw_params.m'))
tuned = load(fullfile(root, 'data', 'tuned_params.mat'));
A = tuned.A_sw;             % hardware-tuned 4x4 (B_eq fitted to data)
B = tuned.B_sw;             % hardware-tuned 4x1

n = size(A, 1);
poles_ol   = eig(A);
p_unstable = max(real(poles_ol));

fprintf('Open-loop poles: '); fprintf('%+.3f  ', poles_ol); fprintf('\n')
fprintf('Unstable mode:   p_OL = %+.3f rad/s (seesaw fall rate)\n', p_unstable)
fprintf('Controllability rank: %d/%d\n\n', rank(ctrb(A, B)), n)
if rank(ctrb(A, B)) < n
    error('Plant not controllable -- SMC surface cannot be placed.')
end

%% 2. Sliding surface via regular form + pole placement
%
% Orthonormal T maps the input to the last channel: T*B = [0;0;0;bbar].
% In z = T*x the dynamics split into a 3-state "sliding" subsystem driven
% by the scalar z2, which is treated as a virtual control:
%
%   z1_dot = A11*z1 + A12*z2
%   z2_dot = A21*z1 + A22*z2 + bbar*u
%
% Choosing z2 = -F*z1 places the 3 poles of (A11 - A12*F): these are the
% poles the system shows WHILE sliding. The surface is sigma = F*z1 + z2,
% hence S = [F 1]*T in original coordinates.

N  = null(B');                      % 4x3, orthonormal, spans null(B')
Tr = [N'; (B/norm(B))'];            % orthonormal regular-form transform
Abar = Tr * A * Tr';
Bbar = Tr * B;                      % = [0;0;0;norm(B)]

A11 = Abar(1:n-1, 1:n-1);
A12 = Abar(1:n-1, n);

% Sliding-mode poles (order n-1 = 3). Must outrun the open-loop divergence.
% Dominant complex pair governs the alpha-balancing motion; the real pole
% damps the cart. Kept in the same bandwidth window as the PP/LQR designs
% so the required voltage stays within saturation.
sigma_s = 4.0;                      % dominant decay rate [rad/s]
zeta_s  = 0.80;                     % dominant pair damping
wn_s    = sigma_s / zeta_s;
p_real  = -1.6 * sigma_s;           % cart-mode real pole
p_slide = [ -sigma_s + 1j*wn_s*sqrt(1-zeta_s^2);
            -sigma_s - 1j*wn_s*sqrt(1-zeta_s^2);
             p_real ];

F = place(A11, A12, p_slide);
S = [F, 1] * Tr;                    % sliding surface row vector, s = S*x

SB = S * B;
if abs(SB) < 1e-9
    error('S*B is singular -- sliding surface is not well posed.')
end

% Equivalent control gain: u_eq = -K_eq * x
K_eq = (SB) \ (S * A);

% Verify the achieved sliding dynamics
A_slide = A11 - A12 * F;
fprintf('Sliding-surface poles (achieved): '); fprintf('%+.3f%+.3fj  ', ...
    [real(eig(A_slide)) imag(eig(A_slide))]'); fprintf('\n')
fprintf('S   = [%+.4f %+.4f %+.4f %+.4f]\n', S)
fprintf('S*B = %+.4f\n\n', SB)

%% 3. Super-twisting reaching-law gains
%
% On the surface the perturbed reaching dynamics are
%   s_dot = -k1*sqrt(|s|)*sign(s) + v + d(t)
%   v_dot = -k2*sign(s)
% Finite-time convergence holds for k1 = 1.5*sqrt(L), k2 = 1.1*L, where L
% bounds the rate of the matched perturbation expressed in s-coordinates.
% L is the single robustness knob -- raise it for stronger rejection,
% lower it for an even smoother voltage.

L_dist = 18;                        % matched-perturbation rate bound [1/s^2]
k1 = 1.5 * sqrt(L_dist);
k2 = 1.1 * L_dist;
fprintf('Super-twisting gains: L = %.1f -> k1 = %.3f, k2 = %.3f\n', ...
    L_dist, k1, k2)

% --- Boundary layer: the chattering-killer under encoder quantization ---
% Pure super-twisting is continuous in CONTINUOUS time, but on hardware the
% sliding variable s = S*x is reconstructed from QUANTIZED encoder counts.
% s therefore never reaches exactly zero -- it jitters within a band
%   ds ~ |S(1)|*q_xc + |S(3)|*q_alpha
% and a bare sign(s) flips every sample inside that band. Replacing sign(s)
% by the continuous sat(s/phi_bl), with phi_bl sized to swallow the
% quantization band, makes the commanded voltage continuous and removes
% the sampling-induced chattering, at the price of a thin phi_bl-sized
% accuracy layer around s = 0.
q_alpha = K_E_SW / K_gs;            % seesaw encoder resolution [rad]
q_xc    = K_ec;                     % cart encoder resolution [m]
ds_quant = abs(S(1))*q_xc + abs(S(3))*q_alpha;
phi_bl  = 2.5 * ds_quant;           % boundary-layer half-width
fprintf('Quantization band on s: %.4f -> boundary layer phi = %.4f\n\n', ...
    ds_quant, phi_bl)

%% 4. Closed-loop simulation -- boundary-layer STA vs. bare-sign STA
%
% Both runs use the SAME surface, equivalent control and super-twisting
% gains; only the switching function differs (sat(s/phi) vs. sign(s)).
% A matched disturbance (offset mass + Coulomb friction) and encoder
% quantization are injected so the chattering comparison is meaningful.

Ts       = 0.002;                   % QUARC fixed step [s]
t        = (0:Ts:6)';
x0       = [0; 0; deg2rad(2.5); 0]; % initial seesaw tilt

% Matched disturbance d(t) entering through the input channel [N-equiv].
% Constant bias-load torque + a slow drift to exercise the STA integrator.
d_fun = @(tt) 0.6 + 0.4 * sin(0.8 * tt);

sta = sim_smc(A, B, S, K_eq, t, x0, V_sat, q_xc, q_alpha, d_fun, ...
              'sta_bl',   k1, k2, phi_bl);
raw = sim_smc(A, B, S, K_eq, t, x0, V_sat, q_xc, q_alpha, d_fun, ...
              'sta_sign', k1, k2, phi_bl);

% Chattering metric: total variation of the control signal per second.
tv_sta = sum(abs(diff(sta.u))) / (t(end) - t(1));
tv_raw = sum(abs(diff(raw.u))) / (t(end) - t(1));
fprintf('Control total variation [V/s]:  boundary-layer STA = %.2f   bare-sign STA = %.2f\n', ...
    tv_sta, tv_raw)
fprintf('Chattering reduction: %.1fx smoother voltage with the boundary layer\n\n', ...
    tv_raw / max(tv_sta, eps))

fprintf('Settling (|alpha| < 0.2 deg):    STA = %.2f s\n', ...
    settle_time(t, sta.x(:,3), deg2rad(0.2)))
fprintf('Peak voltage:                    STA = %.2f V (limit %.2f V)\n', ...
    max(abs(sta.u)), V_sat)

%% 5. Figures
figure('Name', 'SMC -- super-twisting response')
subplot(3,1,1)
plot(t, sta.x(:,3)*180/pi, 'LineWidth', 1.3); grid on
ylabel('$\alpha$ [deg]')
title(sprintf('Super-Twisting SMC -- IC response (%.1f deg)', ...
    x0(3)*180/pi))
subplot(3,1,2)
plot(t, sta.x(:,1)*100, 'LineWidth', 1.3); grid on
ylabel('Cart [cm]')
subplot(3,1,3)
plot(t, sta.u, 'LineWidth', 1.3); grid on
yline( V_sat, 'r--'); yline(-V_sat, 'r--')
ylabel('$V_m$ [V]'); xlabel('Time [s]')
saveas(gcf, fullfile(figdir, 'SMC-STA-Response.png'))

figure('Name', 'SMC -- chattering comparison')
subplot(2,1,1); hold on; grid on
plot(t, raw.u, 'LineWidth', 1.0)
plot(t, sta.u, 'LineWidth', 1.3)
ylabel('$V_m$ [V]')
legend('Bare-sign STA', 'Boundary-layer STA (deployed)', ...
    'Location', 'best')
title('Commanded Voltage -- Chattering Comparison')
subplot(2,1,2); hold on; grid on
plot(t, raw.s, 'LineWidth', 1.0)
plot(t, sta.s, 'LineWidth', 1.3)
ylabel('Sliding variable $s$'); xlabel('Time [s]')
legend('Bare-sign STA', 'Boundary-layer STA', 'Location', 'best')
saveas(gcf, fullfile(figdir, 'SMC-Chattering-Comparison.png'))

figure('Name', 'SMC -- phase portrait')
plot(sta.s, sta.s_dot, 'LineWidth', 1.3); grid on
xlabel('$s$'); ylabel('$\dot{s}$')
title('Super-Twisting Reaching Trajectory ($s,\dot{s}$)')
saveas(gcf, fullfile(figdir, 'SMC-Phase-Portrait.png'))

%% 6. Save controller
save(fullfile(root, 'data', 'controller_smc.mat'), ...
     'S', 'K_eq', 'k1', 'k2', 'L_dist', 'p_slide', 'sigma_s', 'zeta_s', ...
     'p_real', 'V_sat', 'Ts', 'phi_bl')
fprintf('Saved data/controller_smc.mat\n')
fprintf(['Deploy: s = S*x;  sigma = sat(s/phi_bl);  v is an integrator state\n' ...
         '        v_dot = -k2*sigma;\n' ...
         '        u = sat( -K_eq*x + (-k1*sqrt(|s|)*sigma + v)/(S*B), V_sat )\n'])

%% Helpers --------------------------------------------------------------
function out = sim_smc(A, B, S, K_eq, t, x0, Vsat, q_xc, q_al, d_fun, ...
                       law, k1, k2, phi)
    % Fixed-step simulation of the plant under SMC. The plant is integrated
    % with RK4; the controller runs at the sample rate with quantized
    % measurements, mirroring the QUARC deployment.
    N  = numel(t);
    h  = t(2) - t(1);
    SB = S * B;
    x  = x0(:);
    v  = 0;                                  % STA integrator state
    X  = zeros(N, 4); U = zeros(N,1); Sg = zeros(N,1); Sd = zeros(N,1);

    f = @(xx, uu, dd) A*xx + B*(uu + dd);    % matched disturbance dd

    for k = 1:N
        % --- quantized measurement -> sliding variable ---
        xm = x;
        xm(1) = round(x(1)/q_xc) * q_xc;
        xm(3) = round(x(3)/q_al) * q_al;
        s  = S * xm;

        % --- reaching law ---
        switch law
            case 'sta_bl'
                % boundary-layer super-twisting (deployed): sign -> sat
                sig = sat_fun(s/phi);
                u_n = (-k1*sqrt(abs(s))*sig + v) / SB;
                v   = v - k2*sig*h;              % integrator update
            case 'sta_sign'
                % bare-sign super-twisting (chattering baseline)
                u_n = (-k1*sqrt(abs(s))*sign(s) + v) / SB;
                v   = v - k2*sign(s)*h;
        end
        u = -K_eq*xm + u_n;
        u = max(min(u, Vsat), -Vsat);

        % --- log ---
        d = d_fun(t(k));
        X(k,:) = x';  U(k) = u;  Sg(k) = s;
        Sd(k)  = S * f(x, u, d);

        % --- RK4 plant step ---
        d2 = d_fun(t(k) + h/2);
        k1x = f(x,           u, d);
        k2x = f(x + h/2*k1x, u, d2);
        k3x = f(x + h/2*k2x, u, d2);
        k4x = f(x + h*k3x,   u, d_fun(t(k)+h));
        x   = x + h/6*(k1x + 2*k2x + 2*k3x + k4x);
    end
    out.x = X; out.u = U; out.s = Sg; out.s_dot = Sd;
end

function y = sat_fun(z)
    y = max(min(z, 1), -1);
end

function ts = settle_time(t, y, tol)
    idx = find(abs(y) > tol, 1, 'last');
    if isempty(idx) || idx == numel(t)
        ts = NaN;
    else
        ts = t(idx);
    end
end
