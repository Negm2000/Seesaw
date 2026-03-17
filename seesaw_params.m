%% seesaw_params.m
%  -----------------------------------------------------------------------
%  Quanser SEESAW-E + IP02 System Parameters
%  All values from official user manuals. Run this script first to
%  populate the MATLAB workspace before using seesaw_nonlinear_model.m
%  -----------------------------------------------------------------------
%  Source documents:
%    [1] Seesaw Experiment User Manual v1.0 (Quanser, 2012)
%    [2] IP02 Base Unit User Manual v1.0 (Quanser, 2012)
%    [3] VoltPAQ-X1 User Manual v1.1 (Quanser, 2013)
%  -----------------------------------------------------------------------

clear; clc;
fprintf('Loading Seesaw + IP02 system parameters...\n');

%% ===== Physical Constants =====
g = 9.81;                   % Gravitational acceleration [m/s^2]

%% ===== VoltPAQ-X1 Amplifier [3] =====
K_a   = 1;                  % Amplifier gain [V/V] -- SET SWITCH TO 1x!
V_sat = 22.0;               % Amplifier output saturation under load [V]
                             % (Manual: 22.3V at 2A, 20.8V at 4A)
                             % NOTE: No current limiting circuit exists in this
                             % signal chain. VoltPAQ is a linear voltage amp
                             % (up to 4A continuous). Motor current is governed
                             % entirely by the electrical dynamics (Ohm's law +
                             % back-EMF). I_max = 1A is a thermal rating only.

%% ===== IP02 DC Motor (Faulhaber 2338S006) [2] =====
R_m      = 2.6;             % Armature resistance [Ohm] (+/- 12%)
L_m      = 0.18e-3;         % Armature inductance [H]
k_t      = 7.68e-3;         % Torque constant [N*m/A] (+/- 12%)
k_m      = 7.68e-3;         % Back-EMF constant [V/(rad/s)] (+/- 12%)
eta_m    = 0.69;            % Motor efficiency [-] (+/- 5%)
J_rotor  = 3.90e-7;         % Rotor moment of inertia [kg*m^2]
V_nom    = 6.0;             % Nominal input voltage [V]
I_max    = 1.0;             % Maximum continuous current [A]
I_peak   = 3.0;             % Maximum peak current [A]
f_max    = 50;              % Maximum input voltage frequency [Hz]

%% ===== IP02 Gearbox (Faulhaber Planetary 23/1) [2] =====
K_g    = 3.71;              % Gearbox gear ratio [-]
eta_g  = 0.90;              % Gearbox efficiency [-] (+/- 10%)

%% ===== IP02 Cart and Rack [2] =====
M_c    = 0.38;              % Cart mass (no weight, no pendulum) [kg]
r_mp   = 6.35e-3;           % Motor pinion radius [m]
r_pp   = 0.01483;           % Position (encoder) pinion radius [m]
B_eq_c = 4.3;               % Equivalent viscous damping, cart only [N*m*s/rad]
                             % NOTE: This is referenced to the motor shaft.
                             % At the cart: B_cart = B_eq_c * eta_g * K_g / r_mp
L_track = 0.990;            % Track length [m]
T_c     = 0.814;            % Cart travel (usable) [m]
x_c_max = T_c / 2;          % Max cart displacement from center [m]

%% ===== IP02 Encoders [2] =====
K_ec   = 2.275e-5;          % Cart encoder resolution [rad/count]
% Cart position from encoder: x_c = K_ec * r_pp * counts

%% ===== SEESAW-E Module [1] =====
M_SW    = 3.6;              % Mass of one SEESAW-E + one IP02 track [kg]
K_gs    = 3;                % Seesaw geartrain gear ratio [-]
D_T     = 0.125;            % Distance: pivot to IP02 track [m]
D_C     = 0.058;            % Distance: pivot to center of gravity [m]
J_SW_cg = 0.395;            % Moment of inertia about center of gravity [kg*m^2]
B_SW    = 0.0;              % Viscous damping at pivot [N*m*s/rad]
                             % Manual says ~0; increase if model overshoots.
                             % Try 0.01-0.1 if needed during tuning.
K_E_SW  = 0.0015;           % Seesaw encoder resolution [rad/count]
alpha_max = 11.5 * pi/180;  % Maximum seesaw tilt angle [rad]

%% ===== Derived Parameters =====

% Moment of inertia about the PIVOT (parallel axis theorem)
% This is J_sw in the Good ref (Quanser Seesaw Laboratory Guide)
J_pivot = J_SW_cg + M_SW * D_C^2;
fprintf('  J_pivot = %.4f kg*m^2 (J_cg + M*Dc^2 = %.3f + %.4f)\n', ...
    J_pivot, J_SW_cg, M_SW * D_C^2);

% Cart force from motor voltage (Good ref Eq. 2.3, reduced model with L_m = 0):
%   F_c = (eta_g * K_g * k_t) / (R_m * r_mp) * (-K_g * k_m * x_c_dot / r_mp + eta_m * V_m)
%
% This is an algebraic equation (no electrical state). The motor inductance
% L_m = 0.18 mH is negligible compared to the mechanical time constants,
% so current reaches steady state instantaneously.
%
% The back-EMF damping is embedded in F_c via the -K_g*k_m*x_c_dot/r_mp term.
% B_eq below is ADDITIONAL mechanical friction beyond what F_c already captures.

% Equivalent damping at cart [N*s/m] (maps to B_eq in Good ref Eq. 2.1)
% This is the mechanical friction only -- back-EMF damping is already inside F_c.
% TUNE THIS against hardware step response data.
B_eq = 5.0;
fprintf('  B_eq = %.2f N*s/m (cart friction, tunable -- Good ref Eq. 2.1)\n', B_eq);

% Motor force gain factor (Good ref Eq. 2.3 prefactor)
alpha_f = (eta_g * K_g * k_t) / (R_m * r_mp);
fprintf('  alpha_f = %.4f (motor force constant)\n', alpha_f);

% Back-EMF damping embedded in F_c
B_emf = alpha_f * K_g * k_m / r_mp;
fprintf('  B_emf = %.2f N*s/m (inside F_c, not added separately)\n', B_emf);

% Total effective damping at cart
B_total = B_eq + B_emf;
fprintf('  B_total = %.2f N*s/m (B_eq + B_emf)\n', B_total);

%% ===== Linear State-Space Model 1: Cart on Table =====
% QUARC-compatible: uses standard State-Space block (no S-function/TLC needed)
%
% Input:  V_m (motor voltage, after amplifier + saturation)
% States: [x_c; x_c_dot]
% Output: [x_c; x_c_dot]
%
%   x_dot = A_cart * x + B_cart * V_m
%   y     = C_cart * x + D_cart * V_m
%
A_cart = [0,  1;
          0, -B_total/M_c];
B_cart = [0;  alpha_f*eta_m/M_c];
C_cart = eye(2);        % output both states [x_c; x_c_dot]
D_cart = zeros(2, 1);
fprintf('  [Phase 1] A_cart, B_cart, C_cart, D_cart computed.\n');

%% ===== Linear State-Space Model 2: Cart on Seesaw (Coupled, Linearised) =====
% Linearised about equilibrium: x_c=0, x_c_dot=0, alpha=0, alpha_dot=0
% QUARC-compatible: uses standard State-Space block (no S-function/TLC needed)
%
% Input:  V_m (motor voltage, after amplifier + saturation)
% States: [x_c; x_c_dot; alpha; alpha_dot]
% Output: [x_c; x_c_dot; alpha; alpha_dot]
%
% Equations of motion (Good ref, linearised, L_m=0):
%
%   (M_c + M_SW)*x_c_ddot + M_SW*D_T*alpha_ddot + B_total*x_c_dot
%       = alpha_f * eta_m * V_m                         ... (Cart EOM)
%
%   (J_pivot + M_SW*D_T^2)*alpha_ddot
%       + M_SW*D_T*x_c_ddot
%       - M_SW*g*D_C*alpha + B_SW*alpha_dot
%       = 0                                              ... (Seesaw EOM)
%
% Solve for [x_c_ddot; alpha_ddot] via inertia matrix inversion:
%
%   M_eff = [ M_c+M_SW,        M_SW*D_T       ]
%           [ M_SW*D_T,  J_pivot+M_SW*D_T^2   ]
%
%   rhs_x = alpha_f*eta_m*V_m  - B_total*x_c_dot
%   rhs_a = M_SW*g*D_C*alpha   - B_SW*alpha_dot
%
%   [x_c_ddot; alpha_ddot] = inv(M_eff) * [rhs_x; rhs_a]

M_eff = [M_c + M_SW,           M_SW*D_T;
         M_SW*D_T,    J_pivot + M_SW*D_T^2];

% Inverse of effective inertia matrix
M_inv = inv(M_eff);

% A matrix coefficients from linearised equations
%   state vector: z = [x_c; x_c_dot; alpha; alpha_dot]
%   rhs damping/stiffness terms (no V_m):
%     x_c row  : -B_total in x_c_dot column
%     alpha row:  M_SW*g*D_C in alpha column, -B_SW in alpha_dot column

G_rhs = [0, -B_total,         0,           0;   % rhs of cart EOM  (vel-dep)
         0,       0,   M_SW*g*D_C,    -B_SW];    % rhs of seesaw EOM (pos+vel-dep)

% Full A_sw (4x4):  state derivative = M_inv * G_rhs * z
%   rows 1,3 (positions) are just velocities; rows 2,4 are accelerations
A_sw = [0, 1, 0, 0;
        M_inv(1,:) * G_rhs;
        0, 0, 0, 1;
        M_inv(2,:) * G_rhs];

% B matrix for voltage input V_m
%   Only cart EOM has input: rhs_x_input = alpha_f*eta_m*V_m
G_inp = [alpha_f*eta_m;
                     0];        % seesaw EOM has no direct voltage input

B_sw = [0;
        M_inv(1,:) * G_inp;
        0;
        M_inv(2,:) * G_inp];

C_sw = eye(4);                  % output all four states
D_sw = zeros(4, 1);

fprintf('  [Phase 2] A_sw, B_sw, C_sw, D_sw computed (linearised seesaw).\n');

% Print eigenvalues so we can quickly check stability
ev = eig(A_sw);
fprintf('  Seesaw SS eigenvalues:\n');
for k = 1:length(ev)
    if imag(ev(k)) ~= 0
        fprintf('    lambda_%d = %.4f %+.4fi\n', k, real(ev(k)), imag(ev(k)));
    else
        fprintf('    lambda_%d = %.4f\n', k, real(ev(k)));
    end
end
if any(real(ev) > 1e-6)
    fprintf('  WARNING: open-loop seesaw is UNSTABLE (expected -- needs control).\n');
end

%% ===== Sensor Calibration =====
% Cart position from encoder counts:
%   x_c [m] = (encoder_counts * K_ec) * r_pp
% Seesaw angle from encoder counts:
%   alpha [rad] = (encoder_counts * K_E_SW) / K_gs
%   (The gear ratio K_gs is between pivot and encoder shaft)

fprintf('\nAll parameters loaded successfully.\n');
fprintf('Amplifier gain K_a = %d (VERIFY switch on VoltPAQ!)\n', K_a);
fprintf('Voltage saturation = +/- %.1f V\n', V_sat);
fprintf('Max angle = +/- %.1f deg (= physical stops)\n', alpha_max * 180/pi);
fprintf('Motor model: L_m = 0 (reduced, matches Good ref Eq. 2.3)\n');
fprintf('  Cart mass m_c = %.3f kg (no reflected rotor inertia -- matches Good ref)\n', M_c);
fprintf('SS matrices ready: A_cart/B_cart/C_cart/D_cart (Phase 1), A_sw/B_sw/C_sw/D_sw (Phase 2)\n');
