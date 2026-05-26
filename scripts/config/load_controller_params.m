%% load_controller_params.m
% Run ONE section (Ctrl+Enter) to populate the base workspace with the
% tuned params for that controller. Then open the matching test model.
%
% Prereq: run `startup` once per session.
%
% Sections:
%   1. Tuned plant (always run first)
%   2. Pole-placement + integral (SSi)         → SSi_tracking_test.slx
%   3. LQR / LQI                               → SSi_tracking_test.slx (alt gains)
%   4. Cascade PID  (inner cart + outer theta) → PID_seesaw_test.slx
%   5. Super-twisting SMC                      → SMC_STA_HW_2_r2024b_test.slx
%   6. Luenberger observer
%   7. Kalman observer
% -------------------------------------------------------------------------

%% 1. Tuned plant (B_eq, A_sw, B_sw, A_cart, B_cart, deadzone) -- ALWAYS FIRST
load(fullfile(SEESAW_ROOT, 'data/tuned/tuned_params.mat'));
load(fullfile(SEESAW_ROOT, 'data/params/param_nonlinear.mat'));   % ud_pos, ud_neg, ud_sym
fprintf('[1] Plant loaded.  B_eq = %.3f  (was %.3f nominal)\n', B_eq, 5.40);

%% 2. Pole-placement + integral  (for SSi_tracking_test.slx)
% Loads Kf (1x4 pure-state-feedback gains from pole_placement_design.m)
% and aliases to K5d by padding with a zero integral gain for the
% augmented (5-state) controller. Use section 3 instead for LQR-tuned
% integral gain.
load(fullfile(SEESAW_ROOT, 'data/controllers/controller_freq.mat'));   % Kf (1x4)
K5d = [Kf, 0];   % SSi model expects 5-element row; section 3 overrides w/ true LQI
fprintf('[2] PP loaded.  Kf  = %s\n', mat2str(Kf,4));
fprintf('              K5d = [Kf, 0] = %s\n', mat2str(K5d,4));

%% 3. LQR / LQI gains  (alternate K5d for SSi_tracking_test.slx)
load(fullfile(SEESAW_ROOT, 'data/controllers/controller_lqr.mat'));    % K_lqr, Q5, R5, A5, B5
K5d = K_lqr;
fprintf('[3] LQI loaded.  K_lqr = %s\n', mat2str(K_lqr,4));

%% 4. Cascade PID  (for PID_seesaw_test.slx)
load(fullfile(SEESAW_ROOT, 'data/controllers/controller_inner_pid.mat'));  % Kp_in,Ki_in,Kd_in,N_in,...
load(fullfile(SEESAW_ROOT, 'data/controllers/controller_outer_pid.mat'));  % Kp_out,Kd_out,N_out,...
fprintf('[4] PID loaded.  inner Kp=%.2f Ki=%.2f Kd=%.2f   outer Kp=%.3f Kd=%.3f\n', ...
        Kp_in, Ki_in, Kd_in, Kp_out, Kd_out);

%% 5. Super-twisting SMC  (for SMC_STA_HW_2_r2024b_test.slx)
% SMC gains are nested in a `sta_design` struct in the .mat file.
% Pull out scalar vars and skip V_sat / Ts so seesaw_params values stay.
smc = load(fullfile(SEESAW_ROOT, 'data/controllers/controller_smc.mat'), ...
           'sta_design','p_real');
S       = smc.sta_design.S;
K_eq    = smc.sta_design.K_eq;
k1      = smc.sta_design.k1;
k2      = smc.sta_design.k2;
phi_bl  = smc.sta_design.phi_bl;
L_dist  = smc.sta_design.L_dist;
p_real  = smc.p_real;
clear smc
fprintf('[5] SMC loaded.  k1=%.2f  k2=%.2f  phi_bl=%.3f\n', k1, k2, phi_bl);
fprintf('              S = %s\n', mat2str(S,4));

%% 6. Luenberger observer
load(fullfile(SEESAW_ROOT, 'data/params/observer.mat'));   % L, A_obs, B_obs, C_obs, D_obs
fprintf('[6] Luenberger loaded.  L size = %s\n', mat2str(size(L)));

%% 7. Kalman observer
load(fullfile(SEESAW_ROOT, 'data/params/observer_kalman.mat'));  % L (cont), Ld (disc), Qn, Rn, ...
fprintf('[7] Kalman loaded.  L size = %s  Ld size = %s\n', mat2str(size(L)), mat2str(size(Ld)));
