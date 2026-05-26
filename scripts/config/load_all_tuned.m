function load_all_tuned(varargin)
% LOAD_ALL_TUNED  Push every tuned controller / observer / plant param
%                 into the base workspace, in one call.
%
% Usage (from any script or the command window):
%   load_all_tuned                  % load everything that exists; warn on missing
%   load_all_tuned('strict')        % error instead of warn on any missing artifact
%   load_all_tuned('quiet')         % no per-file messages, only final summary
%
% After this call the base workspace contains (when files exist on disk):
%
%   Plant (tuned)
%     B_eq, B_total, alpha_f, B_emf, eta_g
%     A_cart, B_cart, C_cart, D_cart
%     A_sw,  B_sw,  C_sw,  D_sw
%     ud_pos, ud_neg, ud_sym                 (cart deadzone)
%
%   Pole-placement (frequency-domain design)
%     Kf            (1x4)
%     K5d           (1x5, integral-augmented; aliased from controller_freq if absent)
%     p_final, sigma_th, zeta_th, M_c_added
%
%   LQR / LQI
%     K_lqr  (=K5, 1x5),  Q5, R5, A5, B5
%
%   Cascade PID
%     Inner (cart):  Kp_in, Ki_in, Kd_in, N_in, antiwindup_in, C_in
%     Outer (theta): Kp_out, Ki_out, Kd_out, N_out, antiwindup_out, C_theta
%     K_aug          (1x5, if controller_pid.mat present - augmented PID variant)
%
%   Luenberger observer
%     L_obs, A_obs_L, B_obs_L, C_obs_L, D_obs_L, p_obs
%
%   Kalman observer
%     L_kf, Ld, Qn, Rn, A_obs_K, B_obs_K, C_obs_K, D_obs_K
%
%   Sliding-mode (super-twisting)
%     S, K_eq, k1, k2, phi_bl, L_dist, p_slide
%     V_sat_smc, Ts_smc   (renamed to avoid clobbering seesaw_params)
%
% Notes
%   - Requires `SEESAW_ROOT` in base workspace. Calls startup if missing.
%   - Reads from the SUBFOLDER locations (data/controllers, data/params,
%     data/tuned), not the flat data/ paths that some design scripts write to.
%   - Idempotent. Safe to call repeatedly.
%   - V_sat from controller_smc.mat is renamed to V_sat_smc so the +/-6 V
%     limit set by your test models / seesaw_params is preserved.
%
% See also: SEESAW_PARAMS, STARTUP

% ------------------------------------------------------------------------
% Parse options
% ------------------------------------------------------------------------
strict = any(strcmpi(varargin, 'strict'));
quiet  = any(strcmpi(varargin, 'quiet'));

% ------------------------------------------------------------------------
% Resolve SEESAW_ROOT
% ------------------------------------------------------------------------
try
    root = evalin('base', 'SEESAW_ROOT');
catch
    warning('load_all_tuned:NoRoot', ...
        'SEESAW_ROOT not set in base workspace. Running startup...');
    evalin('base', 'startup');
    root = evalin('base', 'SEESAW_ROOT');
end

% Make sure seesaw_params has run (sets V_sat, Ts, nominal A/B/C/D, ...)
if ~evalin('base', 'exist(''V_sat'',''var'') && exist(''Ts'',''var'')')
    if ~quiet, fprintf('  (running seesaw_params first)\n'); end
    evalin('base', 'seesaw_params');
end

% ------------------------------------------------------------------------
% Spec table: each row is one .mat file to load
%   {  relative_path,
%      cellarray of {src_var_in_mat,  base_ws_name}   }
% Use {} as the second column to mean "load everything as-is".
% ------------------------------------------------------------------------
spec = {
    % --- Plant (tuned) ----------------------------------------------------
    'data/tuned/tuned_params.mat',           {}    % B_eq, A_sw, B_sw, A_cart, B_cart, etc.
    'data/params/param_nonlinear.mat',       {'ud_pos','ud_pos'; ...
                                              'ud_neg','ud_neg'; ...
                                              'ud_sym','ud_sym'}

    % --- Pole placement (Kf, also exports K_pp alias) ---------------------
    'data/controllers/controller_freq.mat',  {'Kf','Kf'; ...
                                              'p_final','p_final'; ...
                                              'sigma_th','sigma_th'; ...
                                              'zeta_th','zeta_th'; ...
                                              'p3','p3'; ...
                                              'p4','p4'; ...
                                              'M_c_added','M_c_added'; ...
                                              'V_noise_th','V_noise_th'}

    % --- LQR / LQI --------------------------------------------------------
    'data/controllers/controller_lqr.mat',   {'K_lqr','K_lqr'; ...
                                              'Q5','Q5'; ...
                                              'R5','R5'; ...
                                              'A5','A5'; ...
                                              'B5','B5'}

    % --- Cascade PID ------------------------------------------------------
    'data/controllers/controller_inner_pid.mat', {'Kp_in','Kp_in'; ...
                                                  'Ki_in','Ki_in'; ...
                                                  'Kd_in','Kd_in'; ...
                                                  'N_in','N_in'; ...
                                                  'antiwindup_in','antiwindup_in'; ...
                                                  'C_in','C_in'; ...
                                                  'L_in','L_in'; ...
                                                  'T_in','T_in'}
    'data/controllers/controller_outer_pid.mat', {'Kp_out','Kp_out'; ...
                                                  'Ki_out','Ki_out'; ...
                                                  'Kd_out','Kd_out'; ...
                                                  'N_out','N_out'; ...
                                                  'antiwindup_out','antiwindup_out'; ...
                                                  'C_theta','C_theta'; ...
                                                  'L_theta','L_theta'; ...
                                                  'T_theta','T_theta'}
    'data/controllers/controller_pid.mat',       {'K_aug','K_aug'}

    % --- Luenberger observer (rename to *_L to avoid Kalman clash) --------
    'data/params/observer.mat',              {'L','L_obs'; ...
                                              'A_obs','A_obs_L'; ...
                                              'B_obs','B_obs_L'; ...
                                              'C_obs','C_obs_L'; ...
                                              'D_obs','D_obs_L'; ...
                                              'p_obs','p_obs'}

    % --- Kalman observer (rename to *_K) ---------------------------------
    'data/params/observer_kalman.mat',       {'L','L_kf'; ...
                                              'Ld','Ld'; ...
                                              'Qn','Qn'; ...
                                              'Rn','Rn'; ...
                                              'A_obs','A_obs_K'; ...
                                              'B_obs','B_obs_K'; ...
                                              'C_obs','C_obs_K'; ...
                                              'D_obs','D_obs_K'}

    % --- SMC (rename V_sat/Ts to avoid clobbering seesaw_params) ---------
    'data/controllers/controller_smc.mat',   {'S','S'; ...
                                              'K_eq','K_eq'; ...
                                              'k1','k1'; ...
                                              'k2','k2'; ...
                                              'phi_bl','phi_bl'; ...
                                              'L_dist','L_dist'; ...
                                              'p_slide','p_slide'; ...
                                              'V_sat','V_sat_smc'; ...
                                              'Ts','Ts_smc'}
};

% ------------------------------------------------------------------------
% Load each entry
% ------------------------------------------------------------------------
loaded  = {};   % cellstr of files successfully loaded
missing = {};   % cellstr of files not on disk
nVars   = 0;

for k = 1:size(spec,1)
    relpath = spec{k,1};
    mapping = spec{k,2};
    fullp   = fullfile(root, relpath);

    if ~exist(fullp, 'file')
        missing{end+1} = relpath; %#ok<AGROW>
        if strict
            error('load_all_tuned:MissingFile', ...
                'Required artifact not found: %s', relpath);
        end
        continue
    end

    try
        S = load(fullp);
    catch ME
        warning('load_all_tuned:LoadFailed', ...
            'Could not load %s: %s', relpath, ME.message);
        continue
    end

    if isempty(mapping)
        % Push every variable from the .mat as-is
        fn = fieldnames(S);
        for j = 1:numel(fn)
            assignin('base', fn{j}, S.(fn{j}));
        end
        nVars = nVars + numel(fn);
    else
        % Push only the explicitly mapped variables
        for j = 1:size(mapping,1)
            src = mapping{j,1};
            dst = mapping{j,2};
            if isfield(S, src)
                assignin('base', dst, S.(src));
                nVars = nVars + 1;
            end
        end
    end

    loaded{end+1} = relpath; %#ok<AGROW>
    if ~quiet
        fprintf('  loaded  %s\n', relpath);
    end
end

% ------------------------------------------------------------------------
% Optional alias: K5d  <-  Kf padded with zero (some test models expect K5d)
% ------------------------------------------------------------------------
if evalin('base', 'exist(''K_lqr'',''var'')')
    evalin('base', 'K5d = K_lqr;');
elseif evalin('base', 'exist(''Kf'',''var'')')
    evalin('base', 'K5d = [Kf, 0];');
end

% ------------------------------------------------------------------------
% Summary
% ------------------------------------------------------------------------
fprintf('\nload_all_tuned: %d file(s) loaded, %d variable(s) pushed to base ws\n', ...
        numel(loaded), nVars);
if ~isempty(missing)
    fprintf('  Missing (skipped):\n');
    for k = 1:numel(missing)
        fprintf('    - %s\n', missing{k});
    end
    fprintf('  Run the corresponding design script to regenerate.\n');
end

end
