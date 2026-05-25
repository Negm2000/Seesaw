function import_hardware_validation_log(log_file, experiment_name)
%IMPORT_HARDWARE_VALIDATION_LOG Convert HardwareValidation_HWTest logs.
%
% NEW SINGLE-RUN FORMAT (22 data columns + time):
%   [time | seg_id | x_c | alpha | u_ctrl | u_presat | V_m | d |
%    x_fb(1:4) | x_obs_active(1:4) | x_obs_luenb(1:4) | x_obs_kalm(1:4)]
%
% LEGACY FORMAT (13 columns):
%   [time | x_c | alpha | V_m | d | x_fb(1:4) | x_obs(1:4)]
%
% Examples:
%   import_hardware_validation_log('data/hw_raw_single.mat', 'single')
%   import_hardware_validation_log('data/hw_raw_free.mat', 'free')
%   import_hardware_validation_log('data/hw_raw_step.mat', 'step')
%   import_hardware_validation_log('data/hw_raw_prbs.mat', 'prbs')
%   import_hardware_validation_log('data/hw_raw_obs.mat', 'obs')

if nargin < 2
    error('Usage: import_hardware_validation_log(log_file, experiment_name)');
end

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
valdir = fullfile(root, 'validation');
experiment_name = validatestring(lower(experiment_name), ...
    {'single', 'free', 'step', 'prbs', 'chirp', 'obs'}, mfilename, 'experiment_name');

raw = load(log_file);
Y = extract_log_matrix(raw);
if size(Y, 1) < size(Y, 2)
    Y = Y';
end

%% SINGLE-RUN PROTOCOL (new format)
if strcmp(experiment_name, 'single')
    % Expected columns:
    % [time | seg_id | x_c | alpha | u_ctrl | u_presat | V_m | d |
    %  x_fb(4) | x_obs_active(4) | x_obs_luenb(4) | x_obs_kalm(4)]
    % = 24 columns total (or fewer if some observers are disabled)
    
    n_cols = size(Y, 2);
    fprintf('  Single-run log: %d samples x %d columns\n', size(Y, 1), n_cols);
    
    hw_t        = Y(:, 1);
    hw_seg_id   = Y(:, 2);
    hw_xc       = Y(:, 3);
    hw_alpha    = Y(:, 4);
    hw_u_ctrl   = Y(:, 5);
    hw_u_presat = Y(:, 6);
    hw_vm       = Y(:, 7);
    hw_d        = Y(:, 8);
    
    % Feedback state (4 columns)
    if n_cols >= 12
        hw_x_fb = Y(:, 9:12);
    else
        hw_x_fb = NaN(size(Y,1), 4);
    end
    
    % Raw derivative state vector (from x_fb when feedback = dirty derivative)
    hw_xc_dot_raw   = hw_x_fb(:, 2);
    hw_alpha_dot_raw = hw_x_fb(:, 4);
    
    % Active observer state (4 columns)
    if n_cols >= 16
        hw_x_obs_active = Y(:, 13:16);
    else
        hw_x_obs_active = NaN(size(Y,1), 4);
    end
    
    % Luenberger observer state (4 columns)
    if n_cols >= 20
        hw_x_obs_luenb = Y(:, 17:20);
    else
        hw_x_obs_luenb = NaN(size(Y,1), 4);
    end
    
    % Kalman observer state (4 columns)
    if n_cols >= 24
        hw_x_obs_kalm = Y(:, 21:24);
    else
        hw_x_obs_kalm = NaN(size(Y,1), 4);
    end
    
    out_file = fullfile(valdir, 'data', 'hw_single_run.mat');
    save(out_file, 'hw_t', 'hw_seg_id', 'hw_xc', 'hw_alpha', ...
        'hw_u_ctrl', 'hw_u_presat', 'hw_vm', 'hw_d', ...
        'hw_xc_dot_raw', 'hw_alpha_dot_raw', ...
        'hw_x_fb', 'hw_x_obs_active', 'hw_x_obs_luenb', 'hw_x_obs_kalm');
    fprintf('  Saved: %s\n', out_file);
    return;
end

%% LEGACY FORMATS (backward compatible)
if size(Y, 2) < 5
    error('Expected at least 5 columns: [time x_c alpha V_m d]. Got %d.', size(Y, 2));
end

if size(Y, 2) == 10
    % Legacy LQR observer log format:
    % [time, x_c_ref, alpha, alpha_ref, x_c, V_m, x_c_hat, alpha_hat, x_c_dot_hat, alpha_dot_hat]
    hw_t = Y(:, 1);
    hw_xc = Y(:, 5);
    hw_alpha = Y(:, 3);
    hw_vm = Y(:, 6);
    hw_d = zeros(size(hw_t));
    
    hw_xc_fb = Y(:, 5);
    hw_xcdot_fb = NaN(size(hw_t));
    hw_alpha_fb = Y(:, 3);
    hw_alphadot_fb = NaN(size(hw_t));
    
    hw_xc_hat = Y(:, 7);
    hw_xcdot_hat = Y(:, 9);
    hw_alpha_hat = Y(:, 8);
    hw_alphadot_hat = Y(:, 10);
else
    if size(Y, 2) < 13
        warning('Expected 13 columns from HardwareValidation_HWTest; missing feedback/observer columns will be NaN.');
        Y(:, end+1:13) = NaN;
    end
    
    hw_t = Y(:, 1);
    hw_xc = Y(:, 2);
    hw_alpha = Y(:, 3);
    hw_vm = Y(:, 4);
    hw_d = Y(:, 5);
    
    hw_xc_fb = Y(:, 6);
    hw_xcdot_fb = Y(:, 7);
    hw_alpha_fb = Y(:, 8);
    hw_alphadot_fb = Y(:, 9);
    
    hw_xc_hat = Y(:, 10);
    hw_xcdot_hat = Y(:, 11);
    hw_alpha_hat = Y(:, 12);
    hw_alphadot_hat = Y(:, 13);
end

switch experiment_name
    case 'free'
        out_file = fullfile(valdir, 'data', 'hw_free_run.mat');
        save(out_file, 'hw_t', 'hw_xc', 'hw_alpha', 'hw_vm');

    case 'step'
        out_file = fullfile(valdir, 'data', 'hw_step_response.mat');
        save(out_file, 'hw_t', 'hw_d', 'hw_xc', 'hw_alpha', 'hw_vm');

    case 'prbs'
        out_file = fullfile(valdir, 'data', 'hw_prbs_response.mat');
        save(out_file, 'hw_t', 'hw_d', 'hw_xc', 'hw_alpha', 'hw_vm');

    case 'chirp'
        out_file = fullfile(valdir, 'data', 'hw_chirp_response.mat');
        save(out_file, 'hw_t', 'hw_d', 'hw_xc', 'hw_alpha', 'hw_vm');

    case 'obs'
        out_file = fullfile(valdir, 'data', 'hw_obs_free.mat');
        save(out_file, 'hw_t', 'hw_xc', 'hw_alpha', 'hw_vm', ...
            'hw_xc_hat', 'hw_xcdot_hat', 'hw_alpha_hat', 'hw_alphadot_hat', ...
            'hw_xc_fb', 'hw_xcdot_fb', 'hw_alpha_fb', 'hw_alphadot_fb');
end

fprintf('Saved %s\n', out_file);
end

function Y = extract_log_matrix(raw)
fields = fieldnames(raw);
if isscalar(fields) && isnumeric(raw.(fields{1}))
    Y = raw.(fields{1});
    return;
end

preferred = {'data', 'ans', 'y', 'Y', 'logs', 'log'};
for k = 1:numel(preferred)
    if isfield(raw, preferred{k}) && isnumeric(raw.(preferred{k}))
        Y = raw.(preferred{k});
        return;
    end
end

for k = 1:numel(fields)
    value = raw.(fields{k});
    if isnumeric(value) && ismatrix(value) && size(value, 2) >= 5
        Y = value;
        return;
    end
end

error('Could not find a numeric log matrix in the file.');
end
