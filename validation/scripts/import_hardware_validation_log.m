function import_hardware_validation_log(log_file, experiment_name)
%IMPORT_HARDWARE_VALIDATION_LOG Convert Seesaw_Validation logs.
%
% SINGLE-RUN FORMAT (17 data columns + time = 18 total):
%   [time | seg_id | xc | alpha | xc_dot | alpha_dot | u_ctrl | u_presat | Vm | d_inj |
%    xc_hat_L | alpha_hat_L | xc_dot_hat_L | alpha_dot_hat_L |
%    xc_hat_K | alpha_hat_K | xc_dot_hat_K | alpha_dot_hat_K]
%
% LEGACY FORMAT (13 columns):
%   [time | x_c | alpha | V_m | d | x_fb(1:4) | x_obs(1:4)]
%
% Examples:
%   import_hardware_validation_log('data/hw_raw_single.mat', 'single')
%   import_hardware_validation_log('data/hw_raw_free.mat', 'free')

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

%% SINGLE-RUN PROTOCOL (new format with observer states)
if strcmp(experiment_name, 'single')
    % Expected columns from Seesaw_Validation.slx:
    % [time | seg_id | xc | alpha | xc_dot | alpha_dot | u_ctrl | u_presat | Vm | d_inj |
    %  xc_hat_L | alpha_hat_L | xc_dot_hat_L | alpha_dot_hat_L |
    %  xc_hat_K | alpha_hat_K | xc_dot_hat_K | alpha_dot_hat_K]
    % = 18 columns total
    
    n_cols = size(Y, 2);
    fprintf('  Single-run log: %d samples x %d columns\n', size(Y, 1), n_cols);
    
    if n_cols < 18
        warning('Expected 18 columns [time + 17 signals]. Got %d. Padding with NaN.', n_cols);
        Y(:, end+1:18) = NaN;
    end
    
    hw_t         = Y(:, 1);
    hw_seg_id    = Y(:, 2);
    hw_xc        = Y(:, 3);
    hw_alpha     = Y(:, 4);
    hw_xc_dot    = Y(:, 5);
    hw_alpha_dot = Y(:, 6);
    hw_u_ctrl    = Y(:, 7);
    hw_u_presat  = Y(:, 8);
    hw_vm        = Y(:, 9);
    hw_d         = Y(:, 10);
    
    % Luenberger observer estimates
    hw_xc_hat_L        = Y(:, 11);
    hw_alpha_hat_L     = Y(:, 12);
    hw_xc_dot_hat_L    = Y(:, 13);
    hw_alpha_dot_hat_L = Y(:, 14);
    
    % Kalman observer estimates
    hw_xc_hat_K        = Y(:, 15);
    hw_alpha_hat_K     = Y(:, 16);
    hw_xc_dot_hat_K    = Y(:, 17);
    hw_alpha_dot_hat_K = Y(:, 18);
    
    out_file = fullfile(valdir, 'data', 'hw_single_run.mat');
    save(out_file, 'hw_t', 'hw_seg_id', 'hw_xc', 'hw_alpha', ...
        'hw_xc_dot', 'hw_alpha_dot', ...
        'hw_u_ctrl', 'hw_u_presat', 'hw_vm', 'hw_d', ...
        'hw_xc_hat_L', 'hw_alpha_hat_L', 'hw_xc_dot_hat_L', 'hw_alpha_dot_hat_L', ...
        'hw_xc_hat_K', 'hw_alpha_hat_K', 'hw_xc_dot_hat_K', 'hw_alpha_dot_hat_K');
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
