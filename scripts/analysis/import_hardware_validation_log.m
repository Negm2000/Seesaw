function import_hardware_validation_log(log_file, experiment_name)
%IMPORT_HARDWARE_VALIDATION_LOG Convert HardwareValidation_HWTest logs.
%
% The model logs these To Host File columns:
%   [time | x_c | alpha | V_m | d | x_fb(1:4) | x_obs(1:4)]
%
% This helper saves the named variables consumed by hardware_verification.m.
%
% Examples:
%   import_hardware_validation_log('data/hw_raw_free.mat', 'free')
%   import_hardware_validation_log('data/hw_raw_step.mat', 'step')
%   import_hardware_validation_log('data/hw_raw_prbs.mat', 'prbs')
%   import_hardware_validation_log('data/hw_raw_obs.mat', 'obs')

if nargin < 2
    error('Usage: import_hardware_validation_log(log_file, experiment_name)');
end

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
experiment_name = validatestring(lower(experiment_name), ...
    {'free', 'step', 'prbs', 'chirp', 'obs'}, mfilename, 'experiment_name');

raw = load(log_file);
Y = extract_log_matrix(raw);
if size(Y, 2) < 5
    error('Expected at least 5 columns: [time x_c alpha V_m d]. Got %d.', size(Y, 2));
end
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

switch experiment_name
    case 'free'
        out_file = fullfile(root, 'data', 'hw_free_run.mat');
        save(out_file, 'hw_t', 'hw_xc', 'hw_alpha', 'hw_vm');

    case 'step'
        out_file = fullfile(root, 'data', 'hw_step_response.mat');
        save(out_file, 'hw_t', 'hw_d', 'hw_xc', 'hw_alpha', 'hw_vm');

    case 'prbs'
        out_file = fullfile(root, 'data', 'hw_prbs_response.mat');
        save(out_file, 'hw_t', 'hw_d', 'hw_xc', 'hw_alpha', 'hw_vm');

    case 'chirp'
        out_file = fullfile(root, 'data', 'hw_chirp_response.mat');
        save(out_file, 'hw_t', 'hw_d', 'hw_xc', 'hw_alpha', 'hw_vm');

    case 'obs'
        out_file = fullfile(root, 'data', 'hw_obs_free.mat');
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
