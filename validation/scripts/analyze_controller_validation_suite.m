function summary = analyze_controller_validation_suite(varargin)
%ANALYZE_CONTROLLER_VALIDATION_SUITE Summarize theta-tracking sim results.
%
% This is intentionally lightweight for now: it establishes the common output
% schema used by the report. Detailed sine-fit Bode extraction can be added on
% top of this without changing the result file organization.

opts = parse_inputs(varargin{:});
result_dir = fullfile(opts.root, 'validation', 'data', 'sim_results');
files = dir(fullfile(result_dir, '*_theta.mat'));

summary = table('Size', [0 8], ...
    'VariableTypes', {'string','string','string','double','double','double','double','double'}, ...
    'VariableNames', {'case_id','controller_id','feedback_source_id','theta_rms_rad', ...
    'theta_peak_rad','V_m_peak_v','V_m_rms_v','sim_stop_s'});

for k = 1:numel(files)
    file = fullfile(files(k).folder, files(k).name);
    d = load(file, 'out', 'c', 'validation_log_columns');
    log = d.out.validation_log;
    t = log.Time;
    data = log.Data;

    % Logger mux order is defined by validation_log_columns in the case config.
    if isfield(d, 'validation_log_columns')
        [V_m_idx, theta_idx] = validation_log_indices(d.validation_log_columns);
    else
        V_m_idx = 7;
        theta_idx = 8;
    end
    theta = squeeze(data(:, theta_idx));
    V_m = squeeze(data(:, V_m_idx));

    row = {erase(files(k).name, '.mat'), string(d.c.controller_id), string(d.c.feedback_source_id), ...
        rms(theta), max(abs(theta)), max(abs(V_m)), rms(V_m), t(end)};
    summary = [summary; row]; %#ok<AGROW>
end

out_file = fullfile(result_dir, 'controller_validation_summary.mat');
save(out_file, 'summary');
disp(summary);
fprintf('Saved validation summary: %s\n', out_file);
end

function opts = parse_inputs(varargin)
if evalin('base', 'exist(''SEESAW_ROOT'', ''var'')')
    default_root = evalin('base', 'SEESAW_ROOT');
else
    default_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
p = inputParser;
addParameter(p, 'root', default_root, @(x) ischar(x) || isstring(x));
parse(p, varargin{:});
opts = p.Results;
opts.root = char(opts.root);
end

function [V_m_idx, theta_idx] = validation_log_indices(columns)
widths = zeros(1, numel(columns));
for i = 1:numel(columns)
    switch columns{i}
        case {'x_ref', 'x_feedback', 'x_measured', 'x_luenberger', 'x_kalman', 'x_state'}
            widths(i) = 4;
        otherwise
            widths(i) = 1;
    end
end
starts = [1 1 + cumsum(widths(1:end-1))];
V_m_idx = starts(strcmp(columns, 'V_m'));
theta_idx = starts(strcmp(columns, 'theta'));
end
