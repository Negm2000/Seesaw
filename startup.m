% startup.m
% -------------------------------------------------------------------------
% Seesaw Project Paths Initialization
% -------------------------------------------------------------------------

fprintf('Initializing Seesaw Project...\n');

% Get the directory where startup.m is located
root_dir = fileparts(mfilename('fullpath'));

% Define subdirectories
subfolders = {'data', 'docs', 'models', 'scripts', 'src'};

% Add all subdirectories to the MATLAB path
for i = 1:length(subfolders)
    folder_path = fullfile(root_dir, subfolders{i});
    if exist(folder_path, 'dir')
        addpath(genpath(folder_path));
        fprintf('  Added to path recursively: %s\n', subfolders{i});
    end
end

% Set the project root as a base workspace variable for scripts to use
assignin('base', 'SEESAW_ROOT', root_dir);

fprintf('>>> Seesaw project paths initialized successfully.\n');
fprintf('    Project Root: %s\n\n', root_dir);
