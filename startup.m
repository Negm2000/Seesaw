% startup.m
% -------------------------------------------------------------------------
% Seesaw Project Paths Initialization
% -------------------------------------------------------------------------

fprintf('Initializing Seesaw Project...\n');

% setting default parameter with LaTeX interpreter
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

% Get the directory where startup.m is located
root_dir = fileparts(mfilename('fullpath'));

% Initialize the Simulink Agentic Toolkit from the current user's profile
user_home = getenv('USERPROFILE');
if isempty(user_home)
    user_home = getenv('HOME');
end

% Try both work PC and personal PC install locations
work_pc_dir = fullfile(user_home, '.config', 'opencode', 'servers', 'simulink-agentic-toolkit');
personal_pc_dir = fullfile(user_home, '.matlab', 'agentic-toolkits', 'simulink');

simulink_toolkit_dir = '';
mcp_server_path = '';

if exist(work_pc_dir, 'dir')
    simulink_toolkit_dir = work_pc_dir;
    mcp_server_path = fullfile(user_home, '.config', 'opencode', 'bin', 'matlab-mcp-core-server-win64-v0.9.1.exe');
elseif exist(personal_pc_dir, 'dir')
    simulink_toolkit_dir = personal_pc_dir;
    mcp_server_path = fullfile(user_home, '.matlab', 'agentic-toolkits', 'bin', 'matlab-mcp-core-server.exe');
end

if ~isempty(simulink_toolkit_dir)
    addpath(simulink_toolkit_dir);
    if exist('satk_initialize', 'file')
        try
            if exist(mcp_server_path, 'file')
                satk_initialize(MCPServerPath=mcp_server_path);
            else
                satk_initialize;
            end
            fprintf('  Simulink Agentic Toolkit initialized.\n');
        catch ME
            fprintf('  Simulink Agentic Toolkit initialization skipped: %s\n', ME.message);
        end
    else
        fprintf('  Simulink Agentic Toolkit path added, but satk_initialize was not found.\n');
    end
end


% Define subdirectories
subfolders = {'data', 'docs', 'models', 'scripts', 'src', 'validation'};

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

% Configure Simulink build artifacts redirection
build_dir = fullfile(root_dir, 'build');
if ~exist(build_dir, 'dir')
    mkdir(build_dir);
end
Simulink.fileGenControl('set', 'CacheFolder', build_dir, 'CodeGenFolder', build_dir);

% Load system parameters
seesaw_params;

fprintf('>>> Seesaw project paths and parameters initialized successfully.\n');
fprintf('    Project Root: %s\n\n', root_dir);
