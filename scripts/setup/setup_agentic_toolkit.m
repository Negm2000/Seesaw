% setup_agentic_toolkit.m
% -------------------------------------------------------------------------
% Dynamically locates and initializes the Simulink Agentic Toolkit (OpenCode)
% -------------------------------------------------------------------------

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
