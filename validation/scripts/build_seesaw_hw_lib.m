function build_seesaw_hw_lib(varargin)
%BUILD_SEESAW_HW_LIB  Regenerate validation/lib/seesaw_hw_lib.slx.
%
%   build_seesaw_hw_lib()
%   build_seesaw_hw_lib('Force', true)
%
% Two reusable subsystems, deliberately small:
%   DeadzoneInverse   the friction-compensation MATLAB Function from PID_seesaw
%   DisturbanceBank   five sources + MultiPort Switch (Zero/Step/PRBS/Chirp/SteppedSine)
%
% QUARC I/O blocks (HIL Initialize, Encoders, Motor Command, To Host File)
% are NOT in this library on purpose. They live at the top level of each
% HW_*.slx the way SMC_STA_HW_2.slx does it -- you see your I/O flow at a
% glance instead of through a wrapper subsystem.

opts = struct('Force', false);
for k = 1:2:numel(varargin), opts.(varargin{k}) = varargin{k+1}; end

valdir = fullfile(fileparts(fileparts(mfilename('fullpath'))));
lib_path = fullfile(valdir, 'lib', 'seesaw_hw_lib.slx');

if opts.Force && isfile(lib_path), delete(lib_path); end
if isfile(lib_path)
    error('build_seesaw_hw_lib:exists', ...
        'Library exists at %s. Pass ''Force'', true to overwrite.', lib_path);
end

warning('off', 'all');
lib = 'seesaw_hw_lib';
try, bdclose(lib); catch, end
new_system(lib, 'Library');
set_param(lib, 'EnableLBRepository', 'on');

% =========================================================================
% DeadzoneInverse  (from PID_seesaw / Non-linear Motor / MATLAB Function)
% =========================================================================
dz = [lib '/DeadzoneInverse'];
add_block('built-in/Subsystem', dz, 'Position', [60 60 220 160]);
add_block('built-in/Inport',  [dz '/u_lin'],  'Position', [40 60 70 80]);
add_block('built-in/Outport', [dz '/u_real'], 'Position', [400 60 430 80]);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [dz '/Deadzone Inverse'], 'Position', [140 30 320 110]);
sf = sfroot;
chart = sf.find('-isa', 'Stateflow.EMChart', '-and', 'Path', [dz '/Deadzone Inverse']);
chart.Script = sprintf([ ...
    'function u_real = deadzone_inverse(u_lin, ud_pos, ud_neg, V_sat)\n' ...
    '%%#codegen\n' ...
    '%% Replicates PID_seesaw/Non-linear Motor: adds the static-friction\n' ...
    '%% bias the motor needs to start moving, then clamps to the rail.\n' ...
    'epsilon = 0.1;\n' ...
    'if u_lin >  epsilon\n' ...
    '    u_real = u_lin + ud_pos;\n' ...
    'elseif u_lin < -epsilon\n' ...
    '    u_real = u_lin - ud_neg;\n' ...
    'else\n' ...
    '    u_real = 0;\n' ...
    'end\n' ...
    'if u_real >  V_sat, u_real =  V_sat; end\n' ...
    'if u_real < -V_sat, u_real = -V_sat; end\n' ]);

add_block('built-in/Constant', [dz '/ud_pos'], 'Value', 'ud_pos', 'Position', [40 130 70 150]);
add_block('built-in/Constant', [dz '/ud_neg'], 'Value', 'ud_neg', 'Position', [40 160 70 180]);
add_block('built-in/Constant', [dz '/V_sat'],  'Value', 'V_sat',  'Position', [40 190 70 210]);
add_line(dz, 'u_lin/1',  'Deadzone Inverse/1', 'autorouting', 'on');
add_line(dz, 'ud_pos/1', 'Deadzone Inverse/2', 'autorouting', 'on');
add_line(dz, 'ud_neg/1', 'Deadzone Inverse/3', 'autorouting', 'on');
add_line(dz, 'V_sat/1',  'Deadzone Inverse/4', 'autorouting', 'on');
add_line(dz, 'Deadzone Inverse/1', 'u_real/1', 'autorouting', 'on');

% =========================================================================
% DisturbanceBank  (5 sources + MultiPort Switch + selector Constant)
% =========================================================================
db = [lib '/DisturbanceBank'];
add_block('built-in/Subsystem', db, 'Position', [60 200 220 320]);
add_block('built-in/Outport', [db '/d'], 'Position', [400 100 430 120]);

add_block('built-in/Constant', [db '/1 Zero'], 'Value', '0', ...
    'Position', [40 30 90 60]);
add_block('built-in/Step', [db '/2 Step'], 'Time', 'step_time', ...
    'After', 'step_amp', 'Before', '0', 'Position', [40 80 90 110]);
add_block('built-in/FromFile', [db '/3 PRBS'], ...
    'FileName', '_signals/prbs.mat', 'Position', [40 130 110 160]);
add_block('built-in/FromFile', [db '/4 Chirp'], ...
    'FileName', '_signals/chirp.mat', 'Position', [40 180 110 210]);
add_block('built-in/FromFile', [db '/5 SteppedSine'], ...
    'FileName', '_signals/stepped_sine.mat', 'Position', [40 230 110 260]);
add_block('built-in/Constant', [db '/d_select'], 'Value', 'd_select', ...
    'Position', [40 290 110 320]);

add_block('built-in/MultiPortSwitch', [db '/Switch'], 'Inputs', '5', ...
    'DataPortOrder', 'One-based contiguous', ...
    'Position', [240 30 280 280]);
add_line(db, 'd_select/1',      'Switch/1', 'autorouting', 'on');
add_line(db, '1 Zero/1',        'Switch/2', 'autorouting', 'on');
add_line(db, '2 Step/1',        'Switch/3', 'autorouting', 'on');
add_line(db, '3 PRBS/1',        'Switch/4', 'autorouting', 'on');
add_line(db, '4 Chirp/1',       'Switch/5', 'autorouting', 'on');
add_line(db, '5 SteppedSine/1', 'Switch/6', 'autorouting', 'on');
add_line(db, 'Switch/1', 'd/1', 'autorouting', 'on');

save_system(lib, lib_path);
fprintf('Saved library: %s\n', lib_path);
end
