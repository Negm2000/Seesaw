%% CART PI PIPELINE — Quanser IP02 Cart on Table
%  PI position control, tuned via sisotool. 8V transient motor limit.
%
%  Plant:  G(s) = alpha_f * eta_m / (M_c * s^2 + B_total * s)
%  Controller: PI  →  C(s) = Kp + Ki/s
%
%  Sections:
%    §1  Load parameters & build plant
%    §2  PI tuning (pidtune → sisotool), constrained to 8V
%    §3  Step response check (with 8V saturation)
%    §4  Build Simulink model
%    §5  QUARC hardware deployment
%
%  Prerequisites: Run startup.m

%% 1. LOAD PARAMETERS & BUILD PLANT

if ~exist('SEESAW_ROOT', 'var')
    SEESAW_ROOT = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end

seesaw_params;

% Use tuned B_eq if available
tp = fullfile(SEESAW_ROOT, 'data', 'tuned_params.mat');
if exist(tp, 'file')
    tuned = load(tp);
    B_eq = tuned.B_eq;
    fprintf('Using tuned B_eq = %.3f N*s/m\n', B_eq);
end

% Override V_sat for transient operation
%   seesaw_params sets V_sat = 6V (motor continuous/nominal rating).
%   The real constraint is peak current: I_peak = 3A (demagnetization).
%   At stall: V_max = I_peak * R_m = 3 * 2.6 = 7.8V.
%   Use 8V — right at I_peak at stall, and once the cart moves,
%   back-EMF reduces current well below that.
V_sat = 8;

alpha_f = (eta_g * K_g * k_t) / (R_m * r_mp);
B_emf   = alpha_f * K_g * k_m / r_mp;
B_total = B_eq + B_emf;

% State-space
A_cart    = [0, 1; 0, -B_total/M_c];
B_cart_ss = [0; alpha_f*eta_m/M_c];
C_cart    = [1, 0];   % output = position only
D_cart    = 0;

% Transfer function: V_m → x_c
s = tf('s');
G_xc = alpha_f * eta_m / (M_c * s^2 + B_total * s);

fprintf('Plant: G(s) = %.4f / (%.4f s^2 + %.4f s)\n', ...
    alpha_f*eta_m, M_c, B_total);
fprintf('Poles: '); fprintf('%.4f  ', pole(G_xc)); fprintf('\n');
fprintf('V_sat = %.1f V\n', V_sat);

%% 2. PI TUNING (pidtune → sisotool)
%  pidtune('PI') gives a starting point that respects 8V for a 25cm step.
%  Then sisotool opens for interactive refinement.
%  Close sisotool when done — gains are extracted automatically.

fprintf('\n--- §2: PI Tuning ---\n');

% --- Find max crossover frequency that stays within V_sat ---
%  For a step of x_ref [m], the peak voltage from C(s)*S(s)*x_ref
%  must stay under V_sat.
x_ref_design = 0.25;  % 25 cm design step (~61% of usable track ±40.7cm)

% Sensitivity TF: S = 1/(1+C*G),  voltage = C*S*ref
peak_V = @(C_pi) max(abs(step( ...
    tf(C_pi) * feedback(1, tf(C_pi)*G_xc) * x_ref_design, 0:0.001:5)));

% Binary search for highest bandwidth that keeps V_peak <= V_sat
C_default = pidtune(G_xc, 'PI');
[~, ~, ~, wc_max] = margin(C_default * G_xc);

wc_lo = 0.5;  wc_hi = wc_max;  wc_best = wc_lo;
for iter = 1:20
    wc = (wc_lo + wc_hi) / 2;
    C_try = pidtune(G_xc, 'PI', wc);
    if peak_V(C_try) <= V_sat
        wc_best = wc;
        wc_lo = wc;
    else
        wc_hi = wc;
    end
    if (wc_hi - wc_lo) < 0.05, break; end
end

C_pi0 = pidtune(G_xc, 'PI', wc_best);
fprintf('pidtune (V_sat-constrained):  Kp=%.3f  Ki=%.3f  wc=%.2f rad/s\n', ...
    C_pi0.Kp, C_pi0.Ki, wc_best);
fprintf('Peak voltage for %.0fcm step: %.2f V (limit %.1f V)\n', ...
    x_ref_design*100, peak_V(C_pi0), V_sat);

% Clear previous exports so we detect a fresh one
evalin('base', 'clear C DesignTask');

fprintf('\nOpening sisotool with voltage-constrained starting point.\n');
fprintf('  1. Tune your PI (drag root locus, reshape Bode, etc.)\n');
fprintf('  2. Close sisotool → click "Export" when prompted\n');
fprintf('     (exports compensator as variable "C" to workspace)\n');
fprintf('  3. Come back here and press ENTER\n');
sisotool(G_xc, tf(C_pi0));
input('  >> Press ENTER after exporting from sisotool...', 's');

% --- Extract tuned compensator ---
C_tuned = [];

% Method 1: 'C' in workspace (standard export name)
try
    if evalin('base', 'exist(''C'',''var'')')
        C_tuned = pid(tf(evalin('base', 'C')));
        fprintf('Extracted compensator from workspace variable "C".\n');
    end
catch, end

% Method 2: 'DesignTask' (some MATLAB versions)
if isempty(C_tuned)
    try
        if evalin('base', 'exist(''DesignTask'',''var'')')
            DT = evalin('base', 'DesignTask');
            C_tuned = pid(tf(DT.Compensator));
            fprintf('Extracted compensator from DesignTask.\n');
        end
    catch, end
end

% Method 3: manual fallback
if isempty(C_tuned)
    fprintf('\nNo exported compensator found in workspace.\n');
    fprintf('Enter Kp and Ki manually (read from sisotool display):\n');
    Kp_cart = input('  Kp = ');
    Ki_cart = input('  Ki = ');
    C_tuned = pid(Kp_cart, Ki_cart);
end

if ~isa(C_tuned, 'pid'), C_tuned = pid(C_tuned); end

Kp_cart = C_tuned.Kp;
Ki_cart = C_tuned.Ki;

fprintf('\nFinal PI:  Kp=%.4f  Ki=%.4f\n', Kp_cart, Ki_cart);

% Quick voltage check on the tuned controller
V_check = peak_V(C_tuned);
if V_check > V_sat
    fprintf('WARNING: Peak voltage %.2f V exceeds %.1f V limit!\n', V_check, V_sat);
    fprintf('Re-run sisotool with lower bandwidth.\n');
else
    fprintf('Peak voltage: %.2f V — OK (limit %.1f V)\n', V_check, V_sat);
end

%% 3. STEP RESPONSE CHECK (with saturation)

fprintf('\n--- §3: Step Response ---\n');

T_sim = 6;  dt = 0.001;
t = (0:dt:T_sim)';
x_ref = x_ref_design;  t_step = 1.0;

sat_v = @(v) max(-V_sat, min(V_sat, v));

% States: [x_c, x_c_dot, xi]
x = zeros(3,1);
x_hist = zeros(length(t), 3);
u_hist = zeros(length(t), 1);

for k = 1:length(t)
    r = (t(k) >= t_step) * x_ref;
    x_hist(k,:) = x';

    e = r - x(1);
    u_raw = Kp_cart*e + Ki_cart*x(3);
    u = sat_v(u_raw);
    u_hist(k) = u;

    % Plant
    x(1) = x(1) + dt * x(2);
    x(2) = x(2) + dt * (-B_total*x(2) + alpha_f*eta_m*u) / M_c;

    % Integrator — freeze when saturated (simple anti-windup)
    if abs(u_raw) > V_sat && sign(u_raw) == sign(e)
        % clamped
    else
        x(3) = x(3) + dt * e;
    end
end

figure('Name', 'Cart PI — Step Response', 'Position', [100 100 800 500]);

subplot(2,1,1);
plot(t, x_hist(:,1)*100, 'b', 'LineWidth', 1.5); hold on;
yline(x_ref*100, 'k--'); xline(t_step, ':');
ylabel('x_c [cm]'); title('Cart Position'); grid on;

subplot(2,1,2);
plot(t, u_hist, 'r', 'LineWidth', 1.5);
yline([V_sat, -V_sat], 'k--');
ylabel('V_m [V]'); xlabel('Time [s]'); title('Motor Voltage'); grid on;

sgtitle(sprintf('PI Step Response — Kp=%.2f Ki=%.2f  (V_{sat}=%.0fV)', ...
    Kp_cart, Ki_cart, V_sat));

fprintf('Peak voltage: %.2f V (limit %.1f V)\n', max(abs(u_hist)), V_sat);

%% 4. BUILD SIMULINK MODEL

fprintf('\n--- §4: Build Simulink Model ---\n');

mdl = 'IP02_CartPI';

assignin('base', 'Kp_cart', Kp_cart);
assignin('base', 'Ki_cart', Ki_cart);
assignin('base', 'V_sat',   V_sat);
assignin('base', 'A_cart',  A_cart);
assignin('base', 'B_cart_ss', B_cart_ss);
assignin('base', 'C_cart',  C_cart);
assignin('base', 'D_cart',  D_cart);

% Detect QUARC
quarc_available = exist('quarc_library', 'file') == 4 || ...
                  ~isempty(which('hil_initialize_block'));
if ~quarc_available
    try, quarc_available = ~isempty(ver('quarc')); catch, end
end

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);

if quarc_available
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', ...
        'FixedStep', '0.002', 'StopTime', 'inf');
else
    set_param(mdl, 'Solver', 'ode45', 'StopTime', '10', 'MaxStep', '1e-3');
end

% Reference
add_block('simulink/Sources/Step', [mdl '/x_c_ref'], ...
    'Time', '2', 'Before', '0', 'After', num2str(x_ref_design), ...
    'Position', [50 95 90 135]);

% Error sum
add_block('simulink/Math Operations/Sum', [mdl '/Sum_err'], ...
    'Inputs', '+-', 'Position', [170 100 190 120]);

% PI controller with anti-windup
add_block('simulink/Continuous/PID Controller', [mdl '/PI_Cart'], ...
    'Controller', 'PI', ...
    'P', 'Kp_cart', 'I', 'Ki_cart', ...
    'AntiWindupMode', 'clamping', ...
    'UpperSaturationLimit', 'V_sat', ...
    'LowerSaturationLimit', '-V_sat', ...
    'Position', [260 87 350 133]);

% Voltage saturation
add_block('simulink/Discontinuities/Saturation', [mdl '/V_Sat'], ...
    'UpperLimit', 'V_sat', 'LowerLimit', '-V_sat', ...
    'Position', [410 98 460 132]);

% Forward path
add_line(mdl, 'x_c_ref/1', 'Sum_err/1',   'autorouting', 'smart');
add_line(mdl, 'Sum_err/1',  'PI_Cart/1',   'autorouting', 'smart');
add_line(mdl, 'PI_Cart/1',  'V_Sat/1',     'autorouting', 'smart');

% Scopes
add_block('simulink/Math Operations/Gain', [mdl '/m_to_cm'], ...
    'Gain', '100', 'Position', [800 115 840 135]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_Cart'], ...
    'NumInputPorts', '2', 'Position', [920 80 960 130]);
set_param([mdl '/Scope_Cart'], 'Name', 'Cart [cm]');
add_block('simulink/Sinks/Scope', [mdl '/Scope_Vm'], ...
    'NumInputPorts', '1', 'Position', [920 200 960 230]);
set_param([mdl '/Scope_Vm'], 'Name', 'Voltage [V]');

% To Workspace
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_xc'], ...
    'VariableName', 'cart_xc', 'SaveFormat', 'Timeseries', ...
    'Position', [920 310 990 330]);
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_Vm'], ...
    'VariableName', 'cart_Vm', 'SaveFormat', 'Timeseries', ...
    'Position', [920 345 990 365]);

% Ref → scope
add_block('simulink/Math Operations/Gain', [mdl '/ref_cm'], ...
    'Gain', '100', 'Position', [800 65 840 85]);
add_line(mdl, 'x_c_ref/1', 'ref_cm/1',    'autorouting', 'smart');
add_line(mdl, 'ref_cm/1',  'Cart [cm]/1',  'autorouting', 'smart');

% Voltage → scope + workspace
add_line(mdl, 'V_Sat/1', 'Voltage [V]/1', 'autorouting', 'smart');
add_line(mdl, 'V_Sat/1', 'ToWS_Vm/1',     'autorouting', 'smart');

% Feedback path
if quarc_available
    fprintf('Adding QUARC hardware I/O...\n');
    add_block('quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize', ...
        [mdl '/HIL Initialize'], 'Position', [50 350 134 425]);
    add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Write Analog', ...
        [mdl '/Motor Command'], 'Position', [520 98 605 132]);
    add_block('quarc_library/Data Acquisition/Generic/Immediate I//O/HIL Read Encoder', ...
        [mdl '/Encoder'], 'Position', [50 450 135 510]);
    add_block('simulink/Math Operations/Gain', [mdl '/Enc_to_m'], ...
        'Gain', 'K_ec', 'Position', [200 465 260 495]);

    add_line(mdl, 'V_Sat/1',    'Motor Command/1', 'autorouting', 'smart');
    add_line(mdl, 'Encoder/1',  'Enc_to_m/1',      'autorouting', 'smart');
    add_line(mdl, 'Enc_to_m/1', 'Sum_err/2',       'autorouting', 'smart');
    add_line(mdl, 'Enc_to_m/1', 'm_to_cm/1',       'autorouting', 'smart');
    add_line(mdl, 'm_to_cm/1',  'Cart [cm]/2',     'autorouting', 'smart');
    add_line(mdl, 'Enc_to_m/1', 'ToWS_xc/1',       'autorouting', 'smart');
else
    fprintf('Adding simulation plant...\n');
    add_block('simulink/Continuous/State-Space', [mdl '/Cart_SS'], ...
        'A', 'A_cart', 'B', 'B_cart_ss', 'C', 'C_cart', 'D', 'D_cart', ...
        'X0', '[0; 0]', 'Position', [540 80 650 160]);

    add_line(mdl, 'V_Sat/1',    'Cart_SS/1',   'autorouting', 'smart');
    add_line(mdl, 'Cart_SS/1',  'Sum_err/2',   'autorouting', 'smart');
    add_line(mdl, 'Cart_SS/1',  'm_to_cm/1',   'autorouting', 'smart');
    add_line(mdl, 'm_to_cm/1',  'Cart [cm]/2', 'autorouting', 'smart');
    add_line(mdl, 'Cart_SS/1',  'ToWS_xc/1',   'autorouting', 'smart');
end

mdl_path = fullfile(SEESAW_ROOT, 'models', [mdl '.slx']);
save_system(mdl, mdl_path);
fprintf('Model saved: models/%s.slx\n', mdl);

if ~quarc_available
    sim(mdl, 'StopTime', '10');
    fprintf('Simulation complete.\n');
end

save_path = fullfile(SEESAW_ROOT, 'data', 'controller_cart_pi.mat');
save(save_path, 'Kp_cart', 'Ki_cart', 'A_cart', 'B_cart_ss', 'C_cart', 'D_cart');
fprintf('Controller saved: data/controller_cart_pi.mat\n');

%% 5. QUARC HARDWARE DEPLOYMENT

if ~quarc_available
    fprintf('\nQUARC not available — install QUARC and re-run.\n');
    return;
end

fprintf('\n--- §5: Hardware Deployment ---\n');
fprintf('PI:  Kp=%.3f  Ki=%.3f  V_sat=%.1fV\n', Kp_cart, Ki_cart, V_sat);
fprintf('\nChecklist:\n');
fprintf('  [ ] Q2-USB connected & powered\n');
fprintf('  [ ] VoltPAQ powered, 1x gain\n');
fprintf('  [ ] Cart near center of track\n\n');

fprintf('Building QUARC target...\n');
rtwbuild(mdl);
fprintf('Build OK.\n');

reply = input('Press ENTER to connect (q to abort): ', 's');
if strcmpi(reply, 'q'), return; end
set_param(mdl, 'SimulationCommand', 'connect');
fprintf('Connected.\n');

reply = input('Type "go" to start controller: ', 's');
if ~strcmpi(reply, 'go')
    set_param(mdl, 'SimulationCommand', 'disconnect');
    return;
end
set_param(mdl, 'SimulationCommand', 'start');
fprintf('RUNNING — cart tracks %.0fcm step at t=2s.\n', x_ref_design*100);

input('Press ENTER to stop...', 's');
set_param(mdl, 'SimulationCommand', 'stop');
fprintf('Stopped.\n');
