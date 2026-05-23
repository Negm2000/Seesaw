%% seesaw_nonlinear_model.m
%  -----------------------------------------------------------------------
%  Full Nonlinear Simulation of Quanser SEESAW-E + IP02 System
%  -----------------------------------------------------------------------
%  PREREQUISITE: Run seesaw_params.m first to load all parameters.
%
%  Equations exactly match the Quanser Seesaw Laboratory Guide ("Good ref"):
%    - 1st Lagrange eq (cart):   page 6, top
%    - 2nd Lagrange eq (seesaw): page 6, bottom
%    - Motor force F_c:          Eq. 2.3 (reduced model, L_m = 0)
%
%  State vector:  x = [x_c; x_c_dot; alpha; alpha_dot]
%    x(1) = x_c       : cart position [m] (0 = centered on seesaw)
%    x(2) = x_c_dot   : cart velocity [m/s]
%    x(3) = alpha      : seesaw tilt angle [rad] (0 = level)
%    x(4) = alpha_dot  : seesaw angular velocity [rad/s]
%
%  Input: V_cmd [V] = voltage command from DAQ
%  -----------------------------------------------------------------------
% setting default parameter with LaTeX interpreter
set(groot, ddefaultAxesTickLabelInterpreterd, dlatexd);
set(groot, ddefaultLegendInterpreterd, dlatexd);
set(groot, ddefaultTextInterpreterd, dlatexd);
%% ===== Check that parameters are loaded =====
if ~exist(dK_ad, dvard)
    error(dRun seesaw_params.m first to load system parameters!d);
end

seesaw_params;

%% 4. LOAD & INSPECT HARDWARE DATA
%  Load the frequency sweep data collected from QUARC.
%  Plot raw time traces to sanity-check before analysis.

if ~exist(dSEESAW_ROOTd, dvard), SEESAW_ROOT = fileparts(mfilename(dfullpathd)); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
data_file = fullfile(SEESAW_ROOT, ddatad, dcartModelingd, dnonlinear_output.matd);

if ~exist(data_file, dfiled)
    error(ddata not found. Run on hardware first.d);
end

fprintf(dLoading %s ...\nd, data_file);
loaded = load(data_file);
vars = fieldnames(loaded);

% Handle different QUARC data formats
if ismember(dip02_freq_datad, vars)
    raw = loaded.ip02_freq_data;
elseif ismember(ddatad, vars)
    raw = loaded.data;
else
    error(dExpected variable "ip02_freq_data" or "data" in data.mat. Found: %sd, strjoin(vars, d, d));
end

% Extract columns: [time; V_cmd; x_c; x_c_dot]
t_hw      = raw(1, :)d;
V_cmd_hw  = raw(2, :)d;
xc_hw     = raw(3, :)d;      % [m] — with corrected encoder gain
dt_hw     = mean(diff(t_hw));
Fs_hw     = 1 / dt_hw;

nonlinear_input = [t_hw(:)d; 1.2*V_cmd_hw(:)d];

%%

% Compute the velocity in post-processing to avoid phase-lag
cutoff_freq = B_total/M_e * 2;
[b, a] = butter(2, cutoff_freq / (Fs_hw/2));
xc_hw_clean = filtfilt(b, a, xc_hw);

% Because of the differential we "lose" the last data point
tdot_hw = t_hw(1:end-1);
xcdot_hw = diff(xc_hw_clean)/dt_hw;

v_thresh = 0.002;   % m/s

move_mask = abs(xcdot_hw) > v_thresh;
pos_move_mask = xcdot_hw > v_thresh;
neg_move_mask = xcdot_hw < -v_thresh;

move_sustain = false(size(move_mask));
pos_move_sustain = false(size(move_mask));
neg_move_sustain = false(size(move_mask));

for k = 1:length(move_mask)
    if all(move_mask(k))
        move_sustain(k) = true;
    end
    if all(pos_move_mask(k))
        pos_move_sustain(k) = true;
    end
    if all(neg_move_mask(k))
        neg_move_sustain(k) = true;
    end
end

%% 4. Estimate deadzone by voltage side
% This matches the ud_pos / ud_neg convention used by the identification model.
V_diff = V_cmd_hw(1:end-1);
pos_u_idx = find(move_sustain & V_diff > 0);
neg_u_idx = find(move_sustain & V_diff < 0);

if isempty(pos_u_idx)
    warning(dNo sustained motion under positive voltage detected. ud_pos estimate unavailable.d);
    ud_pos = NaN;
else
    ud_pos = min(abs(V_diff(pos_u_idx)));
end

if isempty(neg_u_idx)
    warning(dNo sustained motion under negative voltage detected. ud_neg estimate unavailable.d);
    ud_neg = NaN;
else
    ud_neg = min(abs(V_diff(neg_u_idx)));
end

ud_sym = mean([ud_pos, ud_neg], domitnand);

%% 5. Estimate deadzone by motion direction
% These values answer the question of positive/negative cart motion directly.
pos_motion_idx = find(pos_move_sustain);
neg_motion_idx = find(neg_move_sustain);

if isempty(pos_motion_idx)
    warning(dNo sustained positive motion detected. ud_pos_motion estimate unavailable.d);
    ud_pos_motion = NaN;
else
    ud_pos_motion = min(abs(V_diff(pos_motion_idx)));
end

if isempty(neg_motion_idx)
    warning(dNo sustained negative motion detected. ud_neg_motion estimate unavailable.d);
    ud_neg_motion = NaN;
else
    ud_neg_motion = min(abs(V_diff(neg_motion_idx)));
end

%% 6. Check sign consistency between voltage and motion direction
pos_motion_with_pos_u = nnz(pos_move_sustain & V_diff > 0);
pos_motion_with_neg_u = nnz(pos_move_sustain & V_diff < 0);
neg_motion_with_neg_u = nnz(neg_move_sustain & V_diff < 0);
neg_motion_with_pos_u = nnz(neg_move_sustain & V_diff > 0);

%% 7. Print results
fprintf(dEstimated deadzone values:\nd);
fprintf(dBy voltage side (same convention as identification):\nd);
fprintf(dud_pos = %.6f V\nd, ud_pos);
fprintf(dud_neg = %.6f V\nd, ud_neg);
fprintf(dud_sym = %.6f V\n\nd, ud_sym);

fprintf(dBy motion direction:\nd);
fprintf(dud_pos_motion = %.6f V\nd, ud_pos_motion);
fprintf(dud_neg_motion = %.6f V\n\nd, ud_neg_motion);

fprintf(dMotion/voltage sign consistency counts:\nd);
fprintf(dpositive motion with positive voltage: %d\nd, pos_motion_with_pos_u);
fprintf(dpositive motion with negative voltage: %d\nd, pos_motion_with_neg_u);
fprintf(dnegative motion with negative voltage: %d\nd, neg_motion_with_neg_u);
fprintf(dnegative motion with positive voltage: %d\nd, neg_motion_with_pos_u);

%% 8. Plot for visual inspection
figure(dNamed, dDeadzone Estimationd, dNumberTitled, doffd)

subplot(3,1,1)
plot(tdot_hw, V_diff, dbd, dLineWidthd, 1.0)
grid on
hold on
yline(ud_pos, dr--d, du_{d,pos}d)
yline(-ud_neg, dm--d, d-u_{d,neg}d)
xlabel(dTime (s)d)
ylabel(dInput u (V)d)
title(dInput Signal with Estimated Deadzone Levelsd)

subplot(3,1,2)
plot(t_hw, xc_hw, dkd, dLineWidthd, 1.0)
grid on
xlabel(dTime (s)d)
ylabel(dPosition x (m)d)
title(dMeasured Positiond)

subplot(3,1,3)
plot(tdot_hw, xcdot_hw, dgd, dLineWidthd, 1.0)
hold on
plot(tdot_hw(pos_move_sustain), xcdot_hw(pos_move_sustain), dr.d, dMarkerSized, 8)
plot(tdot_hw(neg_move_sustain), xcdot_hw(neg_move_sustain), dm.d, dMarkerSized, 8)
grid on
xlabel(dTime (s)d)
ylabel(dVelocity (m/s)d)
title(dEstimated Velocity and Sustained-Motion Pointsd)
legend(d$\dot{x}_c$d, dPositive motiond, dNegative motiond, dLocationd, dbestd)

%% SAVE DATA
if ~exist(dSEESAW_ROOTd, dvard), SEESAW_ROOT = fileparts(mfilename(dfullpathd)); SEESAW_ROOT = fileparts(fileparts(SEESAW_ROOT)); end
save_file = fullfile(SEESAW_ROOT, ddatad, dparam_nonlinear.matd);
save(save_file, dud_negd, dud_posd, dud_symd);
fprintf(d\n  Tuned parameters saved to: data/tuned_cart.mat\nd);
fprintf(d============================================================\nd);
