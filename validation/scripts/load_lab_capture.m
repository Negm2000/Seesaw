function cap = load_lab_capture(file)
%LOAD_LAB_CAPTURE  Normalize a 2026-05-26 lab .mat capture into a common struct.
%
%   cap = LOAD_LAB_CAPTURE(file) reads a QUARC "To Host File" capture saved as
%   a single numeric matrix (variable usually named 'data') shaped
%   [channels x samples], and returns a struct with named, unit-consistent
%   signals plus protocol-segment slicing.
%
%   Three logging layouts are auto-detected by channel count:
%
%     5-ch  (older cascaded-PID export):
%        [t, r_theta, theta, xc, V_m]
%     7-ch  (cascaded PID, PID_seesaw_test_2024.slx):
%        [t, r_theta, theta, xc, V_m, seg_id, V_real]
%     15-ch (state-space integral tracking, SSi_tracking.slx):
%        [t, r_xc, r_theta, r_xcdot, r_thetadot, ...
%         xc, theta, xc_dot, theta_dot, ...            (states; vel = dirty-deriv used in loop)
%         xc_hat, theta_hat, xc_dot_hat, theta_dot_hat,...(parallel estimator channels)
%         seg_id, V_cmd]
%
%   Output struct fields (SI units, rad for angles):
%     .file .name .fmt .controller .n .fs .t
%     .r_theta .theta .theta_raw .theta_offset .xc .V (commanded motor voltage)
%     .has_seg .seg_id .seg_table          (seg_id present for 7/15-ch)
%     .has_states .xc_dot .theta_dot        (15-ch: dirty-derivative velocities)
%     .has_obs .xc_hat .theta_hat .xc_dot_hat .theta_dot_hat
%     .has_vreal .V_real                    (7-ch only)
%     .seg(id)  -> logical index mask helper via cap.segmask(id)
%
%   theta_offset is estimated from the r=0 prep/baseline segments (or the
%   pre-excitation hold) and removed to form .theta; .theta_raw keeps the
%   un-zeroed signal. This corrects the un-zeroed seesaw encoder seen in
%   some captures without disturbing step/sine tracking content.

% -------- locate & load --------
if exist(file, 'file') ~= 2
    error('load_lab_capture:fileNotFound', 'File not found: %s', file);
end
raw = load(file);
D = local_extract_matrix(raw);
if size(D,1) > size(D,2)      % ensure [channels x samples]
    D = D.';
end
nc = size(D,1);

[~, base] = fileparts(file);
cap = struct();
cap.file = file;
cap.name = base;
cap.controller = local_controller_label(base);
cap.t  = D(1,:).';
cap.fs = 1/median(diff(cap.t));
cap.n  = numel(cap.t);

% -------- channel map by layout --------
switch nc
    case 5
        cap.fmt = '5ch_PID';
        cap.r_theta = D(2,:).';  cap.theta_raw = D(3,:).';
        cap.xc = D(4,:).';       cap.V = D(5,:).';
        cap.has_seg = false; cap.has_states = false;
        cap.has_obs = false; cap.has_vreal = false;
    case 7
        cap.fmt = '7ch_PID';
        cap.r_theta = D(2,:).';  cap.theta_raw = D(3,:).';
        cap.xc = D(4,:).';       cap.V = D(5,:).';
        cap.seg_id = D(6,:).';   cap.V_real = D(7,:).';
        cap.has_seg = true; cap.has_states = false;
        cap.has_obs = false; cap.has_vreal = true;
    case 14
        % 15-ch SSi layout minus seg_id (manual references, e.g. staircase)
        cap.fmt = '14ch_SS';
        cap.r_theta = D(3,:).';  cap.theta_raw = D(7,:).';
        cap.xc = D(6,:).';       cap.V = D(14,:).';
        cap.xc_dot = D(8,:).';   cap.theta_dot = D(9,:).';
        cap.xc_hat = D(10,:).';  cap.theta_hat = D(11,:).';
        cap.xc_dot_hat = D(12,:).'; cap.theta_dot_hat = D(13,:).';
        cap.has_seg = false; cap.has_states = true;
        cap.has_obs = true;  cap.has_vreal = false;
    case 15
        cap.fmt = '15ch_SS';
        cap.r_theta = D(3,:).';  cap.theta_raw = D(7,:).';
        cap.xc = D(6,:).';       cap.V = D(15,:).';
        cap.xc_dot = D(8,:).';   cap.theta_dot = D(9,:).';      % dirty-deriv (in loop)
        cap.xc_hat = D(10,:).';  cap.theta_hat = D(11,:).';
        cap.xc_dot_hat = D(12,:).'; cap.theta_dot_hat = D(13,:).'; % Luenberger (logged only)
        cap.seg_id = D(14,:).';
        cap.has_seg = true; cap.has_states = true;
        cap.has_obs = true;  cap.has_vreal = false;
    otherwise
        error('load_lab_capture:badLayout', ...
            '%s has %d channels — not a known 5/7/15-ch layout.', base, nc);
end

% -------- estimate & remove seesaw-angle offset (un-zeroed encoder) --------
cap.theta_offset = local_theta_offset(cap);
cap.theta = cap.theta_raw - cap.theta_offset;

% -------- protocol segment table + slicing helper --------
if cap.has_seg
    cap.seg_table = local_seg_table(cap.seg_id, cap.t);
end
cap.segmask = @(id) cap.has_seg & ismember_seg(cap.seg_id, id);

end % ===== main =====


% ---------------------------------------------------------------------------
function D = local_extract_matrix(raw)
% Pull the logged numeric matrix out of whatever variable name it was saved as.
fn = fieldnames(raw);
if isscalar(fn) && isnumeric(raw.(fn{1}))
    D = raw.(fn{1}); return;
end
for key = {'data','ans','y','Y','logs','log'}
    if isfield(raw, key{1}) && isnumeric(raw.(key{1}))
        D = raw.(key{1}); return;
    end
end
% fall back to the largest numeric matrix
best = ''; nbest = -1;
for k = 1:numel(fn)
    v = raw.(fn{k});
    if isnumeric(v) && ismatrix(v) && numel(v) > nbest
        best = fn{k}; nbest = numel(v);
    end
end
if isempty(best)
    error('load_lab_capture:noMatrix', 'No numeric log matrix in file.');
end
D = raw.(best);
end


function lbl = local_controller_label(base)
b = upper(base);
if contains(b,'PPDD'),            lbl = 'PP + dirty-deriv';
    elseif contains(b,'LQRDD'),       lbl = 'LQR + dirty-deriv';
elseif strcmp(b,'PID_TEST_FULL'), lbl = 'Cascaded PID (full protocol)';
elseif strcmp(b,'PID_STEP'),      lbl = 'Cascaded PID (step ref)';
elseif strcmp(b,'PID'),           lbl = 'Cascaded PID (regulation)';
else,                              lbl = base;
end
end


function off = local_theta_offset(cap)
% Offset = mean angle while the reference is held at zero, controller settled.
% Prefer the free-run BASELINE (seg 1). Seg 0 is the 15 s release/prep
% transient and must NOT be used — it biases the zero by ~1.5 deg. Fall back
% to seg 0, then to the longest leading r=0 run, then 0.
if cap.has_seg
    m1 = ismember_seg(cap.seg_id, 1);
    if any(m1), off = mean(cap.theta_raw(m1)); return; end
    m0 = ismember_seg(cap.seg_id, 0);
    if any(m0), off = mean(cap.theta_raw(m0)); return; end
end
zr = abs(cap.r_theta) < 1e-6;
if zr(1)
    last0 = find(~zr, 1, 'first');
    if isempty(last0), last0 = numel(zr)+1; end
    idx = 1:max(1,last0-1);
    off = mean(cap.theta_raw(idx)); return;
end
off = 0;
end


function tbl = local_seg_table(seg_id, t)
% Contiguous-run table of segment ids: [id, t_start, t_end, dur].
chg = [true; diff(seg_id(:)) ~= 0];
starts = find(chg);
ends   = [starts(2:end)-1; numel(seg_id)];
ids = seg_id(starts);
tbl = table(ids, t(starts), t(ends), t(ends)-t(starts), ...
    'VariableNames', {'seg_id','t_start','t_end','dur_s'});
end


function m = ismember_seg(seg_id, ids)
m = ismember(round(seg_id(:)), round(ids(:)));
end
