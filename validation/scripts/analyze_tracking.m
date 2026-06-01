function results = analyze_tracking(files, outdir)
%ANALYZE_TRACKING  Tracking-validation analysis for the 2026-05-26 lab captures.
%
%   results = ANALYZE_TRACKING() runs the full pipeline on the five named
%   captures (PID_TEST_FULL, PPDDTEST, LQRDDTEST, PID, PID_STEP), saving
%   figures to validation/docs/figures and a metrics struct to
%   validation/data/tracking_results.mat.
%
%   results = ANALYZE_TRACKING(files, outdir) overrides the inputs.
%
%   Analyses (reference-tracking adaptation of the original HVP):
%     1. Per-file tracking dashboard (steps, pulse, sines, control effort)
%     2. Stepped-sine tracking FRF  T(jw)=theta/r_theta  (quality-gated)
%     3. Step/pulse time-domain tracking metrics (the reliable 1-deg evidence)
%     4. Bounded-oscillation / dead-zone fingerprint on recovery segments
%     5. Three-way controller comparison (PID vs PP+DD vs LQR+DD)
%     6. Observer benchmark: dirty-deriv vs Luenberger vs numerical (15-ch)
%     7. Model-vs-hardware tracking FRF overlay (linear closed loop)
%
%   Requires load_lab_capture.m on the path and scripts/config/seesaw_params.m.

root = local_root();
if nargin < 1 || isempty(files)
    files = fullfile(root, {'PID_TEST_FULL.mat','PPDDTEST.mat','LQRDDTEST.mat', ...
                            'PID.mat','PID_STEP.mat'});
end
if nargin < 2 || isempty(outdir)
    outdir = fullfile(root, 'validation', 'docs', 'figures');
end
if ~exist(outdir,'dir'); mkdir(outdir); end
addpath(fullfile(root,'validation','scripts'));
addpath(genpath(fullfile(root,'scripts')));

% --- parameters & linear model (suppress the script's chatter + latex default) ---
P = local_load_params(root);
local_light_theme();    % force clean light figures (MATLAB may default to dark)

proto = load(fullfile(root,'validation','data','tracking_protocol.mat'));
sine_freqs = proto.sine_freqs_Hz(:);
P.r_step_deg = proto.r_step_deg;

% --- load every capture ---
caps = cellfun(@load_lab_capture, files, 'UniformOutput', false);

results = struct();
results.params = struct('V_sat',P.V_sat,'theta_max_deg',rad2deg(P.theta_max), ...
    'enc_res_deg',rad2deg(P.K_E_SW/P.K_gs), 'fs', 500);
results.files = cellfun(@(c) c.name, caps, 'UniformOutput', false);

% =====================================================================
% Per-file analysis
% =====================================================================
per = struct([]);
for i = 1:numel(caps)
    c = caps{i};
    fprintf('\n================ %s (%s) ================\n', c.name, c.controller);
    % consistent field template so per-file structs concatenate cleanly
    m = struct('name',[],'controller',[],'fmt',[],'duration_s',[], ...
        'theta_offset_deg',[],'V_rms',[],'V_peak',[],'V_clamp',[],'V_pct_motor',[], ...
        'xc_rms_cm',[],'xc_pp_cm',[],'angle_rms_deg',[],'angle_p2p_deg',[], ...
        'step_pos',[],'step_neg',[],'pulse',[],'frf',[],'bw_hz',[], ...
        'frf_peak',[],'track_rms_sines_deg',[],'osc',[],'model',[],'observer',[]);
    m.name = c.name; m.controller = c.controller; m.fmt = c.fmt;
    m.duration_s = c.t(end); m.theta_offset_deg = rad2deg(c.theta_offset);

    % --- control effort (whole run, excluding prep) ---
    runmask = c.t > 1;
    if c.has_seg, runmask = runmask & ~c.segmask(0); end
    m.V_rms = rms(c.V(runmask));
    m.V_peak = max(abs(c.V(runmask)));
    m.V_clamp = round(m.V_peak);                 % per-run clamp (PID~6V, SS~9V)
    m.V_pct_motor = m.V_peak / P.V_nom * 100;    % vs 6 V motor rating (common, fair)
    % cart travel = actuator authority used (track usable half-stroke ~41 cm)
    m.xc_rms_cm = rms(c.xc(runmask) - mean(c.xc(runmask))) * 100;
    m.xc_pp_cm  = (max(c.xc(runmask)) - min(c.xc(runmask))) * 100;
    % whole-run angle-tracking error
    m.angle_rms_deg = rad2deg(rms(c.theta(runmask) - c.r_theta(runmask)));
    m.angle_p2p_deg = rad2deg(max(c.theta(runmask)) - min(c.theta(runmask)));

    % --- step & pulse tracking (segmented runs only) ---
    if c.has_seg
        m.step_pos = local_step_metrics(c, 2, +P.r_step_deg);
        m.step_neg = local_step_metrics(c, 4, -P.r_step_deg);
        m.pulse    = local_pulse_metrics(c, 6);
    end

    % --- stepped-sine tracking FRF ---
    if c.has_seg
        frf = local_tracking_frf(c, sine_freqs);
        m.frf = frf;
        m.bw_hz = local_bandwidth(frf);
        gd = frf.mag(frf.ok);
        m.frf_peak = max([gd; NaN]);
        m.track_rms_sines_deg = rad2deg(local_rms_err(c, 10:21));
        m.model = local_fit_2nd_order(c);   % closed-loop 2nd-order model from data
    end

    % --- bounded oscillation on recovery / baseline (r=0) ---
    if c.has_seg
        oscmask = c.segmask([1 3 5 7 30]);
    else
        oscmask = abs(c.r_theta) < 1e-6 & c.t > 3;
    end
    m.osc = local_osc_stats(c.theta(oscmask), c.fs);

    per = [per, m]; %#ok<AGROW>

    % --- per-file dashboard figure ---
    try
        local_file_dashboard(c, m, P, outdir);
    catch ME
        fprintf('  [warn] dashboard failed: %s\n', ME.message);
    end
end
results.per_file = per;

% =====================================================================
% Cross-file: controller comparison (the three full-protocol runs)
% =====================================================================
cmp_idx = find(arrayfun(@(m) isfield(m,'frf') && ~isempty(m.frf), per));
try
    local_controller_comparison(caps(cmp_idx), per(cmp_idx), P, outdir);
catch ME
    fprintf('[warn] controller comparison failed: %s\n', ME.message);
end
try
    local_tradeoff(per(cmp_idx), outdir);
catch ME
    fprintf('[warn] trade-off plot failed: %s\n', ME.message);
end
try
    local_model_summary(caps(cmp_idx), per(cmp_idx), outdir);
catch ME
    fprintf('[warn] model summary failed: %s\n', ME.message);
end
for i = 1:numel(caps)
    if caps{i}.has_states
        try, local_phase_portrait(caps{i}, outdir); catch ME
            fprintf('[warn] phase portrait (%s) failed: %s\n', caps{i}.name, ME.message);
        end
    end
end

% =====================================================================
% Observer benchmark (15-ch captures)
% =====================================================================
for i = 1:numel(caps)
    if caps{i}.has_obs
        try
            ob = local_observer_benchmark(caps{i}, P, outdir);
            per(i).observer = ob;
        catch ME
            fprintf('[warn] observer benchmark (%s) failed: %s\n', caps{i}.name, ME.message);
        end
    end
end
results.per_file = per;

% =====================================================================
% Model-vs-hardware tracking FRF overlay (linear closed loop)
% =====================================================================
try
    results.model = local_step_validation(caps, per, P, root, outdir);
catch ME
    fprintf('[warn] step validation failed: %s\n', ME.message);
end

% --- save ---
save(fullfile(root,'validation','data','tracking_results.mat'), 'results');
fprintf('\nSaved metrics -> validation/data/tracking_results.mat\n');
fprintf('Figures       -> %s\n', outdir);
end % ===== main =====


%% ===================================================================
%  Local analysis helpers
%  ===================================================================
function root = local_root()
root = getenv('SEESAW_ROOT');
if isempty(root) || exist(root,'dir')~=7
    root = 'C:\Users\Karim Negm\Seesaw';
end
end

function P = local_load_params(root)
% Run seesaw_params quietly and harvest the constants/matrices we need.
here = pwd; cleaner = onCleanup(@() cd(here));
cd(fullfile(root,'scripts','config'));
evalc('seesaw_params');                 %#ok<*NASGU> suppress output
P = struct();
for v = {'V_sat','V_nom','theta_max','K_E_SW','K_gs','K_ec', ...
         'A_sw','B_sw','C_sw','D_sw','A5','B5','C5','Ts'}
    P.(v{1}) = eval(v{1});
end
end

function s = local_step_metrics(c, segid, amp_deg)
% Tracking metrics for a constant +/- reference step segment.
m = c.segmask(segid); s = struct('valid',any(m));
if ~s.valid, return; end
t = c.t(m) - c.t(find(m,1)); th = rad2deg(c.theta(m)); r = rad2deg(c.r_theta(m));
steady = t > 0.7*t(end);
yf = mean(th(steady));                  % final value (last 30%)
y0 = th(1); A = amp_deg;
s.final_deg = yf; s.cmd_deg = A;
s.dc_gain = yf / A;                     % steady tracking gain (~1 is ideal)
s.ss_err_deg = yf - A;
tpk = t < min(2.5, 0.5*t(end));         % transient window only (not whole seg)
s.peak_deg = sign(A) * max(sign(A)*th(tpk));   % signed transient peak
s.overshoot_pct = max(0, (max(sign(A)*th(tpk)) - sign(A)*yf) / abs(A) * 100);
s.p2p_deg = max(th) - min(th);                 % angle peak-to-peak over the step
s.rms_angle_deg = rms(th(steady) - yf);        % steady-state rocking about final
% 10-90 rise time
tgt10 = y0 + 0.1*(yf-y0); tgt90 = y0 + 0.9*(yf-y0);
i10 = find((th-tgt10)*sign(A) >= 0, 1); i90 = find((th-tgt90)*sign(A) >= 0, 1);
if ~isempty(i10) && ~isempty(i90) && i90>=i10, s.rise_s = t(i90)-t(i10); else s.rise_s = NaN; end
% settling: within the larger of 5% of A or the bounded-rocking floor (2*sigma)
tol = max(0.05*abs(A), 2*std(th(steady)));
s.settle_tol_deg = tol;
outside = find(abs(th-yf) > tol, 1, 'last');
if isempty(outside), s.settle_s = 0; elseif outside<numel(t), s.settle_s = t(outside+1); else s.settle_s = NaN; end
s.track_rms_deg = rms(th - r);
end

function s = local_pulse_metrics(c, segid)
m = c.segmask(segid); s = struct('valid',any(m));
if ~s.valid, return; end
t = c.t(m)-c.t(find(m,1)); th = rad2deg(c.theta(m)); r = rad2deg(c.r_theta(m));
s.peak_track_deg = max(th);
s.r_peak_deg = max(r);
s.residual_deg = rms(th(t>2));          % settle after pulse
end

function frf = local_tracking_frf(c, sine_freqs)
% Single-sinusoid least-squares fit per stepped-sine segment ->
% complementary sensitivity T=theta/r_theta with a fit-quality gate.
nf = numel(sine_freqs);
frf = struct('f',sine_freqs,'mag',nan(nf,1),'phase_deg',nan(nf,1), ...
             'r2',nan(nf,1),'r_amp_deg',nan(nf,1),'th_amp_deg',nan(nf,1), ...
             'rms_err_deg',nan(nf,1),'ok',false(nf,1));
for k = 1:nf
    id = 9+k; m = c.segmask(id); if ~any(m), continue; end
    t = c.t(m); t = t - t(1); r = c.r_theta(m); th = c.theta(m);
    w = 2*pi*sine_freqs(k); n = numel(t);
    idx = max(1,round(0.15*n)):round(0.85*n);   % drop ramp-in/out
    X = [sin(w*t(idx)) cos(w*t(idx)) ones(numel(idx),1)];
    ar = X\r(idx); at = X\th(idx);
    ra = hypot(ar(1),ar(2)); ta = hypot(at(1),at(2));
    resid = th(idx) - X*at;
    r2 = 1 - var(resid)/max(var(th(idx)),eps);
    frf.rms_err_deg(k) = rad2deg(rms(th(idx)-r(idx)));
    frf.r_amp_deg(k)=rad2deg(ra); frf.th_amp_deg(k)=rad2deg(ta);
    frf.mag(k) = ta/ra;
    frf.phase_deg(k) = rad2deg(atan2(at(1),at(2)) - atan2(ar(1),ar(2)));
    frf.phase_deg(k) = mod(frf.phase_deg(k)+180,360)-180;
    frf.r2(k) = r2;
    % quality gate: well-explained by the drive sinusoid AND response above
    % the encoder/limit-cycle floor (excludes trivial near-zero fits).
    frf.ok(k) = r2 > 0.6 && ra > 0 && frf.th_amp_deg(k) > 0.08;
end
end

function bw = local_bandwidth(frf)
% -3 dB tracking bandwidth relative to low-frequency gain (gated points).
ok = frf.ok; if nnz(ok) < 2, bw = NaN; return; end
f = frf.f(ok); mag = frf.mag(ok); g0 = mag(1);
below = find(mag < g0/sqrt(2), 1);
if isempty(below), bw = f(end); else bw = f(below); end
end

function e = local_rms_err(c, segids)
m = c.segmask(segids); e = rms(c.theta(m) - c.r_theta(m));
end

function s = local_osc_stats(theta, fs)
% Bounded-oscillation envelope + dead-zone harmonic fingerprint.
s = struct();
if numel(theta) < 16, s.rms_deg=NaN; return; end
x = theta - mean(theta);
s.rms_deg = rad2deg(rms(x));
s.p95_deg = rad2deg(prctile(abs(x),95));
s.pp_deg  = rad2deg(max(x)-min(x));
n = 2^nextpow2(numel(x)); w = local_hann(numel(x));
Pxx = abs(fft(x.*w,n)).^2; f = fs*(0:n/2)'/n; Pxx = Pxx(1:n/2+1);
band = f>=0.2 & f<=15;
[~,im] = max(Pxx(band)); fb = f(band);
s.dom_freq_hz = fb(im);
s.thd_proxy = max(Pxx(band))/mean(Pxx(band));   % peakiness (limit-cycle sharpness)
end


%% ===================================================================
%  Figures
%  ===================================================================
function local_file_dashboard(c, m, P, outdir)
f = local_newfig([40 40 1500 950]);

subplot(3,2,1:2);
plot(c.t, rad2deg(c.r_theta),'k-','LineWidth',1.1); hold on;
plot(c.t, rad2deg(c.theta),'r-','LineWidth',0.5); grid on;
ylabel('Seesaw angle \theta [deg]'); xlim([0 c.t(end)]);
legend({'\theta_{ref}','\theta measured'},'Location','northeast');
title('Seesaw angle: measured vs reference');

subplot(3,2,3);
plot(c.t, c.V,'Color',[0 .55 0],'LineWidth',0.5); grid on; hold on;
yline(m.V_clamp,'r--'); yline(-m.V_clamp,'r--');
ylabel('Motor voltage [V]'); xlabel('Time [s]');
title(sprintf('Motor voltage: %.2f V rms, %.1f V peak (%.0f%% of 6 V rating)', ...
    m.V_rms, m.V_peak, m.V_pct_motor));

subplot(3,2,4);
if ~isempty(m.frf)
    g = m.frf; mdb = 20*log10(g.mag);
    semilogx(g.f, mdb, '-','Color',[.6 .6 .85]); hold on;
    h_lo = semilogx(g.f(~g.ok), mdb(~g.ok),'o','MarkerEdgeColor',[.4 .4 .4],'MarkerFaceColor','w');
    h_hi = semilogx(g.f(g.ok),  mdb(g.ok), 'bo-','LineWidth',1.4,'MarkerFaceColor','b');
    r_amp = median(g.r_amp_deg(g.r_amp_deg>0));
    floor_db = 20*log10(m.osc.rms_deg / r_amp);
    yline(floor_db,'r--','limit-cycle floor','LabelHorizontalAlignment','left','Color',[.85 .2 .2]);
    grid on; yline(0,'k:'); xlim([0.09 11]); ylim([-40 25]);
    xlabel('Frequency [Hz]'); ylabel('|\theta/\theta_{ref}| [dB]');
    if ~isempty(h_lo)
        legend([h_hi h_lo],{'reference-dominated','below resolution'},'Location','southwest');
    end
    title(sprintf('Closed-loop frequency response  (bandwidth %.2f Hz)', m.bw_hz));
else
    axis off; text(0.1,0.5,'No swept-sine data (regulation run)');
end

subplot(3,2,5);
if c.has_seg && m.step_pos.valid
    mm = c.segmask(2); t=c.t(mm)-c.t(find(mm,1));
    plot(t, rad2deg(c.theta(mm)),'r-','LineWidth',1.2); hold on;
    plot(t, rad2deg(c.r_theta(mm)),'k--','LineWidth',1.1); grid on;
    xlabel('Time [s]'); ylabel('Seesaw angle \theta [deg]');
    legend({'\theta measured','\theta_{ref}'},'Location','southeast');
    title(sprintf('1{\\circ} step: rise %.2f s, overshoot %.0f%%, steady-state error %.2f{\\circ}', ...
        m.step_pos.rise_s, m.step_pos.overshoot_pct, m.step_pos.ss_err_deg));
else
    axis off; text(0.1,0.5,'No step segment');
end

subplot(3,2,6);
if c.has_seg, mask=c.segmask([1 3 5 7 30]); else, mask=abs(c.r_theta)<1e-6 & c.t>3; end
histogram(rad2deg(c.theta(mask)-mean(c.theta(mask))),'BinWidth',0.05,...
    'FaceColor','r','EdgeColor','none'); grid on;
xlabel('Angle deviation from mean [deg]'); ylabel('Samples');
title(sprintf('Regulation spread (\\theta_{ref}=0): %.2f{\\circ} rms, %.2f{\\circ} 95th pct, %.2f Hz', ...
    m.osc.rms_deg, m.osc.p95_deg, m.osc.dom_freq_hz));

sgtitle(c.controller,'FontWeight','bold');
saveas(f, fullfile(outdir, sprintf('Tracking-%s.png', c.name))); close(f);
fprintf('  saved Tracking-%s.png\n', c.name);
end

function local_controller_comparison(caps, per, P, outdir)
f = local_newfig([40 40 1250 850]);
cols = lines(numel(caps));

% (1) FRF magnitude overlay
subplot(2,2,1);
hmag = gobjects(1,numel(per));
for i=1:numel(per)
    g=per(i).frf; ok=g.ok; mdb=20*log10(g.mag);
    semilogx(g.f, mdb, '-','Color',[cols(i,:) 0.30],'HandleVisibility','off'); hold on; % all freqs faded
    semilogx(g.f(~ok), mdb(~ok),'o','MarkerEdgeColor',cols(i,:),'MarkerFaceColor','w', ...
        'HandleVisibility','off');                                                       % below floor (hollow)
    hmag(i)=semilogx(g.f(ok), mdb(ok),'o-','Color',cols(i,:),'LineWidth',1.7, ...
        'MarkerFaceColor',cols(i,:));                                                    % trusted
end
grid on; yline(0,'k:','HandleVisibility','off'); xlim([0.09 11]); ylim([-40 25]);
xlabel('Frequency [Hz]'); ylabel('|\theta/\theta_{ref}| [dB]');
title('Closed-loop magnitude  (filled: reference-dominated;  open: below resolution)');
legend(hmag, {per.controller}, 'Location','southwest','Interpreter','none');

% (2) FRF phase overlay
subplot(2,2,2);
for i=1:numel(per)
    g=per(i).frf; ok=g.ok;
    semilogx(g.f(~ok),g.phase_deg(~ok),'o','MarkerEdgeColor',cols(i,:),'MarkerFaceColor','w'); hold on;
    semilogx(g.f(ok),g.phase_deg(ok),'o-','Color',cols(i,:),'LineWidth',1.7,...
        'MarkerFaceColor',cols(i,:));
end
grid on; xlim([0.09 11]); xlabel('Frequency [Hz]'); ylabel('\angle (\theta/\theta_{ref}) [deg]');
title('Closed-loop phase');

% (3) tracking error vs frequency (swept sine) + steps as reference markers
subplot(2,2,3);
for i=1:numel(per)
    g=per(i).frf; ok=g.ok;
    semilogx(g.f(ok), g.rms_err_deg(ok),'o-','Color',cols(i,:),'LineWidth',1.6, ...
        'MarkerFaceColor',cols(i,:)); hold on;
end
for i=1:numel(per)
    yline(per(i).step_pos.track_rms_deg, '--','Color',cols(i,:),'Alpha',0.5,'HandleVisibility','off');
end
grid on; xlim([0.09 11]); xlabel('Frequency [Hz]'); ylabel('RMS angle error [deg]');
title('Tracking error vs frequency  (dashed = 1{\circ}-step error)');
legend({per.controller},'Location','northwest','Interpreter','none');

% (4) performance table (clean, mixed units)
subplot(2,2,4); axis off;
rows = {'Bandwidth [Hz]','Peak |\theta/\theta_{ref}| [-]','Damping \zeta [-]', ...
        'Voltage rms [V]','Voltage peak [V]','Cart travel p-p [cm]', ...
        'Angle p-p [deg]','Rocking rms [deg]','Step error [deg]'};
vals = zeros(numel(rows), numel(per));
for i=1:numel(per)
    zeta = local_getf(per(i).model,'zeta',NaN);
    vals(:,i) = [per(i).bw_hz; per(i).frf_peak; zeta; per(i).V_rms; per(i).V_peak; ...
                 per(i).xc_pp_cm; per(i).angle_p2p_deg; per(i).osc.rms_deg; per(i).step_pos.ss_err_deg];
end
short = {'PID','PP+DD','LQR+DD'};
tx = sprintf('%-22s %8s %8s %8s\n','metric', short{:});
for r=1:numel(rows)
    tx = [tx sprintf('%-22s %8.2f %8.2f %8.2f\n', rows{r}, vals(r,:))]; %#ok<AGROW>
end
text(0.0, 0.98, tx, 'FontName','FixedWidth','FontSize',10,'VerticalAlignment','top','Interpreter','tex');
title('Performance summary');

sgtitle('Controller comparison — identical reference protocol','FontWeight','bold');
saveas(f, fullfile(outdir,'Compare-Controllers.png')); close(f);
fprintf('  saved Compare-Controllers.png\n');
end

function ob = local_observer_benchmark(c, P, outdir)
% Dirty-derivative (in-loop) vs Luenberger observer vs filtered numerical diff.
dt = 1/c.fs; ob = struct();
% filtered numerical derivative of measured angle (zero-phase LPF @ 30 Hz)
fc = 30;
th_f = local_lpf(c.theta, fc, c.fs);
thd_num = local_lpf([diff(th_f);0]/dt, fc, c.fs);

% Compare on the quiet recovery segments so the startup convergence spike
% (large initial innovation) does not dominate the noise-floor RMS.
qm = c.segmask([3 5 7 30]); if ~any(qm), qm = c.t>5; end
sm = c.t > 5;                                  % steady (post-convergence)
ob.dd_rms   = rms(c.theta_dot(qm));
ob.obs_rms  = rms(c.theta_dot_hat(qm));
ob.num_rms  = rms(thd_num(qm));
% innovation: measured - observer estimate of angle (should be ~white/small)
innov = c.theta - c.theta_hat;
ob.innov_rms_deg = rad2deg(rms(innov(sm)));
ob.innov_bias_deg = rad2deg(mean(innov(sm)));
d = innov(sm) - mean(innov(sm));
ob.innov_lag1 = (d(2:end)'*d(1:end-1))/(d'*d);
% time for the initial estimate to converge (|innov| below 0.2 deg)
ic = find(abs(rad2deg(innov)) < 0.2, 1);
ob.conv_s = c.t(max(ic,1));
nseg = min(2048, nnz(qm));
[Pdd,fp] = local_welch(c.theta_dot(qm), nseg, c.fs);
[Pob,~]  = local_welch(c.theta_dot_hat(qm), nseg, c.fs);
[Pnum,~] = local_welch(thd_num(qm), nseg, c.fs);

f = local_newfig([40 40 1150 800]);
subplot(3,1,1);
zoom = c.t>40 & c.t<55;
plot(c.t(zoom), c.theta_dot(zoom),'-','Color',[.85 .3 .1],'LineWidth',0.6); hold on;
plot(c.t(zoom), c.theta_dot_hat(zoom),'b-','LineWidth',1.0);
plot(c.t(zoom), thd_num(zoom),'k:','LineWidth',0.8); grid on;
ylabel('Angular rate $\dot\theta$ [rad/s]','Interpreter','latex');
legend({'dirty derivative (used in loop)','Luenberger observer','filtered finite difference'},'Location','best');
title('Angular-rate estimates (45–55 s detail)');

subplot(3,1,2);
loglog(fp,Pdd,'-','Color',[.85 .3 .1],'LineWidth',1.0); hold on;
loglog(fp,Pob,'b-','LineWidth',1.3); loglog(fp,Pnum,'k:','LineWidth',1.0);
grid on; xlim([0.1 250]); xlabel('Frequency [Hz]'); ylabel('PSD [(rad/s)^2/Hz]');
legend({'dirty derivative','Luenberger observer','filtered difference'},'Location','best');
title(sprintf('Angular-rate spectrum, level segments  (rms: %.3f / %.3f / %.3f rad/s)', ...
    ob.dd_rms, ob.obs_rms, ob.num_rms));

subplot(3,1,3);
plot(c.t, rad2deg(innov),'b-','LineWidth',0.4); grid on;
ylabel('\theta - \theta_{est} [deg]'); xlabel('Time [s]');
title(sprintf('Observer estimation error  (rms %.3f{\\circ}, bias %+.3f{\\circ}, lag-1 corr %+.2f)', ...
    ob.innov_rms_deg, ob.innov_bias_deg, ob.innov_lag1));

sgtitle(sprintf('Angular-rate estimation — %s', c.controller),'FontWeight','bold');
saveas(f, fullfile(outdir, sprintf('Observer-%s.png', c.name))); close(f);
fprintf('  saved Observer-%s.png\n', c.name);
end

function md = local_step_validation(caps, per, P, root, outdir)
% Honest model validation on the CLEAN data: the 1-deg angle step.
% Overlays the measured +1 deg step of every segmented controller and the
% linear closed-loop (integral-servo) step from the saved LQR gain.
md = struct('available',false);
cols = lines(numel(caps));
f = local_newfig([60 60 1050 650]); hold on; h = []; leg = {};

% measured +1 deg steps (segment 2), time-aligned to step onset
for i=1:numel(caps)
    c = caps{i}; if ~c.has_seg, continue; end
    mm = c.segmask(2); if ~any(mm), continue; end
    t = c.t(mm) - c.t(find(mm,1));
    h(end+1) = plot(t, rad2deg(c.theta(mm)),'-','Color',cols(i,:),'LineWidth',1.4); %#ok<AGROW>
    leg{end+1} = sprintf('%s (measured)', c.controller); %#ok<AGROW>
end

% linear closed-loop step from the saved (designed, stable) LQR servo.
% Use the stored closed-loop A_cl; normalise to unit DC gain so the
% transient SHAPE is shown even if the logged augmentation differs.
lqrf = fullfile(root,'data','controller_lqr.mat'); src = 'none';
if exist(lqrf,'file')
    S = load(lqrf);
    if isfield(S,'A_cl')
        Acl = S.A_cl; Br = [0;0;0;0;1]; Cth = [0 1 0 0 0];
        if all(real(eig(Acl)) < 0)
            sysT = ss(Acl, Br, Cth, 0); g0 = dcgain(sysT);
            if isfinite(g0) && abs(g0) > 1e-6
                tt = linspace(0,12,3000)';
                y = step(sysT, tt) / g0;        % normalised: settles to 1
                h(end+1) = plot(tt, y,'k--','LineWidth',2); %#ok<AGROW>
                leg{end+1} = 'linear model (LQR servo, normalised)'; %#ok<AGROW>
                src = 'controller_lqr.mat (A_cl)';
            end
        end
    end
end

yline(1,'k:','reference','LabelHorizontalAlignment','left');
grid on; xlim([0 12]); ylim([-0.5 4]);
xlabel('Time from step [s]'); ylabel('Seesaw angle \theta [deg]');
legend(h, leg, 'Location','northeast','Interpreter','none');
if strcmp(src,'none')
    title('Measured 1{\circ} angle-step response — controller comparison','FontWeight','bold');
else
    title('1{\circ} angle-step response — measured vs linear model','FontWeight','bold');
end
saveas(f, fullfile(outdir,'StepResponse-Comparison.png')); close(f);
fprintf('  saved StepResponse-Comparison.png (model gain: %s)\n', src);
md.available = true; md.gain_source = src;
end


%% ===================================================================
%  Base-MATLAB DSP helpers (no Signal Processing Toolbox dependency)
%  ===================================================================
function v = local_getf(s, f, d)
% Safe struct field fetch with default.
if isstruct(s) && isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

function md = local_fit_2nd_order(c)
% Fit a 2nd-order closed-loop model theta/theta_ref from the (stable) logged
% reference-tracking data. Returns natural frequency, damping, resonant peak.
md = struct('valid',false,'wn',NaN,'fn_hz',NaN,'zeta',NaN,'Mr',NaN,'dc',NaN,'fit_pct',NaN);
mask = c.t > 15; dec = 2; idx = find(mask); idx = idx(1:dec:end);
if numel(idx) < 200, return; end
z = iddata(c.theta(idx), c.r_theta(idx), 0.002*dec);
try
    sys = tfest(z, 2, 0);
    [~,fit] = compare(z, sys); if iscell(fit), fit = fit{1}; end
    p = pole(sys); wn = abs(p(1)); ze = -real(p(1))/wn;
    md.valid = true; md.sys = sys; md.wn = wn; md.fn_hz = wn/(2*pi);
    md.zeta = ze; md.dc = dcgain(sys); md.fit_pct = fit;
    if ze < 1, md.Mr = 1/(2*ze*sqrt(1-ze^2)); else, md.Mr = md.dc; end
catch
end
end

function local_model_summary(caps, per, outdir)
% Per-controller 2nd-order closed-loop model: step + frequency-response fit,
% with natural frequency and damping. Ties the time- and frequency-domain
% behaviour together through one identified model per controller.
cols = lines(numel(per));
f = local_newfig([50 50 1300 560]);

% (left) measured +1 deg step vs identified 2nd-order step
subplot(1,2,1); hold on; h=[]; leg={};
for i=1:numel(per)
    c = caps{i}; md = per(i).model;
    mm = c.segmask(2); t = c.t(mm)-c.t(find(mm,1));
    h(end+1) = plot(t, rad2deg(c.theta(mm)),'-','Color',cols(i,:),'LineWidth',1.0); %#ok<AGROW>
    leg{end+1} = sprintf('%s  (\\zeta=%.2f, f_n=%.2f Hz, fit %.0f%%)', per(i).controller, ...
        local_getf(md,'zeta',NaN), local_getf(md,'fn_hz',NaN), local_getf(md,'fit_pct',NaN)); %#ok<AGROW>
    if local_getf(md,'valid',false)
        tt = linspace(0,t(end),1500)';
        ys = step(md.sys, tt) * deg2rad(1);     % 1 deg reference step
        plot(tt, rad2deg(ys),'--','Color',cols(i,:),'LineWidth',1.8);
    end
end
yline(1,'k:'); grid on; xlim([0 12]); ylim([-0.5 4]);
xlabel('Time from step [s]'); ylabel('Seesaw angle \theta [deg]');
legend(h, leg, 'Location','northeast','Interpreter','tex');
title('1{\circ} step: measured (solid) vs 2nd-order model (dashed)');

% (right) measured FRF points vs identified model magnitude
subplot(1,2,2);
fline = logspace(log10(0.09), log10(11), 200)';
for i=1:numel(per)
    g = per(i).frf; ok = g.ok; md = per(i).model;
    semilogx(g.f(ok), 20*log10(g.mag(ok)),'o','Color',cols(i,:), ...
        'MarkerFaceColor',cols(i,:),'HandleVisibility','off'); hold on;
    if local_getf(md,'valid',false)
        mgm = squeeze(bode(md.sys, 2*pi*fline));
        semilogx(fline, 20*log10(mgm),'-','Color',cols(i,:),'LineWidth',1.6);
    end
end
grid on; yline(0,'k:','HandleVisibility','off'); xlim([0.09 11]); ylim([-40 25]);
xlabel('Frequency [Hz]'); ylabel('|\theta/\theta_{ref}| [dB]');
legend({per.controller},'Location','southwest','Interpreter','none');
title('Closed-loop response: measured (points) vs 2nd-order model (line)');

sgtitle({'Data-driven 2nd-order closed-loop model', ...
    'reduced fit (small-signal, limit-cycle-contaminated) — damping \zeta is indicative, not a precise identification'}, ...
    'FontWeight','bold');
saveas(f, fullfile(outdir,'Model-ClosedLoop.png')); close(f);
fprintf('  saved Model-ClosedLoop.png\n');
end

function local_tradeoff(per, outdir)
% Intuitive trade-off scatter: tracking accuracy vs control effort,
% bubble size = cart travel used. Lower-left = better.
f = local_newfig([60 60 820 640]);
cols = lines(numel(per)); hold on;
for i=1:numel(per)
    x = per(i).angle_rms_deg; y = per(i).V_rms;
    sz = 200 + 60*per(i).xc_pp_cm;
    scatter(x, y, sz, cols(i,:), 'filled', 'MarkerFaceAlpha',0.6, 'MarkerEdgeColor','k');
    text(x, y, ['  ' per(i).controller], 'FontSize',10,'Interpreter','none');
end
grid on; xlabel('Angle tracking error (rms) [deg]'); ylabel('Motor voltage (rms) [V]');
title('Accuracy vs effort trade-off  (bubble \propto cart travel; lower-left is better)');
xl=xlim; yl=ylim; xlim([0 xl(2)*1.15]); ylim([0 yl(2)*1.15]);
saveas(f, fullfile(outdir,'Compare-Tradeoff.png')); close(f);
fprintf('  saved Compare-Tradeoff.png\n');
end

function local_phase_portrait(c, outdir)
% Phase portrait theta vs theta_dot — shows the limit cycle / dead-zone shape.
% Trajectories are broken at segment gaps so non-contiguous spans are not
% joined by spurious straight lines.
f = local_newfig([60 60 900 760]);
% use the smooth observer rate (the dirty-derivative is too quantization-spiky
% for a legible phase portrait)
thd = c.theta_dot_hat;
[xr,yr] = local_break(rad2deg(c.theta), thd, c.segmask([1 3 5 7 30]));
[xs,ys] = local_break(rad2deg(c.theta), thd, c.segmask([2 4]));
h2 = plot(xs, ys, '-', 'Color',[.85 .3 .1 0.8],'LineWidth',0.7); hold on;
h1 = plot(xr, yr, '-', 'Color',[.2 .4 .9 0.6],'LineWidth',0.6);
grid on; xlabel('Seesaw angle \theta [deg]');
ylabel('Angular rate $\dot\theta$ (observer) [rad/s]','Interpreter','latex');
legend([h1 h2],{'reference held at 0 (rocking)','\pm1{\circ} steps'},'Location','northeast');
title(sprintf('Phase portrait — %s', c.controller),'FontWeight','bold');
saveas(f, fullfile(outdir, sprintf('Phase-%s.png', c.name))); close(f);
fprintf('  saved Phase-%s.png\n', c.name);
end

function [xb, yb] = local_break(x, y, mask)
% Return x,y over mask with NaNs inserted at non-contiguous index gaps.
idx = find(mask); if isempty(idx), xb=[]; yb=[]; return; end
gap = [false; diff(idx)>1];
xb = x(idx); yb = y(idx);
xb(gap) = NaN; yb(gap) = NaN;
end

function local_light_theme()
% Force clean light-on-white figures regardless of the MATLAB app theme.
set(groot,'defaultAxesTickLabelInterpreter','tex');
set(groot,'defaultTextInterpreter','tex');
set(groot,'defaultLegendInterpreter','tex');
set(groot,'defaultFigureColor','w');
set(groot,'defaultAxesColor','w');
set(groot,'defaultAxesXColor',[.15 .15 .15]);
set(groot,'defaultAxesYColor',[.15 .15 .15]);
set(groot,'defaultAxesZColor',[.15 .15 .15]);
set(groot,'defaultAxesGridColor',[.65 .65 .65]);
set(groot,'defaultAxesGridAlpha',0.35);
set(groot,'defaultTextColor','k');
set(groot,'defaultAxesFontSize',10);
set(groot,'defaultLegendTextColor','k');
set(groot,'defaultLegendColor','w');
set(groot,'defaultLegendEdgeColor',[.6 .6 .6]);
end

function f = local_newfig(pos)
f = figure('Position',pos,'Visible','off','Color','w');
try, theme(f,'light'); catch, end   %#ok<CTCH>  (R2025a+; harmless if absent)
end

function w = local_hann(n)
n = max(1,round(n));
if n==1, w = 1; return; end
w = 0.5*(1 - cos(2*pi*(0:n-1)'/(n-1)));
end

function y = local_lpf(x, fc, fs)
% Zero-phase low-pass: two forward-backward passes of a 1st-order IIR.
x = x(:); a = exp(-2*pi*fc/fs); b = 1-a;
y = filter(b,[1 -a],x);
y = flipud(filter(b,[1 -a],flipud(y)));
end

function [Pxx, f] = local_welch(x, nseg, fs)
% One-sided Welch PSD with 50% overlap and a Hann window (base MATLAB).
x = x(:); N = numel(x); nseg = max(8, min(round(nseg), N));
nov = floor(nseg/2); step = max(1, nseg - nov);
w = local_hann(nseg); U = sum(w.^2);
starts = 1:step:(N - nseg + 1); if isempty(starts), starts = 1; end
nfft = nseg; half = floor(nfft/2)+1; Pxx = zeros(half,1);
for s = starts
    seg = x(s:s+nseg-1) .* w;
    X = fft(seg, nfft);
    P = abs(X(1:half)).^2 / (fs*U);
    P(2:end-1) = 2*P(2:end-1);
    Pxx = Pxx + P;
end
Pxx = Pxx / numel(starts);
f = (0:half-1)' * fs / nfft;
end
