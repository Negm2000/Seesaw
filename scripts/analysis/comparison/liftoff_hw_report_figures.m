%% Lift-off HW report figures -- File 2 (manual calibration, Lyapunov supervisor, beta=0.30)
%
% Generates four publication-quality figures for the thesis Experience 3:
%   1. Full-run timeline with catch episodes annotated
%   2. Catch zoom: state during one clean catch event
%   3. Steady-state characterisation in the longest caught window
%   4. Phase plane and Lyapunov V(x) trajectory
%
% Source: data/hw/liftoff/successful/14-52-39_lyap_supervisor_manual_cal.mat
% Supervisor: lift-off K_lift = 0.30*Kf; catch trigger V(x) < V_rest/3 latched;
%             balance K_pp = Kf (from data/controller_freq.mat).

clear; close all; clc
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

%% Load run and recompute supervisor constants -----------------------------
s = load(fullfile(root, 'data', 'hw', 'liftoff', 'successful', ...
                  '14-52-39_lyap_supervisor_manual_cal.mat'));
d = s.data;
t = d(1,:)'; x_c = d(2,:)'; al = d(3,:)'; Vm = d(4,:)';

% Supervisor design constants (same as build_pp_liftoff_hw.m)
run(fullfile(root, 'scripts', 'config', 'seesaw_params.m'))
tuned = load(fullfile(root, 'data', 'tuned', 'tuned_params.mat'));
ctrl  = load(fullfile(root, 'data', 'controllers', 'controller_freq.mat'));
Kf = ctrl.Kf;
A_sw = tuned.A_sw; B_sw = tuned.B_sw;
Q_lyap = diag([1, 0.1, 10, 1]);
P_lyap = lyap((A_sw - B_sw*Kf)', Q_lyap);
x_rest = [0; 0; alpha_max; 0];
V_rest = x_rest' * P_lyap * x_rest;
c_catch = V_rest / 3;

% Reconstruct alpha_dot, x_c_dot via centered difference + light LPF
Ts = mean(diff(t));
alp_dot = [0; (al(3:end) - al(1:end-2)) / (2*Ts); 0];
xc_dot  = [0; (x_c(3:end) - x_c(1:end-2)) / (2*Ts); 0];
b_lpf = ones(1, 25)/25;
alp_dot_f = filter(b_lpf, 1, alp_dot);
xc_dot_f  = filter(b_lpf, 1, xc_dot);

% Compute Lyapunov V(x) over the run
state = [x_c, xc_dot_f, al, alp_dot_f];
Vt = sum((state * P_lyap) .* state, 2);

% Find sustained-balance windows (|alpha|<5deg for >5s)
basin = abs(al) < deg2rad(5);
de = diff([0; basin; 0]);
rs = find(de == 1); re = find(de == -1) - 1;
durs = t(re) - t(rs);
keep = durs > 5;
rs = rs(keep); re = re(keep); durs = durs(keep);
[durs, oi] = sort(durs, 'descend');
rs = rs(oi); re = re(oi);

fprintf('Caught windows (|alpha|<5deg sustained > 5s):\n');
for k = 1:numel(rs)
    fprintf('  #%d: t = %6.2fs to %6.2fs  dur=%5.1fs  mean=%+.2f deg\n', ...
        k, t(rs(k)), t(re(k)), durs(k), rad2deg(mean(al(rs(k):re(k)))));
end

%% Figure 1: Full-run timeline with episodes ------------------------------
fig1 = figure('Position', [60 40 1200 700], 'Color', 'w');

% Shade caught windows
shade_clr = [0.85 0.95 0.85];

ax1 = subplot(3,1,1); hold on; grid on
for k = 1:numel(rs)
    fill([t(rs(k)) t(re(k)) t(re(k)) t(rs(k))], [-15 -15 15 15], shade_clr, ...
         'EdgeColor','none', 'HandleVisibility','off');
end
plot(t, rad2deg(al), 'b-', 'LineWidth', 0.8)
yline( rad2deg(alpha_max), 'k--', 'stop')
yline(-rad2deg(alpha_max), 'k--')
yline(0, 'k:')
xline(27.32, 'r--', 'controller engage')
ylabel('\alpha [deg]', 'Interpreter', 'tex')
title('File 2 -- manual-calibration lift-off with Lyapunov supervisor (\beta=0.30)', ...
      'Interpreter', 'tex')
ylim([-15 15])

ax2 = subplot(3,1,2); hold on; grid on
for k = 1:numel(rs)
    fill([t(rs(k)) t(re(k)) t(re(k)) t(rs(k))], [-50 -50 50 50], shade_clr, ...
         'EdgeColor','none', 'HandleVisibility','off');
end
plot(t, 100*x_c, 'b-', 'LineWidth', 0.8)
yline(0, 'k:')
ylabel('x_c [cm]')
ylim([-30 50])

ax3 = subplot(3,1,3); hold on; grid on
for k = 1:numel(rs)
    fill([t(rs(k)) t(re(k)) t(re(k)) t(rs(k))], [-8 -8 8 8], shade_clr, ...
         'EdgeColor','none', 'HandleVisibility','off');
end
plot(t, Vm, 'b-', 'LineWidth', 0.5)
yline(7, 'k--'); yline(-7, 'k--')
ylabel('V_m [V]'); xlabel('Time [s]')
ylim([-8 8])

linkaxes([ax1, ax2, ax3], 'x'); xlim([0 t(end)])
saveas(fig1, fullfile(figdir, 'HW-Liftoff-File2-Timeline.png'));

%% Figure 2: Catch zoom (first catch event at ~t=28s) ---------------------
% The first lift-off, from controller engage to the first sustained catch
t_engage = 27.32;
t_zoom_end = t_engage + 8;
i_zoom = t >= t_engage - 0.5 & t <= t_zoom_end;
% Find catch event within zoom: first time V(x) < c_catch sustained 0.2s
V_zoom = Vt(i_zoom); t_zoom = t(i_zoom);
catch_mask = V_zoom < c_catch;
% Find first index where catch is true and stays true for 100 samples
de2 = diff([0; catch_mask; 0]);
rs2 = find(de2 == 1); re2 = find(de2 == -1) - 1;
catch_dur = (re2 - rs2) * Ts;
ic = find(catch_dur > 0.2, 1);
if ~isempty(ic)
    t_catch = t_zoom(rs2(ic));
else
    t_catch = NaN;
end

fig2 = figure('Position', [60 40 1200 800], 'Color', 'w');
ax21 = subplot(4,1,1); hold on; grid on
plot(t(i_zoom), rad2deg(al(i_zoom)), 'b-', 'LineWidth', 1.2)
yline(0, 'k:'); xline(t_engage, 'r--', 'engage')
if ~isnan(t_catch), xline(t_catch, 'g--', 'V(x)<c\_catch'); end
ylabel('\alpha [deg]')
title('Catch zoom -- supervisor handoff during first lift-off')
ylim([-15 15])

ax22 = subplot(4,1,2); hold on; grid on
plot(t(i_zoom), 100*x_c(i_zoom), 'b-', 'LineWidth', 1.2)
yline(0, 'k:'); xline(t_engage, 'r--'); if ~isnan(t_catch), xline(t_catch, 'g--'); end
ylabel('x_c [cm]')

ax23 = subplot(4,1,3); hold on; grid on
plot(t(i_zoom), Vm(i_zoom), 'b-', 'LineWidth', 0.8)
yline(7, 'k--'); yline(-7, 'k--'); yline(0, 'k:')
xline(t_engage, 'r--'); if ~isnan(t_catch), xline(t_catch, 'g--'); end
ylabel('V_m [V]'); ylim([-8 8])

ax24 = subplot(4,1,4); hold on; grid on
semilogy(t(i_zoom), max(Vt(i_zoom), 1e-4), 'b-', 'LineWidth', 1.2)
yline(c_catch, 'g--', 'c\_catch'); set(gca, 'YScale', 'log')
xline(t_engage, 'r--'); if ~isnan(t_catch), xline(t_catch, 'g--'); end
ylabel('V(x) = x^T P x'); xlabel('Time [s]')

linkaxes([ax21, ax22, ax23, ax24], 'x')
xlim([t_engage - 0.5, t_zoom_end])
saveas(fig2, fullfile(figdir, 'HW-Liftoff-File2-Catch.png'));

%% Figure 3: Steady-state characterisation (FIRST caught window) ---------
% Only the first catch is known to use K_lift; subsequent catches in the
% same run have the latch already tripped, so K_lift never acts.  Window
% duration is just when the operator toggled the switch, not a metric.
% Skip windows that pre-date controller engage (those are the manual hold
% period with V_m=0).
t_engage = 27.32;     % first time |V_m| > 0.05
[~, srt_by_time] = sort(rs);
first_idx = srt_by_time(find(t(rs(srt_by_time)) >= t_engage, 1));
% Trim 3 s after catch (catch transient) and 0.5 s before window end (manual
% release transient) so the metrics describe the limit cycle, not the boundaries.
i_lo = find(t >= t(rs(first_idx)) + 3.0, 1);
i_hi = find(t >= t(re(first_idx)) - 0.5, 1);
ssrng = i_lo:i_hi;
fig3 = figure('Position', [60 40 1200 700], 'Color', 'w');

subplot(3,2,1); hold on; grid on
plot(t(ssrng), rad2deg(al(ssrng)), 'b-', 'LineWidth', 0.8)
yline(0, 'k:'); yline(rad2deg(mean(al(ssrng))), 'r--', 'mean')
ylabel('\alpha [deg]'); xlabel('Time [s]')
title(sprintf('Limit cycle after first catch (t = %.1f .. %.1f s, %.1f s window)', ...
    t(ssrng(1)), t(ssrng(end)), t(ssrng(end))-t(ssrng(1))))

subplot(3,2,2); hold on; grid on
histogram(rad2deg(al(ssrng)), 50, 'FaceColor', [0.4 0.6 1.0])
xline(rad2deg(mean(al(ssrng))), 'r--', 'mean')
xlabel('\alpha [deg]'); ylabel('count'); title('\alpha distribution')

subplot(3,2,3); hold on; grid on
plot(t(ssrng), 100*x_c(ssrng), 'b-', 'LineWidth', 0.8)
yline(0, 'k:'); yline(100*mean(x_c(ssrng)), 'r--')
ylabel('x_c [cm]'); xlabel('Time [s]')

subplot(3,2,4); hold on; grid on
histogram(100*x_c(ssrng), 50, 'FaceColor', [0.4 0.6 1.0])
xline(100*mean(x_c(ssrng)), 'r--')
xlabel('x_c [cm]'); ylabel('count'); title('x_c distribution')

subplot(3,2,5); hold on; grid on
plot(t(ssrng), Vm(ssrng), 'b-', 'LineWidth', 0.5)
yline(0, 'k:'); yline(7, 'k--'); yline(-7, 'k--')
ylabel('V_m [V]'); xlabel('Time [s]')

subplot(3,2,6); hold on; grid on
histogram(Vm(ssrng), 50, 'FaceColor', [0.4 0.6 1.0])
xlabel('V_m [V]'); ylabel('count'); title('V_m distribution')

saveas(fig3, fullfile(figdir, 'HW-Liftoff-File2-SteadyState.png'));

%% Figure 4: Phase plane + V(x) trajectory --------------------------------
fig4 = figure('Position', [60 40 1200 500], 'Color', 'w');

subplot(1,2,1); hold on; grid on
% Plot color-coded by time
sc = scatter(rad2deg(al), alp_dot_f, 4, t, 'filled');
xlabel('\alpha [deg]'); ylabel('\alpha-dot [rad/s]')
title('Phase plane (colour = time)')
cb = colorbar; ylabel(cb, 'Time [s]')
xlim([-15 15])

subplot(1,2,2); hold on; grid on
semilogy(t, max(Vt, 1e-4), 'b-', 'LineWidth', 0.6)
yline(c_catch, 'g--', 'c\_catch')
yline(V_rest, 'r--', 'V_{rest}')
set(gca, 'YScale', 'log')
xlabel('Time [s]'); ylabel('V(x) = x^T P x')
title('Lyapunov score over the full run')
xlim([0 t(end)])

saveas(fig4, fullfile(figdir, 'HW-Liftoff-File2-PhaseLyap.png'));

%% Report numbers ---------------------------------------------------------
fprintf('\n========== Report numbers for thesis ==========\n');
fprintf('Lyapunov design:\n');
fprintf('  Q              = diag([1, 0.1, 10, 1])\n');
fprintf('  V_rest         = %.4f\n', V_rest);
fprintf('  c_catch        = V_rest/3 = %.4f\n', c_catch);
fprintf('Supervisor gains:\n');
fprintf('  K_pp           = [%s]\n', sprintf('%+.2f ', Kf));
fprintf('  beta_lift      = 0.30\n');
fprintf('  K_lift         = [%s]\n', sprintf('%+.2f ', 0.30*Kf));
fprintf('\nRun summary:\n');
fprintf('  total duration : %.1f s\n', t(end));
fprintf('  caught windows : %d (durations %.1f / %.1f / %.1f s)\n', ...
    numel(rs), durs(1), durs(2), durs(3));
ss = ssrng;   % already trimmed (skip catch + manual-release transients)
fprintf('\nLimit cycle after first catch (window t = %.1f .. %.1f s, %.1f s long):\n', ...
    t(ss(1)), t(ss(end)), t(ss(end))-t(ss(1)));
% Robust metrics: percentile-based to ignore one-sample transient spikes
al_deg = rad2deg(al(ss));
fprintf('  alpha p2p (5-95 pct)  : %.2f deg     (robust limit-cycle amplitude)\n', ...
    prctile(al_deg, 95) - prctile(al_deg, 5));
fprintf('  alpha p2p (1-99 pct)  : %.2f deg\n', prctile(al_deg, 99) - prctile(al_deg, 1));
fprintf('  alpha 95-pct |.|      : %.2f deg\n', prctile(abs(al_deg), 95));
fprintf('  alpha mean (bias)     : %+.2f deg\n', mean(al_deg));
fprintf('  alpha RMS (about mean): %.2f deg\n', rms(al_deg - mean(al_deg)));
fprintf('  x_c mean              : %+.2f cm\n', 100*mean(x_c(ss)));
fprintf('  x_c p2p (5-95 pct)    : %.2f cm\n', 100*(prctile(x_c(ss),95)-prctile(x_c(ss),5)));
fprintf('  V_m RMS               : %.2f V\n', rms(Vm(ss)));
fprintf('  V_m 95-pct |.|        : %.2f V\n', prctile(abs(Vm(ss)), 95));
fprintf('\nFigures saved to %s\n', figdir);