function res = analyze_staircase(file, outdir)
%ANALYZE_STAIRCASE  Increasing-then-decreasing reference-angle test.
%
%   The seesaw reference angle is stepped up in small increments to a large
%   value (~10 deg, near the +-11.5 deg physical stop) and then back down.
%   This probes (a) how far the linear controller tracks before it degrades
%   or saturates, and (b) whether the up-leg and down-leg differ — a
%   hysteresis signature of Coulomb friction / backlash.
%
%   Default file: the 2026-05-26 14-channel run that ramps to 10 deg.
%   Produces validation/docs/figures/Staircase-*.png and returns a table of
%   per-level steady-state results.

root = getenv('SEESAW_ROOT');
if isempty(root) || exist(root,'dir')~=7, root = 'C:\Users\Karim Negm\Seesaw'; end
if nargin < 1 || isempty(file), file = fullfile(root,'LQRDD_STAIRCASE.mat'); end
if nargin < 2 || isempty(outdir), outdir = fullfile(root,'validation','docs','figures'); end
addpath(fullfile(root,'validation','scripts'));
local_theme();

c = load_lab_capture(file);
% Use RAW angle and re-zero from the run's own r=0 dwells. The loader's
% leading-baseline offset is unreliable here: the first ~29 s is a release
% transient (seesaw settling ~-5 deg), whereas the run ends back at r=0 with
% the encoder reading ~0 — so the true zero is the trailing baseline.
r = rad2deg(c.r_theta); th = rad2deg(c.theta_raw); t = c.t;

% --- detect constant-reference dwells (>= 4 s) and their steady-state angle ---
rr = round(r/0.25)*0.25;
edges = [1; find(diff(rr)~=0)+1; numel(rr)+1];
L = struct('r',{},'t0',{},'t1',{},'th',{},'Vrms',{},'Vpk',{},'xc',{});
for k = 1:numel(edges)-1
    i0 = edges(k); i1 = edges(k+1)-1;
    if t(i1)-t(i0) < 4, continue; end          % ignore brief transitions
    ss = i0 + round(0.5*(i1-i0)) : i1;         % steady = last half of dwell
    e.r = rr(i0); e.t0 = t(i0); e.t1 = t(i1);
    e.th = mean(th(ss));
    e.Vrms = rms(c.V(ss)); e.Vpk = max(abs(c.V(ss)));
    e.xc = mean(c.xc(ss))*100;
    L(end+1) = e; %#ok<AGROW>
end
% re-zero: offset = mean measured angle over r=0 dwells, excluding the first
z0 = find([L.r]==0);
if numel(z0) >= 2, off0 = mean([L(z0(2:end)).th]); elseif ~isempty(z0), off0 = L(z0(end)).th; else, off0 = 0; end
th = th - off0;
for k=1:numel(L)
    L(k).th = L(k).th - off0;
    L(k).err = L(k).th - L(k).r;
    L(k).gain = L(k).th / L(k).r;
end
[~,kmax] = max([L.r]);
up = 1:kmax; dn = kmax:numel(L);          % up-leg and down-leg (share the peak)

res.levels = L; res.file = c.name;

% ================= Figure 1: time series =================
f = local_fig([50 50 1250 620]);
yyaxis left;
plot(t, r,'k-','LineWidth',1.3); hold on;
plot(t, th,'r-','LineWidth',0.7);
ylabel('Seesaw angle \theta [deg]'); ylim([-2 12]);
yline(11.5,'--','Color',[.5 .5 .5]); yline(-11.5,'--','Color',[.5 .5 .5]);
yyaxis right;
plot(t, c.V,'Color',[0 .55 0],'LineWidth',0.4);
ylabel('Motor voltage [V]'); ylim([-12 12]);
ax=gca; ax.YAxis(1).Color='k'; ax.YAxis(2).Color=[0 .45 0];
grid on; xlabel('Time [s]'); xlim([0 t(end)]);
legend({'\theta_{ref}','\theta measured','physical stop','motor voltage'},'Location','northwest');
title('Staircase reference test: increasing then decreasing angle','FontWeight','bold');
saveas(f, fullfile(outdir,'Staircase-TimeSeries.png')); close(f);
fprintf('  saved Staircase-TimeSeries.png\n');

% ================= Figure 2: static curve + gain/error vs amplitude =====
f = local_fig([60 60 1250 560]);

subplot(1,2,1);
plot([0 11],[0 11],'k:','LineWidth',1); hold on;
plot([L(up).r],[L(up).th],'o-','Color',[.85 .33 .1],'LineWidth',1.6,'MarkerFaceColor',[.85 .33 .1]);
plot([L(dn).r],[L(dn).th],'s--','Color',[.1 .4 .85],'LineWidth',1.6,'MarkerFaceColor',[.1 .4 .85]);
grid on; axis equal; xlim([0 11]); ylim([0 11]);
xlabel('Reference angle \theta_{ref} [deg]'); ylabel('Measured steady angle \theta [deg]');
legend({'ideal (unity)','increasing','decreasing'},'Location','northwest');
title('Static tracking: up-leg vs down-leg (gap = hysteresis)');

subplot(1,2,2);
yyaxis left;
plot([L.r],[L.err],'o-','Color',[.85 .33 .1],'LineWidth',1.5,'MarkerFaceColor',[.85 .33 .1]);
ylabel('Steady-state error \theta-\theta_{ref} [deg]'); hold on; yline(0,'k:');
yyaxis right;
plot([L.r],[L.Vpk],'^-','Color',[0 .5 0],'LineWidth',1.3,'MarkerFaceColor',[0 .5 0]);
ylabel('Peak motor voltage [V]'); yline(9,'--','Color',[.5 .5 .5]);
ax=gca; ax.YAxis(1).Color=[.85 .33 .1]; ax.YAxis(2).Color=[0 .45 0];
grid on; xlabel('Reference angle \theta_{ref} [deg]');
title('Tracking error and control effort vs commanded angle');

sgtitle(sprintf('Staircase angle test  (peak %.0f{\\circ}, state-feedback + dirty-derivative)', max([L.r])),'FontWeight','bold');
saveas(f, fullfile(outdir,'Staircase-Static.png')); close(f);
fprintf('  saved Staircase-Static.png\n');

% --- console summary ---
fprintf('\n  ref[deg]  meas[deg]  err[deg]  gain   Vpk[V]  cart[cm]\n');
for k=1:numel(L)
    leg = '^'; if k>=kmax, leg='v'; end
    fprintf('   %s %5.1f   %6.2f   %+6.2f  %5.2f  %5.1f   %+5.1f\n', ...
        leg, L(k).r, L(k).th, L(k).err, L(k).gain, L(k).Vpk, L(k).xc);
end
end

% --------------------------------------------------------------------------
function local_theme()
set(groot,'defaultAxesTickLabelInterpreter','tex');
set(groot,'defaultTextInterpreter','tex'); set(groot,'defaultLegendInterpreter','tex');
set(groot,'defaultFigureColor','w'); set(groot,'defaultAxesColor','w');
set(groot,'defaultAxesXColor',[.15 .15 .15]); set(groot,'defaultAxesYColor',[.15 .15 .15]);
set(groot,'defaultAxesGridColor',[.65 .65 .65]); set(groot,'defaultAxesGridAlpha',0.35);
set(groot,'defaultTextColor','k'); set(groot,'defaultAxesFontSize',10);
set(groot,'defaultLegendTextColor','k');
end

function f = local_fig(pos)
f = figure('Position',pos,'Visible','off','Color','w');
end
