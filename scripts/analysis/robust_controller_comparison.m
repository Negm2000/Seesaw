%% Robust Controller Comparison -- Rocking / Effort Objective
%
% Compares several robust-control candidates on the same deadzone-driven
% rocking simulation used for SMC-vs-PP validation.  The ranking priority is:
%   1) steady-state theta peak-to-peak, 2) voltage effort, 3) smoothness.
%
% Exact Hinf synthesis and LMI synthesis require Robust Control Toolbox and
% Optimization Toolbox/LMI Lab respectively.  This script remains runnable
% without those toolboxes and uses:
%   - an LMI-screened state-feedback sweep: candidates must satisfy a fixed-P
%     quadratic Lyapunov inequality over uncertainty vertices;
%   - a state-feedback Hinf Riccati fallback when hinfsyn is unavailable;
%   - a Takagi-Sugeno fuzzy gain scheduler built with Fuzzy Logic Toolbox
%     when available, with the same rule base implemented manually otherwise.
%
% Requires: startup.m or direct access to scripts/config/seesaw_params.m,
%           data/tuned_params.mat, data/controller_freq.mat.
% Optional: data/controller_smc.mat.  If missing, the SMC design is rebuilt
%           in memory using the same equations as scripts/control/smc_design.m.
% Outputs:  data/robust_controller_comparison.mat
%           docs/figures/Robust-Control-Comparison.png
%           docs/figures/Robust-Control-SteadyState.png
%           docs/figures/Robust-Control-ThetaPhasePlane.png
%           docs/figures/Robust-Control-RateLimitSurvival.png

clear; close all; clc
set(groot, 'defaultAxesTickLabelInterpreter', 'none');
set(groot, 'defaultLegendInterpreter', 'none');
set(groot, 'defaultTextInterpreter', 'none');

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figdir = fullfile(root, 'docs', 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

fprintf('\n========================================\n')
fprintf(' Robust Controller Comparison\n')
fprintf('========================================\n')

toolbox_report();

%% Load plant and baseline controller
run(fullfile(root, 'scripts', 'config', 'seesaw_params.m'))
tuned = load(fullfile(root, 'data', 'tuned_params.mat'));
ctrl = load(fullfile(root, 'data', 'controller_freq.mat'));

plant = build_benchmark_plant(tuned, ctrl, M_c);

fprintf('\nPlant used for comparison:\n')
fprintf('  B_eq = %.4f N*s/m\n', plant.B_eq)
fprintf('  M_c  = %.3f kg (includes %.3f kg added mass)\n', ...
    plant.M_c, ctrl.M_c_added)
fprintf('  Open-loop max real pole = %.3f rad/s\n', max(real(eig(plant.A))))
fprintf('  Voltage limit = +/- %.1f V\n', V_sat)
fprintf('  Rail limit    = +/- %.1f mm\n', x_c_max * 1000)
fprintf('  Angle limit   = +/- %.1f deg\n', rad2deg(alpha_max))

constraints.V_sat = V_sat;
constraints.x_max = x_c_max;
constraints.theta_max = alpha_max;
constraints.q_xc = K_ec;
constraints.q_theta = K_E_SW / K_gs;
constraints.deadzone = 0.12;
constraints.actuator_gain_pos = 1.0;
constraints.actuator_gain_neg = 0.5;
constraints.coulomb_force = 0.18;
constraints.M_c = plant.M_c;
constraints.u_rate_limit = Inf;       % [V/s], Inf = no command slew limit

scenario.T = 20;
scenario.Ts = 0.002;
scenario.trim_time = 15;
scenario.x0 = [0; 0; deg2rad(2.0); 0];

tune_scenario = scenario;
tune_scenario.T = 12;
tune_scenario.Ts = 0.004;
tune_scenario.trim_time = 8;

%% Controller candidates
controllers = struct('name', {}, 'type', {}, 'K', {}, 'data', {}, 'notes', {});

controllers(end+1) = gain_controller('Pole placement', ctrl.Kf, ...
    'Existing baseline from controller_freq.mat.');

fprintf('\nTuning LMI-screened robust gain...\n')
[K_lmi, lmi_info] = tune_lmi_screened_gain(plant, constraints, tune_scenario);
controllers(end+1) = gain_controller('LMI-screened SF', K_lmi, lmi_info.note);
controllers(end).data = lmi_info;

fprintf('\nTuning Hinf/Riccati gain...\n')
[K_hinf, hinf_info] = tune_hinf_riccati_gain(plant, constraints, tune_scenario);
controllers(end+1) = gain_controller('Hinf Riccati SF', K_hinf, hinf_info.note);
controllers(end).data = hinf_info;

if exist('hinfsyn', 'file') == 2
    fprintf('\nTuning true dynamic Hinf controller...\n')
    [hinf_dyn_data, hinf_dyn_info] = tune_dynamic_hinf_controller(plant, constraints, tune_scenario);
    if hinf_dyn_info.success
        controllers(end+1) = dynamic_hinf_controller('Hinf dynamic', hinf_dyn_data, hinf_dyn_info.note);
        controllers(end).data.info = hinf_dyn_info;
    else
        fprintf('  No true dynamic hinfsyn candidate survived the nonlinear constraints.\n')
    end
end

fprintf('\nTuning actuator-aware Hinf/Riccati gain...\n')
[act_data, act_info] = tune_actuator_aware_gain(plant, constraints, tune_scenario);
controllers(end+1) = actuator_aware_controller('Hinf actuator-aware', act_data, act_info.note);
controllers(end).data.info = act_info;

fprintf('\nPreparing existing super-twisting SMC...\n')
smc_data = load_or_design_smc(root, plant.A, plant.B, constraints);
controllers(end+1) = smc_controller('SMC super-twist', smc_data, ...
    'Existing boundary-layer super-twisting SMC design.');

fprintf('\nTuning softened scheduled SMC...\n')
[smc_soft_data, smc_soft_info] = tune_scheduled_smc(plant, constraints, tune_scenario, smc_data);
controllers(end+1) = smc_scheduled_controller('SMC scheduled soft', smc_soft_data, smc_soft_info.note);
controllers(end).data.info = smc_soft_info;

fprintf('\nTuning u-augmented super-twisting SMC (5-state, V/s as virtual input)...\n')
[smc_aug_data, smc_aug_info] = tune_augmented_smc(plant, constraints, tune_scenario);
controllers(end+1) = smc_augmented_controller('SMC u-aug super-twist', smc_aug_data, smc_aug_info.note);
controllers(end).data.info = smc_aug_info;

fprintf('\nTuning manual fuzzy gain scheduler...\n')
[fuzzy_data, fuzzy_info] = tune_fuzzy_controller(plant, constraints, tune_scenario, ...
    ctrl.Kf, K_lmi, K_hinf);
controllers(end+1) = fuzzy_controller('Fuzzy scheduled SF', fuzzy_data, fuzzy_info.note);
controllers(end).data.info = fuzzy_info;

%% Final comparison at full resolution
fprintf('\nRunning final %g s simulation at Ts = %.4f s...\n', scenario.T, scenario.Ts)
for i = 1:numel(controllers)
    fprintf('  %-22s', controllers(i).name)
    result = simulate_controller(plant.A, plant.B, controllers(i), constraints, scenario);
    controllers(i).data.result = result;
    fprintf(' theta p2p = %6.3f deg, u_rms = %5.3f V, u_pk = %5.3f V\n', ...
        result.metrics.theta_p2p_deg, result.metrics.u_rms, result.metrics.u_peak)
end

results = collect_results(controllers);
results = sortrows(results, {'constraint_ok', 'theta_p2p_deg', 'u_rms', 'u_peak'}, ...
                   {'descend', 'ascend', 'ascend', 'ascend'});

fprintf('\n=== Final Ranking (constraints first, then theta p2p, then effort) ===\n')
print_results_table(results)

best_name = string(results.name(1));
fprintf('\nBest in this simulation: %s\n', best_name)

rate_limits = [Inf 300 200 150 100 75 50 35 25 15 10 7.5 5];
fprintf('\nRunning voltage slew-rate survival sweep...\n')
rate_results = run_rate_limit_sweep(plant.A, plant.B, controllers, constraints, ...
                                    scenario, rate_limits);
print_rate_limit_summary(rate_results, rate_limits)

save(fullfile(root, 'data', 'robust_controller_comparison.mat'), ...
     'controllers', 'results', 'rate_results', 'rate_limits', ...
     'plant', 'constraints', 'scenario')
fprintf('Saved data/robust_controller_comparison.mat\n')

plot_results(controllers, results, scenario, constraints, figdir)
plot_rate_limit_results(rate_results, figdir)

%% Local helpers ---------------------------------------------------------
function toolbox_report()
    fprintf('\nToolbox availability:\n')
    fprintf('  Robust Control Toolbox / hinfsyn: %s\n', yesno(exist('hinfsyn', 'file') == 2))
    fprintf('  LMI Lab / setlmis:               %s\n', yesno(exist('setlmis', 'file') == 2))
    fprintf('  Optimization / quadprog:         %s\n', yesno(exist('quadprog', 'file') == 2))
    fprintf('  Fuzzy Logic / mamfis:            %s\n', yesno(exist('mamfis', 'file') == 2))
    if exist('hinfsyn', 'file') ~= 2
        fprintf('  -> Exact hinfsyn is unavailable; using a state-feedback Riccati fallback.\n')
    end
    if exist('setlmis', 'file') ~= 2
        fprintf('  -> Exact LMI synthesis is unavailable; using fixed-P Lyapunov LMI screening.\n')
    end
    if exist('mamfis', 'file') ~= 2
        fprintf('  -> Fuzzy Toolbox is unavailable; using manual Takagi-Sugeno scheduling.\n')
    end
end

function s = yesno(tf)
    if tf, s = 'yes'; else, s = 'no'; end
end

function plant = build_benchmark_plant(tuned, ctrl, M_c_base)
    % Match scripts/analysis/smc_vs_pp_nonlinear.m exactly: use the tuned
    % linear A/B matrices from identification, and use the controller's added
    % cart mass only when converting Coulomb friction to acceleration.
    plant.B_eq = tuned.B_eq;
    plant.M_c = M_c_base + ctrl.M_c_added;
    plant.A = tuned.A_sw;
    plant.B = tuned.B_sw;
end

function ctrl = gain_controller(name, K, notes)
    ctrl.name = name;
    ctrl.type = 'gain';
    ctrl.K = K(:)';
    ctrl.data = struct();
    ctrl.notes = notes;
end

function ctrl = smc_controller(name, data, notes)
    ctrl.name = name;
    ctrl.type = 'smc';
    ctrl.K = [];
    ctrl.data = data;
    ctrl.notes = notes;
end

function ctrl = smc_scheduled_controller(name, data, notes)
    ctrl.name = name;
    ctrl.type = 'smc_scheduled';
    ctrl.K = [];
    ctrl.data = data;
    ctrl.notes = notes;
end

function ctrl = smc_augmented_controller(name, data, notes)
    ctrl.name = name;
    ctrl.type = 'smc_augmented';
    ctrl.K = [];
    ctrl.data = data;
    ctrl.notes = notes;
end

function ctrl = actuator_aware_controller(name, data, notes)
    ctrl.name = name;
    ctrl.type = 'actuator_aware';
    ctrl.K = [];
    ctrl.data = data;
    ctrl.notes = notes;
end

function ctrl = dynamic_hinf_controller(name, data, notes)
    ctrl.name = name;
    ctrl.type = 'hinf_dynamic';
    ctrl.K = [];
    ctrl.data = data;
    ctrl.notes = notes;
end

function ctrl = fuzzy_controller(name, data, notes)
    ctrl.name = name;
    ctrl.type = 'fuzzy';
    ctrl.K = [];
    ctrl.data = data;
    ctrl.notes = notes;
end

function [K_best, info] = tune_lmi_screened_gain(plant, constraints, scenario)
    vertices = uncertainty_vertices(plant);
    theta_weights = [1500 3000 6000 10000 18000 30000];
    rate_weights = [0 10 40];
    r_weights = [0.08 0.15 0.3 0.6 1.2 2.5];

    best_score = inf;
    K_best = [];
    info = struct('candidates', [], 'seed_candidates', [], 'note', '');
    rows = [];
    seed_rows = [];
    seed_gains = {};
    for qth = theta_weights
        for qrate = rate_weights
            for r = r_weights
                Q = diag([0.5, 0.03, qth, qrate]);
                try
                    K = lqr(plant.A, plant.B, Q, r);
                catch
                    continue
                end
                c = gain_controller('candidate', K, '');
                sim = simulate_controller(plant.A, plant.B, c, constraints, scenario);
                vertex = vertex_stability(vertices, K);
                score = priority_score(sim.metrics) + 500 * max(0, vertex.worst_real + 0.05);
                seed_gains{end+1} = K; %#ok<AGROW>
                seed_rows = [seed_rows; qth, qrate, r, score, ...
                             sim.metrics.theta_p2p_deg, sim.metrics.u_rms, ...
                             sim.metrics.u_peak, vertex.worst_real]; %#ok<AGROW>
            end
        end
    end

    if isempty(seed_rows)
        warning('No LMI seed candidates could be built. Falling back to nominal PP gain.')
        K_best = load_pp_fallback();
        info.note = 'Fallback: no LQR seed candidates could be built.';
        return
    end

    [~, seed_order] = sort(seed_rows(:, 4), 'ascend');
    seed_count = min(6, numel(seed_order));
    for idx = 1:seed_count
        seed_idx = seed_order(idx);
        K_seed = seed_gains{seed_idx};
        cert = certify_fixed_gain_lmi(vertices, K_seed, 0.05);
        if ~cert.pass
            continue
        end
        lmi = fixed_p_lmi_screen(vertices, K_seed);
        c = gain_controller('candidate', K_seed, '');
        sim = simulate_controller(plant.A, plant.B, c, constraints, scenario);
        score = priority_score(sim.metrics);
        rows = [rows; seed_idx, score, sim.metrics.theta_p2p_deg, ...
                sim.metrics.u_rms, sim.metrics.u_peak, cert.margin, lmi.margin]; %#ok<AGROW>
                if score < best_score
                    best_score = score;
                    K_best = K_seed;
                    best_lmi = cert; %#ok<NASGU>
                    best_metrics = sim.metrics; %#ok<NASGU>
                end
    end

    if isempty(K_best)
        warning('No common quadratic LMI certificate found. Using best robust LQR seed.')
        K_best = seed_gains{seed_order(1)};
        info.note = ['Fallback: common-quadratic LMI did not certify a gain; ' ...
            'best vertex-stable robust LQR seed used.'];
    else
        info.note = sprintf(['High-performing state-feedback seed certified by ' ...
            'common quadratic Lyapunov LMIs over %d uncertainty vertices.'], numel(vertices));
    end
    info.candidates = rows;
    info.seed_candidates = seed_rows;

    function Kfb = load_pp_fallback()
        persistent Kfallback
        if isempty(Kfallback)
            Kfallback = place(plant.A, plant.B, [-5.5+4.1j -5.5-4.1j -8.25 -6.6]);
        end
        Kfb = Kfallback;
    end
end

function vertices = uncertainty_vertices(plant)
    a_scales = [0.95 1.00 1.05];
    b_scales = [0.85 1.00 1.15];
    vertices = struct('A', {}, 'B', {});
    for ia = 1:numel(a_scales)
        for ib = 1:numel(b_scales)
            v.A = plant.A;
            v.B = plant.B * b_scales(ib);
            grav_rows = [2 4];
            grav_cols = [1 3];
            v.A(grav_rows, grav_cols) = v.A(grav_rows, grav_cols) * a_scales(ia);
            vertices(end+1) = v; %#ok<AGROW>
        end
    end
end

function lmi = fixed_p_lmi_screen(vertices, K)
    Acl0 = vertices(ceil(numel(vertices)/2)).A - vertices(ceil(numel(vertices)/2)).B * K;
    lmi.pass = false;
    lmi.margin = -inf;
    if max(real(eig(Acl0))) >= -1e-6
        return
    end
    try
        P = lyap(Acl0', eye(size(Acl0)));
    catch
        return
    end
    if min(eig((P + P') / 2)) <= 1e-8
        return
    end
    worst = -inf;
    for i = 1:numel(vertices)
        Acl = vertices(i).A - vertices(i).B * K;
        M = Acl' * P + P * Acl;
        worst = max(worst, max(real(eig((M + M') / 2))));
    end
    lmi.margin = -worst;
    lmi.pass = worst < -1e-4;
end

function v = vertex_stability(vertices, K)
    v.pass = true;
    v.worst_real = -inf;
    for i = 1:numel(vertices)
        Acl = vertices(i).A - vertices(i).B * K;
        wr = max(real(eig(Acl)));
        v.worst_real = max(v.worst_real, wr);
        v.pass = v.pass && wr < -1e-6;
    end
end

function cert = certify_fixed_gain_lmi(vertices, K, decay_rate)
    nx = size(vertices(1).A, 1);
    if exist('setlmis', 'file') == 2 && exist('feasp', 'file') == 2
        setlmis([])
        P = lmivar(1, [nx 1]);
        lmiterm([-1 1 1 P], 1, 1);
        for i = 1:numel(vertices)
            Acl = vertices(i).A - vertices(i).B * K;
            lmiterm([i+1 1 1 P], Acl', 1, 's');
            lmiterm([i+1 1 1 P], decay_rate, 1, 's');
        end
        lmis = getlmis;
        [tmin, xfeas] = feasp(lmis, [1e-4 0 0 0 0], -1);
        cert.pass = tmin < -1e-7;
        cert.margin = -tmin;
        cert.min_p = NaN;
        if cert.pass
            Pval = dec2mat(lmis, xfeas, P);
            cert.min_p = min(real(eig((Pval + Pval') / 2)));
        end
        return
    end

    cert = fixed_p_lmi_screen(vertices, K);
    cert.min_p = NaN;
end

function [K, cert] = solve_lmi_state_feedback(vertices, K_seed, decay_rate)
    % Solve P=P'>0 and Y=K*P satisfying the standard full-state-feedback LMI:
    %   A_i*P + P*A_i' - B_i*Y - Y'*B_i' + 2*a*P < 0
    % at all uncertainty vertices.
    if exist('setlmis', 'file') == 2 && exist('feasp', 'file') == 2
        [K, cert] = solve_lmi_state_feedback_lmilab(vertices, decay_rate);
        if cert.pass
            return
        end
    end

    % Fallback local search if LMI Lab is unavailable or infeasible.
    A0 = vertices(ceil(numel(vertices)/2)).A;
    B0 = vertices(ceil(numel(vertices)/2)).B;
    Acl0 = A0 - B0 * K_seed;
    P0 = lyap(Acl0', eye(size(Acl0)));
    P0 = P0 / max(trace(P0), eps);
    Y0 = K_seed * P0;
    z0 = pack_lmi_variables(P0, Y0);
    opts = optimset('Display', 'off', 'MaxIter', 2500, 'MaxFunEvals', 9000, ...
                    'TolX', 1e-7, 'TolFun', 1e-9);
    objective = @(z) lmi_penalty(z, vertices, decay_rate);
    z = fminsearch(objective, z0, opts);
    [P, Y, worst, min_p] = unpack_lmi_variables(z, vertices, decay_rate);

    cert.pass = worst < -1e-7 && min_p > 1e-8;
    cert.margin = -worst;
    cert.min_p = min_p;
    if cert.pass
        K = Y / P;
    else
        K = K_seed;
    end
end

function [K, cert] = solve_lmi_state_feedback_lmilab(vertices, decay_rate)
    nx = size(vertices(1).A, 1);
    nu = size(vertices(1).B, 2);
    setlmis([])
    P = lmivar(1, [nx 1]);
    Y = lmivar(2, [nu nx]);

    lmiterm([-1 1 1 P], 1, 1);              % P > I*eps via objective margin
    for i = 1:numel(vertices)
        A = vertices(i).A;
        B = vertices(i).B;
        lmiterm([i+1 1 1 P], A, 1, 's');
        lmiterm([i+1 1 1 P], decay_rate, 1, 's');
        lmiterm([i+1 1 1 Y], -B, 1, 's');
    end

    lmis = getlmis;
    opts = [1e-6, 0, 0, 0, 0];
    [tmin, xfeas] = feasp(lmis, opts, -1);
    cert.pass = tmin < -1e-7;
    cert.margin = -tmin;
    cert.min_p = NaN;
    if cert.pass
        Pval = dec2mat(lmis, xfeas, P);
        Yval = dec2mat(lmis, xfeas, Y);
        cert.min_p = min(real(eig((Pval + Pval') / 2)));
        K = Yval / Pval;
    else
        K = zeros(nu, nx);
    end
end

function z = pack_lmi_variables(P, Y)
    z = [P(1,1), P(2,1), P(2,2), P(3,1), P(3,2), P(3,3), ...
         P(4,1), P(4,2), P(4,3), P(4,4), Y(:)'];
end

function val = lmi_penalty(z, vertices, decay_rate)
    [P, Y, worst, min_p] = unpack_lmi_variables(z, vertices, decay_rate);
    pos = max(0, worst / max(trace(P), eps));
    pd = max(0, 1e-7 - min_p);
    val = 1e8 * pos^2 + 1e10 * pd^2 + 1e-8 * sum(Y.^2) + 1e-10 * sum(z.^2);
end

function [P, Y, worst, min_p] = unpack_lmi_variables(z, vertices, decay_rate)
    P = [z(1), z(2), z(4), z(7);
         z(2), z(3), z(5), z(8);
         z(4), z(5), z(6), z(9);
         z(7), z(8), z(9), z(10)];
    P = (P + P') / 2;
    Y = z(11:14);
    min_p = min(real(eig(P)));
    worst = -inf;
    for i = 1:numel(vertices)
        M = vertices(i).A * P - vertices(i).B * Y;
        S = M + M' + 2 * decay_rate * P;
        worst = max(worst, max(real(eig((S + S') / 2))));
    end
end

function [K_best, info] = tune_hinf_riccati_gain(plant, constraints, scenario)
    theta_weights = [1500 3000 6000 10000 18000 30000];
    r_weights = [0.08 0.15 0.3 0.6 1.2 2.5];
    gammas = [0.75 1.0 1.5 2.5 4.0 8.0];
    B1 = [0; 1; 0; 0.25];

    best_score = inf;
    K_best = [];
    rows = [];
    for qth = theta_weights
        Q = diag([0.5, 0.03, qth, 10]);
        for r = r_weights
            for gamma = gammas
                try
                    K = hinf_state_feedback_gain(plant.A, plant.B, B1, Q, r, gamma);
                catch
                    continue
                end
                if any(~isfinite(K)) || max(real(eig(plant.A - plant.B * K))) >= -0.05
                    continue
                end
                c = gain_controller('candidate', K, '');
                sim = simulate_controller(plant.A, plant.B, c, constraints, scenario);
                score = priority_score(sim.metrics);
                rows = [rows; qth, r, gamma, score, sim.metrics.theta_p2p_deg, ...
                        sim.metrics.u_rms, sim.metrics.u_peak]; %#ok<AGROW>
                if score < best_score
                    best_score = score;
                    K_best = K;
                end
            end
        end
    end

    if isempty(K_best)
        warning('Hinf Riccati candidates failed. Falling back to LQR-like gain.')
        K_best = lqr(plant.A, plant.B, diag([0.5 0.03 8000 10]), 0.5);
        info.note = 'Fallback: Hinf Riccati solve failed; nominal LQR-like gain used.';
    else
        info.note = ['State-feedback Hinf/Riccati fallback selected by simulation; ' ...
            'install Robust Control Toolbox for exact hinfsyn/mixsyn.'];
    end
    info.candidates = rows;
end

function K = hinf_state_feedback_gain(A, B2, B1, Q, R, gamma)
    % Continuous-time bounded-real state-feedback Riccati fallback.  This is
    % not a replacement for hinfsyn; it gives a comparable Hinf-shaped gain
    % without Robust Control Toolbox when a stabilizing solution exists.
    Bhat = [B2, B1];
    Rhat = blkdiag(R, -gamma^2 * eye(size(B1, 2)));
    [X, ~, report] = care(A, Bhat, Q, Rhat);
    if report < 0
        error('care did not find a stabilizing solution')
    end
    K = R \ (B2' * X);
end

function [data_best, info] = tune_dynamic_hinf_controller(plant, constraints, scenario)
    theta_scales = [1 2 5 10 20];
    cart_scales = [0.1 0.5 1];
    effort_scales = [10 50 100 300];
    output_scales = [0.02 0.05 0.10 0.20 0.35];
    output_taus = [0.02 0.05 0.10 0.20];
    state_leaks = [0 1 3 6];

    best_score = inf;
    data_best = struct();
    rows = [];
    opts = hinfsynOptions('Display', 'off', 'Method', 'RIC', ...
                         'Regularize', 'on', 'AutoScale', 'on', ...
                         'LimitGain', 'on');
    for wt = theta_scales
        for wx = cart_scales
            for wu = effort_scales
                try
                    P = build_hinf_generalized_plant(plant.A, plant.B, wt, wx, wu);
                    [K, ~, gamma] = hinfsyn(P, 4, 1, opts);
                catch
                    continue
                end
                if isempty(K) || order(K) == 0 || any(~isfinite(K.A(:)))
                    continue
                end
                data = dynamic_controller_data(K);
                data.weights = [wt, wx, wu];
                data.gamma = gamma;
                for scale = output_scales
                    for tau = output_taus
                        for leak = state_leaks
                            d = data;
                            d.output_scale = scale;
                            d.output_tau = tau;
                            d.state_leak = leak;
                            c = dynamic_hinf_controller('candidate', d, '');
                            sim = simulate_controller(plant.A, plant.B, c, constraints, scenario);
                            score = hardware_friendly_score(sim.metrics);
                            rows = [rows; wt, wx, wu, gamma, scale, tau, leak, score, ...
                                    sim.metrics.theta_p2p_deg, sim.metrics.u_rms, ...
                                    sim.metrics.u_slew_peak, sim.metrics.u_peak, ...
                                    sim.metrics.constraint_ok]; %#ok<AGROW>
                            if sim.metrics.constraint_ok && score < best_score
                                best_score = score;
                                data_best = d;
                            end
                        end
                    end
                end
            end
        end
    end

    if isempty(fieldnames(data_best))
        warning('Dynamic hinfsyn candidates failed nonlinear constraint validation.')
        info.note = 'No true dynamic hinfsyn candidate survived nonlinear validation.';
        info.success = false;
    else
        info.note = ['True dynamic Hinf controller synthesized with hinfsyn and ' ...
            'deployed through tuned output scaling/leakage/actuator smoothing.'];
        info.success = true;
    end
    info.candidates = rows;
end

function P = build_hinf_generalized_plant(A, B, wt, wx, wu)
    % Inputs are [w; u], outputs are [z; y].  z penalizes weighted states and
    % control; y exposes all states to the dynamic controller.  A tiny D12
    % regularization keeps hinfsyn well-posed for state penalties.
    nx = size(A, 1);
    B1 = [0; 1; 0; 0.25];
    B2 = B;
    C1 = [wx 0 0 0;
          0 0 wt 0;
          0 0 0 0.2 * wt;
          zeros(1, nx)];
    D11 = zeros(4, 1);
    D12 = [1e-4; 1e-4; 1e-4; wu];
    C2 = eye(nx);
    D21 = zeros(nx, 1);
    D22 = zeros(nx, 1);
    P = ss(A, [B1 B2], [C1; C2], [D11 D12; D21 D22]);
end

function data = dynamic_controller_data(K)
    Kss = ss(K);
    data.A = Kss.A;
    data.B = Kss.B;
    data.C = Kss.C;
    data.D = Kss.D;
    data.xk0 = zeros(size(Kss.A, 1), 1);
    data.output_scale = 1;
    data.output_tau = 0;
    data.state_leak = 0;
end

function [data_best, info] = tune_actuator_aware_gain(plant, constraints, scenario)
    tau_sets = [0.035 0.05 0.075 0.10];
    theta_weights = [6000 10000 18000];
    u_state_weights = [5 15 30];
    r_weights = [0.15 0.3 0.6];

    best_score = inf;
    data_best = struct();
    rows = [];
    for tau = tau_sets
        A_aug = [plant.A, plant.B; zeros(1,4), -1/tau];
        B_aug = [zeros(4,1); 1/tau];
        B1 = [0; 1; 0; 0.25; 0];
        for qth = theta_weights
            for qu = u_state_weights
                Q = diag([0.5, 0.03, qth, 10, qu]);
                for r = r_weights
                    try
                        K = hinf_state_feedback_gain(A_aug, B_aug, B1, Q, r, 2.5);
                    catch
                        continue
                    end
                    if any(~isfinite(K)) || max(real(eig(A_aug - B_aug * K))) >= -0.05
                        continue
                    end
                    data.K_aug = K;
                    data.tau_act = tau;
                    c = actuator_aware_controller('candidate', data, '');
                    sim = simulate_controller(plant.A, plant.B, c, constraints, scenario);
                    score = hardware_friendly_score(sim.metrics);
                    rows = [rows; tau, qth, qu, r, score, sim.metrics.theta_p2p_deg, ...
                            sim.metrics.u_rms, sim.metrics.u_slew_peak, sim.metrics.u_peak]; %#ok<AGROW>
                    if score < best_score
                        best_score = score;
                        data_best = data;
                    end
                end
            end
        end
    end

    if isempty(fieldnames(data_best))
        warning('Actuator-aware candidates failed. Falling back to nominal Hinf gain with tau=0.05.')
        [K, ~] = tune_hinf_riccati_gain(plant, constraints, scenario);
        data_best.K_aug = [K, 0];
        data_best.tau_act = 0.05;
        info.note = 'Fallback: actuator-aware augmented design failed.';
    else
        info.note = 'Augmented actuator-state Hinf/Riccati design penalizing voltage state and command effort.';
    end
    info.candidates = rows;
end

function score = hardware_friendly_score(metrics)
    penalty = 0;
    if ~metrics.constraint_ok, penalty = penalty + 1e4; end
    score = penalty ...
          + 130 * metrics.theta_p2p_deg ...
          + 20 * metrics.u_rms ...
          + 0.08 * metrics.u_slew_peak ...
          + 0.4 * metrics.u_tv_per_s ...
          + 20 * metrics.saturation_pct;
end

function [fuzzy, info] = tune_fuzzy_controller(plant, constraints, scenario, K_pp, K_lmi, K_hinf)
    threshold_sets = [0.25 0.8  2.0;
                      0.35 1.0  2.5;
                      0.50 1.4  3.0;
                      0.75 1.8  4.0];
    rate_sets = [10 25 45];
    gain_sets = {
        0.70 * K_hinf, K_lmi, 1.10 * K_pp;
        0.55 * K_hinf, K_lmi, 1.00 * K_pp;
        0.45 * K_lmi,  K_hinf, 1.15 * K_pp;
        0.60 * K_lmi,  K_lmi,  0.95 * K_pp};

    best_score = inf;
    rows = [];
    fuzzy = struct();
    for ig = 1:size(gain_sets, 1)
        for it = 1:size(threshold_sets, 1)
            for ir = 1:numel(rate_sets)
                data.K_small = gain_sets{ig, 1};
                data.K_mid = gain_sets{ig, 2};
                data.K_large = gain_sets{ig, 3};
                data.theta_breaks_deg = threshold_sets(it, :);
                data.rate_break_deg_s = rate_sets(ir);
                data.use_fis = false;
                c = fuzzy_controller('candidate', data, '');
                sim = simulate_controller(plant.A, plant.B, c, constraints, scenario);
                score = priority_score(sim.metrics);
                rows = [rows; ig, threshold_sets(it, :), rate_sets(ir), score, ...
                        sim.metrics.theta_p2p_deg, sim.metrics.u_rms, sim.metrics.u_peak]; %#ok<AGROW>
                if score < best_score
                    best_score = score;
                    fuzzy = data;
                end
            end
        end
    end
    if exist('sugfis', 'file') == 2 && exist('evalfis', 'file') == 2
        fuzzy.fis = build_fuzzy_weight_fis(fuzzy.theta_breaks_deg, fuzzy.rate_break_deg_s);
        fuzzy.fis_lookup = build_fuzzy_lookup(fuzzy.fis, fuzzy.theta_breaks_deg, ...
                                              fuzzy.rate_break_deg_s);
        fuzzy.use_fis = true;
        info.note = 'Takagi-Sugeno fuzzy scheduler evaluated through Fuzzy Logic Toolbox.';
    else
        fuzzy.use_fis = false;
        info.note = 'Manual Takagi-Sugeno fuzzy scheduler; Fuzzy Logic Toolbox unavailable.';
    end
    info.candidates = rows;
end

function fis = build_fuzzy_weight_fis(theta_breaks_deg, rate_break_deg_s)
    b = theta_breaks_deg;
    fis = sugfis('Name', 'seesaw_gain_scheduler');
    fis = addInput(fis, [0 max(2 * b(3), b(3) + 1)], 'Name', 'theta_abs_deg');
    fis = addInput(fis, [0 max(2 * rate_break_deg_s, rate_break_deg_s + 1)], ...
                   'Name', 'theta_dot_abs_deg_s');
    fis = addInput(fis, [0 1], 'Name', 'moving_away');

    fis = addMF(fis, 'theta_abs_deg', 'trapmf', [0 0 b(1) b(2)], 'Name', 'small');
    fis = addMF(fis, 'theta_abs_deg', 'trimf', [b(1) b(2) b(3)], 'Name', 'mid');
    fis = addMF(fis, 'theta_abs_deg', 'trapmf', [b(2) b(3) max(2*b(3), b(3)+1) max(2*b(3), b(3)+1)], 'Name', 'large');
    fis = addMF(fis, 'theta_dot_abs_deg_s', 'trapmf', [0 0 0.4 1] * rate_break_deg_s, 'Name', 'slow');
    fis = addMF(fis, 'theta_dot_abs_deg_s', 'trapmf', [0.4 1 2 2] * rate_break_deg_s, 'Name', 'fast');
    fis = addMF(fis, 'moving_away', 'trapmf', [0 0 0.25 0.75], 'Name', 'no');
    fis = addMF(fis, 'moving_away', 'trapmf', [0.25 0.75 1 1], 'Name', 'yes');

    fis = addOutput(fis, [1 3], 'Name', 'gain_index');
    fis = addMF(fis, 'gain_index', 'constant', 1, 'Name', 'small_gain');
    fis = addMF(fis, 'gain_index', 'constant', 2, 'Name', 'mid_gain');
    fis = addMF(fis, 'gain_index', 'constant', 3, 'Name', 'large_gain');

    fis = addRule(fis, [1 0 0 1 1 1]);
    fis = addRule(fis, [2 0 0 2 1 1]);
    fis = addRule(fis, [3 0 0 3 1 1]);
    fis = addRule(fis, [2 2 2 3 1 1]);
    fis = addRule(fis, [1 2 2 2 1 1]);
end

function lookup = build_fuzzy_lookup(fis, theta_breaks_deg, rate_break_deg_s)
    theta_max = max(2 * theta_breaks_deg(3), theta_breaks_deg(3) + 1);
    rate_max = max(2 * rate_break_deg_s, rate_break_deg_s + 1);
    lookup.theta_axis = linspace(0, theta_max, 81);
    lookup.rate_axis = linspace(0, rate_max, 81);
    [theta_grid, rate_grid] = meshgrid(lookup.theta_axis, lookup.rate_axis);

    n = numel(theta_grid);
    near = [theta_grid(:), rate_grid(:), zeros(n, 1)];
    away = [theta_grid(:), rate_grid(:), ones(n, 1)];
    lookup.idx_near = reshape(evalfis(fis, near), size(theta_grid));
    lookup.idx_away = reshape(evalfis(fis, away), size(theta_grid));
end

function smc = load_or_design_smc(root, A, B, constraints)
    smc_file = fullfile(root, 'data', 'controller_smc.mat');
    if exist(smc_file, 'file') == 2
        d = load(smc_file);
        smc.S = d.S;
        smc.K_eq = d.K_eq;
        smc.k1 = d.k1;
        smc.k2 = d.k2;
        smc.phi_bl = d.phi_bl;
        smc.source = 'data/controller_smc.mat';
        return
    end

    fprintf('  data/controller_smc.mat not found; rebuilding SMC parameters in memory.\n')
    n = size(A, 1);
    N = null(B');
    Tr = [N'; (B / norm(B))'];
    Abar = Tr * A * Tr';
    A11 = Abar(1:n-1, 1:n-1);
    A12 = Abar(1:n-1, n);
    sigma_s = 4.0;
    zeta_s = 0.80;
    wn_s = sigma_s / zeta_s;
    p_slide = [-sigma_s + 1j * wn_s * sqrt(1 - zeta_s^2);
               -sigma_s - 1j * wn_s * sqrt(1 - zeta_s^2);
               -1.6 * sigma_s];
    F = place(A11, A12, p_slide);
    S = [F, 1] * Tr;
    SB = S * B;
    L_dist = 18;
    smc.S = S;
    smc.K_eq = SB \ (S * A);
    smc.k1 = 1.5 * sqrt(L_dist);
    smc.k2 = 1.1 * L_dist;
    ds_quant = abs(S(1)) * constraints.q_xc + abs(S(3)) * constraints.q_theta;
    smc.phi_bl = 2.5 * ds_quant;
    smc.source = 'rebuilt in robust_controller_comparison.m';
end

function [best, info] = tune_scheduled_smc(plant, constraints, scenario, smc_base)
    small_gain_sets = [0.25 0.40];
    mid_gain_sets = [0.55 0.75];
    theta_soft_sets = [0.30 0.50];
    theta_full_sets = [1.0 1.6];
    slew_penalty_sets = [0.02 0.06];

    best_score = inf;
    best = struct();
    rows = [];
    for gs = small_gain_sets
        for gm = mid_gain_sets
            for ths = theta_soft_sets
                for thf = theta_full_sets
                    if thf <= ths
                        continue
                    end
                    for slew_pen = slew_penalty_sets
                        data = smc_base;
                        data.small_gain = gs;
                        data.mid_gain = gm;
                        data.theta_soft_deg = ths;
                        data.theta_full_deg = thf;
                        data.slew_penalty = slew_pen;
                        c = smc_scheduled_controller('candidate', data, '');
                        sim = simulate_controller(plant.A, plant.B, c, constraints, scenario);
                        score = hardware_friendly_score(sim.metrics);
                        rows = [rows; gs, gm, ths, thf, slew_pen, score, ...
                                sim.metrics.theta_p2p_deg, sim.metrics.u_rms, ...
                                sim.metrics.u_slew_peak, sim.metrics.u_peak]; %#ok<AGROW>
                        if score < best_score
                            best_score = score;
                            best = data;
                        end
                    end
                end
            end
        end
    end

    if isempty(fieldnames(best))
        best = smc_base;
        best.small_gain = 1;
        best.mid_gain = 1;
        best.theta_soft_deg = 0;
        best.theta_full_deg = 0;
        best.slew_penalty = 0;
        info.note = 'Fallback: scheduled SMC tuning failed; base SMC used.';
    else
        info.note = 'Boundary-layer SMC with theta-dependent reaching gains and slew damping.';
    end
    info.candidates = rows;
end

function [best, info] = tune_augmented_smc(plant, constraints, scenario)
    % Super-twisting SMC on the lifted plant [x; u] with u_dot (V/s) as the
    % virtual control input.  With B_aug = [0;0;0;0;1] the regular-form
    % transform is identity, so the 4-state sliding subsystem is just the
    % original plant and the surface is s = F*x + u with F placing
    % eig(A - B*F).  Integrating the SMC output directly bounds the
    % commanded voltage slew rate, which is the metric we want to optimise.
    A_aug = [plant.A, plant.B; zeros(1, 4), 0];

    sigma_set = [3.0 4.0 5.0 6.0];
    zeta_set = [0.7 0.8 0.9];
    L_set = [10 18 30 50];
    phi_scale_set = [1.5 2.5 4.0 6.0];

    best_score = inf;
    best = struct();
    rows = [];
    for sigma_s = sigma_set
        for zeta_s = zeta_set
            wn_s = sigma_s / zeta_s;
            p_slide = [-sigma_s + 1j * wn_s * sqrt(1 - zeta_s^2);
                       -sigma_s - 1j * wn_s * sqrt(1 - zeta_s^2);
                       -1.6 * sigma_s;
                       -1.2 * sigma_s];
            try
                F = place(plant.A, plant.B, p_slide);
            catch
                continue
            end
            S = [F, 1];                    % already in regular form
            K_eq = S * A_aug;              % S*B_aug = S(5) = 1
            ds_quant = abs(S(1)) * constraints.q_xc + ...
                       abs(S(3)) * constraints.q_theta;
            for L = L_set
                for phi_scale = phi_scale_set
                    data = struct();
                    data.S = S;
                    data.K_eq = K_eq;
                    data.k1 = 1.5 * sqrt(L);
                    data.k2 = 1.1 * L;
                    data.phi_bl = phi_scale * ds_quant;
                    data.sigma_s = sigma_s;
                    data.zeta_s = zeta_s;
                    data.L_dist = L;
                    c = smc_augmented_controller('candidate', data, '');
                    sim = simulate_controller(plant.A, plant.B, c, constraints, scenario);
                    score = hardware_friendly_score(sim.metrics);
                    rows = [rows; sigma_s, zeta_s, L, phi_scale, score, ...
                            sim.metrics.theta_p2p_deg, sim.metrics.u_rms, ...
                            sim.metrics.u_slew_peak, sim.metrics.u_peak]; %#ok<AGROW>
                    if score < best_score
                        best_score = score;
                        best = data;
                    end
                end
            end
        end
    end

    if isempty(fieldnames(best))
        warning('Augmented SMC tuning failed; using nominal fixed design.')
        sigma_s = 4.0; zeta_s = 0.80;
        wn_s = sigma_s / zeta_s;
        p_slide = [-sigma_s + 1j * wn_s * sqrt(1 - zeta_s^2);
                   -sigma_s - 1j * wn_s * sqrt(1 - zeta_s^2);
                   -1.6 * sigma_s; -1.2 * sigma_s];
        F = place(plant.A, plant.B, p_slide);
        best.S = [F, 1];
        best.K_eq = best.S * A_aug;
        L = 18;
        best.k1 = 1.5 * sqrt(L);
        best.k2 = 1.1 * L;
        ds_quant = abs(best.S(1)) * constraints.q_xc + ...
                   abs(best.S(3)) * constraints.q_theta;
        best.phi_bl = 2.5 * ds_quant;
        best.sigma_s = sigma_s;
        best.zeta_s = zeta_s;
        best.L_dist = L;
        info.note = 'Fallback: augmented SMC sweep failed; nominal fixed design used.';
    else
        info.note = sprintf(['Augmented STA SMC over [x; u]; voltage slew is the ' ...
            'virtual control. sigma=%.1f, zeta=%.2f, L=%.0f.'], ...
            best.sigma_s, best.zeta_s, best.L_dist);
    end
    info.candidates = rows;
end

function result = simulate_controller(A, B, controller, constraints, scenario)
    Ts = scenario.Ts;
    N = round(scenario.T / Ts) + 1;
    t = (0:N-1)' * Ts;
    x = scenario.x0(:);
    X = zeros(N, 4);
    U = zeros(N, 1);
    Ueff = zeros(N, 1);
    smc_v = 0;
    u_act_state = 0;
    xk = controller_initial_state(controller);
    u_prev = 0;
    rail_hit = false;
    theta_hit = false;
    saturated = false(N, 1);
    rate_limited = false(N, 1);

    for k = 1:N
        x_ctrl = quantized_state(x, constraints);
        [u_raw, smc_v, u_act_state, xk] = controller_voltage(controller, x_ctrl, ...
            smc_v, u_act_state, xk, Ts, A, B, constraints);
        u_sat = clamp(u_raw, -constraints.V_sat, constraints.V_sat);
        [u, hit_rate] = apply_rate_limit(u_sat, u_prev, constraints.u_rate_limit, Ts);
        u_prev = u;
        saturated(k) = abs(u_raw) > constraints.V_sat + 1e-9;
        rate_limited(k) = hit_rate;
        u_eff = actuator_path(u, constraints);

        X(k,:) = x';
        U(k) = u;
        Ueff(k) = u_eff;

        dx = A * x + B * u_eff + coulomb_disturbance(x, constraints);
        x = x + Ts * dx;
        [x, hit_rail, hit_theta] = enforce_stops(x, constraints);
        rail_hit = rail_hit || hit_rail;
        theta_hit = theta_hit || hit_theta;
    end

    metrics = compute_metrics(t, X, U, Ueff, saturated, rate_limited, rail_hit, theta_hit, ...
                              constraints, scenario.trim_time);
    result.t = t;
    result.x = X;
    result.u = U;
    result.u_eff = Ueff;
    result.metrics = metrics;
end

function xk = controller_initial_state(controller)
    if strcmp(controller.type, 'hinf_dynamic') && isfield(controller.data, 'xk0')
        xk = controller.data.xk0;
    else
        xk = [];
    end
end

function [u, limited] = apply_rate_limit(u_target, u_prev, rate_limit, Ts)
    if isinf(rate_limit)
        u = u_target;
        limited = false;
        return
    end
    du_max = rate_limit * Ts;
    u = u_prev + clamp(u_target - u_prev, -du_max, du_max);
    limited = abs(u - u_target) > 1e-10;
end

function xq = quantized_state(x, constraints)
    xq = x;
    xq(1) = round(x(1) / constraints.q_xc) * constraints.q_xc;
    xq(3) = round(x(3) / constraints.q_theta) * constraints.q_theta;
end

function [u, smc_v, u_act_state, xk] = controller_voltage(controller, x, smc_v, ...
                                                           u_act_state, xk, Ts, ~, B, constraints)
    switch controller.type
        case 'gain'
            u = -controller.K * x;
        case 'actuator_aware'
            d = controller.data;
            u_cmd = -d.K_aug * [x; u_act_state];
            u_cmd = clamp(u_cmd, -constraints.V_sat, constraints.V_sat);
            u_act_state = u_act_state + Ts * (u_cmd - u_act_state) / d.tau_act;
            u = u_act_state;
        case 'hinf_dynamic'
            d = controller.data;
            if isempty(xk)
                xk = d.xk0;
            end
            y = x;
            u_core = d.output_scale * (d.C * xk + d.D * y);
            xk_dot = d.A * xk + d.B * y - d.state_leak * xk;
            xk = xk + Ts * xk_dot;
            if d.output_tau > 0
                u_core = clamp(u_core, -constraints.V_sat, constraints.V_sat);
                u_act_state = u_act_state + Ts * (u_core - u_act_state) / d.output_tau;
                u = u_act_state;
            else
                u = u_core;
            end
        case 'fuzzy'
            K = fuzzy_gain(controller.data, x);
            u = -K * x;
        case 'smc'
            d = controller.data;
            S = d.S;
            % Match the pulled SMC benchmark: use the weaker negative-path
            % actuator gain in S*B so the reaching term never under-actuates
            % under the asymmetric drive model.
            SB = S * B * constraints.actuator_gain_neg;
            s = S * x;
            sig = clamp(s / max(d.phi_bl, eps), -1, 1);
            smc_v = smc_v - d.k2 * sig * Ts;
            smc_v = clamp(smc_v, -abs(SB) * constraints.V_sat, abs(SB) * constraints.V_sat);
            u = -d.K_eq * x + (-d.k1 * sqrt(abs(s)) * sig + smc_v) / SB;
        case 'smc_scheduled'
            d = controller.data;
            S = d.S;
            SB = S * B * constraints.actuator_gain_neg;
            s = S * x;
            sig = clamp(s / max(d.phi_bl, eps), -1, 1);
            th = abs(rad2deg(x(3)));
            moving_away = x(3) * x(4) > 0;
            if th <= d.theta_soft_deg
                gain_scale = d.small_gain;
            elseif th >= d.theta_full_deg || moving_away
                gain_scale = 1;
            else
                blend = (th - d.theta_soft_deg) / (d.theta_full_deg - d.theta_soft_deg);
                gain_scale = d.mid_gain + blend * (1 - d.mid_gain);
            end
            k1 = d.k1 * gain_scale;
            k2 = d.k2 * gain_scale;
            smc_v = smc_v - k2 * sig * Ts;
            smc_v = clamp(smc_v, -abs(SB) * constraints.V_sat, abs(SB) * constraints.V_sat);
            u_unsmoothed = -d.K_eq * x + (-k1 * sqrt(abs(s)) * sig + smc_v) / SB;
            u = (1 - d.slew_penalty) * u_unsmoothed + d.slew_penalty * u_act_state;
            u_act_state = u;
        case 'smc_augmented'
            % Augmented state x_aug = [x; u_prev].  The plant is lifted to
            %   x_aug_dot = [A B; 0 0] x_aug + [0;0;0;0;1] * u_dot,
            % so u_dot (V/s) is the SMC virtual control.  With B_aug having
            % only the last entry non-zero, the regular-form transform is
            % the identity: sliding subsystem dynamics are A - B*F and the
            % surface is s = F*x + u.  Integrating u_dot inherently bounds
            % the commanded voltage slew rate.
            d = controller.data;
            S = d.S;
            x_aug = [x; u_act_state];
            SB = S(5);                              % S * B_aug
            s = S * x_aug;
            sig = clamp(s / max(d.phi_bl, eps), -1, 1);
            smc_v = smc_v - d.k2 * sig * Ts;
            v_clip = 10 * d.L_dist;                 % loose bound on STA integrator
            smc_v = clamp(smc_v, -v_clip, v_clip);
            u_dot = -d.K_eq * x_aug + (-d.k1 * sqrt(abs(s)) * sig + smc_v) / SB;
            u_new = u_act_state + Ts * u_dot;
            u_act_state = clamp(u_new, -constraints.V_sat, constraints.V_sat);
            u = u_act_state;
        otherwise
            error('Unknown controller type: %s', controller.type)
    end
end

function K = fuzzy_gain(fuzzy, x)
    th = abs(rad2deg(x(3)));
    thd = abs(rad2deg(x(4)));
    moving_away = x(3) * x(4) > 0;

    if isfield(fuzzy, 'use_fis') && fuzzy.use_fis
        idx = fuzzy_lookup_index(fuzzy.fis_lookup, th, thd, moving_away);
        w_small = clamp(2 - idx, 0, 1);
        w_large = clamp(idx - 2, 0, 1);
        w_mid = 1 - w_small - w_large;
        w = [w_small, w_mid, w_large];
    else
        b = fuzzy.theta_breaks_deg;
        mu_small = clamp((b(2) - th) / max(b(2) - b(1), eps), 0, 1);
        mu_large = clamp((th - b(2)) / max(b(3) - b(2), eps), 0, 1);
        mu_mid = max(0, 1 - mu_small - mu_large);

        if moving_away
            rate_boost = clamp(thd / fuzzy.rate_break_deg_s, 0, 1);
            mu_large = mu_large + 0.35 * rate_boost;
            mu_small = mu_small * (1 - 0.25 * rate_boost);
        end

        w = [mu_small, mu_mid, mu_large];
    end
    w = w / max(sum(w), eps);
    K = w(1) * fuzzy.K_small + w(2) * fuzzy.K_mid + w(3) * fuzzy.K_large;
end

function idx = fuzzy_lookup_index(lookup, theta_abs_deg, theta_dot_abs_deg_s, moving_away)
    theta_abs_deg = clamp(theta_abs_deg, lookup.theta_axis(1), lookup.theta_axis(end));
    theta_dot_abs_deg_s = clamp(theta_dot_abs_deg_s, lookup.rate_axis(1), lookup.rate_axis(end));
    if moving_away
        table = lookup.idx_away;
    else
        table = lookup.idx_near;
    end
    idx = interp2(lookup.theta_axis, lookup.rate_axis, table, ...
                  theta_abs_deg, theta_dot_abs_deg_s, 'linear');
end

function u_eff = actuator_path(u, constraints)
    if abs(u) < constraints.deadzone
        u_eff = 0;
    elseif u > 0
        u_eff = u - constraints.deadzone;
    else
        u_eff = u + constraints.deadzone;
    end
    if u_eff >= 0
        u_eff = constraints.actuator_gain_pos * u_eff;
    else
        u_eff = constraints.actuator_gain_neg * u_eff;
    end
end

function d = coulomb_disturbance(x, constraints)
    if abs(x(2)) < 1e-9
        f = 0;
    else
        f = -constraints.coulomb_force * sign(x(2));
    end
    d = [0; f / constraints.M_c; 0; 0];
end

function [x, rail_hit, theta_hit] = enforce_stops(x, constraints)
    rail_hit = false;
    theta_hit = false;
    if x(1) > constraints.x_max
        x(1) = constraints.x_max;
        x(2) = min(0, x(2));
        rail_hit = true;
    elseif x(1) < -constraints.x_max
        x(1) = -constraints.x_max;
        x(2) = max(0, x(2));
        rail_hit = true;
    end
    if x(3) > constraints.theta_max
        x(3) = constraints.theta_max;
        x(4) = min(0, x(4));
        theta_hit = true;
    elseif x(3) < -constraints.theta_max
        x(3) = -constraints.theta_max;
        x(4) = max(0, x(4));
        theta_hit = true;
    end
end

function metrics = compute_metrics(t, X, U, Ueff, saturated, rate_limited, rail_hit, theta_hit, constraints, trim_time)
    win = t >= trim_time;
    if ~any(win), win = true(size(t)); end
    th = X(win, 3);
    xc = X(win, 1);
    uw = U(win);
    metrics.theta_p2p_deg = rad2deg(max(th) - min(th));
    metrics.theta_rms_deg = rad2deg(rms(th));
    metrics.theta_abs95_deg = rad2deg(prctile(abs(th), 95));
    metrics.cart_p2p_mm = (max(xc) - min(xc)) * 1000;
    metrics.u_rms = rms(uw);
    metrics.u_mean_abs = mean(abs(uw));
    metrics.u_peak = max(abs(U));
    metrics.u_eff_rms = rms(Ueff(win));
    metrics.u_tv_per_s = sum(abs(diff(U))) / max(t(end) - t(1), eps);
    metrics.u_slew_peak = max(abs(diff(U))) / mean(diff(t));
    metrics.saturation_pct = 100 * mean(saturated);
    metrics.rate_limited_pct = 100 * mean(rate_limited);
    metrics.rail_hit = rail_hit;
    metrics.theta_stop_hit = theta_hit;
    metrics.diverged = ~all(isfinite(X(:))) || max(abs(X(:,3))) >= constraints.theta_max - 1e-9;
    metrics.constraint_ok = metrics.u_peak <= constraints.V_sat + 1e-8 && ...
                            metrics.u_slew_peak <= constraints.u_rate_limit + 1e-8 && ...
                            ~rail_hit && ~theta_hit && all(isfinite(X(:)));
end

function score = priority_score(metrics)
    penalty = 0;
    if ~metrics.constraint_ok, penalty = penalty + 1e4; end
    score = penalty ...
          + 100 * metrics.theta_p2p_deg ...
          + 12 * metrics.u_rms ...
          + 4 * metrics.u_mean_abs ...
          + 0.25 * metrics.u_tv_per_s ...
          + 20 * metrics.saturation_pct;
end

function results = collect_results(controllers)
    n = numel(controllers);
    names = strings(n, 1);
    theta_p2p = zeros(n, 1);
    theta_rms = zeros(n, 1);
    theta_p95 = zeros(n, 1);
    cart_p2p = zeros(n, 1);
    u_rms = zeros(n, 1);
    u_abs = zeros(n, 1);
    u_peak = zeros(n, 1);
    u_tv = zeros(n, 1);
    u_slew_peak = zeros(n, 1);
    sat_pct = zeros(n, 1);
    rate_pct = zeros(n, 1);
    ok = false(n, 1);
    rail = false(n, 1);
    theta_stop = false(n, 1);

    for i = 1:n
        m = controllers(i).data.result.metrics;
        names(i) = string(controllers(i).name);
        theta_p2p(i) = m.theta_p2p_deg;
        theta_rms(i) = m.theta_rms_deg;
        theta_p95(i) = m.theta_abs95_deg;
        cart_p2p(i) = m.cart_p2p_mm;
        u_rms(i) = m.u_rms;
        u_abs(i) = m.u_mean_abs;
        u_peak(i) = m.u_peak;
        u_tv(i) = m.u_tv_per_s;
        u_slew_peak(i) = m.u_slew_peak;
        sat_pct(i) = m.saturation_pct;
        rate_pct(i) = m.rate_limited_pct;
        ok(i) = m.constraint_ok;
        rail(i) = m.rail_hit;
        theta_stop(i) = m.theta_stop_hit;
    end

    results = table(names, ok, theta_p2p, theta_rms, theta_p95, cart_p2p, ...
                    u_rms, u_abs, u_peak, u_tv, u_slew_peak, sat_pct, ...
                    rate_pct, rail, theta_stop, ...
        'VariableNames', {'name', 'constraint_ok', 'theta_p2p_deg', ...
        'theta_rms_deg', 'theta_abs95_deg', 'cart_p2p_mm', 'u_rms', ...
        'u_mean_abs', 'u_peak', 'u_tv_per_s', 'u_slew_peak', ...
        'saturation_pct', 'rate_limited_pct', 'rail_hit', 'theta_stop_hit'});
end

function print_results_table(results)
    fprintf('%-22s %3s %10s %9s %9s %9s %9s %9s %9s %8s\n', ...
        'Controller', 'OK', 'th_p2p', 'th_rms', 'cart_pp', 'u_rms', 'u_abs', ...
        'u_peak', 'slew_pk', 'sat%')
    for i = 1:height(results)
        fprintf('%-22s %3s %10.3f %9.3f %9.2f %9.3f %9.3f %9.3f %9.1f %8.2f\n', ...
            char(results.name(i)), yesno_short(results.constraint_ok(i)), ...
            results.theta_p2p_deg(i), results.theta_rms_deg(i), ...
            results.cart_p2p_mm(i), results.u_rms(i), results.u_mean_abs(i), ...
            results.u_peak(i), results.u_slew_peak(i), results.saturation_pct(i))
    end
end

function rate_results = run_rate_limit_sweep(A, B, controllers, constraints, scenario, rate_limits)
    rows = strings(numel(rate_limits) * numel(controllers), 1);
    rate_col = zeros(numel(rows), 1);
    ok_col = false(numel(rows), 1);
    th_col = zeros(numel(rows), 1);
    rms_col = zeros(numel(rows), 1);
    peak_col = zeros(numel(rows), 1);
    slew_col = zeros(numel(rows), 1);
    sat_col = zeros(numel(rows), 1);
    rate_pct_col = zeros(numel(rows), 1);
    rail_col = false(numel(rows), 1);
    theta_col = false(numel(rows), 1);
    idx = 0;

    for ir = 1:numel(rate_limits)
        c_lim = constraints;
        c_lim.u_rate_limit = rate_limits(ir);
        for ic = 1:numel(controllers)
            idx = idx + 1;
            sim = simulate_controller(A, B, controllers(ic), c_lim, scenario);
            m = sim.metrics;
            rows(idx) = string(controllers(ic).name);
            rate_col(idx) = rate_limits(ir);
            ok_col(idx) = m.constraint_ok;
            th_col(idx) = m.theta_p2p_deg;
            rms_col(idx) = m.u_rms;
            peak_col(idx) = m.u_peak;
            slew_col(idx) = m.u_slew_peak;
            sat_col(idx) = m.saturation_pct;
            rate_pct_col(idx) = m.rate_limited_pct;
            rail_col(idx) = m.rail_hit;
            theta_col(idx) = m.theta_stop_hit;
        end
    end

    rate_results = table(rows, rate_col, ok_col, th_col, rms_col, peak_col, ...
                         slew_col, sat_col, rate_pct_col, rail_col, theta_col, ...
        'VariableNames', {'name', 'rate_limit_v_s', 'constraint_ok', ...
        'theta_p2p_deg', 'u_rms', 'u_peak', 'u_slew_peak', ...
        'saturation_pct', 'rate_limited_pct', 'rail_hit', 'theta_stop_hit'});
end

function print_rate_limit_summary(rate_results, rate_limits)
    names = unique(rate_results.name, 'stable');
    fprintf('\n=== Slew-Rate Survival Summary ===\n')
    fprintf('%-22s %14s %14s %12s %10s\n', ...
        'Controller', 'lowest OK V/s', 'theta p2p', 'u RMS', 'notes')
    for i = 1:numel(names)
        rows = rate_results(rate_results.name == names(i), :);
        finite_ok = rows.constraint_ok & isfinite(rows.rate_limit_v_s);
        if any(finite_ok)
            [min_rate, idx_rel] = min(rows.rate_limit_v_s(finite_ok));
            ok_rows = rows(finite_ok, :);
            r = ok_rows(idx_rel, :);
            fprintf('%-22s %14.1f %11.3f deg %10.3f %10s\n', ...
                char(names(i)), min_rate, r.theta_p2p_deg, r.u_rms, 'survives')
        else
            base = rows(isinf(rows.rate_limit_v_s), :);
            if ~isempty(base) && base.constraint_ok(1)
                note = 'only Inf';
            else
                note = 'fails';
            end
            fprintf('%-22s %14s %14s %12s %10s\n', ...
                char(names(i)), '-', '-', '-', note)
        end
    end

    fprintf('\nBest controller at each finite slew limit:\n')
    fprintf('%10s %-22s %12s %10s\n', 'V/s', 'best OK controller', 'theta p2p', 'u RMS')
    for i = 1:numel(rate_limits)
        if isinf(rate_limits(i)), continue, end
        rows = rate_results(rate_results.rate_limit_v_s == rate_limits(i) & rate_results.constraint_ok, :);
        if isempty(rows)
            fprintf('%10.1f %-22s %12s %10s\n', rate_limits(i), 'none', '-', '-')
            continue
        end
        rows = sortrows(rows, {'theta_p2p_deg', 'u_rms'}, {'ascend', 'ascend'});
        fprintf('%10.1f %-22s %9.3f deg %10.3f\n', ...
            rate_limits(i), char(rows.name(1)), rows.theta_p2p_deg(1), rows.u_rms(1))
    end
end

function s = yesno_short(tf)
    if tf, s = 'Y'; else, s = 'N'; end
end

function plot_results(controllers, results, scenario, constraints, figdir)
    order = zeros(height(results), 1);
    for i = 1:height(results)
        order(i) = find(strcmp({controllers.name}, char(results.name(i))), 1);
    end

    t = controllers(order(1)).data.result.t;
    colors = lines(numel(order));

    f1 = figure('Name', 'Robust controller comparison', ...
        'Position', [80 80 1000 780], 'Color', 'w');
    subplot(3,1,1); hold on; grid on; box on
    for j = 1:numel(order)
        r = controllers(order(j)).data.result;
        plot(r.t, rad2deg(r.x(:,3)), 'LineWidth', 1.2, 'Color', colors(j,:))
    end
    yline(rad2deg(constraints.theta_max), 'k--')
    yline(-rad2deg(constraints.theta_max), 'k--')
    ylabel('\theta [deg]')
    title('Rocking Response -- Robust Controller Comparison')
    legend(cellstr(results.name), 'Location', 'best')

    subplot(3,1,2); hold on; grid on; box on
    for j = 1:numel(order)
        r = controllers(order(j)).data.result;
        plot(r.t, r.x(:,1) * 1000, 'LineWidth', 1.2, 'Color', colors(j,:))
    end
    yline(constraints.x_max * 1000, 'k--')
    yline(-constraints.x_max * 1000, 'k--')
    ylabel('x_c [mm]')

    subplot(3,1,3); hold on; grid on; box on
    for j = 1:numel(order)
        r = controllers(order(j)).data.result;
        plot(r.t, r.u, 'LineWidth', 1.1, 'Color', colors(j,:))
    end
    yline(constraints.V_sat, 'k--')
    yline(-constraints.V_sat, 'k--')
    yline(constraints.deadzone, 'k:')
    yline(-constraints.deadzone, 'k:')
    ylabel('V_m [V]'); xlabel('Time [s]')
    saveas(f1, fullfile(figdir, 'Robust-Control-Comparison.png'))

    z0 = max(0, scenario.T - 5);
    f2 = figure('Name', 'Robust controller steady state', ...
        'Position', [100 100 1000 600], 'Color', 'w');
    subplot(2,1,1); hold on; grid on; box on
    for j = 1:numel(order)
        r = controllers(order(j)).data.result;
        plot(r.t, rad2deg(r.x(:,3)), 'LineWidth', 1.3, 'Color', colors(j,:))
    end
    xlim([z0 t(end)])
    ylabel('\theta [deg]')
    title('Steady-State Rocking Window')
    legend(cellstr(results.name), 'Location', 'best')

    subplot(2,1,2); hold on; grid on; box on
    for j = 1:numel(order)
        r = controllers(order(j)).data.result;
        plot(r.t, r.u, 'LineWidth', 1.2, 'Color', colors(j,:))
    end
    xlim([z0 t(end)])
    yline(constraints.V_sat, 'k--')
    yline(-constraints.V_sat, 'k--')
    ylabel('V_m [V]'); xlabel('Time [s]')
    saveas(f2, fullfile(figdir, 'Robust-Control-SteadyState.png'))

    f3 = figure('Name', 'Robust controller theta phase plane', ...
        'Position', [60 60 1400 850], 'Color', 'w');
    tile_cols = 3;
    tile_rows = ceil((numel(order) + 1) / tile_cols);
    tiledlayout(tile_rows, tile_cols, 'TileSpacing', 'compact', 'Padding', 'compact')
    ss_theta = [];
    ss_theta_dot = [];
    for j = 1:numel(order)
        r = controllers(order(j)).data.result;
        win = r.t >= z0;
        ss_theta = [ss_theta; rad2deg(r.x(win,3))]; %#ok<AGROW>
        ss_theta_dot = [ss_theta_dot; rad2deg(r.x(win,4))]; %#ok<AGROW>
    end
    x_lim = padded_limits(ss_theta, 0.15, 0.15);
    y_lim = padded_limits(ss_theta_dot, 0.15, 0.15);

    for j = 1:numel(order)
        r = controllers(order(j)).data.result;
        win = r.t >= z0;
        m = controllers(order(j)).data.result.metrics;
        nexttile; hold on; grid on; box on
        plot(rad2deg(r.x(win,3)), rad2deg(r.x(win,4)), ...
            'LineWidth', 1.5, 'Color', colors(j,:))
        plot(rad2deg(r.x(find(win, 1, 'first'),3)), ...
             rad2deg(r.x(find(win, 1, 'first'),4)), 'o', ...
             'MarkerFaceColor', colors(j,:), 'MarkerEdgeColor', colors(j,:), ...
             'MarkerSize', 4)
        xline(0, 'k:')
        yline(0, 'k:')
        xlim(x_lim)
        ylim(y_lim)
        xlabel('theta [deg]')
        ylabel('theta_dot [deg/s]')
        title(sprintf('%s: %.3f deg p2p', controllers(order(j)).name, ...
            m.theta_p2p_deg), 'Interpreter', 'none')
    end
    nexttile; hold on; grid on; box on
    for j = 1:numel(order)
        r = controllers(order(j)).data.result;
        win = r.t >= z0;
        plot(rad2deg(r.x(win,3)), rad2deg(r.x(win,4)), ...
            'LineWidth', 1.2, 'Color', colors(j,:))
    end
    xline(0, 'k:')
    yline(0, 'k:')
    xlim(x_lim)
    ylim(y_lim)
    xlabel('theta [deg]')
    ylabel('theta_dot [deg/s]')
    title('Overlay, same zoom')
    legend(cellstr(results.name), 'Location', 'best', 'Interpreter', 'none')
    saveas(f3, fullfile(figdir, 'Robust-Control-ThetaPhasePlane.png'))

    fprintf(['Saved Robust-Control-Comparison.png, Robust-Control-SteadyState.png, ' ...
             'and Robust-Control-ThetaPhasePlane.png\n'])
end

function plot_rate_limit_results(rate_results, figdir)
    finite_rows = rate_results(isfinite(rate_results.rate_limit_v_s), :);
    names = unique(finite_rows.name, 'stable');
    colors = lines(numel(names));

    f = figure('Name', 'Robust controller rate-limit survival', ...
        'Position', [100 100 1050 680], 'Color', 'w');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact')

    nexttile; hold on; grid on; box on
    for i = 1:numel(names)
        rows = finite_rows(finite_rows.name == names(i), :);
        [rate, ord] = sort(rows.rate_limit_v_s, 'ascend');
        y = rows.theta_p2p_deg(ord);
        y(~rows.constraint_ok(ord)) = NaN;
        plot(rate, y, '-o', 'LineWidth', 1.3, 'MarkerSize', 4, 'Color', colors(i,:))
    end
    set(gca, 'XScale', 'log')
    ylabel('theta p2p [deg]')
    title('Surviving Controllers Under Voltage Slew-Rate Limit')
    legend(cellstr(names), 'Location', 'best', 'Interpreter', 'none')

    nexttile; hold on; grid on; box on
    for i = 1:numel(names)
        rows = finite_rows(finite_rows.name == names(i), :);
        [rate, ord] = sort(rows.rate_limit_v_s, 'ascend');
        y = rows.u_rms(ord);
        y(~rows.constraint_ok(ord)) = NaN;
        plot(rate, y, '-o', 'LineWidth', 1.3, 'MarkerSize', 4, 'Color', colors(i,:))
    end
    set(gca, 'XScale', 'log')
    xlabel('voltage slew-rate limit [V/s]')
    ylabel('u RMS [V]')
    saveas(f, fullfile(figdir, 'Robust-Control-RateLimitSurvival.png'))
    fprintf('Saved Robust-Control-RateLimitSurvival.png\n')
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function lim = padded_limits(x, frac, min_half_width)
    lo = min(x);
    hi = max(x);
    mid = (lo + hi) / 2;
    half_width = max((hi - lo) * (0.5 + frac), min_half_width);
    lim = [mid - half_width, mid + half_width];
end
