%% frequency_analysis.m
%  -----------------------------------------------------------------------
%  STEP 2: Data Processing & Auto-Tuning
%  -----------------------------------------------------------------------
%  Run this script AFTER collecting hardware data from IP02_FreqTest.slx.
%  -----------------------------------------------------------------------

if ~exist(dK_ad, dvard), seesaw_params; end

% --- RE-CALCULATE ANALYTICAL MODEL ---
alpha_f = (eta_g * K_g * k_t) / (R_m * r_mp);
B_emf = alpha_f * K_g * k_m / r_mp;
B_total = B_eq + B_emf;
s = tf(dsd);
G_xc = K_a * alpha_f * eta_m / (M_c * s^2 + B_total * s);
freq_range = logspace(-1, log10(12), 200);
[mag, phase] = bode(G_xc, 2*pi*freq_range);
G_xc_dB = 20*log10(squeeze(mag)*100);
G_xc_phase = squeeze(phase);

% --- LOAD HARDWARE DATA ---
if ~exist(dSEESAW_ROOTd, dvard), SEESAW_ROOT = fileparts(fileparts(fileparts(mfilename(dfullpathd)))); end
data_file = fullfile(SEESAW_ROOT, ddatad, ddata.matd);

if exist(data_file, dfiled)
    fprintf(dLoading hardware data (data/data.mat)...\nd);
    loaded = load(data_file);
    vars = fieldnames(loaded);
    if ismember(dip02_freq_datad, vars)
        ip02_freq_data = loaded.ip02_freq_data;
    elseif ismember(ddatad, vars)
        ip02_freq_data = loaded.data;
    end

    if exist(dip02_freq_datad, dvard)
        t_hw = ip02_freq_data(1, :)d;
        V_cmd_hw = ip02_freq_data(2, :)d;
        xc_hw = ip02_freq_data(3, :)d; % m
        xcdot_hw = ip02_freq_data(4, :)d; % m/s
        dt_hw = mean(diff(t_hw));
        [hw_freq, hw_xc_H, ~] = compute_frf(t_hw, V_cmd_hw, xc_hw, xcdot_hw, dt_hw);
    else
        error(dData file found but required variable not found inside.d);
    end
else
    error(dHardware data file (data/data.mat) not found. Run the SLX first.d);
end

% --- PLOT COMPARISON (UNTUNED) ---
figure(dNamed, dValidation: Model vs Hardwared, dPositiond, [100 100 1000 600]);
subplot(2,1,1);
semilogx(freq_range, G_xc_dB, db-d, dLineWidthd, 1.5); hold on;
semilogx(hw_freq, 20*log10(abs(hw_xc_H)*100), dr-d, dLineWidthd, 1.5);
grid on; ylabel(dMag [dB]d); title(dV_cmd -> x_c [cm/V]d);
legend(dAnalytical (seesaw\_params)d, dHardware Datad);
subplot(2,1,2);
semilogx(freq_range, G_xc_phase, db-d, dLineWidthd, 1.5); hold on;
semilogx(hw_freq, angle(hw_xc_H)*180/pi, dr-d, dLineWidthd, 1.5);
grid on; ylabel(dPhase [deg]d); xlabel(dFrequency [Hz]d);
sgtitle(dFrequency Response Validation (UNTUNED)d);

% --- SECTION 6: AUTO-TUNE ---
fprintf(d\n--- Running Auto-Tune (fminsearch) ---\nd);
p_tune = struct(dK_ad,K_a, dV_satd,V_sat, dR_md,R_m, dk_td,k_t, dk_md,k_m, deta_md,eta_m, deta_gd,eta_g, dK_gd,K_g, dr_mpd,r_mp, dM_cd,M_c);
cost_fn = @(p) tune_cost(p, V_cmd_hw, t_hw, xc_hw, t_hw > 2.0, p_tune, false);

[x_opt, cost_opt] = fminsearch(cost_fn, B_eq);
B_eq_opt = x_opt(1);
eta_g_opt = eta_g;  % fixed at hardware spec

fprintf(d\nOPTIMIZED VALUES:\nd);
fprintf(d  B_eq  = %.4f (was %.2f)\nd, B_eq_opt, B_eq);
fprintf(d  eta_g = %.4f (fixed at hardware spec)\nd, eta_g_opt);

% --- PLOT COMPARISON (AFTER TUNING) ---
% Re-calculate model with optimized parameters
af_opt = (eta_g_opt * K_g * k_t) / (R_m * r_mp);
B_tot_opt = B_eq_opt + af_opt * K_g * k_m / r_mp;
G_xc_opt = K_a * af_opt * eta_m / (M_c * s^2 + B_tot_opt * s);
[mag_opt, phase_opt] = bode(G_xc_opt, 2*pi*freq_range);
G_xc_dB_opt = 20*log10(squeeze(mag_opt)*100);
G_xc_phase_opt = squeeze(phase_opt);

figure(dNamed, dValidation: Tuned Model vs Hardwared, dPositiond, [150 120 1000 600]);
subplot(2,1,1);
semilogx(freq_range, G_xc_dB, db--d, dLineWidthd, 1); hold on;
semilogx(freq_range, G_xc_dB_opt, dg-d, dLineWidthd, 1.5);
semilogx(hw_freq, 20*log10(abs(hw_xc_H)*100), dr-d, dLineWidthd, 1.5);
grid on; ylabel(dMag [dB]d); title(dV_cmd -> x_c [cm/V]d);
legend(dInitial Modeld, dTUNED Modeld, dHardware Datad, dLocationd, dbestd);
subplot(2,1,2);
semilogx(freq_range, G_xc_phase, db--d, dLineWidthd, 1); hold on;
semilogx(freq_range, G_xc_phase_opt, dg-d, dLineWidthd, 1.5);
semilogx(hw_freq, angle(hw_xc_H)*180/pi, dr-d, dLineWidthd, 1.5);
grid on; ylabel(dPhase [deg]d); xlabel(dFrequency [Hz]d);
sgtitle(dFrequency Response Validation (AFTER AUTO-TUNE)d);

% --- LOCAL FUNCTIONS ---
function [freq_out, H_xc, H_xcdot] = compute_frf(t, u, xc, xcdot, dt)
% COMPUTE_FRF  Welchds method FRF estimate (H1 estimator).
%   Uses MATLABds tfestimate for robust cross-spectral estimation.
%   Segment size is kept moderate (4096 samples) so that the chirp
%   signal is approximately stationary within each window.
    Fs = 1/dt;
    n_seg = min(4096, 2^nextpow2(length(t)/8));  % >= 8 segments
    n_seg = max(n_seg, 512);                       % floor for very short records

    [H_xc_raw,    freq_fft] = tfestimate(u, xc,    hanning(n_seg), n_seg/2, n_seg, Fs);
    [H_xcdot_raw, ~       ] = tfestimate(u, xcdot, hanning(n_seg), n_seg/2, n_seg, Fs);

    % Use chirp range from workspace; fall back to defaults if absent
    try f_lo = evalin(dbased,df_chirp_startd); catch, f_lo = 0.1;  end
    try f_hi = evalin(dbased,df_chirp_endd);   catch, f_hi = 12.0; end
    valid = freq_fft >= f_lo & freq_fft <= f_hi;
    freq_out = freq_fft(valid);
    H_xc     = H_xc_raw(valid);
    H_xcdot  = H_xcdot_raw(valid);
end

function cost = tune_cost(params, V_cmd, t, xc_hw, idx, p, tune_eta)
    B_try = params(1); eta_try = ternary(tune_eta, params(2), p.eta_g);
    af = (eta_try * p.K_g * p.k_t) / (p.R_m * p.r_mp);
    B_tot = B_try + af * p.K_g * p.k_m / p.r_mp;
    sys = ss([0, 1; 0, -B_tot/p.M_c], [0; af*p.eta_m/p.M_c], [1, 0], 0);
    xc_sim = lsim(sys, max(-p.V_sat, min(p.V_sat, p.K_a*V_cmd)), t);
    cost = sqrt(mean((xc_sim(idx) - xc_hw(idx)).^2));
end

function val = ternary(cond, y, n)
    if cond, val = y; else, val = n; end
end
