function [H, coh, f_bin] = stepped_sine_frf(t, d, y, f_vec, t_block, settle_frac)
% STEPPED_SINE_FRF  Per-frequency FRF from stepped-sine hardware data.
%
%   [H, coh] = stepped_sine_frf(t, d, y, f_vec, t_block, settle_frac)
%
%   Computes the frequency response H(f_k) = Y(f_k)/D(f_k) at each
%   excitation frequency using a single-bin DFT over the steady-state
%   portion of each block.  This is the H1 estimator specialised for
%   discrete-frequency excitation and is immune to the spectral leakage
%   and non-stationarity issues that degrade chirp-based tfestimate
%   on a continuously rocking seesaw.
%
%   Coherence is estimated by splitting each block's analysis window
%   into sub-segments and computing the cross-spectral statistics.
%
%   Inputs:
%     t           — time vector [s], Nx1
%     d           — disturbance (excitation) signal, Nx1
%     y           — output signal (e.g. x_c, alpha, V_m), Nx1
%     f_vec       — excitation frequencies [Hz], as returned by stepped_sine
%     t_block     — duration per frequency block [s]
%     settle_frac — fraction skipped at start of each block
%
%   Outputs:
%     H      — complex FRF at each excitation frequency, n_freqs x 1
%     coh    — coherence (0–1) at each excitation frequency, n_freqs x 1
%     f_bin  — actual bin-centre frequency for each estimate (≈ f_vec)

    n_freqs = length(f_vec);
    H   = NaN(n_freqs, 1);
    coh = NaN(n_freqs, 1);
    f_bin = f_vec;

    dt = mean(diff(t));
    if abs(max(diff(t)) - min(diff(t))) > 1e-6 * dt
        warning('stepped_sine_frf:NonUniformTime', ...
            'Time vector is not uniformly sampled. Results may be degraded.');
    end

    n_per_block = round(t_block / dt);
    n_settle    = round(n_per_block * settle_frac);

    for k = 1:n_freqs
        i0 = (k-1) * n_per_block + n_settle + 1;
        i1 = k * n_per_block;

        if i0 > length(t) || i1 > length(t)
            warning('stepped_sine_frf:BlockOOB', ...
                'Block %d (%.3f Hz) extends past data.  Truncating.', k, f_vec(k));
            i1 = min(i1, length(t));
            if i0 >= i1, continue; end
        end

        t_k = t(i0:i1) - t(i0);
        d_k = d(i0:i1);
        y_k = y(i0:i1);

        Nk = length(t_k);
        if Nk < 4
            warning('stepped_sine_frf:TooFewSamples', ...
                'Block %d (%.3f Hz): only %d samples in analysis window.', k, f_vec(k), Nk);
            continue;
        end

        omega_k = 2*pi * f_vec(k);

        % Single-bin DFT at f_k on the full analysis window
        e = exp(-1j * omega_k * t_k);
        D_full = mean(d_k .* e);
        Y_full = mean(y_k .* e);

        H(k) = Y_full / D_full;

        % Coherence via sub-segment H1 estimator
        n_sub = min(3, floor(Nk / 8));
        n_sub = max(n_sub, 2);
        n_per_sub = floor(Nk / n_sub);

        D_sub = zeros(n_sub, 1);
        Y_sub = zeros(n_sub, 1);

        for s = 1:n_sub
            idx_s = (s-1)*n_per_sub + 1 : s*n_per_sub;
            t_s = t_k(idx_s) - t_k(idx_s(1));
            e_s = exp(-1j * omega_k * t_s);
            D_sub(s) = mean(d_k(idx_s) .* e_s);
            Y_sub(s) = mean(y_k(idx_s) .* e_s);
        end

        Sdd = mean(D_sub .* conj(D_sub));
        Sdy = mean(Y_sub .* conj(D_sub));
        Syy = mean(Y_sub .* conj(Y_sub));

        if Sdd > 0 && Syy > 0
            coh(k) = abs(Sdy)^2 / (Sdd * Syy);
            coh(k) = min(coh(k), 1);
        end
    end
end
