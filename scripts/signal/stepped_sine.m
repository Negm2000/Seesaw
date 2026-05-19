function [t, d, f_vec, info] = stepped_sine(f_start, f_end, n_freqs, A, dt, t_block, settle_frac)
% STEPPED_SINE  Generate stepped-sine excitation for Quanser hardware.
%
%   [t, d, f_vec, info] = stepped_sine(f_start, f_end, n_freqs, A, dt, t_block, settle_frac)
%
%   Produces a sequence of pure sine tones at logarithmically-spaced
%   frequencies.  Each tone is held at constant frequency for t_block
%   seconds.  The hardware sees clean constant-frequency blocks rather
%   than a continuously-sweeping chirp, which the VoltPAQ-X1 amplifier
%   and Faulhaber motor track far more smoothly.
%
%   The first settle_frac fraction of each block is reserved for
%   transient decay after the frequency switch and should be excluded
%   from FRF analysis.
%
%   Inputs (all optional — defaults are sensible for seesaw closed-loop):
%     f_start     — lowest frequency [Hz]           (default 0.1)
%     f_end       — highest frequency [Hz]          (default 10)
%     n_freqs     — number of discrete frequencies  (default 18)
%     A           — sine amplitude [V]              (default 0.5)
%     dt          — sample time [s]                 (default 0.002)
%     t_block     — duration per frequency [s]      (default 5)
%     settle_frac — fraction of block for settling  (default 0.2)
%
%   Outputs:
%     t      — time vector [s], Nx1
%     d      — disturbance / excitation signal, Nx1  (units of A)
%     f_vec  — excitation frequencies [Hz], n_freqs x 1 (log-spaced)
%     info   — struct with block boundaries and diagnostic fields:
%                .f_vec, .t_block, .settle_frac, .n_per_block,
%                .total_duration, .block_start_idx
%
%   For Simulink "From Workspace" block, use:  [t, d]  with sample time dt.

    if nargin < 1 || isempty(f_start),     f_start = 0.1;   end
    if nargin < 2 || isempty(f_end),       f_end   = 10;    end
    if nargin < 3 || isempty(n_freqs),     n_freqs = 18;    end
    if nargin < 4 || isempty(A),           A       = 0.5;   end
    if nargin < 5 || isempty(dt),          dt      = 0.002; end
    if nargin < 6 || isempty(t_block),     t_block = 5;     end
    if nargin < 7 || isempty(settle_frac), settle_frac = 0.2; end

    f_vec = logspace(log10(f_start), log10(f_end), n_freqs)';
    n_per_block = round(t_block / dt);
    n_settle = round(n_per_block * settle_frac);
    n_analysis = n_per_block - n_settle;
    n_total = n_freqs * n_per_block;
    t = (0:n_total-1)' * dt;
    d = zeros(n_total, 1);

    block_start_idx = zeros(n_freqs, 1);
    for k = 1:n_freqs
        i0 = (k-1) * n_per_block + 1;
        i1 = k * n_per_block;
        block_start_idx(k) = i0;
        t_k = t(i0:i1) - t(i0);
        d(i0:i1) = A * sin(2*pi*f_vec(k) * t_k);
    end

    info = struct();
    info.f_vec          = f_vec;
    info.t_block        = t_block;
    info.settle_frac    = settle_frac;
    info.n_per_block    = n_per_block;
    info.n_settle       = n_settle;
    info.n_analysis     = n_analysis;
    info.total_duration = t(end);
    info.n_freqs        = n_freqs;
    info.A              = A;
    info.dt             = dt;
    info.block_start_idx = block_start_idx;
end
