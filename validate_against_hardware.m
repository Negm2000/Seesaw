%% validate_against_hardware.m
%  -----------------------------------------------------------------------
%  Post-experiment script: compare Simulink simulation vs QUARC hardware data.
%
%  USAGE:
%    1. Run seesaw_params.m
%    2. Open IP02_CartOnTable.slx or Seesaw_Full.slx
%    3. Run simulation (Normal mode) to get sim_xc, sim_alpha, sim_im
%    4. Run on hardware (External mode) to get hw_xc, hw_alpha
%    5. Run this script to compare and tune parameters
%  -----------------------------------------------------------------------

fprintf('\n=== Hardware Validation ===\n');

%% Check what data is available
has_sim = exist('sim_xc', 'var');
has_hw  = exist('hw_xc', 'var');
has_alpha_sim = exist('sim_alpha', 'var');
has_alpha_hw  = exist('hw_alpha', 'var');

if ~has_sim && ~has_hw
    error('No data found. Run a simulation or hardware experiment first.');
end

%% ---- Phase 1: Cart Position Comparison ----
if has_sim && has_hw
    figure('Name', 'Validation: Sim vs Hardware', 'Position', [100 100 900 600]);

    subplot(3,1,1);
    plot(sim_xc.Time, sim_xc.Data*100, 'b-', 'LineWidth', 1.5); hold on;
    plot(hw_xc.Time, hw_xc.Data*100, 'r-', 'LineWidth', 1.5);
    ylabel('x_c [cm]');
    title('Cart Position: Simulation vs Hardware');
    legend('Simulation', 'Hardware', 'Location', 'best');
    grid on;

    subplot(3,1,2);
    % Compute position error (interpolate to common time base)
    t_common = sim_xc.Time;
    hw_interp = interp1(hw_xc.Time, hw_xc.Data, t_common, 'linear', 0);
    err = (sim_xc.Data - hw_interp) * 100;  % cm
    plot(t_common, err, 'k-', 'LineWidth', 1.2);
    ylabel('Error [cm]');
    title(sprintf('Position Error (RMS = %.2f cm)', rms(err)));
    grid on;

    if has_sim
        subplot(3,1,3);
        plot(sim_im.Time, sim_im.Data, 'b-', 'LineWidth', 1.2);
        ylabel('i_m [A]'); xlabel('Time [s]');
        title('Simulated Motor Current');
        grid on;
    end

    sgtitle('Phase 1: Hardware Validation', 'FontWeight', 'bold');
end

%% ---- Phase 2: Seesaw Angle Comparison ----
if has_alpha_sim && has_alpha_hw
    figure('Name', 'Validation: Seesaw Angle', 'Position', [150 150 900 500]);

    subplot(2,1,1);
    plot(sim_alpha.Time, sim_alpha.Data*180/pi, 'b-', 'LineWidth', 1.5); hold on;
    plot(hw_alpha.Time, hw_alpha.Data*180/pi, 'r-', 'LineWidth', 1.5);
    yline(11.5, 'k--'); yline(-11.5, 'k--');
    ylabel('\alpha [deg]');
    title('Seesaw Angle: Simulation vs Hardware');
    legend('Simulation', 'Hardware', 'Location', 'best');
    grid on;

    subplot(2,1,2);
    t_common = sim_alpha.Time;
    hw_a_interp = interp1(hw_alpha.Time, hw_alpha.Data, t_common, 'linear', 0);
    err_a = (sim_alpha.Data - hw_a_interp) * 180/pi;  % deg
    plot(t_common, err_a, 'k-', 'LineWidth', 1.2);
    ylabel('Error [deg]');
    title(sprintf('Angle Error (RMS = %.2f deg)', rms(err_a)));
    xlabel('Time [s]');
    grid on;

    sgtitle('Phase 2: Seesaw Angle Validation', 'FontWeight', 'bold');
end

%% ---- Parameter Tuning Guide ----
fprintf('\n--- Parameter Tuning Guide ---\n');
fprintf('If simulation lags behind hardware:\n');
fprintf('  -> Reduce B_eq (less friction)\n');
fprintf('  -> Currently B_eq = %.2f N*s/m\n', B_eq);
fprintf('\nIf simulation leads hardware:\n');
fprintf('  -> Increase B_eq (more friction)\n');
fprintf('\nIf seesaw angle is off:\n');
fprintf('  -> Adjust B_SW (seesaw pivot friction)\n');
fprintf('  -> Currently B_SW = %.4f N*m*s/rad\n', B_SW);
fprintf('\nTo change friction: edit seesaw_params.m, re-run, re-simulate.\n');
