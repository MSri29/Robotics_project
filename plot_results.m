function plot_results(Rf, Ra, W, P, outdir)
% PLOT_RESULTS  Publication-quality comparison figures.
%
% Generates up to 9 figures depending on enabled features:
%   Fig 1-6: Always
%   Fig 7:   If sensor noise enabled (R.x_measured exists)
%   Fig 8:   If adaptive weights enabled (w_history varies)
%   Fig 9:   Energy efficiency comparison (always)

    if nargin < 5, outdir = 'results/figures'; end
    if ~exist(outdir, 'dir'), mkdir(outdir); end

    xg = 0:P.grid_res:P.Lx;
    yg = 0:P.grid_res:P.Ly;
    n_saved = 0;

    %% ---------- Figure 1: Trajectory over weed map ----------
    f1 = figure('Visible','off','Position',[100 100 800 700]);
    imagesc(xg, yg, W); axis xy equal tight;
    hold on;
    colormap(flipud(hot));
    contour(xg, yg, W, [P.W_th P.W_th], 'g-', 'LineWidth', 1);
    h1 = plot(Rf.x(1,:), Rf.x(2,:), 'b-',  'LineWidth', 2);
    h2 = plot(Ra.x(1,:), Ra.x(2,:), 'c-',  'LineWidth', 2);
    h3 = plot(Rf.x(1,1), Rf.x(2,1), 'ko',  'MarkerFaceColor','k', 'MarkerSize', 10);
    h4 = plot(P.goal(1), P.goal(2), 'kp',  'MarkerFaceColor','y', 'MarkerSize', 15);
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Trajectory — Fixed (blue) vs Adaptive (cyan) | dt=%.2f s', P.dt));
    legend([h1 h2 h3 h4], {'Fixed MPC', 'Adaptive MPC', 'Start', 'Goal'}, ...
        'Location', 'northeastoutside');
    colorbar;
    print(f1, fullfile(outdir, 'fig1_trajectory.png'), '-dpng', '-r200');
    close(f1); n_saved = n_saved + 1;

    %% ---------- Figure 2: SOC vs time ----------
    f2 = figure('Visible','off','Position',[100 100 800 400]);
    plot(Rf.t, Rf.x(5,:), 'b-', 'LineWidth', 1.5); hold on;
    plot(Ra.t, Ra.x(5,:), 'c-', 'LineWidth', 1.5);
    yline(P.SOC_min, 'r--', 'LineWidth', 1, 'Label', 'SOC_{min}');
    xlabel('Time [s]'); ylabel('SOC');
    title('Battery State of Charge');
    legend('Fixed MPC', 'Adaptive MPC', 'Location', 'best');
    grid on;
    print(f2, fullfile(outdir, 'fig2_soc.png'), '-dpng', '-r200');
    close(f2); n_saved = n_saved + 1;

    %% ---------- Figure 3: Control inputs ----------
    f3 = figure('Visible','off','Position',[100 100 800 700]);
    subplot(3,1,1);
    plot(Rf.t, Rf.u(1,:), 'b-'); hold on;
    plot(Ra.t, Ra.u(1,:), 'c-');
    ylabel('a [m/s^2]'); title('Control inputs'); grid on;
    legend('Fixed', 'Adaptive', 'Location', 'best');
    subplot(3,1,2);
    plot(Rf.t, Rf.u(2,:), 'b-'); hold on;
    plot(Ra.t, Ra.u(2,:), 'c-');
    ylabel('\omega [rad/s]'); grid on;
    subplot(3,1,3);
    stairs(Rf.t, Rf.u(3,:), 'b-'); hold on;
    stairs(Ra.t, Ra.u(3,:), 'c-');
    ylabel('spray'); xlabel('Time [s]'); grid on;
    print(f3, fullfile(outdir, 'fig3_controls.png'), '-dpng', '-r200');
    close(f3); n_saved = n_saved + 1;

    %% ---------- Figure 4: Parameter estimation ----------
    theta_true = [P.kd_true; P.kv_true; P.wx_true; P.wy_true];
    names = {'k_d', 'k_v', 'w_x', 'w_y'};
    f4 = figure('Visible','off','Position',[100 100 800 600]);
    for i = 1:4
        subplot(2,2,i);
        plot(Ra.t, Ra.theta_hat(i,:), 'c-', 'LineWidth', 1.5); hold on;
        yline(theta_true(i), 'k--', 'LineWidth', 1);
        yline(Rf.theta_hat(i,1), 'b:', 'LineWidth', 1);
        xlabel('Time [s]'); ylabel(names{i});
        title(sprintf('%s estimation', names{i}));
        legend('Adaptive \theta', 'True', 'Fixed \theta', 'Location', 'best');
        grid on;
    end
    print(f4, fullfile(outdir, 'fig4_theta.png'), '-dpng', '-r200');
    close(f4); n_saved = n_saved + 1;

    %% ---------- Figure 5: Metrics bar chart ----------
    Mf = compute_metrics(Rf, W, P);
    Ma = compute_metrics(Ra, W, P);
    metrics = {'Coverage (%)', 'Spray eff (%)', 'Energy', 'RMSE (m)'};
    fixed_vals    = [Mf.coverage, Mf.spray_efficiency, Mf.energy, Mf.rmse];
    adaptive_vals = [Ma.coverage, Ma.spray_efficiency, Ma.energy, Ma.rmse];

    f5 = figure('Visible','off','Position',[100 100 800 400]);
    bar([fixed_vals; adaptive_vals]');
    set(gca, 'XTickLabel', metrics);
    ylabel('Value');
    title('Performance Metrics Comparison');
    legend({'Fixed MPC', 'Adaptive MPC'}, 'Location', 'best');
    grid on;
    print(f5, fullfile(outdir, 'fig5_metrics.png'), '-dpng', '-r200');
    close(f5); n_saved = n_saved + 1;

    %% ---------- Figure 6: Side-by-side heatmap ----------
    f6 = figure('Visible','off','Position',[100 100 900 400]);
    subplot(1,2,1);
    imagesc(xg, yg, W); axis xy equal tight; hold on;
    colormap(flipud(hot));
    plot(Rf.x(1,:), Rf.x(2,:), 'b-', 'LineWidth', 1.5);
    plot(Rf.x(1,1), Rf.x(2,1), 'ko', 'MarkerFaceColor', 'k');
    title('Fixed MPC'); xlabel('x [m]'); ylabel('y [m]');
    colorbar;
    subplot(1,2,2);
    imagesc(xg, yg, W); axis xy equal tight; hold on;
    colormap(flipud(hot));
    plot(Ra.x(1,:), Ra.x(2,:), 'c-', 'LineWidth', 1.5);
    plot(Ra.x(1,1), Ra.x(2,1), 'ko', 'MarkerFaceColor', 'k');
    title('Adaptive MPC'); xlabel('x [m]'); ylabel('y [m]');
    colorbar;
    print(f6, fullfile(outdir, 'fig6_compare.png'), '-dpng', '-r200');
    close(f6); n_saved = n_saved + 1;

    %% ---------- Figure 7: Sensor noise + filtering ----------
    if isfield(Ra, 'x_measured') && ~isempty(Ra.x_measured)
        f7 = figure('Visible','off','Position',[100 100 900 700]);

        subplot(2,1,1);
        imagesc(xg, yg, W); axis xy equal tight; hold on;
        colormap(flipud(hot));
        plot(Ra.x(1,:),          Ra.x(2,:),          'k-',  'LineWidth', 2);
        plot(Ra.x_measured(1,:), Ra.x_measured(2,:), 'r.', 'MarkerSize', 6);
        plot(Ra.x_hat(1,:),      Ra.x_hat(2,:),      'b--', 'LineWidth', 1.5);
        legend('True', 'Noisy measurement', 'Filtered', 'Location', 'northeastoutside');
        title('Position: True vs Noisy vs Filtered (Adaptive MPC)');
        xlabel('x [m]'); ylabel('y [m]');

        subplot(2,1,2);
        err_noisy = sqrt((Ra.x_measured(1,:) - Ra.x(1,:)).^2 + ...
                         (Ra.x_measured(2,:) - Ra.x(2,:)).^2);
        err_filt  = sqrt((Ra.x_hat(1,:) - Ra.x(1,:)).^2 + ...
                         (Ra.x_hat(2,:) - Ra.x(2,:)).^2);
        plot(Ra.t, err_noisy, 'r-', 'LineWidth', 1); hold on;
        plot(Ra.t, err_filt,  'b-', 'LineWidth', 1.5);
        yline(mean(err_noisy), 'r--', 'LineWidth', 1);
        yline(mean(err_filt),  'b--', 'LineWidth', 1);
        xlabel('Time [s]'); ylabel('Position error [m]');
        title(sprintf('Estimation error: noisy RMSE=%.2f m, filtered RMSE=%.2f m', ...
              mean(err_noisy), mean(err_filt)));
        legend('Noisy', 'Filtered', 'Mean (noisy)', 'Mean (filtered)', ...
               'Location', 'best');
        grid on;

        print(f7, fullfile(outdir, 'fig7_noise_filter.png'), '-dpng', '-r200');
        close(f7); n_saved = n_saved + 1;
    end

    %% ---------- Figure 8: Adaptive Weight Evolution ----------
    if isfield(Ra, 'w_history') && any(Ra.w_history(:) ~= Ra.w_history(1,1))
        f8 = figure('Visible','off','Position',[100 100 900 700]);

        labels = {'w_1 (smoothness)', 'w_2 (treatment)', ...
                  'w_3 (energy)', 'w_4 (terminal)', 'w_5 (safety)'};
        for i = 1:5
            subplot(5,1,i);
            plot(Ra.t, Ra.w_history(i,:), 'b-', 'LineWidth', 1.5);
            ylabel(labels{i});
            grid on;
            if i == 1
                title('Adaptive Weight Evolution Over Mission');
            end
        end
        xlabel('Time [s]');

        print(f8, fullfile(outdir, 'fig8_adaptive_weights.png'), '-dpng', '-r200');
        close(f8); n_saved = n_saved + 1;
    end

    %% ---------- Figure 9: Energy Efficiency Comparison ----------
    f9 = figure('Visible','off','Position',[100 100 1200 400]);

    % Compute normalized efficiency metrics
    Mf = compute_metrics(Rf, W, P);
    Ma = compute_metrics(Ra, W, P);

    t_f = size(Rf.x, 2) * P.dt;
    t_a = size(Ra.x, 2) * P.dt;

    eff_f = Mf.energy / t_f;    % [SOC/s]
    eff_a = Ma.energy / t_a;

    % Subplot 1: Energy per second
    subplot(1,3,1);
    b1 = bar([eff_f, eff_a]);
    b1.FaceColor = 'flat';
    b1.CData(1,:) = [0.12, 0.47, 0.71];   % blue
    b1.CData(2,:) = [0.17, 0.63, 0.17];   % green
    set(gca, 'XTickLabel', {'Fixed', 'Adaptive'});
    ylabel('Energy [SOC/s]');
    title('Energy Efficiency');
    grid on;
    text(1, eff_f + 0.0003, sprintf('%.4f', eff_f), 'HorizontalAlignment', 'center');
    text(2, eff_a + 0.0003, sprintf('%.4f', eff_a), 'HorizontalAlignment', 'center');

    % Subplot 2: Coverage
    subplot(1,3,2);
    b2 = bar([Mf.coverage, Ma.coverage]);
    b2.FaceColor = 'flat';
    b2.CData(1,:) = [0.12, 0.47, 0.71];
    b2.CData(2,:) = [0.17, 0.63, 0.17];
    set(gca, 'XTickLabel', {'Fixed', 'Adaptive'});
    ylabel('Coverage [%]');
    title('Mission Coverage');
    grid on;
    text(1, Mf.coverage + 0.2, sprintf('%.1f%%', Mf.coverage), 'HorizontalAlignment', 'center');
    text(2, Ma.coverage + 0.2, sprintf('%.1f%%', Ma.coverage), 'HorizontalAlignment', 'center');

    % Subplot 3: Mission duration
    subplot(1,3,3);
    b3 = bar([t_f, t_a]);
    b3.FaceColor = 'flat';
    b3.CData(1,:) = [0.12, 0.47, 0.71];
    b3.CData(2,:) = [0.17, 0.63, 0.17];
    set(gca, 'XTickLabel', {'Fixed', 'Adaptive'});
    ylabel('Duration [s]');
    title('Mission Completion');
    yline(P.T_sim, 'r--', 'Full mission', 'LabelHorizontalAlignment', 'left');
    grid on;
    text(1, t_f + 2, sprintf('%.1f s', t_f), 'HorizontalAlignment', 'center');
    text(2, t_a + 2, sprintf('%.1f s', t_a), 'HorizontalAlignment', 'center');

    sgtitle('Energy Efficiency & Mission Performance Comparison');

    print(f9, fullfile(outdir, 'fig9_energy_efficiency.png'), '-dpng', '-r200');
    close(f9); n_saved = n_saved + 1;

    %% ---------- Summary ----------
    fprintf('  Saved %d figures to %s\n', n_saved, outdir);
end