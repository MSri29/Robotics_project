function R = run_mission(P, M, W, use_adaptive)
% RUN_MISSION  Run mission with weed-hotspot waypoints + optional sensor noise.

    N  = P.N_steps;
    nx = 5;
    nu = 3;

    R.t         = zeros(1, N);
    R.x         = zeros(nx, N);
    R.u         = zeros(nu, N);
    R.theta_hat = zeros(4, N);
    R.adaptive  = use_adaptive;
    R.wp_visited = 0;
    R.w_history = zeros(5, N);

    % --- Noise/filter logging ---
    if isfield(P, 'use_sensor_noise') && P.use_sensor_noise
        R.x_measured = zeros(nx, N);
        R.x_hat      = zeros(nx, N);
    end

    % --- Find weed hotspots ---
    [iy_list, ix_list] = find(W > 0.4);
    hotspots = [(ix_list-1) * P.grid_res, (iy_list-1) * P.grid_res];
    hotspots = round(hotspots / 2) * 2;
    hotspots = unique(hotspots, 'rows');
    fprintf('  Found %d unique weed hotspots\n', size(hotspots, 1));

    if isempty(hotspots)
        hotspots = P.goal(:)';
    end

    % --- Greedy nearest-neighbor ordering ---
    ordered = zeros(size(hotspots));
    current = P.x0(1:2)';
    remaining = hotspots;
    for i = 1:size(hotspots, 1)
        d = sum((remaining - current).^2, 2);
        [~, idx] = min(d);
        ordered(i, :) = remaining(idx, :);
        current = remaining(idx, :);
        remaining(idx, :) = [];
    end
    hotspots = ordered;

    % --- Start mission ---
    wp_idx = 1;
    P_local = P;
    P_local.goal = hotspots(wp_idx, :)';

    x = P.x0;
    theta_hat = P.theta_nominal;
    if use_adaptive
        P_cov = P.rls_P0 * eye(4);
    else
        P_cov = zeros(4);
    end

    U_warm = [];
    wp_radius = 5.0;
    k_end = N;

    % Initialize filtered state (equal to true state at start)
    x_hat = x;

    for k = 1:N
        %% ---------- Sensor measurement + filter ----------
        if isfield(P, 'use_sensor_noise') && P.use_sensor_noise
            % Add noise to px, py, SOC (not to psi, v)
            x_measured = x;
            x_measured(1) = x(1) + P.sigma_px * randn();
            x_measured(2) = x(2) + P.sigma_py * randn();
            x_measured(5) = x(5) + P.sigma_soc * randn();

            % Compute velocity in world frame for prediction
            v_x = x(4) * cos(x(3));
            v_y = x(4) * sin(x(3));
            v_prev = [v_x; v_y; 0; 0; 0];   % only px, py matter for prediction

            % Apply velocity-compensated filter
            x_hat = low_pass_filter(x_measured, x_hat, P.alpha_lpf, v_prev, P.dt);

            % Log for plotting
            R.x_measured(:, k) = x_measured;
            R.x_hat(:, k)      = x_hat;
        else
            x_hat = x;   % perfect state — no noise
        end

        %% ---------- MPC uses FILTERED state ----------
        [u, U_warm] = Mpc_controller(x_hat, W, theta_hat, P_local, U_warm);

        R.t(k)            = (k-1) * P.dt;
        R.x(:, k)         = x;
        R.u(:, k)         = u;
        R.theta_hat(:, k) = theta_hat;

        % Log current weights
        if isfield(P, 'use_adaptive_weights') && P.use_adaptive_weights
            [w1_k, w2_k, w3_k, w4_k, w5_k] = adaptive_weights(x, W, theta_hat, P);
            R.w_history(:, k) = [w1_k; w2_k; w3_k; w4_k; w5_k];
        else
            R.w_history(:, k) = [P.w1; P.w2; P.w3; P.w4; P.w5];
        end

        % --- Waypoint reached? ---
        if norm(x(1:2) - P_local.goal) < wp_radius
            R.wp_visited = R.wp_visited + 1;
            wp_idx = wp_idx + 1;
            if wp_idx > size(hotspots, 1)
                wp_idx = 1;
            end
            P_local.goal = hotspots(wp_idx, :)';
        end

        % --- Propagate plant (TRUE state) ---
        x_prev = x;
        x = uav_dynamics(x_prev, u, P.dt, P);

        % --- Battery safety stop ---
        if x(5) < P.SOC_min
            fprintf('  ⚠️ Battery depleted at t=%.1f s (k=%d)\n', R.t(k), k);
            k_end = k;
            break;
        end

        % --- Adaptive update (uses TRUE measurements) ---
        if use_adaptive
            [theta_hat, P_cov] = adaptive_update(theta_hat, P_cov, x_prev, u, x, P);
        end
    end

    % --- Trim arrays if mission ended early ---
    if k_end < N
        R.t         = R.t(1:k_end);
        R.x         = R.x(:, 1:k_end);
        R.u         = R.u(:, 1:k_end);
        R.theta_hat = R.theta_hat(:, 1:k_end);
        R.w_history = R.w_history(:, 1:k_end);   % ← ADD THIS LINE
        if isfield(R, 'x_measured')
            R.x_measured = R.x_measured(:, 1:k_end);
            R.x_hat      = R.x_hat(:, 1:k_end);
        end
    end
end