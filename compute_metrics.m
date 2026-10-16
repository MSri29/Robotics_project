function M = compute_metrics(R, W, P)
% COMPUTE_METRICS  Compute performance metrics from a mission run.
%
% Inputs:
%   R - results struct from run_mission (has t, x, u, theta_hat)
%   W - weed density map
%   P - struct from sim_params
%
% Outputs:
%   M - struct with fields:
%       .coverage          [%] sprayed cells where W > W_th
%       .spray_efficiency  [%] fraction of spray on weeds
%       .energy            SOC used (initial - final)
%       .mission_time      [s] time until goal reached or end
%       .rmse              [m] tracking error vs straight-line reference
%       .n_spray_events    number of steps with spray > 0.5

    N = size(R.x, 2);

    %% ---------- 1. Weed coverage ----------
    [Ny, Nx] = size(W);
    sprayed_cells = false(Ny, Nx);

    for k = 1:N
        if R.u(3, k) > 0.5
            ix = max(1, min(Nx, round(R.x(1, k) / P.grid_res) + 1));
            iy = max(1, min(Ny, round(R.x(2, k) / P.grid_res) + 1));
            sprayed_cells(iy, ix) = true;
        end
    end

    hotspots = W > P.W_th;
    n_hotspots = sum(hotspots(:));
    if n_hotspots > 0
        M.coverage = 100 * sum(hotspots(:) & sprayed_cells(:)) / n_hotspots;
    else
        M.coverage = 0;
    end

    %% ---------- 2. Spray efficiency ----------
    num = 0;  den = 0;
    xg = 0:P.grid_res:P.Lx;
    yg = 0:P.grid_res:P.Ly;

    for k = 1:N
        s = R.u(3, k);
        if s > 0
            Wq = interp2(xg, yg, W, R.x(1, k), R.x(2, k), 'linear', 0);
            if isnan(Wq), Wq = 0; end
            num = num + s * Wq;
            den = den + s;
        end
    end
    if den > 0
        M.spray_efficiency = 100 * num / den;
    else
        M.spray_efficiency = 0;
    end

    %% ---------- 3. Energy used ----------
    M.energy = R.x(5, 1) - R.x(5, end);

    %% ---------- 4. Mission time (until within 1 m of goal) ----------
    t_goal = R.t(end);
    for k = 1:N
        if norm(R.x(1:2, k) - P.goal) < 1.0
            t_goal = R.t(k);
            break;
        end
    end
    M.mission_time = t_goal;

    %% ---------- 5. RMSE vs straight-line reference ----------
    px = R.x(1, :);
    py = R.x(2, :);
    t_norm = R.t / max(R.t(end), 1e-6);
    px_ref = P.x0(1) + t_norm * (P.goal(1) - P.x0(1));
    py_ref = P.x0(2) + t_norm * (P.goal(2) - P.x0(2));
    M.rmse = sqrt(mean((px - px_ref).^2 + (py - py_ref).^2));

        %% ---------- 6. Spray events ----------
    M.n_spray_events = sum(R.u(3, :) > 0.5);

    %% ---------- 7. Sensor noise metrics (if applicable) ----------
    if isfield(R, 'x_measured') && ~isempty(R.x_measured)
        % Position RMSE of raw noisy measurements vs truth
        dx_raw = R.x_measured(1,:) - R.x(1,:);
        dy_raw = R.x_measured(2,:) - R.x(2,:);
        M.rmse_noisy = sqrt(mean(dx_raw.^2 + dy_raw.^2));

        % Position RMSE of filtered estimate vs truth
        dx_filt = R.x_hat(1,:) - R.x(1,:);
        dy_filt = R.x_hat(2,:) - R.x(2,:);
        M.rmse_filtered = sqrt(mean(dx_filt.^2 + dy_filt.^2));

        % Improvement
        M.filter_improvement = 100 * (M.rmse_noisy - M.rmse_filtered) / ...
                               max(M.rmse_noisy, 1e-6);
    else
        M.rmse_noisy = NaN;
        M.rmse_filtered = NaN;
        M.filter_improvement = NaN;
    end
end