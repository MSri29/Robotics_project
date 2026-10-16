function export_json(Rf, Ra, W, P, filename)
% EXPORT_JSON  Export simulation results to JSON for 3D viewer.

    if nargin < 5
        filename = 'results/exports/simulation.json';
    end
    if ~exist(fileparts(filename), 'dir')
        mkdir(fileparts(filename));
    end

    % Metadata
    out.metadata.field_size = [P.Lx, P.Ly];
    out.metadata.dt         = P.dt;
    out.metadata.n_steps    = size(Rf.x, 2);
    out.metadata.grid_res   = P.grid_res;
    out.metadata.SOC_min    = P.SOC_min;

    % Weed map
    out.weed_map.rows = size(W, 1);
    out.weed_map.cols = size(W, 2);
    out.weed_map.data = round(W, 3);

    % Fixed MPC
    out.simulations.fixed.label      = 'Fixed MPC';
    out.simulations.fixed.color      = '#1f77b4';
    out.simulations.fixed.trajectory = pack_trajectory(Rf);
    out.simulations.fixed.theta_hat  = pack_theta(Rf);
    out.simulations.fixed.w_history  = pack_weights(Rf);   % ← ADD
    out.simulations.fixed.metrics    = pack_metrics(Rf, W, P);

    % Adaptive MPC
    out.simulations.adaptive.label      = 'Adaptive MPC';
    out.simulations.adaptive.color      = '#2ca02c';
    out.simulations.adaptive.trajectory = pack_trajectory(Ra);
    out.simulations.adaptive.theta_hat  = pack_theta(Ra);
    out.simulations.adaptive.w_history  = pack_weights(Ra);   % ← ADD
    out.simulations.adaptive.metrics    = pack_metrics(Ra, W, P);

    % True parameters
    out.true_theta.kd = P.kd_true;
    out.true_theta.kv = P.kv_true;
    out.true_theta.wx = P.wx_true;
    out.true_theta.wy = P.wy_true;

    % Write JSON
    txt = jsonencode(out, 'PrettyPrint', true);
    fid = fopen(filename, 'w');
    fwrite(fid, txt);
    fclose(fid);
    fprintf('  Exported: %s\n', filename);

    % Copy 3D viewer assets
    srcDir = fileparts(mfilename('fullpath'));
    dstDir = fileparts(filename);

    copyfile(fullfile(srcDir, 'template3d.html'), fullfile(dstDir, 'viewer3d.html'));
    copyfile(fullfile(srcDir, 'viewer3d.js'),     fullfile(dstDir, 'viewer3d.js'));
    copyfile(fullfile(srcDir, 'style3d.css'),     fullfile(dstDir, 'style3d.css'));

    fprintf('  Copied 3D viewer assets to %s\n', dstDir);
end

% =========================================================================
function traj = pack_trajectory(R)
    n = numel(R.t);
    traj = cell(n, 1);
    for k = 1:n
        traj{k} = struct( ...
            't',     R.t(k), ...
            'x',     R.x(1, k), ...
            'y',     R.x(2, k), ...
            'psi',   R.x(3, k), ...
            'v',     R.x(4, k), ...
            'soc',   R.x(5, k), ...
            'spray', R.u(3, k));
    end
    traj = [traj{:}];
end

% =========================================================================
function th = pack_theta(R)
    n = size(R.theta_hat, 2);
    th = cell(n, 1);
    for k = 1:n
        th{k} = struct( ...
            't',  R.t(k), ...
            'kd', R.theta_hat(1, k), ...
            'kv', R.theta_hat(2, k), ...
            'wx', R.theta_hat(3, k), ...
            'wy', R.theta_hat(4, k));
    end
    th = [th{:}];
end

% =========================================================================
function wh = pack_weights(R)
% PACK_WEIGHTS  Pack adaptive weight history for JSON.
    if ~isfield(R, 'w_history') || isempty(R.w_history)
        wh = [];
        return;
    end
    n = size(R.w_history, 2);
    wh = cell(n, 1);
    for k = 1:n
        wh{k} = struct( ...
            't',  R.t(k), ...
            'w1', R.w_history(1, k), ...
            'w2', R.w_history(2, k), ...
            'w3', R.w_history(3, k), ...
            'w4', R.w_history(4, k), ...
            'w5', R.w_history(5, k));
    end
    wh = [wh{:}];
end

% =========================================================================
function m = pack_metrics(R, W, P)
    M = compute_metrics(R, W, P);
    m = struct( ...
        'coverage',         M.coverage, ...
        'spray_efficiency', M.spray_efficiency, ...
        'energy',           M.energy, ...
        'rmse',             M.rmse, ...
        'spray_events',     M.n_spray_events);
end