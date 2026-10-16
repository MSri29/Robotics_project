function [u_opt, U_opt, J_opt] = Mpc_controller(x, W, theta_hat, P, U_warm)
% MPC_CONTROLLER  Linear MPC with adaptive weights + reference tracking.

    %% ---------- Setup ----------
    N  = P.N;
    nx = 5;
    nu = 3;

    x         = x(:);
    theta_hat = theta_hat(:);

    u_nom = zeros(nu, 1);
    [~, A, B] = uav_dynamics(x, u_nom, P.dt, P, theta_hat);

    %% ---------- Nominal trajectory (U = 0) ----------
    X_nom = zeros(nx, N+1);
    X_nom(:,1) = x;
    for j = 1:N
        X_nom(:,j+1) = A * X_nom(:,j);
    end

    %% ---------- Sensitivity matrices ----------
    Phi   = zeros(nx*(N+1), nx);
    Gamma = zeros(nx*(N+1), nu*N);
    Phi(1:nx, :) = eye(nx);

    for j = 1:N
        rows = (j*nx+1):((j+1)*nx);
        prev = ((j-1)*nx+1):(j*nx);
        cols = ((j-1)*nu+1):(j*nu);

        Phi(rows, :)   = A * Phi(prev, :);
        Gamma(rows, :) = A * Gamma(prev, :);
        Gamma(rows, cols) = Gamma(rows, cols) + B;
    end

    %% ---------- Reference trajectory ----------
    X_ref = zeros(nx*(N+1), 1);
    goal_p = P.goal(:);
    soc_expected = x(5) - 0.002 * (0:N);
    soc_expected = max(soc_expected, P.SOC_min + 0.05);

    for j = 0:N
        alpha = j / N;
        pj = (1 - alpha) * x(1:2) + alpha * goal_p;
        X_ref(j*nx + 1) = pj(1);
        X_ref(j*nx + 2) = pj(2);
        X_ref(j*nx + 3) = 0;
        X_ref(j*nx + 4) = P.v_max * 0.6;
        X_ref(j*nx + 5) = soc_expected(j+1);
    end

    %% ---------- Adaptive weights ----------
    if isfield(P, 'use_adaptive_weights') && P.use_adaptive_weights
        [w1_k, w2_k, w3_k, w4_k, w5_k] = adaptive_weights(x, W, theta_hat, P);
    else
        w1_k = P.w1;
        w2_k = P.w2;
        w3_k = P.w3;
        w4_k = P.w4;
        w5_k = P.w5;
    end

    %% ---------- Cost matrices ----------
    Q_run  = diag([5.0, 5.0, 0, 2.0*w1_k, 50.0]);
    R_run  = diag([0.5*w1_k, 10.0, 0.1*w2_k]);
    Q_term = diag([w4_k, w4_k, 0, 0, w5_k]);

    Qbar = blkdiag(kron(eye(N), Q_run), Q_term);
    Rbar = kron(eye(N), R_run);

    H = Gamma' * Qbar * Gamma + Rbar;

    %% ---------- Yaw-rate smoothing ----------
    for j = 2:N
        idx_prev = (j-2)*nu + 2;
        idx_curr = (j-1)*nu + 2;
        H(idx_prev, idx_prev) = H(idx_prev, idx_prev) + 50.0;
        H(idx_curr, idx_curr) = H(idx_curr, idx_curr) + 50.0;
        H(idx_prev, idx_curr) = H(idx_prev, idx_curr) - 50.0;
        H(idx_curr, idx_prev) = H(idx_curr, idx_prev) - 50.0;
    end

    H = (H + H') / 2 + 1e-4 * eye(size(H));
    g = Gamma' * Qbar * (Phi * x - X_ref);

    %% ---------- Treatment cost (uses adaptive w2_k) ----------
    for j = 1:N
        s_idx = (j-1)*nu + 3;
        Wq = interp2(W, X_nom(1,j), X_nom(2,j), 'linear', 0);
        if isnan(Wq), Wq = 0; end
        g(s_idx) = g(s_idx) + w2_k * (1 - 2*Wq);
    end

    %% ---------- Bounds ----------
    lb = repmat([-P.a_max; -P.w_max; 0], N, 1);
    ub = repmat([ P.a_max;  P.w_max; 1], N, 1);

    %% ---------- Solve ----------
    if ~isempty(U_warm) && numel(U_warm) == nu*N
        U0 = U_warm;
    else
        U0 = zeros(nu*N, 1);
    end

    opts = optimoptions('fmincon', ...
        'Algorithm', 'sqp', 'Display', 'off', ...
        'MaxIterations', 20, 'OptimalityTolerance', 1e-3, ...
        'StepTolerance', 1e-6, 'SpecifyObjectiveGradient', true);

    Jfun = @(U) qp_objective(U, H, g);
    [U_opt, J_opt, exitflag] = fmincon(Jfun, U0, [], [], [], [], lb, ub, [], opts);

    if exitflag <= 0
        U_opt = U0;
        J_opt = 0.5 * U0' * H * U0 + g' * U0;
    end

    J_const = 0.5 * (Phi * x - X_ref)' * Qbar * (Phi * x - X_ref);
    J_opt = J_const + 0.5 * U_opt' * H * U_opt + g' * U_opt;

    u_opt = U_opt(1:nu);
end

function [J, gradJ] = qp_objective(U, H, g)
    U     = U(:);
    J     = 0.5 * U' * H * U + g' * U;
    gradJ = H * U + g;
end