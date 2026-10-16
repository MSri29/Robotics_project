function [theta_hat, P_cov] = adaptive_update(theta_hat, P_cov, x_prev, u_prev, x_now, P)
% ADAPTIVE_UPDATE  RLS estimation of plant parameters [kd; kv; wx; wy].
%
% kv is FROZEN at its nominal value (weakly observable).

    lambda = P.rls_lambda;
    dt     = P.dt;

    v_prev   = x_prev(4);
    psi_prev = x_prev(3);

    % Regressors and measurements
    phi_kd = -v_prev^2 * dt;
    y_kd   = (x_now(4) - x_prev(4)) - u_prev(1) * dt;

    phi_kv = -v_prev^2 * dt / P.Emax;
    y_kv   = -(x_now(5) - x_prev(5)) ...
             - P.kh * dt / P.Emax ...
             - P.ks * u_prev(3) * dt / P.Emax;

    phi_wx = dt;
    y_wx   = (x_now(1) - x_prev(1)) - v_prev * cos(psi_prev) * dt;

    phi_wy = dt;
    y_wy   = (x_now(2) - x_prev(2)) - v_prev * sin(psi_prev) * dt;

    phi_vec = [phi_kd; phi_kv; phi_wx; phi_wy];
    y_vec   = [y_kd;  y_kv;  y_wx;  y_wy ];

    for i = 1:4
        phi = phi_vec(i);
        if abs(phi) < 1e-8
            continue;
        end
        p_ii = P_cov(i,i);
        den  = lambda + phi * p_ii * phi;
        K    = p_ii * phi / den;
        theta_hat(i) = theta_hat(i) + K * (y_vec(i) - phi * theta_hat(i));
        P_cov(i,i)   = (1/lambda) * (1 - K * phi) * p_ii;
    end

    %% ---------- Physical bounds (safety) ----------
    theta_hat(1) = max(0,    min(theta_hat(1), 1.0));    % kd
    theta_hat(2) = 0.0015;                                % kv FROZEN
    theta_hat(3) = max(-2.0, min(theta_hat(3), 2.0));    % wx
    theta_hat(4) = max(-2.0, min(theta_hat(4), 2.0));    % wy
end