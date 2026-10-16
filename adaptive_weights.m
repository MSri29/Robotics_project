function [w1, w2, w3, w4, w5] = adaptive_weights(x, W, theta_hat, P)
% ADAPTIVE_WEIGHTS  State-dependent MPC weights.
%
% Adaptation rules:
%   w1: increases with wind magnitude (smoothness under disturbance)
%   w2: doubles over weed hotspots (aggressive spraying)
%   w3: increases when SOC drops (energy conservation)
%   w4: increases near goal (tighter terminal tracking)
%   w5: fixed (terminal safety)
%
% Inputs:
%   x         - [5x1] current state
%   W         - [Ny x Nx] weed map
%   theta_hat - [4x1] estimated params [kd; kv; wx; wy]
%   P         - struct from sim_params
%
% Outputs:
%   w1..w5 - scalar adaptive weights

    %% Rule 1: Battery-aware w3
    soc      = x(5);
    soc_min  = P.SOC_min;
    soc_warn = P.SOC_warn;
    soc_slack = (soc - soc_min) / max(soc_warn - soc_min, 1e-3);
    soc_boost = max(0, 1 - soc_slack);       % 0 when SOC high, 1 when low
    w3 = P.w3 * (1 + P.gamma_soc * soc_boost);

    %% Rule 2: Weed-proximity w2
    W_local = interp2(W, x(1), x(2), 'linear', 0);
    if isnan(W_local), W_local = 0; end
    weed_boost = double(W_local > P.W_th);
    w2 = P.w2 * (1 + P.gamma_weed * weed_boost);

    %% Rule 3: Wind-aware w1
    wind_mag = sqrt(theta_hat(3)^2 + theta_hat(4)^2);
    w1 = P.w1 * (1 + P.gamma_wind * wind_mag);

    %% Rule 4: Goal-proximity w4
    dist_goal  = norm(x(1:2) - P.goal);
    goal_boost = exp(-dist_goal / 5);
    w4 = P.w4 * (1 + P.gamma_goal * goal_boost);

    %% Rule 5: Fixed terminal safety
    w5 = P.w5;
end