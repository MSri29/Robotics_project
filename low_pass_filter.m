function x_hat = low_pass_filter(x_measured, x_hat_prev, alpha, v_prev, dt)
% LOW_PASS_FILTER  Velocity-compensated first-order IIR filter.
%
% Implements:  x̂(k) = α·(x̂(k-1) + v·dt) + (1-α)·x_measured(k)
%
% The velocity term predicts where the UAV should be, eliminating the
% lag that a simple IIR filter introduces on a moving target.
%
% Inputs:
%   x_measured - [5x1] current measurement (with noise)
%   x_hat_prev - [5x1] previous estimate
%   alpha      - scalar in [0,1], higher = more smoothing
%   v_prev     - [5x1] previous estimated velocity (optional; default zeros)
%   dt         - scalar time step (optional; default 0)
%
% Outputs:
%   x_hat - [5x1] filtered estimate
%
% Notes:
%   - Only smooths noisy states (indices 1, 2, 5)
%   - Velocity compensation reduces tracking lag

    x_hat = x_measured;   % default: pass-through

    if nargin < 4 || isempty(v_prev)
        v_prev = zeros(5, 1);
    end
    if nargin < 5
        dt = 0;
    end

    if alpha <= 0
        return;
    end

    % Predict where the UAV should be (velocity compensation)
    x_predicted = x_hat_prev;
    x_predicted(1) = x_hat_prev(1) + v_prev(1) * dt;   % px prediction
    x_predicted(2) = x_hat_prev(2) + v_prev(2) * dt;   % py prediction

    % Filter only noisy states
    noisy_idx = [1, 2, 5];
    x_hat(noisy_idx) = alpha * x_predicted(noisy_idx) + ...
                       (1 - alpha) * x_measured(noisy_idx);
end