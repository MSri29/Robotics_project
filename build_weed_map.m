function W = build_weed_map(P)
% BUILD_WEED_MAP  Synthetic Gaussian-mixture weed density field.
%
% Generates a 2D weed density map W(x,y) in [0,1] on a grid of
% resolution P.grid_res over the field [0, Lx] x [0, Ly].
%
% Inputs:
%   P - struct from sim_params (uses Lx, Ly, grid_res)
%
% Outputs:
%   W - [Ny x Nx] double, values in [0, 1]
%
% Notes:
%   - Deterministic: uses rng(42) internally for reproducibility.
%   - Interface identical to a real-data loader: swap this function
%     to load CoFly-WeedDB masks without changing any other file.

    rng(42, 'twister');

    xg = 0:P.grid_res:P.Lx;
    yg = 0:P.grid_res:P.Ly;
    [X, Y] = meshgrid(xg, yg);

    N_c = 8;                          % number of weed clusters
    W = zeros(size(X));

    for i = 1:N_c
        cx = rand * P.Lx;             % cluster center x
        cy = rand * P.Ly;             % cluster center y
        A  = 0.30 + 0.70 * rand;      % amplitude  [0.3, 1.0]
        sg = 2 + 4 * rand;            % spread     [2, 6] m

        W = W + A * exp(-((X - cx).^2 + (Y - cy).^2) / (2 * sg^2));
    end

    W = min(W, 1);                    % clamp to [0, 1]
end