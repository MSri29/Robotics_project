function [x_next, A, B] = uav_dynamics(x, u, dt, P, theta)
% UAV_DYNAMICS  Discrete-time unicycle UAV with wind + battery.

    if nargin < 5 || isempty(theta)
        theta = [P.kd_true; P.kv_true; P.wx_true; P.wy_true];
    end

    kd = theta(1);
    kv = theta(2);
    wx = theta(3);
    wy = theta(4);

    px  = x(1); py = x(2); psi = x(3); v = x(4); SOC = x(5);
    a   = u(1); omega = u(2); spray = u(3);

    x_next = zeros(5, 1);
    x_next(1) = px  + (v * cos(psi) + wx) * dt;
    x_next(2) = py  + (v * sin(psi) + wy) * dt;
    x_next(3) = psi + omega * dt;
    x_next(4) = v   + (a - kd * v^2) * dt;
    x_next(5) = SOC - dt / P.Emax * (kv * v^2 + P.kh + P.ks * spray);

    if nargout >= 2
        Ac = [ 0  0  -v*sin(psi)    cos(psi)             0;
               0  0   v*cos(psi)    sin(psi)             0;
               0  0   0             0                    0;
               0  0   0            -2*kd*v               0;
               0  0   0            -2*kv*v/P.Emax        0 ];

        Bc = [ 0  0   0;
               0  0   0;
               0  1   0;
               1  0   0;
               0  0  -P.ks/P.Emax ];

        A = eye(5) + Ac * dt;
        B = Bc * dt;
    end
end