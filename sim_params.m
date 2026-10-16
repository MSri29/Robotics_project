function P = sim_params()
% SIM_PARAMS  All simulation, UAV, and MPC constants.

    %% ---------- TIME ----------
    P.dt       = 0.1;
    P.T_sim    = 120;                       % was 100 → longer mission
    P.N_steps  = round(P.T_sim/P.dt);       % = 2000

    %% ---------- FIELD ----------
    P.Lx       = 50;
    P.Ly       = 50;
    P.grid_res = 1.0;

    %% ---------- UAV LIMITS ----------
    P.v_max    = 5.0;
    P.a_max    = 2.0;
    P.w_max    = 0.5;
    P.SOC_min  = 0.20;
    P.SOC_0    = 1.00;

    %% ---------- TRUE PLANT PARAMETERS ----------
    P.kd_true  = 0.30;                       % was 0.10 → 3× drag
    P.kv_true  = 0.0015;
    P.wx_true  = 1.20;                       % was 0.30 → 4× wind
    P.wy_true  = -0.80;                      % was -0.20 → 4× wind

    %% ---------- KNOWN CONSTANTS ----------
    P.kh       = 0.0003;
    P.ks       = 0.0010;
    P.Emax     = 1.00;

    %% ---------- WEED ----------
    P.W_th     = 0.30;
    P.gamma    = 0.50;

    %% ---------- INITIAL STATE & GOAL ----------
    P.x0       = [0; 0; 0; 1.0; P.SOC_0];
    P.goal     = [P.Lx; P.Ly];

    %% ---------- MPC PARAMETERS ----------
    P.N        = 6;
    P.w1       = 0.5;
    P.w2       = 1.0;
    P.w3       = 1.0;
    P.w4       = 5.0;
    P.w5       = 10.0;

    P.theta_nominal = [0.05; 0.0015; 0.0; 0.0];   % kd wrong by 6×

    P.mpc_max_iter = 20;
    P.mpc_tol      = 1e-3;

    %% ---------- SENSOR NOISE ----------
    P.use_sensor_noise = false;   % ← toggle true to enable
    P.sigma_px  = 0.5;            % [m]    GPS x noise std
    P.sigma_py  = 0.5;            % [m]    GPS y noise std
    P.sigma_soc = 0.02;           % [-]    SOC noise std
    P.alpha_lpf = 0.5;           % [-]    low-pass filter coefficient
    P.use_adaptive_weights = false;
    P.SOC_warn     = 0.5;              % ← toggle true to enable
    P.gamma_soc    = 2.0;   % was 5.0
    P.gamma_weed   = 1.0;   % was 2.0
    P.gamma_wind   = 0.8;   % was 1.5
    P.gamma_goal   = 0.3;   % was 1.0
    %% ---------- RLS ----------
    P.rls_lambda = 0.98;
    P.rls_P0     = 100;
end