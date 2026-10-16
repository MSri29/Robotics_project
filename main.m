% main.m
% -------------------------------------------------------------------------
% UAV Weed Management — Fixed vs Adaptive MPC
% Entry point: runs both controllers, compares metrics, generates plots
% and interactive 3D viewer.
% -------------------------------------------------------------------------

clear; clc; close all;

% --- Add project root to path ---
rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

% --- Determinism ---
rng(42, 'twister');

% --- Toolbox check ---
assert(exist('fmincon', 'file') > 0, 'fmincon (Optimization Toolbox) not found.');

fprintf('=== UAV Weed MPC ===\n');

% --- Parameters ---
P = sim_params();
M = P;

% --- 1. Build weed map ---
fprintf('[1/6] Building weed map...\n');
W = build_weed_map(P);

% --- 2. Run Fixed MPC ---
fprintf('[2/6] Running Fixed MPC...\n');
R_fixed = run_mission(P, M, W, false);

% --- 3. Run Adaptive MPC ---
fprintf('[3/6] Running Adaptive MPC...\n');
R_adaptive = run_mission(P, M, W, true);

% --- 4. Compute metrics ---
fprintf('[4/6] Computing metrics...\n');
R_fixed.metrics    = compute_metrics(R_fixed, W, P);
R_adaptive.metrics = compute_metrics(R_adaptive, W, P);

% --- 5. Plots ---
fprintf('[5/6] Generating plots...\n');
plot_results(R_fixed, R_adaptive, W, P);

% --- 6. Export JSON + 3D viewer ---
fprintf('[6/6] Exporting JSON and 3D viewer...\n');
export_json(R_fixed, R_adaptive, W, P);

% --- Summary ---
fprintf('\n=== RESULTS ===\n');
fprintf('%-22s %10s %10s\n', 'Metric', 'Fixed', 'Adaptive');
fprintf('%-22s %10.1f %10.1f\n', 'Coverage (%)', ...
    R_fixed.metrics.coverage, R_adaptive.metrics.coverage);
fprintf('%-22s %10.1f %10.1f\n', 'Spray efficiency (%)', ...
    R_fixed.metrics.spray_efficiency, R_adaptive.metrics.spray_efficiency);
fprintf('%-22s %10.3f %10.3f\n', 'Energy (SOC)', ...
    R_fixed.metrics.energy, R_adaptive.metrics.energy);
fprintf('%-22s %10.1f %10.1f\n', 'Mission time (s)', ...
    R_fixed.metrics.mission_time, R_adaptive.metrics.mission_time);
fprintf('%-22s %10.3f %10.3f\n', 'RMSE (m)', ...
    R_fixed.metrics.rmse, R_adaptive.metrics.rmse);

fprintf('\n✅ Done. Open: results/exports/viewer3d.html\n');