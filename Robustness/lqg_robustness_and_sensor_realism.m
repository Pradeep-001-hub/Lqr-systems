%% Robustness to Model Mismatch + Realistic Sensor Noise
% Builds on lqg_vs_lqr_vs_pid_saturation_montecarlo.m. Same plant
% structure, same actuator limits. Adds:
%
%   1. Model mismatch sweep: LQR/Kalman are DESIGNED assuming I_design,
%      but the TRUE plant has I_true != I_design (fuel burn, mismodeling,
%      manufacturing tolerance, etc). Sweep I_true and see where
%      performance degrades or goes unstable.
%   2. Realistic measurement noise derived from an actual IMU spec sheet
%      number instead of a guessed R_kf.

clc; clear; close all;

%% Design-time plant (what the controller/filter THINK the system is)
I_design = 1;
A = [0 1;
     0 0];
B_design = [0; 1/I_design];
H = [1 0];

%% LQR design (fixed once, using design-time model only)
Q_lqr = [10 0;
          0 1];
R_lqr = 1;
K = lqr(A, B_design, Q_lqr, R_lqr);

Kp = 15; Ki = 2; Kd = 5;

%% --- Realistic sensor noise (NEW) ---
% Example: a MEMS IMU gyro/inclinometer-derived angle estimate.
% Typical low-cost MEMS IMU angle noise (post-fusion, e.g. complementary
% filter output) is on the order of 0.1-0.5 deg RMS static noise.
% Using 0.3 deg RMS as a representative number:
angle_noise_deg_rms = 0.3;
angle_noise_rad_rms = deg2rad(angle_noise_deg_rms);
R_kf = angle_noise_rad_rms^2;     % measurement noise covariance
meas_noise_std = angle_noise_rad_rms;
fprintf('Using measurement noise: %.3f deg RMS (%.2e rad^2 covariance)\n', ...
    angle_noise_deg_rms, R_kf);

% Process noise: how much we trust the model between measurements.
% Kept same order-of-magnitude as before; in practice this would be
% derived from gyro bias instability / rate noise density.
Q_kf = [1e-4 0; 0 1e-4];

%% Actuator limits (unchanged)
u_max = 6;
u_rate_max = 40;

%% Simulation setup
dt = 0.01;
t  = 0:dt:10;
N  = length(t);
x0 = [0.5; 0];
n_runs_per_case = 30;   % Monte Carlo trials per mismatch case (lighter than before, we're sweeping many cases now)

%% --- Robustness sweep over I_true (NEW) ---
I_true_list = [0.5 0.7 1.0 1.3 1.6 2.0];  % 50% under to 100% over design value

results = struct('I_true', {}, 'lqg_rmse_mean', {}, 'lqg_rmse_std', {}, ...
                  'lqg_unstable_frac', {}, 'lqg_max_abs', {});

for i = 1:length(I_true_list)
    I_true = I_true_list(i);
    B_true = [0; 1/I_true];   % TRUE plant differs from design model

    run_rmse = zeros(1, n_runs_per_case);
    run_maxabs = zeros(1, n_runs_per_case);
    unstable_count = 0;

    for run = 1:n_runs_per_case
        rng(run + 1000*i);

        x_lqg = x0;
        x_hat = [0; 0];
        P = eye(2);
        u_prev_true = 0;
        resp = zeros(1, N);

        for k = 1:N
            tk = t(k);
            disturbance = 0.2 * sin(2*tk);

            y_meas = x_lqg(1) + meas_noise_std * randn();

            % Kalman filter still uses the DESIGN model (A, B_design) --
            % it doesn't know the true inertia either, same as reality.
            u_prev_cmd = -K * x_hat;
            x_hat_pred = A*x_hat + B_design*u_prev_cmd;
            P_pred = A*P*A' + Q_kf;

            innovation = y_meas - H*x_hat_pred;
            S = H*P_pred*H' + R_kf;
            Kk = P_pred*H' / S;
            x_hat = x_hat_pred + Kk*innovation;
            P = (eye(2) - Kk*H) * P_pred;

            u_cmd = -K * x_hat;
            max_step = u_rate_max*dt;
            delta = max(min(u_cmd - u_prev_true, max_step), -max_step);
            u = max(min(u_prev_true + delta, u_max), -u_max);
            u_prev_true = u;

            % TRUE plant uses B_true -- this is the mismatch
            x_lqg = x_lqg + (A*x_lqg + B_true*u + B_true*disturbance)*dt;
            resp(k) = x_lqg(1);
        end

        run_rmse(run) = sqrt(mean(resp.^2));
        run_maxabs(run) = max(abs(resp));
        if any(~isfinite(resp)) || run_maxabs(run) > 10
            unstable_count = unstable_count + 1;
        end
    end

    results(i).I_true = I_true;
    results(i).lqg_rmse_mean = mean(run_rmse);
    results(i).lqg_rmse_std = std(run_rmse);
    results(i).lqg_unstable_frac = unstable_count / n_runs_per_case;
    results(i).lqg_max_abs = mean(run_maxabs);
end

%% Report
fprintf('\n--- Robustness to inertia mismatch (design I = %.2f) ---\n', I_design);
fprintf('%-10s %-14s %-16s %-14s\n', 'I_true', 'RMSE (rad)', 'Unstable frac', 'Mean |max|(rad)');
for i = 1:length(results)
    fprintf('%-10.2f %6.4f+-%5.4f %14.1f%% %14.4f\n', results(i).I_true, ...
        results(i).lqg_rmse_mean, results(i).lqg_rmse_std, ...
        results(i).lqg_unstable_frac*100, results(i).lqg_max_abs);
end

%% Plot: RMSE vs mismatch
I_vals = [results.I_true];
rmse_vals = [results.lqg_rmse_mean];
rmse_std = [results.lqg_rmse_std];

figure;
errorbar(I_vals, rmse_vals, rmse_std, '-o', 'LineWidth', 2);
xline(I_design, '--k', 'Design I');
xlabel('True inertia I_{true}');
ylabel('RMSE (rad)');
title('LQG performance degradation vs. inertia mismatch');
grid on;

figure;
bar(I_vals, [results.lqg_unstable_frac]*100);
xlabel('True inertia I_{true}');
ylabel('Fraction of runs unstable/divergent (%)');
title('Instability onset vs. model mismatch');
grid on;
