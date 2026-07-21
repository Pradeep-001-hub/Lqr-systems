%% LQG vs LQR vs PID -- with Actuator Limits + Monte Carlo Noise Sweep
% Builds directly on lqg_vs_lqr_vs_pid.m. Same plant, same weights, same
% disturbance. Adds two things that were flagged as missing:
%
%   1. Actuator saturation + rate limiting (no more "free" instantaneous
%      unlimited control torque).
%   2. Monte Carlo sweep over many noise realizations, so the LQG vs PID
%      comparison is a distribution, not a single lucky/unlucky run.

clc; clear; close all;

%% Plant
I = 1;
A = [0 1;
     0 0];
B = [0;
     1/I];
H = [1 0];

%% LQR design
Q_lqr = [10 0;
          0 1];
R_lqr = 1;
K = lqr(A, B, Q_lqr, R_lqr);

%% PID gains
Kp = 15; Ki = 2; Kd = 5;

%% Kalman filter tuning
Q_kf = [1e-4 0; 0 1e-4];
R_kf = 5e-2;
meas_noise_std = sqrt(R_kf);

%% Actuator limits (NEW)
u_max      = 6;     % max control torque magnitude
u_rate_max = 40;     % max change in u per second (slew rate limit)

%% Simulation setup
dt = 0.01;
t  = 0:dt:10;
N  = length(t);
x0 = [0.5; 0];

n_runs = 100;   % Monte Carlo trials (NEW)

lqg_rmse = zeros(1, n_runs);
pid_rmse = zeros(1, n_runs);
lqg_effort = zeros(1, n_runs);
pid_effort = zeros(1, n_runs);
lqg_settle = zeros(1, n_runs);
pid_settle = zeros(1, n_runs);

settle_band = 0.02; % rad, band used to define "settled"

% Keep the response curves from run 1 for plotting a representative case
plot_run = 1;

for run = 1:n_runs
    rng(run); % different noise realization per run, reproducible overall

    x_lqg = x0;
    x_pid = x0;
    x_hat = [0; 0];
    P = eye(2);

    u_lqg_prev_true = 0;   % previous ACTUAL (post-saturation) control, for rate limiting
    u_pid_prev_true = 0;

    lqg_true_resp = zeros(1, N);
    pid_resp      = zeros(1, N);
    lqg_est_resp  = zeros(1, N);
    u_lqg_list = zeros(1, N);
    u_pid_list = zeros(1, N);

    integral = 0;
    prev_error = x_pid(1);

    for k = 1:N
        tk = t(k);
        disturbance = 0.2 * sin(2*tk);

        %% LQG branch
        y_meas = x_lqg(1) + meas_noise_std * randn();

        u_prev_cmd = -K * x_hat;
        x_hat_pred = A*x_hat + B*u_prev_cmd;
        P_pred = A*P*A' + Q_kf;

        innovation = y_meas - H*x_hat_pred;
        S = H*P_pred*H' + R_kf;
        Kk = P_pred*H' / S;
        x_hat = x_hat_pred + Kk*innovation;
        P = (eye(2) - Kk*H) * P_pred;

        u_lqg_cmd = -K * x_hat;
        u_lqg = apply_actuator_limits(u_lqg_cmd, u_lqg_prev_true, u_max, u_rate_max, dt);
        u_lqg_prev_true = u_lqg;

        x_lqg = x_lqg + (A*x_lqg + B*u_lqg + B*disturbance)*dt;
        lqg_true_resp(k) = x_lqg(1);
        lqg_est_resp(k)  = x_hat(1);
        u_lqg_list(k) = u_lqg;

        %% PID branch
        y_pid_meas = x_pid(1) + meas_noise_std * randn();
        error = y_pid_meas;
        integral = integral + error*dt;
        derivative = (error - prev_error) / dt;
        u_pid_cmd = -(Kp*error + Ki*integral + Kd*derivative);
        prev_error = error;

        u_pid = apply_actuator_limits(u_pid_cmd, u_pid_prev_true, u_max, u_rate_max, dt);
        u_pid_prev_true = u_pid;

        % Basic anti-windup: stop integrating further if we're saturated
        if abs(u_pid_cmd) > u_max
            integral = integral - error*dt;
        end

        x_pid = x_pid + (A*x_pid + B*u_pid + B*disturbance)*dt;
        pid_resp(k) = x_pid(1);
        u_pid_list(k) = u_pid;
    end

    lqg_rmse(run) = sqrt(mean(lqg_true_resp.^2));
    pid_rmse(run) = sqrt(mean(pid_resp.^2));
    lqg_effort(run) = sum(abs(u_lqg_list))*dt;
    pid_effort(run) = sum(abs(u_pid_list))*dt;
    lqg_settle(run) = settle_time(t, lqg_true_resp, settle_band);
    pid_settle(run) = settle_time(t, pid_resp, settle_band);

    if run == plot_run
        rep_lqg_resp = lqg_true_resp;
        rep_lqg_est  = lqg_est_resp;
        rep_pid_resp = pid_resp;
        rep_u_lqg    = u_lqg_list;
        rep_u_pid    = u_pid_list;
    end
end

%% Summary statistics across Monte Carlo runs
fprintf('--- Monte Carlo summary over %d noise realizations ---\n', n_runs);
fprintf('%-14s %12s %12s\n', 'Metric', 'LQG', 'PID');
fprintf('%-14s %6.4f +- %5.4f  %6.4f +- %5.4f\n', 'RMSE (rad)', ...
    mean(lqg_rmse), std(lqg_rmse), mean(pid_rmse), std(pid_rmse));
fprintf('%-14s %6.3f +- %5.3f  %6.3f +- %5.3f\n', 'Effort', ...
    mean(lqg_effort), std(lqg_effort), mean(pid_effort), std(pid_effort));
fprintf('%-14s %6.3f +- %5.3f  %6.3f +- %5.3f\n', 'Settle time(s)', ...
    mean(lqg_settle), std(lqg_settle), mean(pid_settle), std(pid_settle));

win_rate = mean(lqg_rmse < pid_rmse) * 100;
fprintf('\nLQG beat PID on RMSE in %.1f%% of the %d runs.\n', win_rate, n_runs);

%% Plot: representative single run (with saturation now visible)
figure;
plot(t, rep_lqg_resp, 'LineWidth', 2); hold on;
plot(t, rep_lqg_est, '--', 'LineWidth', 1.2);
plot(t, rep_pid_resp, 'LineWidth', 2);
yline(settle_band, ':k'); yline(-settle_band, ':k');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('LQG (true state)', 'LQG (Kalman estimate)', 'PID', 'settle band');
title(sprintf('Representative run (#%d) with actuator saturation', plot_run));
grid on;

figure;
plot(t, rep_u_lqg, 'LineWidth', 2); hold on;
plot(t, rep_u_pid, 'LineWidth', 2);
yline(u_max, '--k'); yline(-u_max, '--k');
xlabel('Time (s)'); ylabel('Control Input (saturated)');
legend('LQG', 'PID', 'actuator limit');
title('Control Effort (with saturation + rate limit)');
grid on;

%% Distribution plot across Monte Carlo runs
figure;
boxplot([lqg_rmse(:), pid_rmse(:)], 'Labels', {'LQG', 'PID'});
ylabel('RMSE (rad)');
title(sprintf('RMSE distribution across %d noise realizations', n_runs));
grid on;

%% ---- Local functions ----

function u_out = apply_actuator_limits(u_cmd, u_prev, u_max, u_rate_max, dt)
    % Rate limit first, then saturate magnitude
    max_step = u_rate_max * dt;
    delta = u_cmd - u_prev;
    delta = max(min(delta, max_step), -max_step);
    u_rate_limited = u_prev + delta;
    u_out = max(min(u_rate_limited, u_max), -u_max);
end

function ts = settle_time(t, resp, band)
    % First time after which the response stays within +/- band forever
    idx_outside = find(abs(resp) > band);
    if isempty(idx_outside)
        ts = 0;
    elseif idx_outside(end) == length(resp)
        ts = t(end); % never settles within the simulated window
    else
        ts = t(idx_outside(end) + 1);
    end
end
