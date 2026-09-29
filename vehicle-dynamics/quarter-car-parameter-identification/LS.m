clc
clearvars -except out

%% Known parameters
m2 = 40;
kt = 180000;
cs = 1500;

%% Extract Simulink data
time = out.z1.Time(:);

z1   = out.z1.Data(:);
z2   = out.z2.Data(:);
dz1  = out.dz1.Data(:);
dz2  = out.dz2.Data(:);
ddz1 = out.ddz1.Data(:);
ddz2 = out.ddz2.Data(:);
r    = out.r.Data(:);

N = length(time);

%% Regression signals for ks
phi_ks = z1 - z2;

Y_ks = m2*ddz2 + kt*(z2 - r) - cs*(dz1 - dz2);

%% Regression signals for m1
% m1 needs ks, so we estimate ks cumulatively first

ks_history = nan(N,1);
m1_history = nan(N,1);

%% To avoid division by very small numbers at the beginning
min_samples = 1000;

for k = min_samples:N

    % =========================
    % Cumulative LS for ks
    % =========================
    Phi_k = phi_ks(1:k);
    Y_k   = Y_ks(1:k);

    ks_est_k = Phi_k \ Y_k;

    ks_history(k) = ks_est_k;

    % =========================
    % Cumulative LS for m1
    % =========================
    Phi_m1 = ddz1(1:k);

    Y_m1 = -cs*(dz1(1:k) - dz2(1:k)) ...
           - ks_est_k*(z1(1:k) - z2(1:k));

    m1_est_k = Phi_m1 \ Y_m1;

    m1_history(k) = m1_est_k;
end

%% True values for comparison
ks_true = 20000;
m1_true = 250;

%% Plot ks convergence
figure;
plot(time, ks_history, 'LineWidth', 1.5);
hold on;
yline(ks_true, '--', 'True k_s', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Estimated k_s (N/m)');
title('Convergence of Least Squares Estimate for k_s');
legend('Estimated k_s', 'True k_s', 'Location', 'best');

%% Plot m1 convergence
figure;
plot(time, m1_history, 'LineWidth', 1.5);
hold on;
yline(m1_true, '--', 'True m_1', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Estimated m_1 (kg)');
title('Convergence of Least Squares Estimate for m_1');
legend('Estimated m_1', 'True m_1', 'Location', 'best');
