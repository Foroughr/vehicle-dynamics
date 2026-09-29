clc
clear
close all

%% ============================================================
% Known parameters
%% ============================================================
m2 = 40;          % kg
cs = 1500;        % Ns/m
kt = 180000;      % N/m

%% Unknown parameters initial values
m1_0 = 250;       % kg
ks_0 = 20000;     % N/m

%% Simulation settings
Tend = 120;       % 2 minutes
dt = 0.0001;      % your time step
t = (0:dt:Tend)';

N = length(t);

%% Road input: chirp from 1 Hz to 10 Hz
A = 0.01;         % 10 mm
f0 = 1;           % Hz
f1 = 10;          % Hz

k_chirp = (f1 - f0)/Tend;

r = A * sin(2*pi*(f0*t + 0.5*k_chirp*t.^2));

%% ============================================================
% Nonlinear 20% increase in m1 and ks
%% ============================================================
m1_true = zeros(N,1);
ks_true = zeros(N,1);

for i = 1:N

    if t(i) <= 60

        tau = t(i)/60;

        % Nonlinear smooth increase: 0 to 1
        nonlinear_factor = 3*tau^2 - 2*tau^3;

        m1_true(i) = m1_0 * (1 + 0.2*nonlinear_factor);
        ks_true(i) = ks_0 * (1 + 0.2*nonlinear_factor);

    else

        m1_true(i) = 1.2*m1_0;
        ks_true(i) = 1.2*ks_0;

    end
end

%% ============================================================
% Simulate quarter-car system in MATLAB using Euler integration
%
% State variables:
% x1 = z1
% x2 = dz1
% x3 = z2
% x4 = dz2
%% ============================================================

z1  = zeros(N,1);
dz1 = zeros(N,1);
z2  = zeros(N,1);
dz2 = zeros(N,1);

ddz1 = zeros(N,1);
ddz2 = zeros(N,1);

for i = 1:N-1

    % Accelerations at current time
    ddz1(i) = (-cs*(dz1(i)-dz2(i)) ...
               -ks_true(i)*(z1(i)-z2(i))) / m1_true(i);

    ddz2(i) = ( cs*(dz1(i)-dz2(i)) ...
               +ks_true(i)*(z1(i)-z2(i)) ...
               -kt*(z2(i)-r(i))) / m2;

    % Euler integration
    dz1(i+1) = dz1(i) + ddz1(i)*dt;
    z1(i+1)  = z1(i)  + dz1(i)*dt;

    dz2(i+1) = dz2(i) + ddz2(i)*dt;
    z2(i+1)  = z2(i)  + dz2(i)*dt;
end

% Last acceleration sample
ddz1(N) = (-cs*(dz1(N)-dz2(N)) ...
           -ks_true(N)*(z1(N)-z2(N))) / m1_true(N);

ddz2(N) = ( cs*(dz1(N)-dz2(N)) ...
           +ks_true(N)*(z1(N)-z2(N)) ...
           -kt*(z2(N)-r(N))) / m2;

%% ============================================================
% Downsample for RLS and plotting
% Simulation is done with dt = 0.0001, but estimation uses fewer points
%% ============================================================

ds = 100;    % effective estimation step = 0.01 s

t_rls    = t(1:ds:end);
z1_rls   = z1(1:ds:end);
z2_rls   = z2(1:ds:end);
dz1_rls  = dz1(1:ds:end);
dz2_rls  = dz2(1:ds:end);
ddz1_rls = ddz1(1:ds:end);
ddz2_rls = ddz2(1:ds:end);
r_rls    = r(1:ds:end);

m1_true_rls = m1_true(1:ds:end);
ks_true_rls = ks_true(1:ds:end);

N_rls = length(t_rls);

%% ============================================================
% Add small measurement noise
%% ============================================================

noise_level = 0.005;   % 0.5%

ddz1_measured = ddz1_rls + noise_level*std(ddz1_rls)*randn(size(ddz1_rls));
ddz2_measured = ddz2_rls + noise_level*std(ddz2_rls)*randn(size(ddz2_rls));

%% ============================================================
% RLS settings
%% ============================================================

lambda = 0.995;

ks_hat = 15000;
P_ks = 1e6;
ks_hist = zeros(N_rls,1);

m1_hat = 150;
P_m1 = 1e6;
m1_hist = zeros(N_rls,1);

%% ============================================================
% RLS loop
%% ============================================================

for i = 1:N_rls

    %% Estimate ks
    phi_ks = z1_rls(i) - z2_rls(i);

    y_ks = m2*ddz2_measured(i) ...
           - cs*(dz1_rls(i)-dz2_rls(i)) ...
           + kt*(z2_rls(i)-r_rls(i));

    K_ks = (P_ks*phi_ks) / (lambda + phi_ks*P_ks*phi_ks);

    ks_hat = ks_hat + K_ks*(y_ks - phi_ks*ks_hat);

    P_ks = (P_ks - K_ks*phi_ks*P_ks) / lambda;

    ks_hist(i) = ks_hat;

    %% Estimate m1
    phi_m1 = ddz1_measured(i);

    y_m1 = -cs*(dz1_rls(i)-dz2_rls(i)) ...
           - ks_hat*(z1_rls(i)-z2_rls(i));

    K_m1 = (P_m1*phi_m1) / (lambda + phi_m1*P_m1*phi_m1);

    m1_hat = m1_hat + K_m1*(y_m1 - phi_m1*m1_hat);

    P_m1 = (P_m1 - K_m1*phi_m1*P_m1) / lambda;

    m1_hist(i) = m1_hat;

end

%% ============================================================
% Errors
%% ============================================================

m1_error = m1_true_rls - m1_hist;
ks_error = ks_true_rls - ks_hist;

%% ============================================================
% Figure 1: Tracking / convergence plots
%% ============================================================

figure('Name','RLS Tracking of Time-Varying Parameters', ...
       'Position',[100 100 1100 450]);

subplot(1,2,1)
plot(t_rls, m1_true_rls, 'r', 'LineWidth', 2.5)
hold on
plot(t_rls, m1_hist, 'b', 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('m_1 (kg)')
title('Tracking of m_1')
legend('True m_1','Estimated m_1','Location','best')
ylim([140 320])

subplot(1,2,2)
plot(t_rls, ks_true_rls, 'r', 'LineWidth', 2.5)
hold on
plot(t_rls, ks_hist, 'b', 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('k_s (N/m)')
title('Tracking of k_s')
legend('True k_s','Estimated k_s','Location','best')
ylim([14000 25000])

%% ============================================================
% Figure 2: Estimation error plots
%% ============================================================

figure('Name','RLS Estimation Errors', ...
       'Position',[150 150 1100 450]);

subplot(1,2,1)
plot(t_rls, m1_error, 'k', 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Error in m_1 (kg)')
title('Estimation Error for m_1')

subplot(1,2,2)
plot(t_rls, ks_error, 'k', 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Error in k_s (N/m)')
title('Estimation Error for k_s')

%% ============================================================
% Display final estimated values
%% ============================================================

disp('===== Final RLS Estimated Parameters =====')
disp(['Final estimated m1 = ', num2str(m1_hist(end)), ' kg'])
disp(['Final true m1      = ', num2str(m1_true_rls(end)), ' kg'])
disp(['Final estimated ks = ', num2str(ks_hist(end)), ' N/m'])
disp(['Final true ks      = ', num2str(ks_true_rls(end)), ' N/m'])