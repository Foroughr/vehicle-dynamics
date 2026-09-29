clc;
close all;

%% ============================================================
%  Understeer / Neutral Steer / Oversteer Comparison
%  2-DOF Dynamic Bicycle Model
%
%  Input:
%     Step steering input
%
%  Outputs:
%     yaw rate, lateral acceleration, sideslip angle
%
%  Purpose:
%     Compare theoretical steering behavior using understeer gradient

%% ============================================================
%  Base vehicle parameters
%% ============================================================

m  = 1500;      % Vehicle mass [kg]
Iz = 2250;      % Yaw moment of inertia [kg.m^2]

a  = 1.2;       % Distance from CG to front axle [m]
b  = 1.6;       % Distance from CG to rear axle [m]
L  = a + b;     % Wheelbase [m]

U  = 20;        % Longitudinal speed [m/s]

%% Steering input

step_amp_deg = 3;
step_amp = step_amp_deg*pi/180;

sine_amp_deg = 3;
sine_amp = sine_amp_deg*pi/180;

sine_freq = 0.5;
sim_time = 8;

%% ============================================================
%  Define three steering behavior cases
%% ============================================================

case_names = {'Understeer', 'Neutral Steer', 'Oversteer'};

Cf_list = [80000, 80000, 80000];   % [N/rad]
Cr_list = [80000, 60000, 55000];   % [N/rad]

%% Create folders

main_folder = 'results';
case_folder = fullfile(main_folder, 'steering_behavior_comparison');
figure_folder = fullfile(case_folder, 'figures');
data_folder = fullfile(case_folder, 'data');

if ~exist(main_folder, 'dir')
    mkdir(main_folder);
end

if ~exist(case_folder, 'dir')
    mkdir(case_folder);
end

if ~exist(figure_folder, 'dir')
    mkdir(figure_folder);
end

if ~exist(data_folder, 'dir')
    mkdir(data_folder);
end

%% Storage

Kus_values = zeros(length(case_names),1);

max_yaw_rate_deg = zeros(length(case_names),1);
max_ay = zeros(length(case_names),1);
max_beta_deg = zeros(length(case_names),1);

final_yaw_rate_deg = zeros(length(case_names),1);
final_ay = zeros(length(case_names),1);
final_beta_deg = zeros(length(case_names),1);

peak_overshoot_yaw_percent = zeros(length(case_names),1);

%% Figures

fig_yaw = figure('Color','w','Name','Steering Behavior - Yaw Rate');
hold on
grid on

fig_ay = figure('Color','w','Name','Steering Behavior - Lateral Acceleration');
hold on
grid on

fig_beta = figure('Color','w','Name','Steering Behavior - Sideslip Angle');
hold on
grid on

legend_text = cell(length(case_names),1);

%% ============================================================
%  Run simulations
%% ============================================================

for i = 1:length(case_names)

    Cf = Cf_list(i);
    Cr = Cr_list(i);

    fprintf('\nRunning case: %s\n', case_names{i});
    fprintf('Cf = %.0f N/rad, Cr = %.0f N/rad\n', Cf, Cr);

    %% Understeer gradient

    Kus = m/L * (b/Cf - a/Cr);
    Kus_values(i) = Kus;

    fprintf('Kus = %.6f\n', Kus);

    %% State-space matrices for optional verification block

    A11 = -(Cf + Cr)/(m*U);
    A12 = (-a*Cf + b*Cr)/(m*U) - U;

    A21 = (-a*Cf + b*Cr)/(Iz*U);
    A22 = -(a^2*Cf + b^2*Cr)/(Iz*U);

    B1 = Cf/m;
    B2 = a*Cf/Iz;

    A_sim = [A11 A12;
             A21 A22];

    B_sim = [B1;
             B2];

    C_sim = [0 1;
             1/U 0;
             A_sim(1,1) A_sim(1,2) + U];

    D_sim = [0;
             0;
             B_sim(1)];

    %% Eigenvalues for stability check

    eig_A = eig(A_sim);
    fprintf('Eigenvalues of A:\n');
    disp(eig_A);

    %% Run Simulink

    simOut = sim(model_name);

    %% Read outputs

    yaw_ts  = simOut.get('yaw_rate');
    ay_ts   = simOut.get('ay_out');
    beta_ts = simOut.get('beta_out');

    t = yaw_ts.Time;

    r_data    = squeeze(yaw_ts.Data);
    ay_data   = squeeze(ay_ts.Data);
    beta_data = squeeze(beta_ts.Data);

    r_deg = rad2deg(r_data);
    beta_deg = rad2deg(beta_data);

    %% Metrics

    max_yaw_rate_deg(i) = max(abs(r_deg));
    max_ay(i) = max(abs(ay_data));
    max_beta_deg(i) = max(abs(beta_deg));

    final_yaw_rate_deg(i) = r_deg(end);
    final_ay(i) = ay_data(end);
    final_beta_deg(i) = beta_deg(end);

    if abs(final_yaw_rate_deg(i)) > 1e-6
        peak_overshoot_yaw_percent(i) = ...
            (max_yaw_rate_deg(i) - abs(final_yaw_rate_deg(i))) / abs(final_yaw_rate_deg(i)) * 100;
    else
        peak_overshoot_yaw_percent(i) = NaN;
    end

    %% Plot yaw rate

    figure(fig_yaw)
    plot(t, r_deg, 'LineWidth', 1.8)

    %% Plot lateral acceleration

    figure(fig_ay)
    plot(t, ay_data, 'LineWidth', 1.8)

    %% Plot sideslip angle

    figure(fig_beta)
    plot(t, beta_deg, 'LineWidth', 1.8)

    legend_text{i} = sprintf('%s: C_f=%.0f, C_r=%.0f', ...
        case_names{i}, Cf, Cr);

end

%% ============================================================
%  Format and save figures
%% ============================================================

figure(fig_yaw)
xlabel('Time [s]')
ylabel('Yaw Rate [deg/s]')
title('Yaw Rate Comparison: Understeer, Neutral Steer, Oversteer')
legend(legend_text, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_behavior_yaw_rate_comparison.png'), 'Resolution', 300);

figure(fig_ay)
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
title('Lateral Acceleration Comparison: Understeer, Neutral Steer, Oversteer')
legend(legend_text, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_behavior_lateral_acceleration_comparison.png'), 'Resolution', 300);

figure(fig_beta)
xlabel('Time [s]')
ylabel('Sideslip Angle \beta [deg]')
title('Sideslip Angle Comparison: Understeer, Neutral Steer, Oversteer')
legend(legend_text, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_behavior_sideslip_angle_comparison.png'), 'Resolution', 300);

%% ============================================================
%  Summary table
%% ============================================================

Behavior = string(case_names(:));
Cf_N_per_rad = Cf_list(:);
Cr_N_per_rad = Cr_list(:);

summaryTable = table(Behavior, Cf_N_per_rad, Cr_N_per_rad, Kus_values, ...
    max_yaw_rate_deg, max_ay, max_beta_deg, ...
    final_yaw_rate_deg, final_ay, final_beta_deg, ...
    peak_overshoot_yaw_percent);

disp(' ');
disp('=============================================');
disp('Understeer / Neutral / Oversteer Summary');
disp('=============================================');
disp(summaryTable);

writetable(summaryTable, fullfile(data_folder, 'summary_steering_behavior_comparison.csv'));

%% Save MAT file

save(fullfile(data_folder, 'data_steering_behavior_comparison.mat'), ...
    'case_names', 'Cf_list', 'Cr_list', 'Kus_values', ...
    'max_yaw_rate_deg', 'max_ay', 'max_beta_deg', ...
    'final_yaw_rate_deg', 'final_ay', 'final_beta_deg', ...
    'peak_overshoot_yaw_percent');

fprintf('\nSteering behavior comparison completed successfully.\n');
fprintf('Results saved in: %s\n', case_folder);