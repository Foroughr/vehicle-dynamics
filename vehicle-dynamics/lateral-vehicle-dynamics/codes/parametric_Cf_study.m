clc;
close all;

%% ============================================================
%  Parametric Study 2:
%  Effect of Front Cornering Stiffness Cf
%  Input: Step steering
%  Outputs: yaw rate, lateral acceleration, sideslip angle

%% ============================================================
%  Base vehicle parameters
%% ============================================================

m  = 1500;      % Vehicle mass [kg]
Iz = 2250;      % Yaw moment of inertia [kg.m^2]

a  = 1.2;       % Distance from CG to front axle [m]
b  = 1.6;       % Distance from CG to rear axle [m]
L  = a + b;     % Wheelbase [m]

U  = 20;        % Longitudinal speed [m/s]

Cr = 80000;     % Rear axle cornering stiffness [N/rad]

%% Steering input

step_amp_deg = 3;
step_amp = step_amp_deg*pi/180;

sine_amp_deg = 3;
sine_amp = sine_amp_deg*pi/180;

sine_freq = 0.5;
sim_time = 8;

%% Front cornering stiffness values

Cf_list = [50000 65000 80000 95000 110000];   % [N/rad]

%% Create folders

main_folder = 'results';
case_folder = fullfile(main_folder, 'parametric_Cf');
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

%% Storage for summary table

max_yaw_rate_deg = zeros(length(Cf_list),1);
max_ay = zeros(length(Cf_list),1);
max_beta_deg = zeros(length(Cf_list),1);

final_yaw_rate_deg = zeros(length(Cf_list),1);
final_ay = zeros(length(Cf_list),1);
final_beta_deg = zeros(length(Cf_list),1);

Kus_values = zeros(length(Cf_list),1);

%% Figures for comparison

fig_yaw = figure('Color','w','Name','Effect of Cf on Yaw Rate');
hold on
grid on

fig_ay = figure('Color','w','Name','Effect of Cf on Lateral Acceleration');
hold on
grid on

fig_beta = figure('Color','w','Name','Effect of Cf on Sideslip Angle');
hold on
grid on

legend_text = cell(length(Cf_list),1);

%% ============================================================
%  Run simulations for different Cf values
%% ============================================================

for i = 1:length(Cf_list)

    Cf = Cf_list(i);

    fprintf('\nRunning simulation for Cf = %.0f N/rad\n', Cf);

    %% Understeer gradient

    Kus = m/L * (b/Cf - a/Cr);
    Kus_values(i) = Kus;

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

    %% Run Simulink model

    simOut = sim(model_name);

    %% Read outputs

    yaw_ts  = simOut.get('yaw_rate');
    ay_ts   = simOut.get('ay_out');
    beta_ts = simOut.get('beta_out');

    t = yaw_ts.Time;

    r_data    = squeeze(yaw_ts.Data);
    ay_data   = squeeze(ay_ts.Data);
    beta_data = squeeze(beta_ts.Data);

    %% Convert units

    r_deg = rad2deg(r_data);
    beta_deg = rad2deg(beta_data);

    %% Store metrics

    max_yaw_rate_deg(i) = max(abs(r_deg));
    max_ay(i) = max(abs(ay_data));
    max_beta_deg(i) = max(abs(beta_deg));

    final_yaw_rate_deg(i) = r_deg(end);
    final_ay(i) = ay_data(end);
    final_beta_deg(i) = beta_deg(end);

    %% Plot yaw rate

    figure(fig_yaw)
    plot(t, r_deg, 'LineWidth', 1.7)

    %% Plot lateral acceleration

    figure(fig_ay)
    plot(t, ay_data, 'LineWidth', 1.7)

    %% Plot sideslip angle

    figure(fig_beta)
    plot(t, beta_deg, 'LineWidth', 1.7)

    legend_text{i} = sprintf('C_f = %.0f N/rad', Cf);

end

%% ============================================================
%  Format and save figures
%% ============================================================

figure(fig_yaw)
xlabel('Time [s]')
ylabel('Yaw Rate [deg/s]')
title('Effect of Front Cornering Stiffness on Yaw Rate')
legend(legend_text, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_Cf_effect_yaw_rate.png'), 'Resolution', 300);

figure(fig_ay)
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
title('Effect of Front Cornering Stiffness on Lateral Acceleration')
legend(legend_text, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_Cf_effect_lateral_acceleration.png'), 'Resolution', 300);

figure(fig_beta)
xlabel('Time [s]')
ylabel('Sideslip Angle \beta [deg]')
title('Effect of Front Cornering Stiffness on Sideslip Angle')
legend(legend_text, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_Cf_effect_sideslip_angle.png'), 'Resolution', 300);

%% ============================================================
%  Create and save summary table
%% ============================================================

Cf_N_per_rad = Cf_list(:);

summaryTable = table(Cf_N_per_rad, Kus_values, ...
    max_yaw_rate_deg, max_ay, max_beta_deg, ...
    final_yaw_rate_deg, final_ay, final_beta_deg);

disp(' ');
disp('=============================================');
disp('Front Cornering Stiffness Parametric Study Summary');
disp('=============================================');
disp(summaryTable);

writetable(summaryTable, fullfile(data_folder, 'summary_Cf_parametric_study.csv'));

%% Save MAT file

save(fullfile(data_folder, 'data_Cf_parametric_study.mat'), ...
    'Cf_list', 'Kus_values', ...
    'max_yaw_rate_deg', 'max_ay', 'max_beta_deg', ...
    'final_yaw_rate_deg', 'final_ay', 'final_beta_deg');

fprintf('\nCf parametric study completed successfully.\n');
fprintf('Results saved in: %s\n', case_folder);