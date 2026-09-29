clc;
close all;

%% ============================================================
%  Parametric Study 3:
%  Effect of Rear Cornering Stiffness Cr
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

Cf = 80000;     % Front axle cornering stiffness [N/rad]

%% Steering input

step_amp_deg = 3;
step_amp = step_amp_deg*pi/180;

sine_amp_deg = 3;
sine_amp = sine_amp_deg*pi/180;

sine_freq = 0.5;
sim_time = 8;

%% Rear cornering stiffness values

Cr_list = [50000 65000 80000 95000 110000];   % [N/rad]

%% Create folders

main_folder = 'results';
case_folder = fullfile(main_folder, 'parametric_Cr');
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

max_yaw_rate_deg = zeros(length(Cr_list),1);
max_ay = zeros(length(Cr_list),1);
max_beta_deg = zeros(length(Cr_list),1);

final_yaw_rate_deg = zeros(length(Cr_list),1);
final_ay = zeros(length(Cr_list),1);
final_beta_deg = zeros(length(Cr_list),1);

Kus_values = zeros(length(Cr_list),1);
behavior_type = strings(length(Cr_list),1);

%% Figures for comparison

fig_yaw = figure('Color','w','Name','Effect of Cr on Yaw Rate');
hold on
grid on

fig_ay = figure('Color','w','Name','Effect of Cr on Lateral Acceleration');
hold on
grid on

fig_beta = figure('Color','w','Name','Effect of Cr on Sideslip Angle');
hold on
grid on

legend_text = cell(length(Cr_list),1);

%% ============================================================
%  Run simulations for different Cr values
%% ============================================================

for i = 1:length(Cr_list)

    Cr = Cr_list(i);

    fprintf('\nRunning simulation for Cr = %.0f N/rad\n', Cr);

    %% Understeer gradient

    Kus = m/L * (b/Cf - a/Cr);
    Kus_values(i) = Kus;

    if Kus > 1e-6
        behavior_type(i) = "Understeer";
    elseif abs(Kus) <= 1e-6
        behavior_type(i) = "Neutral";
    else
        behavior_type(i) = "Oversteer";
    end

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

    legend_text{i} = sprintf('C_r = %.0f N/rad', Cr);

end

%% ============================================================
%  Format and save figures
%% ============================================================

figure(fig_yaw)
xlabel('Time [s]')
ylabel('Yaw Rate [deg/s]')
title('Effect of Rear Cornering Stiffness on Yaw Rate')
legend(legend_text, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_Cr_effect_yaw_rate.png'), 'Resolution', 300);

figure(fig_ay)
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
title('Effect of Rear Cornering Stiffness on Lateral Acceleration')
legend(legend_text, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_Cr_effect_lateral_acceleration.png'), 'Resolution', 300);

figure(fig_beta)
xlabel('Time [s]')
ylabel('Sideslip Angle \beta [deg]')
title('Effect of Rear Cornering Stiffness on Sideslip Angle')
legend(legend_text, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_Cr_effect_sideslip_angle.png'), 'Resolution', 300);

%% ============================================================
%  Create and save summary table
%% ============================================================

Cr_N_per_rad = Cr_list(:);

summaryTable = table(Cr_N_per_rad, Kus_values, behavior_type, ...
    max_yaw_rate_deg, max_ay, max_beta_deg, ...
    final_yaw_rate_deg, final_ay, final_beta_deg);

disp(' ');
disp('=============================================');
disp('Rear Cornering Stiffness Parametric Study Summary');
disp('=============================================');
disp(summaryTable);

writetable(summaryTable, fullfile(data_folder, 'summary_Cr_parametric_study.csv'));

%% Save MAT file

save(fullfile(data_folder, 'data_Cr_parametric_study.mat'), ...
    'Cr_list', 'Kus_values', 'behavior_type', ...
    'max_yaw_rate_deg', 'max_ay', 'max_beta_deg', ...
    'final_yaw_rate_deg', 'final_ay', 'final_beta_deg');

fprintf('\nCr parametric study completed successfully.\n');
fprintf('Results saved in: %s\n', case_folder);