clc;
close all;

%% ============================================================
%  Parametric Study for Sinusoidal Steering Input
%  Studies:
%     1. Vehicle speed U
%     2. Front cornering stiffness Cf
%     3. Rear cornering stiffness Cr
%
%  Input:
%     Sinusoidal steering, amplitude = 3 deg, frequency = 0.5 Hz
%
%  Outputs:
%     yaw rate, lateral acceleration, sideslip angle
%
%  Metrics:
%     Peak absolute response
%     RMS response after transient removal

%% ============================================================
%  Base vehicle parameters
%% ============================================================

m  = 1500;      % Vehicle mass [kg]
Iz = 2250;      % Yaw moment of inertia [kg.m^2]

a  = 1.2;       % Distance from CG to front axle [m]
b  = 1.6;       % Distance from CG to rear axle [m]
L  = a + b;     % Wheelbase [m]

U  = 20;        % Longitudinal speed [m/s]
Cf = 80000;     % Front cornering stiffness [N/rad]
Cr = 80000;     % Rear cornering stiffness [N/rad]

%% Steering input

step_amp_deg = 3;
step_amp = step_amp_deg*pi/180;

sine_amp_deg = 3;
sine_amp = sine_amp_deg*pi/180;

sine_freq = 0.5;     % Hz
sim_time = 8;        % seconds

%% Transient removal time for RMS calculation

transient_cut_time = 2;    % Ignore first 2 seconds for RMS metrics

%% Main folders

main_folder = 'results';
case_folder = fullfile(main_folder, 'parametric_sine');
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

%% ============================================================
%  STUDY 1: Effect of vehicle speed U
%% ============================================================

U_list = [10 15 20 25 30];      % m/s
speed_kmh = U_list * 3.6;

peak_yaw_U = zeros(length(U_list),1);
peak_ay_U = zeros(length(U_list),1);
peak_beta_U = zeros(length(U_list),1);

rms_yaw_U = zeros(length(U_list),1);
rms_ay_U = zeros(length(U_list),1);
rms_beta_U = zeros(length(U_list),1);

fig_U_yaw = figure('Color','w','Name','Sine - Effect of Speed on Yaw Rate');
hold on
grid on

fig_U_ay = figure('Color','w','Name','Sine - Effect of Speed on Lateral Acceleration');
hold on
grid on

fig_U_beta = figure('Color','w','Name','Sine - Effect of Speed on Sideslip Angle');
hold on
grid on

legend_U = cell(length(U_list),1);

for i = 1:length(U_list)

    U = U_list(i);
    Cf = 80000;
    Cr = 80000;

    fprintf('\nSINE speed study: U = %.1f m/s = %.1f km/h\n', U, U*3.6);

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

    idx_ss = t >= transient_cut_time;

    %% Metrics

    peak_yaw_U(i) = max(abs(r_deg));
    peak_ay_U(i) = max(abs(ay_data));
    peak_beta_U(i) = max(abs(beta_deg));

    rms_yaw_U(i) = rms(r_deg(idx_ss));
    rms_ay_U(i) = rms(ay_data(idx_ss));
    rms_beta_U(i) = rms(beta_deg(idx_ss));

    %% Plots

    figure(fig_U_yaw)
    plot(t, r_deg, 'LineWidth', 1.7)

    figure(fig_U_ay)
    plot(t, ay_data, 'LineWidth', 1.7)

    figure(fig_U_beta)
    plot(t, beta_deg, 'LineWidth', 1.7)

    legend_U{i} = sprintf('U = %.0f km/h', U*3.6);

end

figure(fig_U_yaw)
xlabel('Time [s]')
ylabel('Yaw Rate [deg/s]')
title('Sine Input: Effect of Vehicle Speed on Yaw Rate')
legend(legend_U, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_sine_speed_effect_yaw_rate.png'), 'Resolution', 300);

figure(fig_U_ay)
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
title('Sine Input: Effect of Vehicle Speed on Lateral Acceleration')
legend(legend_U, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_sine_speed_effect_lateral_acceleration.png'), 'Resolution', 300);

figure(fig_U_beta)
xlabel('Time [s]')
ylabel('Sideslip Angle \beta [deg]')
title('Sine Input: Effect of Vehicle Speed on Sideslip Angle')
legend(legend_U, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_sine_speed_effect_sideslip_angle.png'), 'Resolution', 300);

summary_U_sine = table(U_list(:), speed_kmh(:), ...
    peak_yaw_U, peak_ay_U, peak_beta_U, ...
    rms_yaw_U, rms_ay_U, rms_beta_U, ...
    'VariableNames', {'Speed_mps','Speed_kmh', ...
    'PeakYawRate_deg_s','PeakAy_m_s2','PeakBeta_deg', ...
    'RMSYawRate_deg_s','RMSAy_m_s2','RMSBeta_deg'});

writetable(summary_U_sine, fullfile(data_folder, 'summary_sine_speed_study.csv'));

%% ============================================================
%  STUDY 2: Effect of front cornering stiffness Cf
%% ============================================================

U = 20;
Cr = 80000;

Cf_list = [50000 65000 80000 95000 110000];

peak_yaw_Cf = zeros(length(Cf_list),1);
peak_ay_Cf = zeros(length(Cf_list),1);
peak_beta_Cf = zeros(length(Cf_list),1);

rms_yaw_Cf = zeros(length(Cf_list),1);
rms_ay_Cf = zeros(length(Cf_list),1);
rms_beta_Cf = zeros(length(Cf_list),1);

Kus_Cf = zeros(length(Cf_list),1);

fig_Cf_yaw = figure('Color','w','Name','Sine - Effect of Cf on Yaw Rate');
hold on
grid on

fig_Cf_ay = figure('Color','w','Name','Sine - Effect of Cf on Lateral Acceleration');
hold on
grid on

fig_Cf_beta = figure('Color','w','Name','Sine - Effect of Cf on Sideslip Angle');
hold on
grid on

legend_Cf = cell(length(Cf_list),1);

for i = 1:length(Cf_list)

    Cf = Cf_list(i);

    fprintf('\nSINE Cf study: Cf = %.0f N/rad\n', Cf);

    Kus = m/L * (b/Cf - a/Cr);
    Kus_Cf(i) = Kus;

    %% State-space matrices

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

    idx_ss = t >= transient_cut_time;

    %% Metrics

    peak_yaw_Cf(i) = max(abs(r_deg));
    peak_ay_Cf(i) = max(abs(ay_data));
    peak_beta_Cf(i) = max(abs(beta_deg));

    rms_yaw_Cf(i) = rms(r_deg(idx_ss));
    rms_ay_Cf(i) = rms(ay_data(idx_ss));
    rms_beta_Cf(i) = rms(beta_deg(idx_ss));

    %% Plots

    figure(fig_Cf_yaw)
    plot(t, r_deg, 'LineWidth', 1.7)

    figure(fig_Cf_ay)
    plot(t, ay_data, 'LineWidth', 1.7)

    figure(fig_Cf_beta)
    plot(t, beta_deg, 'LineWidth', 1.7)

    legend_Cf{i} = sprintf('C_f = %.0f N/rad', Cf);

end

figure(fig_Cf_yaw)
xlabel('Time [s]')
ylabel('Yaw Rate [deg/s]')
title('Sine Input: Effect of Front Cornering Stiffness on Yaw Rate')
legend(legend_Cf, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_sine_Cf_effect_yaw_rate.png'), 'Resolution', 300);

figure(fig_Cf_ay)
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
title('Sine Input: Effect of Front Cornering Stiffness on Lateral Acceleration')
legend(legend_Cf, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_sine_Cf_effect_lateral_acceleration.png'), 'Resolution', 300);

figure(fig_Cf_beta)
xlabel('Time [s]')
ylabel('Sideslip Angle \beta [deg]')
title('Sine Input: Effect of Front Cornering Stiffness on Sideslip Angle')
legend(legend_Cf, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_sine_Cf_effect_sideslip_angle.png'), 'Resolution', 300);

summary_Cf_sine = table(Cf_list(:), Kus_Cf, ...
    peak_yaw_Cf, peak_ay_Cf, peak_beta_Cf, ...
    rms_yaw_Cf, rms_ay_Cf, rms_beta_Cf, ...
    'VariableNames', {'Cf_N_per_rad','Kus', ...
    'PeakYawRate_deg_s','PeakAy_m_s2','PeakBeta_deg', ...
    'RMSYawRate_deg_s','RMSAy_m_s2','RMSBeta_deg'});

writetable(summary_Cf_sine, fullfile(data_folder, 'summary_sine_Cf_study.csv'));

%% ============================================================
%  STUDY 3: Effect of rear cornering stiffness Cr
%% ============================================================

U = 20;
Cf = 80000;

Cr_list = [50000 65000 80000 95000 110000];

peak_yaw_Cr = zeros(length(Cr_list),1);
peak_ay_Cr = zeros(length(Cr_list),1);
peak_beta_Cr = zeros(length(Cr_list),1);

rms_yaw_Cr = zeros(length(Cr_list),1);
rms_ay_Cr = zeros(length(Cr_list),1);
rms_beta_Cr = zeros(length(Cr_list),1);

Kus_Cr = zeros(length(Cr_list),1);
behavior_Cr = strings(length(Cr_list),1);

fig_Cr_yaw = figure('Color','w','Name','Sine - Effect of Cr on Yaw Rate');
hold on
grid on

fig_Cr_ay = figure('Color','w','Name','Sine - Effect of Cr on Lateral Acceleration');
hold on
grid on

fig_Cr_beta = figure('Color','w','Name','Sine - Effect of Cr on Sideslip Angle');
hold on
grid on

legend_Cr = cell(length(Cr_list),1);

for i = 1:length(Cr_list)

    Cr = Cr_list(i);

    fprintf('\nSINE Cr study: Cr = %.0f N/rad\n', Cr);

    Kus = m/L * (b/Cf - a/Cr);
    Kus_Cr(i) = Kus;

    if Kus > 1e-6
        behavior_Cr(i) = "Understeer";
    elseif abs(Kus) <= 1e-6
        behavior_Cr(i) = "Neutral";
    else
        behavior_Cr(i) = "Oversteer";
    end

    %% State-space matrices

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

    idx_ss = t >= transient_cut_time;

    %% Metrics

    peak_yaw_Cr(i) = max(abs(r_deg));
    peak_ay_Cr(i) = max(abs(ay_data));
    peak_beta_Cr(i) = max(abs(beta_deg));

    rms_yaw_Cr(i) = rms(r_deg(idx_ss));
    rms_ay_Cr(i) = rms(ay_data(idx_ss));
    rms_beta_Cr(i) = rms(beta_deg(idx_ss));

    %% Plots

    figure(fig_Cr_yaw)
    plot(t, r_deg, 'LineWidth', 1.7)

    figure(fig_Cr_ay)
    plot(t, ay_data, 'LineWidth', 1.7)

    figure(fig_Cr_beta)
    plot(t, beta_deg, 'LineWidth', 1.7)

    legend_Cr{i} = sprintf('C_r = %.0f N/rad', Cr);

end

figure(fig_Cr_yaw)
xlabel('Time [s]')
ylabel('Yaw Rate [deg/s]')
title('Sine Input: Effect of Rear Cornering Stiffness on Yaw Rate')
legend(legend_Cr, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_sine_Cr_effect_yaw_rate.png'), 'Resolution', 300);

figure(fig_Cr_ay)
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
title('Sine Input: Effect of Rear Cornering Stiffness on Lateral Acceleration')
legend(legend_Cr, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_sine_Cr_effect_lateral_acceleration.png'), 'Resolution', 300);

figure(fig_Cr_beta)
xlabel('Time [s]')
ylabel('Sideslip Angle \beta [deg]')
title('Sine Input: Effect of Rear Cornering Stiffness on Sideslip Angle')
legend(legend_Cr, 'Location', 'best')
exportgraphics(gcf, fullfile(figure_folder, 'fig_sine_Cr_effect_sideslip_angle.png'), 'Resolution', 300);

summary_Cr_sine = table(Cr_list(:), Kus_Cr, behavior_Cr, ...
    peak_yaw_Cr, peak_ay_Cr, peak_beta_Cr, ...
    rms_yaw_Cr, rms_ay_Cr, rms_beta_Cr, ...
    'VariableNames', {'Cr_N_per_rad','Kus','Behavior', ...
    'PeakYawRate_deg_s','PeakAy_m_s2','PeakBeta_deg', ...
    'RMSYawRate_deg_s','RMSAy_m_s2','RMSBeta_deg'});

writetable(summary_Cr_sine, fullfile(data_folder, 'summary_sine_Cr_study.csv'));

%% ============================================================
%  Save all data
%% ============================================================

save(fullfile(data_folder, 'data_parametric_sine_study.mat'), ...
    'U_list', 'speed_kmh', ...
    'summary_U_sine', 'summary_Cf_sine', 'summary_Cr_sine', ...
    'Cf_list', 'Cr_list', ...
    'Kus_Cf', 'Kus_Cr', 'behavior_Cr');

%% ============================================================
%  Print all tables
%% ============================================================

disp(' ');
disp('=============================================');
disp('SINE Parametric Study: Speed');
disp('=============================================');
disp(summary_U_sine);

disp(' ');
disp('=============================================');
disp('SINE Parametric Study: Front Cornering Stiffness Cf');
disp('=============================================');
disp(summary_Cf_sine);

disp(' ');
disp('=============================================');
disp('SINE Parametric Study: Rear Cornering Stiffness Cr');
disp('=============================================');
disp(summary_Cr_sine);

fprintf('\nSine parametric study completed successfully.\n');
fprintf('Results saved in: %s\n', case_folder);