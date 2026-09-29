clc;
close all;
%% Check if out exists

if ~exist('out','var')
    error(['Variable "out" does not exist in Workspace. ', ...
           'First run project1.m, then run the Simulink model.']);
end

%% Required signal names inside out

requiredSignals = {'delta_input', 'vy_out', 'yaw_rate', ...
                   'beta_out', 'ay_out', ...
                   'alpha_f_out', 'alpha_r_out', ...
                   'Fyf_out', 'Fyr_out'};

availableSignals = out.who;
missingSignals = setdiff(requiredSignals, availableSignals);

if ~isempty(missingSignals)
    disp('Available signals inside out are:');
    disp(availableSignals);

    error(['These required signals are missing from out: ', ...
           strjoin(missingSignals, ', '), ...
           '. Check the Variable name in the corresponding To Workspace blocks.']);
end

%% Read timeseries data from out

delta_ts    = out.get('delta_input');
vy_ts       = out.get('vy_out');
yaw_ts      = out.get('yaw_rate');
beta_ts     = out.get('beta_out');
ay_ts       = out.get('ay_out');

alpha_f_ts  = out.get('alpha_f_out');
alpha_r_ts  = out.get('alpha_r_out');
Fyf_ts      = out.get('Fyf_out');
Fyr_ts      = out.get('Fyr_out');

%% Extract data

t = yaw_ts.Time;

delta_data   = squeeze(delta_ts.Data);       % steering input [rad]
vy_data      = squeeze(vy_ts.Data);          % lateral velocity [m/s]
r_data       = squeeze(yaw_ts.Data);         % yaw rate [rad/s]
beta_data    = squeeze(beta_ts.Data);        % sideslip angle [rad]
ay_data      = squeeze(ay_ts.Data);          % lateral acceleration [m/s^2]

alpha_f_data = squeeze(alpha_f_ts.Data);     % front slip angle [rad]
alpha_r_data = squeeze(alpha_r_ts.Data);     % rear slip angle [rad]
Fyf_data     = squeeze(Fyf_ts.Data);         % front lateral force [N]
Fyr_data     = squeeze(Fyr_ts.Data);         % rear lateral force [N]

%% ============================================================
%  Automatically detect input type: Step or Sine
%% ============================================================

second_half_index = round(length(delta_data)/2):length(delta_data);
delta_second_half = delta_data(second_half_index);

if std(delta_second_half) < 0.02 * max(abs(delta_data) + eps)
    case_name = 'step';
else
    case_name = 'sine';
end

fprintf('\nDetected input case: %s\n', upper(case_name));

%% ============================================================
%  Automatically create folders
%% ============================================================

main_results_folder = 'results';
case_folder = fullfile(main_results_folder, case_name);
figure_folder = fullfile(case_folder, 'figures');
data_folder = fullfile(case_folder, 'data');

if ~exist(main_results_folder, 'dir')
    mkdir(main_results_folder);
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

%% Save raw data as MAT file

save(fullfile(data_folder, ['data_' case_name '_simulation_outputs.mat']), ...
    't', 'delta_data', 'vy_data', 'r_data', 'beta_data', 'ay_data', ...
    'alpha_f_data', 'alpha_r_data', 'Fyf_data', 'Fyr_data');

%% ============================================================
%  Figure 1: Main required outputs
%% ============================================================

figure('Color','w','Name','Main Vehicle Responses');

subplot(4,1,1)
plot(t, rad2deg(delta_data), 'LineWidth', 1.5)
grid on
ylabel('\delta [deg]')
title(['2-DOF Dynamic Bicycle Model Response - ', upper(case_name)])

subplot(4,1,2)
plot(t, rad2deg(r_data), 'LineWidth', 1.5)
grid on
ylabel('Yaw Rate [deg/s]')

subplot(4,1,3)
plot(t, ay_data, 'LineWidth', 1.5)
grid on
ylabel('a_y [m/s^2]')

subplot(4,1,4)
plot(t, rad2deg(beta_data), 'LineWidth', 1.5)
grid on
xlabel('Time [s]')
ylabel('\beta [deg]')

exportgraphics(gcf, fullfile(figure_folder, ['fig_' case_name '_main_vehicle_response.png']), 'Resolution', 300);

%% ============================================================
%  Figure 2: Yaw rate only
%% ============================================================

figure('Color','w','Name','Yaw Rate Response');

plot(t, rad2deg(r_data), 'LineWidth', 1.8)
grid on
xlabel('Time [s]')
ylabel('Yaw Rate [deg/s]')
title(['Yaw Rate Response - ', upper(case_name)])

exportgraphics(gcf, fullfile(figure_folder, ['fig_' case_name '_yaw_rate_response.png']), 'Resolution', 300);

%% ============================================================
%  Figure 3: Lateral acceleration only
%% ============================================================

figure('Color','w','Name','Lateral Acceleration Response');

plot(t, ay_data, 'LineWidth', 1.8)
grid on
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
title(['Lateral Acceleration Response - ', upper(case_name)])

exportgraphics(gcf, fullfile(figure_folder, ['fig_' case_name '_lateral_acceleration_response.png']), 'Resolution', 300);

%% ============================================================
%  Figure 4: Sideslip angle only
%% ============================================================

figure('Color','w','Name','Sideslip Angle Response');

plot(t, rad2deg(beta_data), 'LineWidth', 1.8)
grid on
xlabel('Time [s]')
ylabel('Sideslip Angle \beta [deg]')
title(['Sideslip Angle Response - ', upper(case_name)])

exportgraphics(gcf, fullfile(figure_folder, ['fig_' case_name '_sideslip_angle_response.png']), 'Resolution', 300);

%% ============================================================
%  Figure 5: Lateral velocity
%% ============================================================

figure('Color','w','Name','Lateral Velocity Response');

plot(t, vy_data, 'LineWidth', 1.8)
grid on
xlabel('Time [s]')
ylabel('Lateral Velocity v_y [m/s]')
title(['Lateral Velocity Response - ', upper(case_name)])

exportgraphics(gcf, fullfile(figure_folder, ['fig_' case_name '_lateral_velocity_response.png']), 'Resolution', 300);

%% ============================================================
%  Figure 6: Front and rear tire slip angles
%% ============================================================

figure('Color','w','Name','Tire Slip Angles');

plot(t, rad2deg(alpha_f_data), 'LineWidth', 1.8)
hold on
plot(t, rad2deg(alpha_r_data), '--', 'LineWidth', 1.8)
grid on
xlabel('Time [s]')
ylabel('Slip Angle [deg]')
title(['Front and Rear Tire Slip Angles - ', upper(case_name)])
legend('\alpha_f','\alpha_r','Location','best')

exportgraphics(gcf, fullfile(figure_folder, ['fig_' case_name '_tire_slip_angles.png']), 'Resolution', 300);

%% ============================================================
%  Figure 7: Front and rear lateral tire forces
%% ============================================================

figure('Color','w','Name','Tire Lateral Forces');

plot(t, Fyf_data, 'LineWidth', 1.8)
hold on
plot(t, Fyr_data, '--', 'LineWidth', 1.8)
grid on
xlabel('Time [s]')
ylabel('Lateral Force [N]')
title(['Front and Rear Lateral Tire Forces - ', upper(case_name)])
legend('F_{yf}','F_{yr}','Location','best')

exportgraphics(gcf, fullfile(figure_folder, ['fig_' case_name '_tire_lateral_forces.png']), 'Resolution', 300);

%% ============================================================
%  Calculate final values
%% ============================================================

final_delta_rad = delta_data(end);
final_delta_deg = rad2deg(delta_data(end));

final_vy = vy_data(end);

final_r_rad = r_data(end);
final_r_deg = rad2deg(r_data(end));

final_beta_rad = beta_data(end);
final_beta_deg = rad2deg(beta_data(end));

final_ay = ay_data(end);

final_alpha_f_rad = alpha_f_data(end);
final_alpha_f_deg = rad2deg(alpha_f_data(end));

final_alpha_r_rad = alpha_r_data(end);
final_alpha_r_deg = rad2deg(alpha_r_data(end));

final_Fyf = Fyf_data(end);
final_Fyr = Fyr_data(end);

%% ============================================================
%  Print final values
%% ============================================================

fprintf('\n=============================================\n');
fprintf('Final / Steady-State Simulation Values\n');
fprintf('Case: %s\n', upper(case_name));
fprintf('=============================================\n');

fprintf('Final steering angle delta = %.4f rad = %.3f deg\n', ...
    final_delta_rad, final_delta_deg);

fprintf('Final lateral velocity vy = %.4f m/s\n', final_vy);

fprintf('Final yaw rate r = %.4f rad/s = %.3f deg/s\n', ...
    final_r_rad, final_r_deg);

fprintf('Final sideslip beta = %.4f rad = %.3f deg\n', ...
    final_beta_rad, final_beta_deg);

fprintf('Final lateral acceleration ay = %.4f m/s^2\n', final_ay);

fprintf('Final front slip angle alpha_f = %.4f rad = %.3f deg\n', ...
    final_alpha_f_rad, final_alpha_f_deg);

fprintf('Final rear slip angle alpha_r = %.4f rad = %.3f deg\n', ...
    final_alpha_r_rad, final_alpha_r_deg);

fprintf('Final front lateral force Fyf = %.2f N\n', final_Fyf);
fprintf('Final rear lateral force Fyr = %.2f N\n', final_Fyr);

%% ============================================================
%  Save summary as text file
%% ============================================================

summary_file = fullfile(data_folder, ['summary_' case_name '_results.txt']);
fid = fopen(summary_file, 'w');

fprintf(fid, '=============================================\n');
fprintf(fid, '2-DOF Dynamic Bicycle Model Simulation Summary\n');
fprintf(fid, 'Case: %s\n', upper(case_name));
fprintf(fid, '=============================================\n\n');

fprintf(fid, 'Final steering angle delta = %.4f rad = %.3f deg\n', ...
    final_delta_rad, final_delta_deg);

fprintf(fid, 'Final lateral velocity vy = %.4f m/s\n', final_vy);

fprintf(fid, 'Final yaw rate r = %.4f rad/s = %.3f deg/s\n', ...
    final_r_rad, final_r_deg);

fprintf(fid, 'Final sideslip beta = %.4f rad = %.3f deg\n', ...
    final_beta_rad, final_beta_deg);

fprintf(fid, 'Final lateral acceleration ay = %.4f m/s^2\n', final_ay);

fprintf(fid, 'Final front slip angle alpha_f = %.4f rad = %.3f deg\n', ...
    final_alpha_f_rad, final_alpha_f_deg);

fprintf(fid, 'Final rear slip angle alpha_r = %.4f rad = %.3f deg\n', ...
    final_alpha_r_rad, final_alpha_r_deg);

fprintf(fid, 'Final front lateral force Fyf = %.2f N\n', final_Fyf);
fprintf(fid, 'Final rear lateral force Fyr = %.2f N\n', final_Fyr);

fclose(fid);

fprintf('\nFigures and data saved successfully.\n');
fprintf('Folder: %s\n', case_folder);

%% ============================================================
%  Additional response metrics
%% ============================================================

max_delta_deg = max(abs(rad2deg(delta_data)));
max_yaw_rate_deg = max(abs(rad2deg(r_data)));
max_ay = max(abs(ay_data));
max_beta_deg = max(abs(rad2deg(beta_data)));
max_vy = max(abs(vy_data));

max_alpha_f_deg = max(abs(rad2deg(alpha_f_data)));
max_alpha_r_deg = max(abs(rad2deg(alpha_r_data)));

max_Fyf = max(abs(Fyf_data));
max_Fyr = max(abs(Fyr_data));

rms_yaw_rate_deg = rms(rad2deg(r_data));
rms_ay = rms(ay_data);
rms_beta_deg = rms(rad2deg(beta_data));

fprintf('\n=============================================\n');
fprintf('Additional Response Metrics\n');
fprintf('Case: %s\n', upper(case_name));
fprintf('=============================================\n');

fprintf('Max steering angle = %.3f deg\n', max_delta_deg);
fprintf('Max absolute yaw rate = %.3f deg/s\n', max_yaw_rate_deg);
fprintf('Max absolute lateral acceleration = %.3f m/s^2\n', max_ay);
fprintf('Max absolute sideslip angle beta = %.3f deg\n', max_beta_deg);
fprintf('Max absolute lateral velocity vy = %.3f m/s\n', max_vy);

fprintf('Max absolute front slip angle alpha_f = %.3f deg\n', max_alpha_f_deg);
fprintf('Max absolute rear slip angle alpha_r = %.3f deg\n', max_alpha_r_deg);

fprintf('Max absolute front lateral force Fyf = %.2f N\n', max_Fyf);
fprintf('Max absolute rear lateral force Fyr = %.2f N\n', max_Fyr);

fprintf('RMS yaw rate = %.3f deg/s\n', rms_yaw_rate_deg);
fprintf('RMS lateral acceleration = %.3f m/s^2\n', rms_ay);
fprintf('RMS sideslip angle = %.3f deg\n', rms_beta_deg);