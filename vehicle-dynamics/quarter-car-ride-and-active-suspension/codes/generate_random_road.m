function [random_road_ts, random_road_info] = generate_random_road

%% Road settings

road_class = 'C';

Gq0 = 256e-6;
n0 = 0.1;
w = 2;

vehicle_speed = 10;
simulation_time = 10;
start_time = 1;
sample_time = 0.001;
fade_time = 0.5;

random_seed = 42;

%% Spatial spectrum

road_length = vehicle_speed*(simulation_time - start_time);
dn = 1/road_length;

n_min = 0.01;
n_max = 10;

n = (ceil(n_min/dn):floor(n_max/dn))*dn;
Gq = Gq0*(n/n0).^(-w);

%% Random road profile

previous_rng = rng;
rng(random_seed,'twister')

phase = 2*pi*rand(size(n));

t = (0:sample_time:simulation_time)';
zr_random = zeros(size(t));

active = t >= start_time;
tau = t(active) - start_time;
x = vehicle_speed*tau;

z = zeros(size(x));

for k = 1:numel(n)

    amplitude = sqrt(2*Gq(k)*dn);

    z = z + amplitude*cos(2*pi*n(k)*x + phase(k));
end

fade = ones(size(tau));
fade_index = tau < fade_time;

fade(fade_index) = ...
    0.5*(1 - cos(pi*tau(fade_index)/fade_time));

z = z.*fade;
zr_random(active) = z;

rng(previous_rng)

%% Simulink input

random_road_ts = timeseries(zr_random,t);
random_road_ts.Name = 'ISO 8608 Class C Road';

%% Road information

evaluation_range = t >= start_time + fade_time;

road_rms = sqrt(mean(zr_random(evaluation_range).^2));
road_min = min(zr_random);
road_max = max(zr_random);

random_road_info = table(string(road_class),...
                         Gq0,...
                         vehicle_speed,...
                         road_length,...
                         random_seed,...
                         1000*road_rms,...
                         1000*road_min,...
                         1000*road_max,...
    'VariableNames',{'RoadClass',...
                     'Gq0_m3',...
                     'VehicleSpeed_m_s',...
                     'RoadLength_m',...
                     'RandomSeed',...
                     'RoadRMS_mm',...
                     'MinimumElevation_mm',...
                     'MaximumElevation_mm'});

%% Save data

project_folder = fileparts(mfilename('fullpath'));

inputs_folder = fullfile(project_folder,'inputs');
figures_folder = fullfile(project_folder,'figures');

if ~exist(inputs_folder,'dir')
    mkdir(inputs_folder)
end

if ~exist(figures_folder,'dir')
    mkdir(figures_folder)
end

road_data = table(t,zr_random,...
    'VariableNames',{'Time_s','RoadElevation_m'});

writetable(road_data,...
    fullfile(inputs_folder,'random_road_class_C.csv'));

save(fullfile(inputs_folder,'random_road_class_C.mat'),...
     'random_road_ts',...
     'random_road_info',...
     'road_data',...
     'n',...
     'Gq');

%% Save figure

fig = figure('Visible','off',...
             'Color','w',...
             'Position',[100 100 1050 480]);

plot(t,1000*zr_random,...
    'Color',[0.15 0.35 0.65],...
    'LineWidth',1.1)

hold on
xline(start_time,'k--','LineWidth',1);

grid on
xlim([0 simulation_time])

xlabel('Time (s)')
ylabel('Road Elevation (mm)')
title('ISO 8608 Class C Random Road Profile')

set(findall(fig,'-property','FontName'),...
    'FontName','Times New Roman')

exportgraphics(fig,...
    fullfile(figures_folder,'random_road_class_C.png'),...
    'Resolution',300);

close(fig)

disp(random_road_info)

end