%% Setup

init_quarter_car

project_folder = fileparts(mfilename('fullpath'));
model_name = bdroot;

if isempty(model_name) || strcmp(model_name,'simulink')
    error('Open the quarter-car Simulink model first.')
end

results_dir = fullfile(project_folder,'results','road_cases');
figures_dir = fullfile(project_folder,'figures','road_cases');

if ~exist(results_dir,'dir')
    mkdir(results_dir)
end

if ~exist(figures_dir,'dir')
    mkdir(figures_dir)
end

%% Road cases

case_names = {'step','speed_bump','sine'};
case_titles = {'Step Road Input',...
               'Speed-Bump Road Input',...
               'Sinusoidal Road Input'};

peak_accel = zeros(3,1);
peak_travel = zeros(3,1);
peak_tyre_force = zeros(3,1);

%% Simulations

for k = 1:3

    road_case = k;
    sim_out = sim(model_name,'StopTime','10');

    sim_ts = sim_out.sim_data;
    t = sim_ts.Time;
    y = sim_ts.Data;

    response = table(t,...
                     y(:,1),y(:,2),y(:,3),...
                     y(:,4),y(:,5),y(:,6),...
        'VariableNames',{'Time_s',...
                         'Road_m',...
                         'BodyDisp_m',...
                         'WheelDisp_m',...
                         'BodyAccel_m_s2',...
                         'SuspTravel_m',...
                         'DynamicTyreForce_N'});

    writetable(response,...
        fullfile(results_dir,[case_names{k} '_response.csv']));

    save(fullfile(results_dir,[case_names{k} '_response.mat']),...
         'response','road_case');

    peak_accel(k) = max(abs(y(:,4)));
    peak_travel(k) = max(abs(y(:,5)));
    peak_tyre_force(k) = max(abs(y(:,6)));

    fig = figure('Visible','off',...
                 'Color','w',...
                 'Position',[100 100 1050 720]);

    layout = tiledlayout(fig,2,2,...
                         'TileSpacing','compact',...
                         'Padding','compact');

    nexttile
    plot(t,y(:,1),'k--','LineWidth',1.4)
    hold on
    plot(t,y(:,2),'b','LineWidth',1.3)
    plot(t,y(:,3),'Color',[0.85 0.33 0.10],'LineWidth',1.2)
    grid on
    xlabel('Time (s)')
    ylabel('Displacement (m)')
    title('Vertical Displacements')
    legend('Road','Body','Wheel','Location','best')

    nexttile
    plot(t,y(:,4),'Color',[0.85 0.33 0.10],'LineWidth',1.3)
    grid on
    xlabel('Time (s)')
    ylabel('Acceleration (m/s^2)')
    title('Body Acceleration')

    nexttile
    plot(t,y(:,5),'Color',[0.49 0.18 0.56],'LineWidth',1.3)
    grid on
    xlabel('Time (s)')
    ylabel('Travel (m)')
    title('Suspension Travel')

    nexttile
    plot(t,y(:,6),'Color',[0.47 0.67 0.19],'LineWidth',1.3)
    grid on
    xlabel('Time (s)')
    ylabel('Force (N)')
    title('Dynamic Tyre Force')

    title(layout,case_titles{k},'FontWeight','bold')

    set(findall(fig,'-property','FontName'),...
        'FontName','Times New Roman')

    exportgraphics(fig,...
        fullfile(figures_dir,[case_names{k} '_response.png']),...
        'Resolution',300);

    savefig(fig,...
        fullfile(figures_dir,[case_names{k} '_response.fig']));

    close(fig)
end

%% Summary

road_number = (1:3)';
profile = case_names';

summary = table(road_number,...
                profile,...
                peak_accel,...
                peak_travel,...
                peak_tyre_force,...
    'VariableNames',{'RoadCase',...
                     'Profile',...
                     'PeakBodyAcceleration_m_s2',...
                     'PeakSuspensionTravel_m',...
                     'PeakDynamicTyreForce_N'});

writetable(summary,...
    fullfile(results_dir,'road_case_summary.csv'));

save(fullfile(results_dir,'road_case_summary.mat'),'summary')

road_case = 1;

disp(summary)