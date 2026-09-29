%% Setup

init_quarter_car

project_folder = fileparts(mfilename('fullpath'));
model_name = bdroot;

if isempty(model_name) || strcmp(model_name,'simulink')
    error('Open the quarter-car Simulink model first.')
end

results_root = fullfile(project_folder,...
    'results','parametric_study');

figures_root = fullfile(project_folder,...
    'figures','parametric_study');

if ~exist(results_root,'dir')
    mkdir(results_root)
end

if ~exist(figures_root,'dir')
    mkdir(figures_root)
end

%% Study definition

road_names = {'step','speed_bump','sine'};

road_titles = {'Step Road Input',...
               'Speed-Bump Road Input',...
               'Sinusoidal Road Input'};

x_limits = [0.8 5
            0.8 5
            0.8 10];

parameter_names = {'spring_stiffness',...
                   'damping_coefficient',...
                   'sprung_mass'};

parameter_titles = {'Spring Stiffness',...
                    'Damping Coefficient',...
                    'Sprung Mass'};

parameter_symbols = {'k_s','c_s','m_s'};

level_names = {'Low','Baseline','High'};
level_tags = {'70_percent','baseline','130_percent'};

multipliers = [0.70 1.00 1.30];

colors = [0.00 0.45 0.74
          0.20 0.20 0.20
          0.85 0.33 0.10];

ks_ref = ks;
cs_ref = cs;
ms_ref = ms;

base_values = [ks_ref cs_ref ms_ref];

n_parameters = numel(parameter_names);
n_roads = numel(road_names);
n_levels = numel(multipliers);
n_rows = n_parameters*n_roads*n_levels;

%% Summary storage

parameter_column = cell(n_rows,1);
road_column = cell(n_rows,1);
level_column = cell(n_rows,1);

multiplier_column = zeros(n_rows,1);
parameter_value = zeros(n_rows,1);

ks_column = zeros(n_rows,1);
cs_column = zeros(n_rows,1);
ms_column = zeros(n_rows,1);

peak_accel = zeros(n_rows,1);
peak_travel_mm = zeros(n_rows,1);
peak_tyre_force = zeros(n_rows,1);

accel_change = zeros(n_rows,1);
travel_change = zeros(n_rows,1);
tyre_change = zeros(n_rows,1);

row = 0;

%% Simulations

for p = 1:n_parameters

    for r = 1:n_roads

        time_data = cell(n_levels,1);
        response_data = cell(n_levels,1);

        for q = 1:n_levels

            ks = ks_ref;
            cs = cs_ref;
            ms = ms_ref;

            current_value = base_values(p)*multipliers(q);

            switch p
                case 1
                    ks = current_value;

                case 2
                    cs = current_value;

                case 3
                    ms = current_value;
            end

            road_case = r;

            sim_out = sim(model_name,'StopTime','10');

            sim_ts = sim_out.sim_data;
            t = sim_ts.Time;
            y = sim_ts.Data;

            time_data{q} = t;
            response_data{q} = y;

            row = row + 1;

            parameter_column{row} = parameter_names{p};
            road_column{row} = road_names{r};
            level_column{row} = level_names{q};

            multiplier_column(row) = multipliers(q);
            parameter_value(row) = current_value;

            ks_column(row) = ks;
            cs_column(row) = cs;
            ms_column(row) = ms;

            peak_accel(row) = max(abs(y(:,4)));
            peak_travel_mm(row) = 1000*max(abs(y(:,5)));
            peak_tyre_force(row) = max(abs(y(:,6)));

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

            case_folder = fullfile(results_root,...
                parameter_names{p},road_names{r});

            if ~exist(case_folder,'dir')
                mkdir(case_folder)
            end

            file_name = [parameter_names{p} '_'...
                         road_names{r} '_'...
                         level_tags{q} '_response.csv'];

            writetable(response,...
                fullfile(case_folder,file_name));
        end

        index = (row-2):row;
        baseline_index = index(2);

        accel_change(index) = ...
            100*(peak_accel(index)/peak_accel(baseline_index) - 1);

        travel_change(index) = ...
            100*(peak_travel_mm(index)/...
            peak_travel_mm(baseline_index) - 1);

        tyre_change(index) = ...
            100*(peak_tyre_force(index)/...
            peak_tyre_force(baseline_index) - 1);

        %% Comparison figure

        fig = figure('Visible','off',...
                     'Color','w',...
                     'Position',[100 70 1050 850]);

        layout = tiledlayout(fig,3,1,...
                             'TileSpacing','compact',...
                             'Padding','compact');

        legend_names = {['0.7 ' parameter_symbols{p}],...
                        parameter_symbols{p},...
                        ['1.3 ' parameter_symbols{p}]};

        nexttile
        hold on

        for q = 1:n_levels
            plot(time_data{q},response_data{q}(:,4),...
                'Color',colors(q,:),...
                'LineWidth',1.4);
        end

        grid on
        xlim(x_limits(r,:))
        ylabel('Acceleration (m/s^2)')
        title('Body Acceleration')
        legend(legend_names,'Location','best')

        nexttile
        hold on

        for q = 1:n_levels
            plot(time_data{q},...
                1000*response_data{q}(:,5),...
                'Color',colors(q,:),...
                'LineWidth',1.4);
        end

        grid on
        xlim(x_limits(r,:))
        ylabel('Travel (mm)')
        title('Suspension Travel')

        nexttile
        hold on

        for q = 1:n_levels
            plot(time_data{q},response_data{q}(:,6),...
                'Color',colors(q,:),...
                'LineWidth',1.4);
        end

        grid on
        xlim(x_limits(r,:))
        xlabel('Time (s)')
        ylabel('Force (N)')
        title('Dynamic Tyre Force')

        title(layout,...
            ['Effect of ' parameter_titles{p}...
             ' - ' road_titles{r}],...
            'FontWeight','bold')

        set(findall(fig,'-property','FontName'),...
            'FontName','Times New Roman')

        figure_name = [parameter_names{p} '_'...
                       road_names{r} '_comparison.png'];

        exportgraphics(fig,...
            fullfile(figures_root,figure_name),...
            'Resolution',300);

        close(fig)
    end
end

%% Final summary

summary = table(parameter_column,...
                road_column,...
                level_column,...
                multiplier_column,...
                parameter_value,...
                ks_column,...
                cs_column,...
                ms_column,...
                peak_accel,...
                accel_change,...
                peak_travel_mm,...
                travel_change,...
                peak_tyre_force,...
                tyre_change,...
    'VariableNames',{'Parameter',...
                     'RoadProfile',...
                     'Level',...
                     'Multiplier',...
                     'ParameterValue',...
                     'ks_N_m',...
                     'cs_Ns_m',...
                     'ms_kg',...
                     'PeakBodyAccel_m_s2',...
                     'AccelChange_percent',...
                     'PeakSuspTravel_mm',...
                     'TravelChange_percent',...
                     'PeakDynamicTyreForce_N',...
                     'TyreForceChange_percent'});

writetable(summary,...
    fullfile(results_root,'parametric_study_summary.csv'));

ks = ks_ref;
cs = cs_ref;
ms = ms_ref;
road_case = 1;

disp(summary)