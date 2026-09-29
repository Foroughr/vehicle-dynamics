%% Quarter-car ride model
% Baseline parameters for the passive suspension system

clear
clc
close all

%% Project folders

project_path = fileparts(mfilename('fullpath'));
cd(project_path)

if ~exist('figures','dir')
    mkdir('figures')
end

if ~exist('inputs','dir')
    mkdir('inputs')
end

if ~exist('report','dir')
    mkdir('report')
end

if ~exist('results','dir')
    mkdir('results')
end

%% Vehicle parameters
% Reference values are selected based on the lecture notes

g = 9.81;              % gravitational acceleration (m/s^2)

ms = 454.5;            % sprung mass (kg)
mu = 0.1*ms;           % unsprung mass (kg)

%% Suspension and tyre parameters

kt = 176e3;            % tyre stiffness (N/m)
ks = kt/8;             % suspension stiffness (N/m)

zeta = 0.3;            % suspension damping ratio
cs = 2*zeta*sqrt(ks*ms);

ct = 0;                % tyre damping is neglected

%% Mass, damping and stiffness matrices

M = [ms  0;
     0   mu];

C = [cs   -cs;
    -cs   cs+ct];

K = [ks   -ks;
    -ks   ks+kt];

%% Static tyre load

Fz0 = (ms+mu)*g;

%% Natural frequencies
% Used as an initial check of the model parameters

lambda = eig(K,M);
fn = sort(sqrt(lambda)/(2*pi));

f_body = fn(1);
f_wheel = fn(2);

%% Road profile

road_case = 4;    % 1: step, 2: speed bump, 3: sine, 4: random

[random_road_ts, random_road_info] = generate_random_road;

%% Active suspension PID parameters

active_mode = 0;       % 0: passive, 1: active

Kp_pid = 10000;        % N/m
Ki_pid = 2000;         % N/(m.s)
Kd_pid = 8000;         % N.s/m

N_pid = 200;           % derivative filter coefficient
Fa_max = 2500;         % actuator force limit (N)

%% Save the parameters

save('quarter_car_parameters.mat',...
    'g','ms','mu','ks','cs','kt','ct',...
    'zeta','M','C','K','Fz0','fn',...
    'f_body','f_wheel','road_case');

%% Save a summary table

T = table(ms,mu,ks,cs,kt,ct,Fz0,f_body,f_wheel,road_case);

writetable(T,...
    fullfile('results','baseline_parameters.csv'))

%% Display the selected values

disp('Baseline quarter-car parameters:')
disp(T)