clc;
clear;
close all;

%% ============================================================
%  Vehicle Lateral Dynamics Project
%  2-DOF Dynamic Bicycle Model
%  Main states:
%     vy = lateral velocity [m/s]
%     r  = yaw rate [rad/s]
%  Input:
%     delta = front steering angle [rad]
%% ============================================================

%% Vehicle parameters

m  = 1500;      % Vehicle mass [kg]
Iz = 2250;      % Yaw moment of inertia [kg.m^2]

a  = 1.2;       % Distance from CG to front axle [m]
b  = 1.6;       % Distance from CG to rear axle [m]
L  = a + b;     % Wheelbase [m]

Cf = 80000;     % Front axle cornering stiffness [N/rad]
Cr = 80000;     % Rear axle cornering stiffness [N/rad]

U  = 20;        % Longitudinal speed [m/s]

%% Steering input settings

step_amp_deg = 3;                 % Step steering amplitude [deg]
step_amp = step_amp_deg*pi/180;   % Step steering amplitude [rad]

sine_amp_deg = 3;                 % Sine steering amplitude [deg]
sine_amp = sine_amp_deg*pi/180;   % Sine steering amplitude [rad]

sine_freq = 0.5;                  % Sine frequency [Hz]

sim_time = 8;                     % Simulation time [s]

%% State-space matrices for verification only

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

% Verification outputs:
% y1 = yaw rate r
% y2 = sideslip beta = vy/U
% y3 = lateral acceleration ay = vy_dot + U*r

C_sim = [0 1;
         1/U 0;
         A_sim(1,1) A_sim(1,2) + U];

D_sim = [0;
         0;
         B_sim(1)];

%% Understeer gradient

Kus = m/L * (b/Cf - a/Cr);

fprintf('\n=============================================\n');
fprintf('Vehicle model initialized successfully.\n');
fprintf('=============================================\n');
fprintf('U = %.2f m/s = %.2f km/h\n', U, U*3.6);
fprintf('Cf = %.0f N/rad\n', Cf);
fprintf('Cr = %.0f N/rad\n', Cr);
fprintf('Step steering = %.2f deg\n', step_amp_deg);
fprintf('Sine steering amplitude = %.2f deg\n', sine_amp_deg);
fprintf('Understeer gradient Kus = %.6f\n', Kus);

if Kus > 0
    fprintf('Vehicle behavior: Understeer\n');
elseif abs(Kus) < 1e-6
    fprintf('Vehicle behavior: Neutral steer\n');
else
    fprintf('Vehicle behavior: Oversteer\n');
end