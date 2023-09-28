clc
clear
close all

% System Parameters 
g = 9.8; % acceleration due to gravity (m/s^2)
m = 0.034; % mass of quadrotor (kg)
L = 0.032; % moment arm (m)
l = L;
kf = 0.005022; % calibration constant kf
k = kf;
km =  1.858e-5; % calibration constant km
b = km;
I = diag([2.3951 2.3951 3.2347])*1e-5; % inertia matrix (kg.m^2)
Iinv = inv(I);
I_M = 0;
MMA = [0 kf*L 0 -kf*L; ...
       -kf*L 0 kf*L 0; ...
       km -km km -km];

scale = 0; % Turn off wind gusts
scale2 = 0; % Turn off wind gusts

%scale = 0.0001; % Turn on wind gusts
%scale2 = 0.01; % Turn on wind gusts

% Choose shape: rectangle = 1; circle = 0;
shape = 0;

% inertia matrix (kg.m^2)
I_xx = I(1,1); 
I_yy = I(2,2);
I_zz = I(3,3);

% Disturbances
Fxg = 0.5*scale2;
Fyg = 0.5*scale2;

% Initial Conditions
if shape == 1
    % Rectangle Initial Conditions
    x_0 = 2;
    y_0 = 2;
    z_0 = 0;
else
    % Circle Initial Conditions
    x_0 = 1;
    y_0 = 0;
    z_0 = 0;
end

%% parameters of the PD controller
Kp_zeta = [2; 1; 1]; % [x, y, z] 3.5
Ki_zeta = [0; 0; 0]; % [x, y, z] 0
Kd_zeta = [3; 100; 5]; % [x, y, z] 4.5

Kp_eta = [20; 50; 50]; % [phi, theta, psi]
Ki_eta = [0; 0; 0]; % [phi, theta, psi]
Kd_eta = [40; 50; 50]; % [phi, theta, psi]

Tf = 60;

%% start simulation
sim('Quadrotor_Model_CP232')

figure(2)
subplot(1,3,1)
plot(t,x_ref, 'r', 'linewidth', 1.5); hold on;
plot(t,x, 'b:', 'linewidth', 2); hold on;
legend('Reference trajectory', 'Quadrotor trajectory', 'Location', 'best')
%axis([-1 11 -1 11])
grid on
xlabel('t');
ylabel('x');

subplot(1,3,2)
plot(t,y_ref, 'r', 'linewidth', 1.5); hold on;
plot(t,y, 'b:', 'linewidth', 2); hold on;
legend('Reference trajectory', 'Quadrotor trajectory', 'Location', 'best')
%axis([-1 11 -1 11])
grid on
xlabel('t');
ylabel('y');

subplot(1,3,3)
plot(t,z_ref, 'r', 'linewidth', 1.5); hold on;
plot(t,z, 'b:', 'linewidth', 2); hold on;
legend('Reference trajectory', 'Quadrotor trajectory', 'Location', 'best')
%axis([-1 11 -1 11])
grid on
xlabel('t');
ylabel('z');

figure(3)
plot3(x_ref, y_ref, z_ref, 'r', 'linewidth', 1.5); hold on;
plot3(x,y,z, 'b:', 'linewidth', 2);
legend('Reference trajectory', 'Quadrotor trajectory', 'Location', 'best')
grid on
xlabel('x');
ylabel('y');
zlabel('z');
axis equal;