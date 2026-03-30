clc
clear
close all

%% Parameters
m = 0.111;
R = 0.015;
g = -9.8;
L = 1.0;
d = 0.03;
J = 9.99e-6;

step_amp = 0.25;

%% Transfer Function
num = (-1*m*g*d)/(L*((J/(R^2))+m));
den = [1 0 0];

G = tf(num, den);

disp('=== Transfer Function G(s) ===')
G

%% PID parameters
kp = 10;
ki = 0;
kd = 10;

%% Model
model = 'CLBandBBD';

%% Run simulation
sim_time = 20;
simOut = sim(model);

%% Get output
resp = simOut.OutputResponse;

t = resp.Time;
y = resp.Data;

%% Plot
figure
plot(t,y)
grid on
xlabel('Time (s)')
ylabel('Output')
title('Closed Loop Response')

%% Step response specifications
info = stepinfo(y, t);

disp('=== Closed-Loop Response Specifications ===')
disp(info)