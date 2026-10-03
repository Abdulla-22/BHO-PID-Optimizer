clc
clear
close all

%% Parameters
m  = 1000;
b  = 50;
u  = 120;

%% Transfer Function
num = 1;
den = [m b];

G = tf(num, den);

disp('=== Transfer Function G(s) ===')
G

%% PID parameters
kp = 250;
ki = 10;
kd = 0;

%% Model
model = 'CLCCBD';

%% Run simulation
sim_time = 20;
simOut = sim(model);

%% Get output
resp = simOut.OutputResponse;

t = resp.Time;
y = resp.Data;

%% Plot
figure
plot(t,y,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Output')
title('Closed Loop Response')

%% Step response specifications
info = stepinfo(y, t);

disp('=== Closed-Loop Response Specifications ===')
disp(info)