clc
clear
close all

%% Parameters
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;

step_amp = 1;

num = K;
den = [(J*L) ((J*R)+(L*b)) ((b*R)+K^2)];

kp = 100;
ki = 200;
kd = 10;

%% Model
model = 'CLMSBD';

%% Run simulation
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