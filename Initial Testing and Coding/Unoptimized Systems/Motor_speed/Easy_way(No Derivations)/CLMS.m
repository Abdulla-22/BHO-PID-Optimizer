clc
clear
close all

%% Parameters
J = 0.01;
b = 0.1;
K = 0.01;
R = 2.18;
L = 2.3e-3;

step_amp = 1;

%% Transfer Function
num = K;
den = [(J*L) ((J*R)+(L*b)) ((b*R)+K^2)];

G = tf(num, den);

disp('=== Transfer Function G(s) ===')
G

%% PID parameters
kp = 100;
ki = 200;
kd = 10;

%% Model
model = 'CLMSBD';

%% Run simulation
sim_time = 20;
simOut = sim(model);

%% Get output
resp = simOut.OutputResponse;

t = resp.Time;
y = resp.Data;

%% Plot
figure
plot(t, y, 'LineWidth', 2)
grid on
xlabel('Time (s)')
ylabel('Output')
title('Closed Loop Response')

%% Step response specifications
info = stepinfo(y, t);

disp('=== Closed-Loop Response Specifications ===')
disp(info)