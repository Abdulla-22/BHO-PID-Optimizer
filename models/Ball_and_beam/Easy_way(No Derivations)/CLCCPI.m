clc
clear
close all

%% Parameters
m  = 1000;
b  = 50;
u  = 120;

kp = 250;
ki = 10;

%% Model
model = 'CLCCBDPI';

%% Run simulation
simOut = sim(model);

%% Get output
resp = simOut.CLCCBDPI_RESPONSE;

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