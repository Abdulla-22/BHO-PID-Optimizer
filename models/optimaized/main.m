clc
clear
close all

%% =========================
% Select system
% 1: Ball & Beam
% 2: Cruise Control
% 3: Motor Speed
%% =========================
system_id = 3;

switch system_id
    
    case 1
        folder = "Ball_and_beam";
        model  = "CLBandBBD";
        costFcn = @(K) cost_ballandbeam(K, model);
        
    case 2
        folder = "cruise_control";
        model  = "CLCCBD";
        costFcn = @(K) cost_cruise(K, model);
        
    case 3
        folder = "Motor_speed";
        model  = "CLMSBD";
        costFcn = @(K) cost_motor(K, model);
        
end

addpath(genpath(folder))
load_system(model)

%% =========================
% Black Hole Optimization
%% =========================
nPop = 20;        % Number of Stars
MaxIt = 30;       % Number of Iterations

VarMin = [0 0 0];     % kp, ki, kd
VarMax = [500 500 100];

bestK = blackHole(costFcn, nPop, MaxIt, VarMin, VarMax);

disp('=== BEST PID ===')
disp(bestK)

%% =========================
% Final Simulation
%% =========================
kp = bestK(1);
ki = bestK(2);
kd = bestK(3);

assignin('base','kp',kp);
assignin('base','ki',ki);
assignin('base','kd',kd);

simOut = sim(model);

resp = simOut.OutputResponse;
t = resp.Time;
y = resp.Data;

figure
plot(t,y,'LineWidth',2)
xlim([0 10])
grid on
title('Optimized Response')

info = stepinfo(y,t);
disp(info)