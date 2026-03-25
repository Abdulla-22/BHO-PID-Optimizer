clc;
clear;
close all;

%% =========================
% Select system
% 1: Ball & Beam
% 2: Cruise Control
% 3: Motor Speed
%% =========================
system_id = 1;

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

addpath(genpath(folder));
load_system(model);

%% =========================
% Initialize storage for comparison
%% =========================
methods = {'BHO','ZN'};
results = struct();

for i = 1:length(methods)
    method = methods{i};
    
    switch upper(method)
        case 'BHO'
            nPop = 20;        % Number of stars
            MaxIt = 30;       % Maximum iterations
            VarMin = [0 0 0]; % kp, ki, kd
            VarMax = [500 500 100];

            bestK = blackHole(costFcn, nPop, MaxIt, VarMin, VarMax);

        case 'ZN'
            % =========================
            % Automatic estimate of Ku & Tu without toolbox
            % =========================
            [Ku, Tu] = estimateUltimateGain(model);

            % Compute PID gains using your pid_ziegler_nichols
            [Kp, Ki, Kd] = pid_ziegler_nichols(Ku, Tu, 'classic');
            bestK = [Kp, Ki, Kd];

        otherwise
            error('Unknown tuning method');
    end

    disp(['=== BEST PID (' method ') ===']);
    disp(bestK);
    
    % Assign PID for simulation
    assignin('base','kp',bestK(1));
    assignin('base','ki',bestK(2));
    assignin('base','kd',bestK(3));
    
    % Simulate
    simOut = sim(model);
    resp = simOut.OutputResponse;
    results.(method).t = resp.Time;
    results.(method).y = resp.Data;
    results.(method).PID = bestK;
    results.(method).stepinfo = stepinfo(resp.Data, resp.Time);
end

%% =========================
% Plot comparison
%% =========================
figure; hold on; grid on;
colors = {'b','r'};
for i = 1:length(methods)
    method = methods{i};
    plot(results.(method).t, results.(method).y, 'LineWidth',2,'Color',colors{i});
end
xlabel('Time [s]');
ylabel('Output');
title('PID Comparison: BHO vs Ziegler-Nichols');
legend(methods,'Location','best');

%% =========================
% Display step info comparison
%% =========================
for i = 1:length(methods)
    method = methods{i};
    disp(['Step info (' method '):']);
    disp(results.(method).stepinfo);
end

%% =========================
% Subfunction: Estimate Ku & Tu without toolbox
%% =========================
function [Ku, Tu] = estimateUltimateGain(model)
    % Start with a small Kp
    Kp = 1;
    Ki = 0;
    Kd = 0;
    assignin('base','kp',Kp);
    assignin('base','ki',Ki);
    assignin('base','kd',Kd);
    
    maxIter = 50;
    dt = 1; % increment Kp in each step
    Ku = NaN;
    Tu = NaN;

    for k = 1:maxIter
        assignin('base','kp',Kp);
        simOut = sim(model,'StopTime','10');
        y = simOut.OutputResponse.Data;
        t = simOut.OutputResponse.Time;
        
        % approximate peaks: number of times the signal changes sign
        dy = diff(y);
        zeroCrossings = sum(dy(1:end-1).*dy(2:end) < 0);
        
        if zeroCrossings >= 6 % clear oscillation
            Ku = Kp;
            crossingTimes = t([false; dy(1:end-1).*dy(2:end) < 0]);
            Tu = mean(diff(crossingTimes))*2; % full period
            return
        end
        
        Kp = Kp + dt;
    end

    warning('No sustained oscillation detected. Use Ku and Tu manually.');
    Ku = Kp;
    Tu = 1;
end